/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../memory/CircularBuffer.hpp"
#include "../../parallel/BackgroundWorker.hpp"
#include "../serialization/serialization.hpp"
#include "NetworkSocketOptions.hpp"
#include "NetworkSocketSignal.hpp"
#include "NetworkSocketType.hpp"
#include "TCPClientOptions.hpp"
#include "TCPServerOptions.hpp"
#include "UDPSocketOptions.hpp"
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_network
     * \{
     */

    //! Abstraction layer for TCP/UDP server/client
    /*!
     * The class abstracts the hardware layer for establishing network
     * connections. You can choose between TCP and UDP server/client modes.
     *
     * A background worker is used as backend to allow multi-threaded sending
     * and receiving of buffered objects. In multi-threaded mode the class
     * automatically (re-)connects to the corresponding counterpart. Optional
     * heartbeat messages allow to detect timeouts. After a timeout the class
     * closes the connection and automatically tries to reconnect. Received
     * heartbeats will not appear in the buffer of received objects, since
     * they are filtered out automatically.
     *
     * One can send and receive objects (serialized as byte stream) by directly
     * accessing the buffers \ref m_objectsToSend and \ref m_receivedObjects (thread-safe).
     * The (de-)serialization before sending/after receiving is done automatically
     * by the background thread for efficiency. The class is implemented as template
     * to specify varying types of objects to send and receive.
     *
     * \par Notes on TCP
     * TCP is a stream protocol (bytes come in continuously), thus we can set
     * the socket buffer size to anything we want. Drawback: we have to detect message
     * start and end on our own. Thus each message is preceded with a **header** which
     * stores the length of the message (payload size). This allows to split the
     * continuous data stream into discrete messages and afterwards parse them to objects.
     *
     * \par Notes on UDP
     * UDP is a datagram protocol. Each call of `recvfrom()` will return **one** datagram.
     * If the buffer is not big enough, the remaining data will be discarded.
     * If fragmentation should be avoided for reliability, the sender should only
     * send datagrams of size 512 bytes or lesser. To be safe one should use a buffer
     * size of >4096 bytes. As each datagram corresponds to a message/object, there is
     * no need for preceding size specifications.
     */
    template <class SendObjectType, class ReceiveObjectType>
    class NetworkSocket final : public parallel::BackgroundWorker {
    public:
        // Type definitions
        typedef serialization::BinaryStream DataStream; //!< Specification of data stream type (byte stream format used by the socket)
        typedef serialization::BinaryStreamSize DataStreamSize; //!< Specifies the used type for describing the size of a data stream

        //! Constructor (runs in parent thread)
        /*!
         * \param [in] options Initializes: \copybrief options()
         */
        NetworkSocket(const NetworkSocketOptions& options)
            : BackgroundWorker(options.m_name, options.m_multiThreaded, options.m_threadPriority, options.m_minimumCycleTime, options.m_blockingWaiting)
            , m_options(options)
            , m_socket(-1)
            , m_listenerSocket(-1)
            , m_isConnected(false)
            , m_connectionAttemptInterval(1)
            , m_lastConnectionAttempt(0)
            , m_sendHeartBeats(true)
            , m_heartBeatSendCode({ 0xFF }) // Default byte stream for sending heartbeats
            , m_heartBeatSendInterval(0.1)
            , m_lastHeartBeatSent(0)
            , m_receiveHeartBeats(true)
            , m_heartBeatReceiveCode({ 0xFF }) // Default byte stream for receiving heartbeats
            , m_heartBeatTimeout(5)
            , m_lastHeartBeatReceived(0)
            , m_signalBuffer(1000)
            , m_receivedObjects(options.m_receivedObjectBufferSize) // Initializes circular buffer
            , m_objectsToSend(options.m_sendObjectBufferSize) // Initializes circular buffer
        {
            // Avoid program exit by "broken pipe"
            // (occurs, when remote socket closes the connection and we want to write to the socket)
            signal(SIGPIPE, SIG_IGN);

            // Pre-allocate memory for socket buffers (to avoid dynamic allocation)
            m_socketSendBuffer.reserve(m_options.m_maximumMessageSize);
            if (m_options.m_type == NetworkSocketType::TCP_CLIENT || m_options.m_type == NetworkSocketType::TCP_SERVER)
                m_socketReceiveBuffer.reserve(2 * m_options.m_maximumMessageSize); // May has to hold more than one message
            if (m_options.m_type == NetworkSocketType::UDP)
                m_socketReceiveBuffer.reserve(m_options.m_maximumMessageSize); // Has to hold only one datagram (=message)
        }

    private:
        //! Copy constructor (internal) (**not** thread-safe -> should only be called by the thread-safe wrapper)
        /*! \param [in] original Reference to original object. */
        NetworkSocket(const NetworkSocket<SendObjectType, ReceiveObjectType>& original, const int& /* <- trick used for locking mutex */)
            : BackgroundWorker(original)
            , m_options(original.m_options)
            , m_socket(-1) // m_socket = DO NOT COPY FROM ORIGINAL (use own socket)
            , m_listenerSocket(-1) // m_listenerSocket = DO NOT COPY FROM ORIGINAL (use own socket)
            , m_socketReceiveBuffer(original.m_socketReceiveBuffer)
            , m_socketSendBuffer(original.m_socketSendBuffer)
            , remoteAddress(original.remoteAddress)
            , ownAddress(original.ownAddress)
            , m_isConnected(original.m_isConnected)
            , m_connectionAttemptInterval(original.m_connectionAttemptInterval)
            , m_lastConnectionAttempt(original.m_lastConnectionAttempt)
            , m_sendHeartBeats(original.m_sendHeartBeats)
            , m_heartBeatSendCode(original.m_heartBeatSendCode)
            , m_heartBeatSendInterval(original.m_heartBeatSendInterval)
            , m_lastHeartBeatSent(original.m_lastHeartBeatSent)
            , m_receiveHeartBeats(original.m_receiveHeartBeats)
            , m_heartBeatReceiveCode(original.m_heartBeatReceiveCode)
            , m_heartBeatTimeout(original.m_heartBeatTimeout)
            , m_lastHeartBeatReceived(original.m_lastHeartBeatReceived)
            , m_signalBuffer(original.m_signalBuffer)
            , m_receivedObjects(original.m_receivedObjects)
            , m_objectsToSend(original.m_objectsToSend)
        {
        }

    public:
        //! Copy constructor (wrapper) (**thread-safe**)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        NetworkSocket(const NetworkSocket<SendObjectType, ReceiveObjectType>& original)
            : NetworkSocket(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (**thread-safe**)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        NetworkSocket& operator=(const NetworkSocket<SendObjectType, ReceiveObjectType>& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            BackgroundWorker::operator=(reference);
            m_options = reference.m_options;
            m_socket = -1; // m_socket = DO NOT COPY FROM ORIGINAL (use own socket)
            m_listenerSocket = -1; // m_listenerSocket = DO NOT COPY FROM ORIGINAL (use own socket)
            m_socketReceiveBuffer = reference.m_socketReceiveBuffer;
            m_socketSendBuffer = reference.m_socketSendBuffer;
            remoteAddress = reference.remoteAddress;
            ownAddress = reference.ownAddress;
            m_isConnected = reference.m_isConnected;
            m_connectionAttemptInterval = reference.m_connectionAttemptInterval;
            m_lastConnectionAttempt = reference.m_lastConnectionAttempt;
            m_sendHeartBeats = reference.m_sendHeartBeats;
            m_heartBeatSendCode = reference.m_heartBeatSendCode;
            m_heartBeatSendInterval = reference.m_heartBeatSendInterval;
            m_lastHeartBeatSent = reference.m_lastHeartBeatSent;
            m_receiveHeartBeats = reference.m_receiveHeartBeats;
            m_heartBeatReceiveCode = reference.m_heartBeatReceiveCode;
            m_heartBeatTimeout = reference.m_heartBeatTimeout;
            m_lastHeartBeatReceived = reference.m_lastHeartBeatReceived;
            m_signalBuffer = reference.m_signalBuffer;
            m_receivedObjects = reference.m_receivedObjects;
            m_objectsToSend = reference.m_objectsToSend;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

        //! Destructor
        ~NetworkSocket()
        {
            // De-initialize (if not already triggered)
            deInitialize();
        }

        //! Initialize the \ref NetworkSocket (runs in parent thread)
        /*!
         * Starts background thread.
         * \return `true` on success, `false` on failure
         */
        bool initialize()
        {
            // Initialize temporary exchange objects
            m_currentReceivedObject = std::unique_ptr<ReceiveObjectType>(new ReceiveObjectType); // Dynamically allocated since this may be too large for static allocation
            m_currentObjectToSend = std::unique_ptr<SendObjectType>(new SendObjectType); // Dynamically allocated since this may be too large for static allocation

            // Start background thread
            return start();
        }

        //! De-initialize the \ref NetworkSocket (runs in parent thread)
        /*!
         * Waits until background thread has finished its work.
         * Tries to close open connections.
         * Clears all buffers.
         * \param [in] timeout Timeout for waiting until background thread finished its work.
         * \return `true` if background worker was shutdown properly, `false` otherwise
         */
        bool deInitialize(const core::Time& timeout = core::Time(1))
        {
            // Stop background thread
            stop();

            // Try to shutdown and close socket connections
            try {
                if (m_listenerSocket != -1) {
                    shutdown(m_listenerSocket, SHUT_RDWR);
                    close(m_listenerSocket);
                }
                if (m_socket != -1) {
                    shutdown(m_socket, SHUT_RDWR);
                    close(m_socket);
                }
                setIsConnected(false);
            } catch (...) {
            }

            // Join background thread and remember result
            const bool threadJoinSuccessful = join(timeout);

            // Clear buffers
            m_signalBuffer.clear();
            m_receivedObjects.clear();
            m_objectsToSend.clear();

            // Pass back result
            return threadJoinSuccessful;
        }

    private:
        // Hide base class functionality (use initialize() and deInitialize() instead!)
        using BackgroundWorker::join;
        using BackgroundWorker::start;
        using BackgroundWorker::stop;

    private:
        //! Main loop cycle (runs in background thread)
        /*!
         * Automatically connects to remote socket (network counterpart). Tries to reconnect, if connection is lost.
         * Handles receive and send buffers. Also handles heartbeats if enabled.
         */
        void execute()
        {
            // Step 1: check heartbeat timeout
            checkHeartbeatTimeout();

            // Emergency exit point
            if (stopTriggered() == true)
                return;

            // Step 2: (re-)connect to remote socket
            reConnect();

            // Emergency exit point
            if (stopTriggered() == true)
                return;

            // Step 3: receive new data
            receiveData();

            // Emergency exit point
            if (stopTriggered() == true)
                return;

            // Step 4: sending heartbeat
            sendHeartBeat();

            // Emergency exit point
            if (stopTriggered() == true)
                return;

            // Step 5: send data
            sendData();
        }

        // Main functionality
        // ------------------
        //! Checks, if the timeout for heartbeats is hit
        /*!
         * If the timeout is hit, the socket is closed automatically.
         */
        void checkHeartbeatTimeout()
        {
            // Only, if we are connected and receiving heartbeats is enabled
            if (isConnected() == true && receiveHeartBeats() == true) {
                // Check, if time limit is hit...
                if (core::Time::currentTime() - lastHeartBeatReceived() > heartBeatTimeout()) {
                    // ... we hit the the time limit -> disconnect
                    m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::DISCONNECT_HEARTBEAT_TIMEOUT));
                    socketClose();
                }
            }
        }

        //! Tries to (re-)connect to remote socket
        /*!
         * Returns without any action, if the connection is already open. Automatically avoids too many connection attempts (see \ref connectionAttemptInterval()).
         * In case a new connection could be setup successfully, all buffers are reset.
         */
        void reConnect()
        {
            // Skip, if we are already connected
            if (isConnected() == false) {
                // Wait between connection attempts
                if (core::Time::currentTime() - lastConnectionAttempt() >= connectionAttemptInterval()) {
                    // Connection is closed -> try to (re-)open it
                    m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::INFORMATION, NetworkSocketSignal::Message::CONNECT_NEW_TRY));

                    // Force close connection (to start properly)
                    socketClose();

                    // Try to open connection
                    setLastConnectionAttempt(core::Time::currentTime());
                    if (socketOpen() == false) {
                        // Check, if stop signal was received
                        if (stopTriggered() == true)
                            return;
                        else
                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::CONNECT_FAILED_SOCKET_OPEN));
                    } else {
                        m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::INFORMATION, NetworkSocketSignal::Message::CONNECT_SUCCESS));

                        // Reinitialize timers
                        core::Time currentTime = core::Time::currentTime();
                        setLastHeartBeatSent(currentTime);
                        setLastHeartBeatReceived(currentTime);

                        // Clear send/receive buffers
                        m_socketReceiveBuffer.clear();
                        m_socketSendBuffer.clear();
                        m_receivedObjects.clear();
                        m_objectsToSend.clear();
                    }
                }
            }
        }

        //! Tries to receive and automatically deserialize data from the remote socket
        /*!
         * Skips, if connection is not open. Automatically reads all available data from the socket buffer and tries
         * to parse it. Heartbeats are filtered out and user objects are deserialized and stored in the \ref m_receivedObjects buffer.
         */
        void receiveData()
        {
            // Skip, if we are not connected
            if (isConnected() == true) {
                // Grab protected parameters in advance (to avoid race conditions)
                const DataStream heartBeatCode = heartBeatReceiveCode();

                // Loop until there is no incoming data left
                while (isConnected() == true && stopTriggered() == false) {
                    // Try to get incoming data
                    if (socketReadData() <= 0) {
                        // ...there was no data or an error occured -> stop checking...
                        break;
                    } else {
                        // ... we received some data

                        // UDP
                        // ---
                        // Each datagram is a complete message -> immediately parse messages to objects
                        if (options().m_type == NetworkSocketType::UDP) {
                            // Check, if heartbeat was received
                            bool heartBeatReceived = false;
                            if (m_socketReceiveBuffer.size() == heartBeatCode.size())
                                if (std::equal(m_socketReceiveBuffer.begin(), m_socketReceiveBuffer.end(), heartBeatCode.begin())) {
                                    heartBeatReceived = true;
                                    setLastHeartBeatReceived(core::Time::currentTime());
                                }
                            if (heartBeatReceived == false) {
                                // Try to deserialize new object from stream
                                if (m_currentReceivedObject->deSerialize(m_socketReceiveBuffer, 0, options().m_endianness) == 0)
                                    m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::DESERIALIZE_FAILED));
                                else
                                    m_receivedObjects.push(*m_currentReceivedObject);
                            }
                        }
                    }

                    // UDP
                    // ---
                    // Clear socket buffer for next datagram
                    if (options().m_type == NetworkSocketType::UDP)
                        m_socketReceiveBuffer.clear(); // Should have no effect on capacity (allocated memory)
                }

                // TCP
                // ---
                // Data comes in as stream -> try to parse every time there is no new data available in the socket
                if (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::TCP_CLIENT) {
                    // Parse messages (loop until an incomplete message occurs)
                    while (stopTriggered() == false) {
                        // Check, if continuous buffer is big enough to hold the message header
                        if (m_socketReceiveBuffer.size() >= sizeof(DataStreamSize)) {
                            // Parse message header
                            DataStreamSize messageLength = 0;
                            serialization::deSerialize(m_socketReceiveBuffer, 0, options().m_endianness, messageLength);

                            // Helper for best performance: we know what the expected message length is -> thus we can already reserve enough memory in the receive buffer for it (with extra buffer -> factor 2)
                            m_socketReceiveBuffer.reserve(2 * messageLength);

                            // Check, if continuous buffer is big enough to hold the header and the complete message
                            if (m_socketReceiveBuffer.size() >= sizeof(DataStreamSize) + messageLength) {
                                // ...yes -> we can cut the message out of the buffer (if not empty)
                                if (messageLength > 0) {
                                    // Check, if heartbeat was received
                                    bool heartBeatReceived = false;
                                    if (messageLength == heartBeatCode.size())
                                        if (std::equal(m_socketReceiveBuffer.begin() + sizeof(DataStreamSize), m_socketReceiveBuffer.begin() + sizeof(DataStreamSize) + messageLength, heartBeatCode.begin())) {
                                            heartBeatReceived = true;
                                            setLastHeartBeatReceived(core::Time::currentTime());
                                        }
                                    if (heartBeatReceived == false) {
                                        // Try to deserialize new object from stream
                                        if (m_currentReceivedObject->deSerialize(m_socketReceiveBuffer, sizeof(DataStreamSize), options().m_endianness) != messageLength)
                                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::DESERIALIZE_FAILED));
                                        else
                                            m_receivedObjects.push(*m_currentReceivedObject);
                                    }
                                }

                                // Check, if data is remaining
                                const DataStreamSize remainingDataLength = m_socketReceiveBuffer.size() - sizeof(DataStreamSize) - messageLength;
                                if (remainingDataLength > 0) {
                                    // Shift contents of buffer to the front
                                    for (DataStreamSize i = 0; i < remainingDataLength; i++)
                                        m_socketReceiveBuffer[i] = m_socketReceiveBuffer[sizeof(DataStreamSize) + messageLength + i];

                                    // Delete duplicate old data at the end
                                    m_socketReceiveBuffer.resize(remainingDataLength); // Should have no effect on capacity (allocated memory)
                                } else {
                                    // Clear complete buffer
                                    m_socketReceiveBuffer.clear(); // Should have no effect on capacity (allocated memory)
                                    break;
                                }
                            } else {
                                // ... data is too short to hold the header and the payload -> wait until payload is complete
                                break;
                            }
                        } else {
                            // ... data is too short for header -> wait until header is complete
                            break;
                        }
                    }
                }
            }
        }

        //! Sends a heartbeat to the remote socket
        /*!
         * Automatically checks, if the connection is open, sending heartbeats is enabled and it is time to send a new heartbeat.
         */
        void sendHeartBeat()
        {
            // Only, if we are connected and sending heartbeats is enabled
            if (isConnected() == true && sendHeartBeats() == true) {
                // Check, if it is time for a new heartbeat
                if (core::Time::currentTime() - lastHeartBeatSent() >= heartBeatSendInterval()) {
                    // Clear send buffer
                    m_socketSendBuffer.clear();

                    // Grab protected parameters in advance (to avoid race conditions)
                    const DataStream heartBeatCode = heartBeatSendCode();

                    // TCP -> send message WITH header
                    if (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::TCP_CLIENT)
                        serialization::serialize(m_socketSendBuffer, options().m_endianness, (DataStreamSize)heartBeatCode.size());

                    // Add actual heartbeat message (=payload)
                    serialization::append(m_socketSendBuffer, heartBeatCode);

                    // Try to send message
                    if (socketSendData(m_socketSendBuffer) == true)
                        setLastHeartBeatSent(core::Time::currentTime()); // Remember event
                }
            }
        }

        //! Sends user objects to the remote socket
        /*!
         * Automatically serializes the objects from the \ref m_objectsToSend buffer and attaches a
         * header if necessary (TCP).
         */
        void sendData()
        {
            // Only, if we are connected
            if (isConnected() == true) {
                // Iterate over all objects in the buffer
                while (m_objectsToSend.pop(*m_currentObjectToSend) > 0 && stopTriggered() == false) {
                    // Clear send buffer
                    m_socketSendBuffer.clear();

                    if (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::TCP_CLIENT) {
                        // TCP -> send message WITH header

                        // Add placeholder for message length (unknown up to now)
                        const DataStreamSize headerSize = serialization::serialize(m_socketSendBuffer, options().m_endianness, (DataStreamSize)0);

                        // Add actual DataStream (=payload)
                        const DataStreamSize reportedPayloadSize = m_currentObjectToSend->serialize(m_socketSendBuffer, options().m_endianness);
                        const DataStreamSize actualPayLoadSize = m_socketSendBuffer.size() - headerSize;
                        if (reportedPayloadSize != actualPayLoadSize) {
                            // Reported and actual payload sizes do not match!
                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SERIALIZE_FAILED));
                            break;
                        }

                        // Replace header-placeholder with correct size information
                        DataStream headerStream;
                        serialization::serialize(headerStream, options().m_endianness, actualPayLoadSize);
                        for (DataStreamSize i = 0; i < headerSize; i++)
                            m_socketSendBuffer[i] = headerStream[i];

                        // Try to send current element
                        if (socketSendData(m_socketSendBuffer) == false)
                            break; // Something went wrong -> exit loop
                    } else {
                        // Write message (=payload without header)
                        m_currentObjectToSend->serialize(m_socketSendBuffer, options().m_endianness);

                        // Size check
                        if (m_socketSendBuffer.size() > options().m_maximumMessageSize)
                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::WARNING, NetworkSocketSignal::Message::SERIALIZE_MAX_SIZE));

                        // UDP -> send message WITHOUT header
                        if (socketSendData(m_socketSendBuffer) == false)
                            break; // Something went wrong -> exit loop
                    }
                }
            }
        }

        // Low level encapsulated socket functions
        // ---------------------------------------
        //! Tries to open socket (runs in background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * \return `true` on success, `false` on failure
         */
        bool socketOpen()
        {
            bool success = false;
            try {
                // Get address objects
                // ------------------
                // Set remote address
                bool gotRemoteAddress = false;
                memset(&remoteAddress, 0, sizeof(remoteAddress)); // Empty struct
                if (options().m_type == NetworkSocketType::TCP_SERVER) {
                    // We don't know the client for now (we have to wait for a connection request by the client)
                    // --> do nothing...
                    gotRemoteAddress = true;
                } else {
                    remoteAddress.sin_family = AF_INET; // Use IPv4 protocol
                    remoteAddress.sin_port = htons(options().m_remotePort); // Set remote port number

                    // Try to convert remote IP address to binary format
                    in_addr_t binaryRemoteAddress; // Binary representation of IP address
                    if ((binaryRemoteAddress = inet_addr(options().m_remoteIP.c_str())) != INADDR_NONE) {
                        // IP is given in numeric format
                        // -> conversion was successful
                        // -> we can use it directly
                        // -> copy binary address to data container
                        memcpy((char*)&remoteAddress.sin_addr, &binaryRemoteAddress, sizeof(binaryRemoteAddress));
                        gotRemoteAddress = true;
                    } else {
                        // IP could not be converted automatically
                        // -> maybe IP is given in text form (e.g. "localhost")
                        // -> convert to valid IP address

                        // Try to get host information
                        hostent* hostDatabaseEntry = gethostbyname(options().m_remoteIP.c_str());
                        if (hostDatabaseEntry == nullptr) {
                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_HOST_IDENTIFICATION));
                            gotRemoteAddress = false;
                        } else {
                            // We could identify the host by its name
                            // -> copy binary address to data container
                            memcpy((char*)&remoteAddress.sin_addr, hostDatabaseEntry->h_addr, hostDatabaseEntry->h_length);
                            gotRemoteAddress = true;
                        }
                    }
                }

                // Set own address
                if (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::UDP) {
                    memset(&ownAddress, 0, sizeof(ownAddress));
                    ownAddress.sin_family = AF_INET; // Use IPv4 protocol
                    ownAddress.sin_port = htons(options().m_ownPort); // Set own port number
                    if (options().m_type == NetworkSocketType::TCP_SERVER)
                        ownAddress.sin_addr.s_addr = htonl(INADDR_ANY); // Accept any client IP address
                }

                // Open connection
                // ---------------
                // Check, if we got the address objects properly
                if (gotRemoteAddress == false)
                    success = false;
                else {
                    // TCP server
                    // ----------
                    if (options().m_type == NetworkSocketType::TCP_SERVER) {
                        success = setupTCPServerSocket();
                    }

                    // TCP client
                    // ----------
                    if (options().m_type == NetworkSocketType::TCP_CLIENT) {
                        success = setupTCPClientSocket();
                    }

                    // UDP
                    // ---
                    if (options().m_type == NetworkSocketType::UDP) {
                        success = setupUDPSocket();
                    }

                    // Try to set socket non-blocking
                    // ------------------------------
                    if (success == true) {
                        // Try to get socket flags (options)
                        int socketFlags = fcntl(m_socket, F_GETFL, 0);
                        if (socketFlags == -1)
                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_GET_FLAGS, strerror(errno)));
                        else {
                            // Add non-blocking option to flags
                            socketFlags = socketFlags | O_NONBLOCK;

                            // Try to set socket non-blocking
                            if (fcntl(m_socket, F_SETFL, socketFlags) == -1)
                                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SET_NONBLOCKING, strerror(errno)));
                        }
                    }

                    // Disable Nagle algorithm for TCP connections (no buffering -> instant transfer, however, less throughput)
                    // -------------------------------------------
                    if (success == true && (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::TCP_CLIENT)) {
                        int enableTCPNoDelay = 1;
                        if (setsockopt(m_socket, IPPROTO_TCP, TCP_NODELAY, &enableTCPNoDelay, sizeof(int)) == -1) {
                            m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SET_TCPNODELAY, strerror(errno)));
                            success = false;
                        }
                    }
                }
            } catch (...) {
                success = false;
            }
            setIsConnected(success);
            return success;
        }

        /*!
         * \brief Setup the socket for TCP server connections
         * @return True on success
         */
        bool setupTCPServerSocket()
        {
            // Create listener socket
            m_listenerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (m_listenerSocket == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_LISTENER_CREATION, strerror(errno)));
                return false;
            }

            // Avoid blocking ports after forced exit of program
            // -> allow reuse of address
            int enableAddressReuse = 1;
            if (setsockopt(m_listenerSocket, SOL_SOCKET, SO_REUSEADDR, &enableAddressReuse, sizeof(int)) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_LISTENER_SET_REUSEADDR, strerror(errno)));
                return false;
            }

            // Bind socket to the address
            if (::bind(m_listenerSocket, (struct sockaddr*)&ownAddress, sizeof(ownAddress)) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_LISTENER_BIND, strerror(errno)));
                return false;
            }

            // Start listening
            if (listen(m_listenerSocket, 5) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_LISTENER_LISTEN, strerror(errno)));
                return false;
            }

            // Accept connection from client (BLOCKING!)
            unsigned int remoteAddressLength = sizeof(remoteAddress);
            m_socket = accept(m_listenerSocket, (struct sockaddr*)&remoteAddress, &remoteAddressLength);
            if (m_socket == -1) {
                // Check, if stop signal was received
                if (stopTriggered() == false)
                    m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_ACCEPT, strerror(errno)));
                return false;
            }

            // Close listener (stop accepting requests)
            if (close(m_listenerSocket) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_LISTENER_CLOSE, strerror(errno)));
                return false;
            }

            return true;
        }

        /*!
         * \brief Setup the socket for TCP client connections
         * @return True on success
         */
        bool setupTCPClientSocket()
        {
            // Create socket
            m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (m_socket == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_CREATION, strerror(errno)));
                return false;
            }

            // Try to connect to remote server (BLOCKING (with timeout)!)
            if (connect(m_socket, (struct sockaddr*)&remoteAddress, sizeof(remoteAddress)) == -1) {
                // Check, if stop signal was received
                if (stopTriggered() == false)
                    m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_CONNECT, strerror(errno)));
                return false;
            }

            return true;
        }

        /*!
         * \brief Setup the socket for UDP connections
         * @return True on success
         */
        bool setupUDPSocket()
        {
            int trueAsInteger = 1;
            // Create socket
            m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (m_socket == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_CREATION, strerror(errno)));
                return false;
            }

            // Avoid blocking ports after forced exit of program -> allow reuse of address
            if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, &trueAsInteger, sizeof(int)) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_SET_REUSEADDR, strerror(errno)));
                return false;
            }

            // Broadcasts require "permissions"
            if (setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, &trueAsInteger, sizeof(int)) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_SET_BROADCAST, strerror(errno)));
                return false;
            }

            // Bind socket to the address
            if (::bind(m_socket, (struct sockaddr*)&ownAddress, sizeof(ownAddress)) == -1) {
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_OPEN_FAILED_SOCKET_BIND, strerror(errno)));
                return false;
            }

            return true;
        }

        //! Tries to close socket (runs in own OR background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * \return `true` on success, `false` on failure
         */
        bool socketClose()
        {
            bool success = false;
            try {
                if (close(m_socket) == -1)
                    success = false;
                else
                    success = true;
            } catch (...) {
                success = false;
            }
            setIsConnected(false);
            return success;
        }

        //! Tries to send data over the socket (runs in background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * \param [in] data Message to send.
         * \return `true` on success, `false` on failure
        */
        bool socketSendData(const DataStream& data)
        {
            bool success = false;
            try {
                // TCP
                // ---
                // To avoid overflow of the socket send buffer, all messages are split into chunks of m_maximumMessageSize
                if (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::TCP_CLIENT) {
                    // Initialize helpers
                    success = true; // No error happened so far
                    size_t currentStartIndex = 0; // Index of starting byte in current iteration
                    while (currentStartIndex < data.size() && stopTriggered() == false) {
                        // Compute index of end byte in current iteration
                        size_t currentEndIndex = currentStartIndex + options().m_maximumMessageSize - 1;
                        if (currentEndIndex > data.size() - 1)
                            currentEndIndex = data.size() - 1;

                        // Compute count of desired bytes to send in current interation
                        const size_t desiredByteCount = currentEndIndex - currentStartIndex + 1;

                        // Try to send desired byte stream
                        const ssize_t actualByteCount = send(m_socket, &data[currentStartIndex], desiredByteCount, 0);

                        // Check result
                        if (actualByteCount >= 0) {
                            // Option A: actualByteCount > desiredByteCount -> this should NEVER happen (but handle as regular success)
                            // Option B: actualByteCount == desiredByteCount -> success -> update start index for next iteration
                            // Option C: actualByteCount < desiredByteCount -> only a part of the message was sent -> try to send rest in next iteration
                            currentStartIndex += actualByteCount;
                        } else {
                            // An error occured -> check type of error
                            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                // The socket send buffer may be full -> try again
                            } else {
                                // Some other error -> sending failed!
                                success = false;
                                break;
                            }
                        }
                    }
                }

                // UDP
                // ---
                // Each object is one datagram -> splitting messages makes no sense since the order of the packets at the receivers side is not guaranteed!
                if (options().m_type == NetworkSocketType::UDP) {
                    // Initialize helpers
                    const size_t desiredByteCount = data.size(); // Send everything all at once
                    while (stopTriggered() == false) {
                        // Try to send desired byte stream
                        const ssize_t actualByteCount = sendto(m_socket, &data[0], desiredByteCount, 0, (struct sockaddr*)&remoteAddress, sizeof(remoteAddress));

                        // Check result
                        if (actualByteCount >= (ssize_t)desiredByteCount) {
                            // Option A: actualByteCount > desiredByteCount -> this should NEVER happen (but handle as regular success)
                            // Option B: actualByteCount == desiredByteCount -> success
                            success = true;
                            break;
                        } else if (actualByteCount < (ssize_t)desiredByteCount && actualByteCount > 0) {
                            // Only a part of the message was sent -> broken datagram!
                            success = false;
                            break;
                        } else {
                            // An error occured -> check type of error
                            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                // The socket send buffer may be full -> try again
                            } else {
                                // Some other error -> sending failed!
                                success = false;
                                break;
                            }
                        }
                    }
                }
            } catch (...) {
                success = false;
            }
            if (success == false) {
                // Something went wrong -> close connection
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_SEND_DATA_FAILED, strerror(errno)));
                socketClose();
            }
            return success;
        }

        //! Tries to receive data over the socket (runs in background thread)
        /*!
         * Convenience function to catch low-level exceptions.
         * Appends new incoming data to the socket buffer. Has to be called more than
         * once to really get all the data (buffered).
         * \return Count of received bytes or -1 in case of failure.
         */
        int64_t socketReadData()
        {
            // Initialize helpers
            int64_t newDataLength = 0; // Length of new received data (-1 on failure)
            size_t startingIndex = m_socketReceiveBuffer.size(); // Starting index of new data in buffer

            // Initialize receive buffer with dummy data
            m_socketReceiveBuffer.resize(m_socketReceiveBuffer.size() + options().m_maximumMessageSize);

            try {
                // TCP
                // ---
                if (options().m_type == NetworkSocketType::TCP_SERVER || options().m_type == NetworkSocketType::TCP_CLIENT) {
                    if ((newDataLength = recv(m_socket, &m_socketReceiveBuffer[startingIndex], options().m_maximumMessageSize, 0)) == -1) {
                        // Check, if no bytes are available (non-blocking socket version)
                        if (errno == EWOULDBLOCK || errno == EAGAIN)
                            newDataLength = 0; // Success, but no data
                        else
                            newDataLength = -1; // Error
                    }
                }

                // UDP
                // ---
                if (options().m_type == NetworkSocketType::UDP) {
                    // Create address memory of sender (may vary with each packet)
                    sockaddr_in senderAddress; // Contains IP address and port of sender
                    unsigned int senderAddressLength = sizeof(senderAddress);

                    if ((newDataLength = recvfrom(m_socket, &m_socketReceiveBuffer[startingIndex], options().m_maximumMessageSize, 0, (struct sockaddr*)&senderAddress, &senderAddressLength)) == -1) {
                        // Check, if no bytes are available (non-blocking socket version)
                        if (errno == EWOULDBLOCK || errno == EAGAIN)
                            newDataLength = 0; // Success, but no data
                        else
                            newDataLength = -1; // Error
                    }
                }
            } catch (...) {
                newDataLength = -1; // Error
            }
            if (newDataLength >= 0) {
                // Resize to actual length of new data
                m_socketReceiveBuffer.resize(startingIndex + newDataLength);
            } else {
                // Something went wrong -> close connection
                m_signalBuffer.push(NetworkSocketSignal(NetworkSocketSignal::Type::ERROR, NetworkSocketSignal::Message::SOCKET_READ_DATA_FAILED, strerror(errno)));
                socketClose();

                // Discard corrupt data
                m_socketReceiveBuffer.resize(startingIndex);
            }
            return newDataLength;
        }

        // Constants
        // ---------
        // No protection necessary, since they are read-only
        NetworkSocketOptions m_options; //!< \copybrief options()

        // Protected memory
        // ----------------
        // Network objects (no mutex necessary, since they are only accessed by the protected execute() function)
        int m_socket; //!< Communication socket
        int m_listenerSocket; //!< Listener socket (TCP server only)
        DataStream m_socketReceiveBuffer; //!< Buffer containing incoming data (for TCP: incomplete messages, for UDP: full datagrams)
        DataStream m_socketSendBuffer; //!< Buffer containing outgoing data (complete messages)
        sockaddr_in remoteAddress; //!< Contains IP address and port of remote socket
        sockaddr_in ownAddress; //!< Contains IP address and port of own socket

        // Start protected area of mutex of background thread
        bool m_isConnected = false; //!< \copybrief isConnected()
        core::Time m_connectionAttemptInterval; //!< \copybrief connectionAttemptInterval()
        core::Time m_lastConnectionAttempt; //!< \copybrief lastConnectionAttempt()
        // Sending heartbeats:
        bool m_sendHeartBeats; //!< \copybrief sendHeartBeats()
        DataStream m_heartBeatSendCode; //!< \copybrief heartBeatSendCode()
        core::Time m_heartBeatSendInterval; //!< \copybrief heartBeatSendInterval()
        core::Time m_lastHeartBeatSent; //!< \copybrief lastHeartBeatSent()
        // Receiving heartbeats:
        bool m_receiveHeartBeats; //!< \copybrief receiveHeartBeats()
        DataStream m_heartBeatReceiveCode; //!< \copybrief heartBeatReceiveCode()
        core::Time m_heartBeatTimeout; //!< \copybrief heartBeatTimeout()
        core::Time m_lastHeartBeatReceived; //!< \copybrief lastHeartBeatReceived()
        // End protected area of mutex of background thread

        // Internal buffers (only accessed by background worker)
    private:
        std::unique_ptr<ReceiveObjectType> m_currentReceivedObject; //!< Buffer for currently processed received objects
        std::unique_ptr<SendObjectType> m_currentObjectToSend; //!< Buffer for currently processed objects to send

        // External buffers (thread-safe on their own)
    public:
        memory::CircularBuffer<NetworkSocketSignal> m_signalBuffer; //!< Buffer containing signals of the socket to a supervisor (thread-safe on its own)
        memory::CircularBuffer<ReceiveObjectType> m_receivedObjects; //!< Buffer containing received objects (already deserialized) (thread-safe on its own)
        memory::CircularBuffer<SendObjectType> m_objectsToSend; //!< Buffer containing objects to send (will be serialized before sending) (thread-safe on its own)

        // Getters (thread-safe)
        // ---------------------
    public:
        // Constant members
        //! Container for options of this socket \details **Thread-safe getter**
        inline const NetworkSocketOptions& options() const { return m_options; }

        // Dynamic members
        //! Flag indicating, if the connection is open or not \details **Thread-safe getter**
        inline bool isConnected() const { return getProtectedData(m_isConnected); }
        //! Time-interval for attempting to establish a new connection \details **Thread-safe getter**
        inline core::Time connectionAttemptInterval() const { return getProtectedData(m_connectionAttemptInterval); }
        //! Time of last connection attempt to remote socket (system time) \details **Thread-safe getter**
        inline core::Time lastConnectionAttempt() const { return getProtectedData(m_lastConnectionAttempt); }
        //! Flag indicating, if cyclic sending of heartbeats is enabled \details **Thread-safe getter**
        inline bool sendHeartBeats() const { return getProtectedData(m_sendHeartBeats); }
        //! Byte sequence used to send as heartbeats \details **Thread-safe getter**
        inline DataStream heartBeatSendCode() const { return getProtectedData(m_heartBeatSendCode); }
        //! Time-interval for sending heartbeat messages \details **Thread-safe getter**
        inline core::Time heartBeatSendInterval() const { return getProtectedData(m_heartBeatSendInterval); }
        //! Time when last heartbeat was sent (system time) \details **Thread-safe getter**
        inline core::Time lastHeartBeatSent() const { return getProtectedData(m_lastHeartBeatSent); }
        //! Flag indicating, if receiving heartbeats is enabled \details **Thread-safe getter**
        inline bool receiveHeartBeats() const { return getProtectedData(m_receiveHeartBeats); }
        //! Expected byte sequence for received heartbeats (used for heartbeat detection) \details **Thread-safe getter**
        inline DataStream heartBeatReceiveCode() const { return getProtectedData(m_heartBeatReceiveCode); }
        //! Maximum allowed time between two heartbeats (defines heartbeat timeout) \details **Thread-safe getter**
        inline core::Time heartBeatTimeout() const { return getProtectedData(m_heartBeatTimeout); }
        //! Time of last received heartbeat (system time) \details **Thread-safe getter**
        inline core::Time lastHeartBeatReceived() const { return getProtectedData(m_lastHeartBeatReceived); }

        // Setters (thread-safe)
        // ---------------------
    public:
        //! **Thread-safe setter for:** \copybrief connectionAttemptInterval()
        inline void setConnectionAttemptInterval(core::Time newValue) { setProtectedData(m_connectionAttemptInterval, newValue); }
        //! **Thread-safe setter for:** \copybrief sendHeartBeats()
        inline void setSendHeartBeats(bool newValue) { setProtectedData(m_sendHeartBeats, newValue); }
        //! **Thread-safe setter for:** \copybrief heartBeatSendCode()
        inline void setHeartBeatSendCode(DataStream newValue) { setProtectedData(m_heartBeatSendCode, newValue); }
        //! **Thread-safe setter for:** \copybrief heartBeatSendInterval()
        inline void setHeartBeatSendInterval(core::Time newValue) { setProtectedData(m_heartBeatSendInterval, newValue); }
        //! **Thread-safe setter for:** \copybrief receiveHeartBeats()
        inline void setReceiveHeartBeats(bool newValue) { setProtectedData(m_receiveHeartBeats, newValue); }
        //! **Thread-safe setter for:** \copybrief heartBeatReceiveCode()
        inline void setHeartBeatReceiveCode(DataStream newValue) { setProtectedData(m_heartBeatReceiveCode, newValue); }
        //! **Thread-safe setter for:** \copybrief heartBeatTimeout()
        inline void setHeartBeatTimeout(core::Time newValue) { setProtectedData(m_heartBeatTimeout, newValue); }

    protected: // Only for "internal" use...
        //! **Thread-safe setter for:** \copybrief isConnected()
        inline void setIsConnected(bool newValue) { setProtectedData(m_isConnected, newValue); }
        //! **Thread-safe setter for:** \copybrief lastConnectionAttempt()
        inline void setLastConnectionAttempt(core::Time newValue) { setProtectedData(m_lastConnectionAttempt, newValue); }
        //! **Thread-safe setter for:** \copybrief lastHeartBeatSent()
        inline void setLastHeartBeatSent(core::Time newValue) { setProtectedData(m_lastHeartBeatSent, newValue); }
        //! **Thread-safe setter for:** \copybrief lastHeartBeatReceived()
        inline void setLastHeartBeatReceived(core::Time newValue) { setProtectedData(m_lastHeartBeatReceived, newValue); }
    };

    //! \}
} // namespace io
} // namespace broccoli
