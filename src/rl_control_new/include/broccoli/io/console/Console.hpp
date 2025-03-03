/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../memory/CircularBuffer.hpp"
#include "../../parallel/BackgroundWorker.hpp"
#include "../filesystem.hpp"
#include "../logging/LogFile.hpp"
#include "../logging/LogFileDataString.hpp"
#include "ConsoleMessage.hpp"
#include "ConsoleOptions.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <stdarg.h>
#include <stdio.h>

namespace broccoli {
namespace io {
    // Forward declarations
    class ConsoleInterface;

    //! Manager class for console output (**thread-safe**)
    /*!
     * \ingroup broccoli_io_console
     *
     * The class is designed as **singleton**. A background thread is used to process the message buffer without blocking the user code. Adding
     * messages to the console is **thread-safe**.
     *
     * The console is configured with \ref configure(). If \ref configure() is not called, the console will be initialized with default parameters
     * on the first call of any method of `Console`. Note that one can call \ref configure() multiple times (each call destroys and recreates the
     * global instance of the class with the new option set).
     *
     * The console supports default and user-defined "interfaces". The default interfaces are inspired by Python's logging levels:
     *
     * Default Interface | Usage
     * ----------------- | -----
     * Plain             | \copybrief ConsoleOptions::DefaultInterfaceType::PLAIN
     * Debug             | \copybrief ConsoleOptions::DefaultInterfaceType::DEBUG
     * Info              | \copybrief ConsoleOptions::DefaultInterfaceType::INFO
     * Warning           | \copybrief ConsoleOptions::DefaultInterfaceType::WARNING
     * Error             | \copybrief ConsoleOptions::DefaultInterfaceType::ERROR
     * Critical          | \copybrief ConsoleOptions::DefaultInterfaceType::CRITICAL
     *
     * Depending on the chosen interface the output is formatted accordingly.
     *
     * Finally, one may specify an output file (during configuration). For each interface the user can enable/disable the output to the console
     * and the log file separately.
     *
     * Typical Usage
     * -------------
     * \code
     * Console::configure(ConsoleOptions(...)); // Optional (for customization only)
     * ...
     * Console::debug().print("test\n"); // For std::string
     * Console::debug().printf("test: %.2f\n", 1.23456789); // Analogous to std::printf
     * Console::debug() << 123 << '\n'; // Analogous to std::cout (however, without support for std::endl, etc.)
     * ...
     * Console::flush(); // Optional (to force flush of console)
     * \endcode
     */
    class Console : protected parallel::BackgroundWorker {
        // Access to global instance
        // -------------------------
    private:
        //! Passes back reference to mutex of global instance
        /*!
         * \note This mutex is here to lock access to the global instance. The idea is to prevent a thread to re-create the global instance, while
         * another thread is using it for output. This mutex is implemented as read-write mutex:
         *  * Read-lock: the calling thread uses the global instance for output. Multiple threads can use the global instance at the same time.
         *  * Write-lock: the calling thread may destroy and re-create the global instance. Only one thread can hold the write-lock such that it is guaranteed, that no other thread uses the global instance while it is re-created.
         */
        static inline pthread_rwlock_t& globalInstanceMutex()
        {
            static pthread_rwlock_t mutex(PTHREAD_RWLOCK_INITIALIZER);
            return mutex;
        }

        //! Returns reference to pointer to **global** instance
        /*! \attention The pointer always exists. However, the (pointed to) instance may not exist yet! */
        static inline std::unique_ptr<Console>& globalInstancePointer()
        {
            static std::unique_ptr<Console> pointer;
            return pointer;
        }

    protected:
        //! Locks the global instance for "reading" (=access to console) (**thread-safe**)
        /*!
         * Creates the global instance, if it does not exist yet.
         *
         * \warning Do not forget to release the access to the global instance with \ref globalInstanceReleaseAccess()!
         *
         * \return Reference to global instance
         */
        static inline Console& globalInstanceGetAccess()
        {
            // Lock for read
            ThreadSafeContainer::lockForRead(globalInstanceMutex());

            // Check, if the global instance exists
            if (globalInstancePointer().get() != nullptr)
                return *globalInstancePointer().get();
            else {
                // Unlock
                ThreadSafeContainer::unlock(globalInstanceMutex());

                // Try to create instance and aquire read lock
                while (true) {
                    // Lock for write
                    ThreadSafeContainer::lockForWrite(globalInstanceMutex());

                    // Check, if the global instance exists (and re-create it if not)
                    // (we have to check this again, since it might have changed in the short time between releasing the read lock and aquiring the write lock)
                    if (globalInstancePointer().get() == nullptr)
                        globalInstancePointer().reset(new Console()); // Triggers constructor

                    // Unlock
                    ThreadSafeContainer::unlock(globalInstanceMutex());

                    // Lock for read
                    ThreadSafeContainer::lockForRead(globalInstanceMutex());

                    // Check, if the global instance exists
                    if (globalInstancePointer().get() != nullptr)
                        return *globalInstancePointer().get();
                    else {
                        // Unlock
                        ThreadSafeContainer::unlock(globalInstanceMutex());

                        // Try again...
                    }
                }
            }
        }

        //! Unlocks the global instance (**thread-safe**)
        static inline void globalInstanceReleaseAccess() { ThreadSafeContainer::unlock(globalInstanceMutex()); }

        //! Re-creates the global instance with the given set of options (**thread-safe**)
        static inline void globalInstanceReCreate(const ConsoleOptions& options = ConsoleOptions())
        {
            // Lock for write
            ThreadSafeContainer::lockForWrite(globalInstanceMutex());

            // Recreate instance
            globalInstancePointer().reset(new Console(options)); // Triggers constructor + may trigger destructor

            // Unlock
            ThreadSafeContainer::unlock(globalInstanceMutex());
        }

        // Construction and destruction
        // ----------------------------
    protected:
        //! Constructor (runs in parent thread)
        /*! \param [in] options Initializes \ref m_options - \copybrief m_options */
        Console(const ConsoleOptions& options = ConsoleOptions())
            : BackgroundWorker(options.m_threadName, true, options.m_threadPriority, 0.001 /* <-- max. 1kHz */, false) // Setup background thread for low CPU usage (data is buffered -> console output is NOT time-critical)
            , m_options(options)
            , m_messageBuffer(options.m_bufferSize)
        {
            // Check, if a logfile should be written
            if (m_options.m_writeLogFile == true) {
                // Create logfile
                m_logFile = std::make_unique<LogFile<LogFileDataString>>(m_options.m_logFileIdentifier, m_options.m_logFileTargetDirectory, m_options.m_logFileFileName, m_options.m_logFileFileExtension, m_options.m_logFileAddTimeStampToName, m_options.m_logFileCompressed, false, 1);

                // Open logfile
                m_logFile->openLogFile();
                m_logFile->writeLogHeader(core::Time::currentTime());
            }

            // Start the background thread
            start();
        }

        //! Copy constructor (deleted -> this class is meant to be used as singleton!)
        Console(const Console& original) = delete;

        //! Copy assignment operator (deleted -> this class is meant to be used as singleton!)
        Console& operator=(const Console& reference) = delete;

    public:
        //! Destructor
        ~Console()
        {
            // Try to lock for write (destructor might be called by main program on exit - without previously locking for write)
            const bool lockedForWrite = ThreadSafeContainer::tryLockForWrite(globalInstanceMutex());

            // Initialize helpers
            const core::Time timeout(3, 0);

            // Flush message buffer -> wait until all messages have been processed
            const core::Time waitEndTime = core::Time::currentTime() + timeout;
            while (m_messageBuffer.elementCount() > 0 && core::Time::currentTime() < waitEndTime)
                core::Time::sleep(core::Time(0, 1));

            // Join background thread
            const bool joinResult = join(timeout);

            // Flush console
            std::cout << std::flush;

            // Close output file (only if background thread shut down properly)
            if (joinResult == true && m_options.m_writeLogFile == true) {
                m_logFile->writeLogFooter(core::Time::currentTime());
                m_logFile->closeLogFile();
            }

            // Unlock
            if (lockedForWrite == true)
                globalInstanceReleaseAccess();
        }

        // Members
        // -------
    protected:
        // Constants (no protection necessary)
        const ConsoleOptions m_options; //!< \copybrief options()
        // Internal data (only accessed by background thread -> no protection necessary)
        std::unique_ptr<LogFile<LogFileDataString>> m_logFile; //!< Output logfile
        ConsoleMessage m_previousMessageConsole; //!< Remembers the previously processed console message
        ConsoleMessage m_previousMessageLogFile; //!< Remembers the previously processed logfile message
        bool m_previousLineCompleteConsole = true; //!< Indicates, if the last line in the console was ended with a linebreak
        bool m_previousLineCompleteLogFile = true; //!< Indicates, if the last line in the logfile was ended with a linebreak
        // Self-managed data
        memory::CircularBuffer<ConsoleMessage, std::allocator<ConsoleMessage>> m_messageBuffer; //!< Buffer for messages to process (**thread-safe** on its own)
        // Protected data
        bool m_doFlushConsole = false; //!< If `true`, the background thread is commanded to flush the console

        // Main user interface
        // -------------------
    public:
        //! Re-configures the console (**thread-safe**) (runs in parent thread)
        /*! \param [in] options Sets \ref m_options - \copybrief m_options */
        static inline void configure(const ConsoleOptions& options = ConsoleOptions()) { globalInstanceReCreate(options); }

        //! Flush the message buffer (**thread-safe**) (runs in parent thread)
        /*!
         * Sends a command to the background thread to flush the message buffer. In blocking mode (timeout > 0) this method
         * waits until the background thread has processed all messages and triggered a flush of the console.
         *
         * \param [in] timeout Optional timeout for waiting (use 0 for non-blocking mode)
         * \return `true` on success, `false` on timeout
         */
        static inline bool flush(const core::Time& timeout = core::Time(0, 0))
        {
            // Initialize helpers
            bool timeoutHit = false;

            // Get access to global instance
            auto& globalInstance = globalInstanceGetAccess();

            // Command flush
            globalInstance.setProtectedData(globalInstance.m_doFlushConsole, true);

            // Wait until all messages have been processed and flush has been executed by the thread
            if (timeout > 0) {
                const core::Time waitEndTime = core::Time::currentTime() + timeout;
                while (globalInstance.m_messageBuffer.elementCount() > 0 || globalInstance.getProtectedData(globalInstance.m_doFlushConsole) == true) {
                    // Check for timeout
                    if (core::Time::currentTime() >= waitEndTime) {
                        timeoutHit = true;
                        break;
                    }

                    // Wait...
                    core::Time::sleep(core::Time(0, 1));
                }
            }

            // Release access to global instance
            globalInstanceReleaseAccess();

            // Pass back type of return
            return timeoutHit;
        }

        // Execution
        // ---------
    protected:
        //! Main execution function (one cylce of execution loop) (runs in background thread)
        void execute()
        {
            // Process signals of logfile
            if (m_options.m_writeLogFile == true)
                processLogFileSignals();

            // Pass through data in buffer
            ConsoleMessage currentMessage;
            while (stopTriggered() == false && m_messageBuffer.pop(currentMessage) > 0) {
                // Only proceed, if interface index is valid
                if (currentMessage.m_interfaceIndex < m_options.interfaceCount()) {
                    const auto currentInterface = m_options.interfaceOptions(currentMessage.m_interfaceIndex);

                    // Print to console
                    // ----------------
                    if (currentInterface.m_writeToConsole == true) {
                        const auto previousInterface = m_options.interfaceOptions(m_previousMessageConsole.m_interfaceIndex);
                        const std::string outputString = ConsoleMessage::createOutputString(currentMessage, currentInterface.m_prefixConsole, currentInterface.m_suffixConsole, m_previousMessageConsole, previousInterface.m_suffixConsole, m_previousLineCompleteConsole);
                        if (outputString.length() > 0)
                            std::cout << outputString;
                    }

                    // Print to file
                    // -------------
                    if (m_options.m_writeLogFile == true && currentInterface.m_writeToLogFile == true) {
                        const auto previousInterface = m_options.interfaceOptions(m_previousMessageLogFile.m_interfaceIndex);
                        const std::string outputString = ConsoleMessage::createOutputString(currentMessage, currentInterface.m_prefixLogFile, currentInterface.m_suffixLogFile, m_previousMessageLogFile, previousInterface.m_suffixLogFile, m_previousLineCompleteLogFile);
                        if (outputString.length() > 0) {
                            m_logFile->addNewDataToBuffer(LogFileDataString(outputString));
                            m_logFile->processLogBuffer();
                        }
                    }
                } else {
                    // Invalid interface index
                    m_messageBuffer.push(ConsoleMessage(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::ERROR), "Console: Invalid interface index '" + std::to_string(currentMessage.m_interfaceIndex) + "' (message text='" + currentMessage.m_text + "')! Skipping this message...\n"));
                }
            }

            // Flush console if desired
            if (getProtectedData(m_doFlushConsole) == true) {
                std::cout << std::flush;
                setProtectedData(m_doFlushConsole, false);
            }
        }

        // Getters (thread-safe)
        // ---------------------
    public:
        // Constants
        //! Returns the currently set options
        static inline ConsoleOptions options()
        {
            const auto& globalInstance = globalInstanceGetAccess();
            const auto returnValue = globalInstance.m_options; // Constant data -> no protection necessary
            globalInstanceReleaseAccess();
            return returnValue;
        }
        // Self-managed data
        //! Returns current count of messages in the message buffer
        static inline size_t bufferedMessageCount()
        {
            const auto& globalInstance = globalInstanceGetAccess();
            const auto returnValue = globalInstance.m_messageBuffer.elementCount(); // Self-managed data -> no protection necessary
            globalInstanceReleaseAccess();
            return returnValue;
        }
        //! Returns total count of "lost" messages (due to buffer overflow)
        static inline uint64_t lostMessageCount()
        {
            const auto& globalInstance = globalInstanceGetAccess();
            const auto returnValue = globalInstance.m_messageBuffer.totalOverwrittenElements(); // Self-managed data -> no protection necessary
            globalInstanceReleaseAccess();
            return returnValue;
        }

        // Setters (thread-safe)
        // ---------------------
    protected:
        // Self-managed data
        //! Adds the given message to the message buffer (**thread-safe**)
        static inline void addMessage(const ConsoleMessage& message)
        {
            auto& globalInstance = globalInstanceGetAccess();
            globalInstance.m_messageBuffer.push(message);
            globalInstanceReleaseAccess();
        }

        // Interfaces
        // ----------
    public:
        //! Interface for output to the console
        class Interface {
        public:
            //! Constructor
            /*! \param [in] index Initializes \ref index() - \copybrief index() */
            Interface(const size_t& index)
                : m_index(index)
            {
            }

            // Members
            // -------
        private:
            const size_t m_index; //!< \copybrief index()

            // Getters (thread-safe)
            // ---------------------
        public:
            // Constants (no protection necessary)
            //! Index of interface in list of interfaces of console
            const size_t& index() const { return m_index; }

            // Message passing to console
            // --------------------------
        public:
            //! Prints the given text string (**thread-safe**)
            inline void print(const std::string& text) const { Console::addMessage(ConsoleMessage(index(), text)); }

            //! Print function (usage equivalent to std::printf) (**thread-safe**)
            /*! \warning The internal buffer can store 4096 bytes. This is the upper bound for the message to write with printf. */
            inline void printf(const char* format, ...) const
            {
                // Print to string
                char messageTextBuffer[4096];
                va_list args;
                va_start(args, format);
                vsnprintf(messageTextBuffer, sizeof(messageTextBuffer), format, args);
                va_end(args);
                const std::string messageText(messageTextBuffer);

                // Add to message buffer
                Console::addMessage(ConsoleMessage(index(), messageText));
            }

            //! Stream operator (usage equivalent to std::cout, however, without support for std::endl, etc.) (**thread-safe**)
            template <typename T>
            inline const Interface& operator<<(const T& data) const
            {
                Console::addMessage(ConsoleMessage::createFrom(index(), data));
                return *this;
            }
        };

        // Default interfaces
        // ------------------
    public:
        //! Global interface for "plain" messages
        static inline const Interface& plain()
        {
            static Interface interface(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::PLAIN));
            return interface;
        }

        //! Global interface for "debug" messages
        static inline const Interface& debug()
        {
            static Interface interface(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::DEBUG));
            return interface;
        }

        //! Global interface for "info" messages
        static inline const Interface& info()
        {
            static Interface interface(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::INFO));
            return interface;
        }

        //! Global interface for "warning" messages
        static inline const Interface& warning()
        {
            static Interface interface(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::WARNING));
            return interface;
        }

        //! Global interface for "error" messages
        static inline const Interface& error()
        {
            static Interface interface(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::ERROR));
            return interface;
        }

        //! Global interface for "critical" messages
        static inline const Interface& critical()
        {
            static Interface interface(static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::CRITICAL));
            return interface;
        }

        // Helpers
        // -------
    protected:
        //! Processes signals of own logfile (runs in background thread)
        /*! Parses all signals of the logfile and creates corresponding messages to print in the console */
        void processLogFileSignals()
        {
            // Abort, if logfile has not been created
            if (!m_logFile)
                return;

            // Pass through all signals of the logfile
            LogFileSignal signal;
            while (m_logFile->m_signalBuffer.pop(signal) > 0) {
                ConsoleMessage message;

                // Map signal type
                if (signal.m_type == LogFileSignal::Type::INFORMATION)
                    message.m_interfaceIndex = static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::INFO);
                if (signal.m_type == LogFileSignal::Type::WARNING)
                    message.m_interfaceIndex = static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::WARNING);
                if (signal.m_type == LogFileSignal::Type::ERROR)
                    message.m_interfaceIndex = static_cast<size_t>(ConsoleOptions::DefaultInterfaceType::ERROR);

                // Concatenate text
                message.m_text = "Console::LogFile::Signal" + signal.messageString() + " (" + signal.m_text + ")\n";

                // Push message to buffer
                m_messageBuffer.push(message);
            }
        }
    };
} // namespace io
} // namespace broccoli
