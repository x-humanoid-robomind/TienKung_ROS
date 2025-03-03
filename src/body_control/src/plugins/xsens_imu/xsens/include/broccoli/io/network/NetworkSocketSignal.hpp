/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    //! Data container for signals of the network socket to a supervising observer
    /*!
     * \ingroup broccoli_io_network
     */
    class NetworkSocketSignal {
    public:
        //! Specification of signal types
        enum class Type : uint8_t {
            INFORMATION = 0, //!< Simple user **information**
            WARNING, //!< Non-critical **warning**
            ERROR, //!< Critical **error**
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given signal type
        static std::string typeString(const Type& type)
        {
            // Check type
            switch (type) {
            case Type::INFORMATION:
                return "INFORMATION";
            case Type::WARNING:
                return "WARNING";
            case Type::ERROR:
                return "ERROR";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }

        //! Returns the string representation of the type of **this** signal
        std::string typeString() const { return typeString(m_type); }

        //! Specification of signal messages
        enum class Message : uint16_t {
            DISCONNECT_HEARTBEAT_TIMEOUT = 0, //!< Disconnected, since heartbeat timeout was hit
            CONNECT_NEW_TRY, //!< New connection attempt triggered
            CONNECT_FAILED_SOCKET_OPEN, //!< Connecting **failed**: could not open socket
            CONNECT_SUCCESS, //!< Connecting was successful
            DESERIALIZE_FAILED, //!< Deserialization of received object **failed**
            DESERIALIZE_MAX_SIZE, //!< Serial representation of received object exceeds maximum message size
            SERIALIZE_FAILED, //!< Serialization of object to send **failed**
            SERIALIZE_MAX_SIZE, //!< Serial representation of object to send exceeds maximum message size
            SOCKET_OPEN_FAILED_HOST_IDENTIFICATION, //!< Low-level socket open **failed**: could not identify host
            SOCKET_OPEN_FAILED_LISTENER_CREATION, //!< Low-level socket open **failed**: could not create listener socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_LISTENER_SET_REUSEADDR, //!< Low-level socket open **failed**: could not set option `SO_REUSEADDR` for listener socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_LISTENER_BIND, //!< Low-level socket open **failed**: could not bind listener socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_LISTENER_LISTEN, //!< Low-level socket open **failed**: could not listen on listener socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_LISTENER_CLOSE, //!< Low-level socket open **failed**: could close listener socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SOCKET_ACCEPT, //!< Low-level socket open **failed**: could not accept connection (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SOCKET_CREATION, //!< Low-level socket open **failed**: could not create socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SOCKET_CONNECT, //!< Low-level socket open **failed**: could not connect socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SOCKET_SET_REUSEADDR, //!< Low-level socket open **failed**: could not set option `SO_REUSEADDR` for socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SOCKET_BIND, //!< Low-level socket open **failed**: could not bind socket (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_GET_FLAGS, //!< Low-level socket open **failed**: could not get socket flags (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SET_NONBLOCKING, //!< Low-level socket open **failed**: could not set socket option "non-blocking" (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SET_TCPNODELAY, //!< Low-level socket open **failed**: could not set socket option `TCP_NODELAY` (details stored in \ref m_text)
            SOCKET_SEND_DATA_FAILED, //!< Low-level socket send-data **failed**: could not send data (details stored in \ref m_text)
            SOCKET_READ_DATA_FAILED, //!< Low-level socket send-data **failed**: could not send data (details stored in \ref m_text)
            SOCKET_OPEN_FAILED_SOCKET_SET_BROADCAST, //!< Low-level socket open **failed**: could not set option `SO_BROADCAST` for socket (details stored in \ref m_text)
            MESSAGE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given message
        static std::string messageString(const Message& message)
        {
            // Check message
            switch (message) {
            case Message::DISCONNECT_HEARTBEAT_TIMEOUT:
                return "DISCONNECT_HEARTBEAT_TIMEOUT";
            case Message::CONNECT_NEW_TRY:
                return "CONNECT_NEW_TRY";
            case Message::CONNECT_FAILED_SOCKET_OPEN:
                return "CONNECT_FAILED_SOCKET_OPEN";
            case Message::CONNECT_SUCCESS:
                return "CONNECT_SUCCESS";
            case Message::DESERIALIZE_FAILED:
                return "DESERIALIZE_FAILED";
            case Message::DESERIALIZE_MAX_SIZE:
                return "DESERIALIZE_MAX_SIZE";
            case Message::SERIALIZE_FAILED:
                return "SERIALIZE_FAILED";
            case Message::SERIALIZE_MAX_SIZE:
                return "SERIALIZE_MAX_SIZE";
            case Message::SOCKET_OPEN_FAILED_HOST_IDENTIFICATION:
                return "SOCKET_OPEN_FAILED_HOST_IDENTIFICATION";
            case Message::SOCKET_OPEN_FAILED_LISTENER_CREATION:
                return "SOCKET_OPEN_FAILED_LISTENER_CREATION";
            case Message::SOCKET_OPEN_FAILED_LISTENER_SET_REUSEADDR:
                return "SOCKET_OPEN_FAILED_LISTENER_SET_REUSEADDR";
            case Message::SOCKET_OPEN_FAILED_LISTENER_BIND:
                return "SOCKET_OPEN_FAILED_LISTENER_BIND";
            case Message::SOCKET_OPEN_FAILED_LISTENER_LISTEN:
                return "SOCKET_OPEN_FAILED_LISTENER_LISTEN";
            case Message::SOCKET_OPEN_FAILED_LISTENER_CLOSE:
                return "SOCKET_OPEN_FAILED_LISTENER_CLOSE";
            case Message::SOCKET_OPEN_FAILED_SOCKET_ACCEPT:
                return "SOCKET_OPEN_FAILED_SOCKET_ACCEPT";
            case Message::SOCKET_OPEN_FAILED_SOCKET_CREATION:
                return "SOCKET_OPEN_FAILED_SOCKET_CREATION";
            case Message::SOCKET_OPEN_FAILED_SOCKET_CONNECT:
                return "SOCKET_OPEN_FAILED_SOCKET_CONNECT";
            case Message::SOCKET_OPEN_FAILED_SOCKET_SET_REUSEADDR:
                return "SOCKET_OPEN_FAILED_SOCKET_SET_REUSEADDR";
            case Message::SOCKET_OPEN_FAILED_SOCKET_BIND:
                return "SOCKET_OPEN_FAILED_SOCKET_BIND";
            case Message::SOCKET_OPEN_FAILED_GET_FLAGS:
                return "SOCKET_OPEN_FAILED_GET_FLAGS";
            case Message::SOCKET_OPEN_FAILED_SET_NONBLOCKING:
                return "SOCKET_OPEN_FAILED_SET_NONBLOCKING";
            case Message::SOCKET_OPEN_FAILED_SET_TCPNODELAY:
                return "SOCKET_OPEN_FAILED_SET_TCPNODELAY";
            case Message::SOCKET_SEND_DATA_FAILED:
                return "SOCKET_SEND_DATA_FAILED";
            case Message::SOCKET_READ_DATA_FAILED:
                return "SOCKET_READ_DATA_FAILED";
            case Message::SOCKET_OPEN_FAILED_SOCKET_SET_BROADCAST:
                return "SOCKET_OPEN_FAILED_SOCKET_SET_BROADCAST";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }

        //! Returns the string representation of the message of **this** signal
        std::string messageString() const { return messageString(m_message); }

        //! Specialized constructor
        /*!
         * \param [in] type Initializes: \ref m_type - \copybrief m_type
         * \param [in] message Initializes: \ref m_message - \copybrief m_message
         * \param [in] text Initializes: \ref m_text - \copybrief m_text
         * \param [in] time Initializes: \ref m_time - \copybrief m_time
         */
        NetworkSocketSignal(const Type& type, const Message& message, const std::string& text, const core::Time& time = core::Time::currentTime())
            : m_type(type)
            , m_message(message)
            , m_text(text)
            , m_time(time)
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] type Initializes: \ref m_type - \copybrief m_type
         * \param [in] message Initializes: \ref m_message - \copybrief m_message
         * \param [in] time Initializes: \ref m_time - \copybrief m_time
         */
        NetworkSocketSignal(const Type& type, const Message& message, const core::Time& time = core::Time::currentTime())
            : NetworkSocketSignal(type, message, "", time)
        {
        }

        //! Default constructor
        NetworkSocketSignal()
            : NetworkSocketSignal(Type::INFORMATION, Message::DISCONNECT_HEARTBEAT_TIMEOUT)
        {
        }

        // Members
        Type m_type; //!< Type of the signal (information, warning, error)
        Message m_message; //!< Message contained in the signal (tells what exactly happened)
        std::string m_text; //!< (Optional) custom text message for additional information
        core::Time m_time; //!< System time of occurance
    };
} // namespace io
} // namespace broccoli
