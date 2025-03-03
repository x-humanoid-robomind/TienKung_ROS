/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \brief Data container for signals of a logfile to a supervising logger
     * \ingroup broccoli_io_logging
     */
    class LogFileSignal {
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
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of the type of **this** signal
        std::string typeString() const { return typeString(m_type); }

        //! Specification of signal messages
        enum class Message : uint16_t {
            FILE_OPEN_FAILED_ALREADY_OPEN = 0, //!< Opening of log file **failed**: file alredy open
            FILE_OPEN_FAILED_FOLDER_CREATION_FAILED, //!< Opening of log file **failed**: directory could not be created
            FILE_OPEN_FAILED_LOWLEVEL, //!< Opening of log file **failed**: low-level error
            FILE_WRITE_FAILED_NOTOPEN, //!< Writing to log file **failed**: file is not open
            FILE_WRITE_FAILED_DATABUFFER, //!< Writing to log file **failed**: data buffer error
            FILE_WRITE_FAILED_LOWLEVEL, //!< Writing to log file **failed**: low-level error
            FILE_WRITE_FAILED_FLUSH, //!< Writing to log file **failed**: flush error
            FILE_ZLIB_NOT_FOUND, //!< Program was not compiled with zlib support.
            MESSAGE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given message
        static std::string messageString(const Message& message)
        {
            // Check message
            switch (message) {
            case Message::FILE_OPEN_FAILED_ALREADY_OPEN:
                return "FILE_OPEN_FAILED_ALREADY_OPEN";
            case Message::FILE_OPEN_FAILED_FOLDER_CREATION_FAILED:
                return "FILE_OPEN_FAILED_FOLDER_CREATION_FAILED";
            case Message::FILE_OPEN_FAILED_LOWLEVEL:
                return "FILE_OPEN_FAILED_LOWLEVEL";
            case Message::FILE_WRITE_FAILED_NOTOPEN:
                return "FILE_WRITE_FAILED_NOTOPEN";
            case Message::FILE_WRITE_FAILED_DATABUFFER:
                return "FILE_WRITE_FAILED_DATABUFFER";
            case Message::FILE_WRITE_FAILED_LOWLEVEL:
                return "FILE_WRITE_FAILED_LOWLEVEL";
            case Message::FILE_WRITE_FAILED_FLUSH:
                return "FILE_WRITE_FAILED_FLUSH";
            case Message::FILE_ZLIB_NOT_FOUND:
                return "FILE_ZLIB_NOT_FOUND";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of the message of **this** signal
        std::string messageString() const { return messageString(m_message); }

        //! Specialized constructor
        /*!
         * \param [in] type Sets \ref m_type - \copybrief m_type
         * \param [in] message Sets \ref m_message - \copybrief m_message
         * \param [in] text Sets \ref m_text - \copybrief m_text
         * \param [in] time Sets \ref m_time - \copybrief m_time
         */
        LogFileSignal(const Type& type, const Message& message, const std::string& text, const core::Time& time = core::Time::currentTime())
            : m_type(type)
            , m_message(message)
            , m_text(text)
            , m_time(time)
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] type Sets \ref m_type - \copybrief m_type
         * \param [in] message Sets \ref m_message - \copybrief m_message
         * \param [in] time Sets \ref m_time - \copybrief m_time
         */
        LogFileSignal(const Type& type, const Message& message, const core::Time& time = core::Time::currentTime())
            : LogFileSignal(type, message, "", time)
        {
        }

        //! Default constructor
        LogFileSignal()
            : LogFileSignal(Type::INFORMATION, Message::FILE_OPEN_FAILED_ALREADY_OPEN)
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
