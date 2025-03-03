/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// The joystick module requires SDL2 library
#ifdef HAVE_SDL2

#include "../../core/Time.hpp"
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \brief Data container for signals of the joystick module to a supervising observer
     * \ingroup broccoli_io_joystick
     */
    class JoystickSignal {
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
            SDL_INIT_FAILED = 0, //!< Initialization of SDL failed (details stored in \ref m_text)
            SDL_QUIT_SIGNAL, //!< SDL quit signal received
            DISCONNECT_DESIRED_DEVICE_CHANGED, //!< Disconnected since desired device differs from currently connected device
            DISCONNECT_DEVICE_DETACHED, //!< Disconnected since device was disconnected from driver
            CONNECT_FAILED_DESIRED_DEVICE_NOT_FOUND, //!< Connecting failed: could not find desired device in list of available devices
            CONNECT_FAILED_SDL_OPEN, //!< Connecting failed: opening device via SDL failed (details stored in \ref m_text)
            CONNECT_FAILED_HAPTICS, //!< Could not initialize haptics during setting up connection (details stored in \ref m_text)
            CONNECT_SUCCESS, //!< Connecting was successful
            ACTION_FAILED_NO_HAPTICS, //!< Performing action failed, since the device does not support haptics
            ACTION_FAILED_HAPTIC_RUMBLE, //!< Performing haptic rumble action failed (details stored in \ref m_text)
            ACTION_FAILED_UNKNOWN, //!< Performing action failed, since the action type is unknown
            UPDATE_DEVICE_LIST, //!< List of available devices has changed
            READ_INFO_FAILED_NAME, //!< Could not read device name (details stored in \ref m_text)
            READ_INFO_FAILED_GUID, //!< Could not read GUID (details stored in \ref m_text)
            READ_INFO_FAILED_AXES, //!< Could not read number of axes (details stored in \ref m_text)
            READ_INFO_FAILED_BALLS, //!< Could not read number of balls (details stored in \ref m_text)
            READ_INFO_FAILED_BUTTONS, //!< Could not read number of buttons (details stored in \ref m_text)
            READ_INFO_FAILED_HATS, //!< Could not read number of hats (details stored in \ref m_text)
            READ_INFO_FAILED_HAPTIC, //!< Could not read haptic device information (details stored in \ref m_text)
            MESSAGE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given message
        static std::string messageString(const Message& message)
        {
            // Check message
            switch (message) {
            case Message::SDL_INIT_FAILED:
                return "SDL_INIT_FAILED";
            case Message::SDL_QUIT_SIGNAL:
                return "SDL_QUIT_SIGNAL";
            case Message::DISCONNECT_DESIRED_DEVICE_CHANGED:
                return "DISCONNECT_DESIRED_DEVICE_CHANGED";
            case Message::DISCONNECT_DEVICE_DETACHED:
                return "DISCONNECT_DEVICE_DETACHED";
            case Message::CONNECT_FAILED_DESIRED_DEVICE_NOT_FOUND:
                return "CONNECT_FAILED_DESIRED_DEVICE_NOT_FOUND";
            case Message::CONNECT_FAILED_SDL_OPEN:
                return "CONNECT_FAILED_SDL_OPEN";
            case Message::CONNECT_FAILED_HAPTICS:
                return "CONNECT_FAILED_HAPTICS";
            case Message::CONNECT_SUCCESS:
                return "CONNECT_SUCCESS";
            case Message::ACTION_FAILED_NO_HAPTICS:
                return "ACTION_FAILED_NO_HAPTICS";
            case Message::ACTION_FAILED_HAPTIC_RUMBLE:
                return "ACTION_FAILED_HAPTIC_RUMBLE";
            case Message::ACTION_FAILED_UNKNOWN:
                return "ACTION_FAILED_UNKNOWN";
            case Message::UPDATE_DEVICE_LIST:
                return "UPDATE_DEVICE_LIST";
            case Message::READ_INFO_FAILED_NAME:
                return "READ_INFO_FAILED_NAME";
            case Message::READ_INFO_FAILED_GUID:
                return "READ_INFO_FAILED_GUID";
            case Message::READ_INFO_FAILED_AXES:
                return "READ_INFO_FAILED_AXES";
            case Message::READ_INFO_FAILED_BALLS:
                return "READ_INFO_FAILED_BALLS";
            case Message::READ_INFO_FAILED_BUTTONS:
                return "READ_INFO_FAILED_BUTTONS";
            case Message::READ_INFO_FAILED_HATS:
                return "READ_INFO_FAILED_HATS";
            case Message::READ_INFO_FAILED_HAPTIC:
                return "READ_INFO_FAILED_HAPTIC";
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
         * \param [in] type The type of the signal (information, warning, error)
         * \param [in] message The actual message.
         * \param [in] text Custom text message for additional information
         * \param [in] time The time the signal was triggered.
         */
        JoystickSignal(const Type& type, const Message& message, const std::string& text, const core::Time& time = core::Time::currentTime())
            : m_type(type)
            , m_message(message)
            , m_text(text)
            , m_time(time)
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] type The type of the signal (information, warning, error)
         * \param [in] message The actual message.
         * \param [in] time The time the signal was triggered.
         */
        JoystickSignal(const Type& type, const Message& message, const core::Time& time = core::Time::currentTime())
            : JoystickSignal(type, message, "", time)
        {
        }

        //! Default constructor
        JoystickSignal()
            : JoystickSignal(Type::INFORMATION, Message::SDL_INIT_FAILED)
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

#endif // HAVE_SDL2
