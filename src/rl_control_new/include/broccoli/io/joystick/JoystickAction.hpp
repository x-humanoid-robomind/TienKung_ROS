/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// The joystick module requires SDL2 library
#ifdef HAVE_SDL2

#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \brief Data container for joystick actions (haptic feedback to perform)
     * \ingroup broccoli_io_joystick
     */
    class JoystickAction {
    public:
        //! Specification of action type to perform
        enum class Type : uint8_t {
            WAIT = 0, //!< Simple waiting (may be used to create a pause between other actions)
            HAPTIC_RUMBLE, //!< Perform generic rumble
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given action type
        static std::string typeString(const Type& type)
        {
            // Check type
            switch (type) {
            case Type::WAIT:
                return "WAIT";
            case Type::HAPTIC_RUMBLE:
                return "HAPTIC_RUMBLE";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of the type of **this** action
        std::string typeString() const { return typeString(m_type); }

        //! Specialized constructor
        /*!
         * \param [in] type Sets \ref m_type - \copybrief m_type
         * \param [in] intensity Sets \ref m_intensity - \copybrief m_intensity
         * \param [in] duration Sets \ref m_duration - \copybrief m_duration
         */
        JoystickAction(const Type& type, const double& intensity, const uint32_t& duration)
            : m_type(type)
            , m_intensity(intensity)
            , m_duration(duration)
        {
        }

        //! Default constructor
        JoystickAction()
            : JoystickAction(JoystickAction::Type::HAPTIC_RUMBLE, 1.0, 300)
        {
        }

        // Members
        Type m_type; //!< Action type to perform
        double m_intensity; //!< Intensity of action (floating point value between 0 and 1)
        uint32_t m_duration; //!< Duration of action in milliseconds
    };
} // namespace io
} // namespace broccoli

#endif // HAVE_SDL2
