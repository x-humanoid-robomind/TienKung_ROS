/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// The joystick module requires SDL2 library
#ifdef HAVE_SDL2

#include <SDL.h>
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \brief Container for device specific information (capabilities, etc.)
     * \ingroup broccoli_io_joystick
     */
    class JoystickInformation {
    public:
        //! Returns the string representation of the given power level
        static std::string powerLevelString(const SDL_JoystickPowerLevel& powerLevel)
        {
            // Check power level
            switch (powerLevel) {
            case SDL_JOYSTICK_POWER_UNKNOWN:
                return "SDL_JOYSTICK_POWER_UNKNOWN";
            case SDL_JOYSTICK_POWER_EMPTY:
                return "SDL_JOYSTICK_POWER_EMPTY";
            case SDL_JOYSTICK_POWER_LOW:
                return "SDL_JOYSTICK_POWER_LOW";
            case SDL_JOYSTICK_POWER_MEDIUM:
                return "SDL_JOYSTICK_POWER_MEDIUM";
            case SDL_JOYSTICK_POWER_FULL:
                return "SDL_JOYSTICK_POWER_FULL";
            case SDL_JOYSTICK_POWER_WIRED:
                return "SDL_JOYSTICK_POWER_WIRED";
            case SDL_JOYSTICK_POWER_MAX:
                return "SDL_JOYSTICK_POWER_MAX";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of the power level of **this** instance
        std::string powerLevelString() const { return powerLevelString(m_powerLevel); }

        //! Default constructor
        JoystickInformation()
            : m_name("UNKNOWN")
            , m_GUIDstring("")
            , m_numberOfAxes(0)
            , m_numberOfBalls(0)
            , m_numberOfButtons(0)
            , m_numberOfHats(0)
            , m_isHaptic(false)
            , m_powerLevel(SDL_JOYSTICK_POWER_UNKNOWN)
        {
            // Clear GUID data (an all-zero GUID is defined as invalid)
            for (size_t i = 0; i < sizeof(m_GUID.data) / sizeof(m_GUID.data[0]); i++)
                m_GUID.data[i] = 0;
        }

        //! Equality operator
        /*!
         * \param [in] reference Reference to other object to compare to
         * \return `true`, if members match, `false` otherwise
         */
        bool operator==(const JoystickInformation& reference) const
        {
            return (m_name == reference.m_name && //
                m_GUID.data == reference.m_GUID.data && //
                m_GUIDstring == reference.m_GUIDstring && //
                m_numberOfAxes == reference.m_numberOfAxes && //
                m_numberOfBalls == reference.m_numberOfBalls && //
                m_numberOfButtons == reference.m_numberOfButtons && //
                m_numberOfHats == reference.m_numberOfHats && //
                m_powerLevel == reference.m_powerLevel);
        }

        //! Inequality operator
        /*!
         * Complement to \ref operator==()
         * \param [in] reference Reference to other object to compare to
         * \return `true`, if members differ, `false` otherwise
         */
        bool operator!=(const JoystickInformation& reference) const
        {
            return !operator==(reference);
        }

        // Members
        std::string m_name; //!< Name of this device
        SDL_JoystickGUID m_GUID; //!< GUID of this device
        std::string m_GUIDstring; //!< GUID represented as string (ASCII)
        int m_numberOfAxes; //!< Number of axes of this device
        int m_numberOfBalls; //!< Number of balls of this device (trackballs - 2D relative motion)
        int m_numberOfButtons; //!< Number of buttons of this device (on/off)
        int m_numberOfHats; //!< Number of hats of this device (discrete directions)
        bool m_isHaptic; //!< Flag indicating if this device supports haptic feedback
        SDL_JoystickPowerLevel m_powerLevel; //!< Power level of this device
    };
} // namespace io
} // namespace broccoli

#endif // HAVE_SDL2
