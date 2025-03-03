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
#include <vector>

namespace broccoli {
namespace io {
    //! Data container for joystick state (current state of axes, buttons and hats)
    /*!
     * \ingroup broccoli_io_joystick
     * \par Possible Hat Values:
     * \code
     * SDL_HAT_LEFTUP   | SDL_HAT_UP       | SDL_HAT_RIGHTUP
     * -----------------|------------------|------------------
     * SDL_HAT_LEFT     | SDL_HAT_CENTERED | SDL_HAT_RIGHT
     * -----------------|------------------|------------------
     * SDL_HAT_LEFTDOWN | SDL_HAT_DOWN     | SDL_HAT_RIGHTDOWN
     * \endcode
     */
    class JoystickState {
    public:
        //! Returns the string representation of the given hat value
        static std::string hatValueString(const uint8_t& hatValue)
        {
            // Check hat value
            switch (hatValue) {
            case SDL_HAT_LEFTUP:
                return "SDL_HAT_LEFTUP";
            case SDL_HAT_UP:
                return "SDL_HAT_UP";
            case SDL_HAT_RIGHTUP:
                return "SDL_HAT_RIGHTUP";
            case SDL_HAT_LEFT:
                return "SDL_HAT_LEFT";
            case SDL_HAT_CENTERED:
                return "SDL_HAT_CENTERED";
            case SDL_HAT_RIGHT:
                return "SDL_HAT_RIGHT";
            case SDL_HAT_LEFTDOWN:
                return "SDL_HAT_LEFTDOWN";
            case SDL_HAT_DOWN:
                return "SDL_HAT_DOWN";
            case SDL_HAT_RIGHTDOWN:
                return "SDL_HAT_RIGHTDOWN";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Constructor
        JoystickState()
        {
        }

        // Members
        std::vector<int16_t> m_axisValue; //!< List containing current values of all axes
        std::vector<bool> m_buttonPressed; //!< List containing current state of all buttons (`true` = pressed, `false` = released)
        std::vector<uint8_t> m_hatValue; //!< List containing value of all hats
    };
} // namespace io
} // namespace broccoli

#endif // HAVE_SDL2
