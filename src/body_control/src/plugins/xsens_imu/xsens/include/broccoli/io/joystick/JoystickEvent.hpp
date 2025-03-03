/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// The joystick module requires SDL2 library
#ifdef HAVE_SDL2

#include <SDL.h>
#include <array>
#include <assert.h>
#include <stdint.h>

namespace broccoli {
namespace io {
    /*!
     * \brief Data container for a joystick event (axis, button, ball and hat events)
     * \ingroup broccoli_io_joystick
     */
    class JoystickEvent {
    public:
        //! Returns the string representation of the given event type
        static std::string typeString(const SDL_EventType& type)
        {
            // Check type
            switch (type) {
            case SDL_JOYAXISMOTION:
                return "SDL_JOYAXISMOTION";
            case SDL_JOYBALLMOTION:
                return "SDL_JOYBALLMOTION";
            case SDL_JOYHATMOTION:
                return "SDL_JOYHATMOTION";
            case SDL_JOYBUTTONDOWN:
                return "SDL_JOYBUTTONDOWN";
            case SDL_JOYBUTTONUP:
                return "SDL_JOYBUTTONUP";
            case SDL_JOYDEVICEADDED:
                return "SDL_JOYDEVICEADDED";
            case SDL_JOYDEVICEREMOVED:
                return "SDL_JOYDEVICEREMOVED";
            case SDL_QUIT:
                return "SDL_QUIT";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of the type of **this** event
        std::string typeString() const { return typeString(m_type); }

        //! Default constructor
        JoystickEvent()
            : m_type(SDL_JOYAXISMOTION)
            , m_timestamp(0)
            , m_index(0)
            , m_value({ { 0 } })
        {
        }

        // Members
        SDL_EventType m_type; //!< Type of the event (`SDL_JOYAXISMOTION`, `SDL_JOYBALLMOTION`, `SDL_JOYHATMOTION`, `SDL_JOYBUTTONDOWN`, `SDL_JOYBUTTONUP`, 'SDL_JOYDEVICEADDED', 'SDL_JOYDEVICEREMOVED', `SDL_QUIT`)
        uint32_t m_timestamp; //!< Timestamp reported by driver in milliseconds
        uint8_t m_index; //!< Number/index of axis/ball/hat/button
        std::array<int16_t, 2> m_value; //!< Value/state of axis/ball/hat/button (2nd entry only for ball: [0]=deltaX, [1]=deltaY)
    };
} // namespace io
} // namespace broccoli

#endif // HAVE_SDL2
