/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <string>

namespace broccoli {
namespace io {
    //! Provides common ANSI code sequences for rich text formatting of console outputs
    /*! \ingroup broccoli_io_console */
    class ConsoleANSICodes {
    public:
        // Generic
        static inline std::string reset() { return "\u001b[0m"; } //!< ANSI "reset" (resets all colors and text effects)

        // Foreground color (=text color)
        class ForeGround {
        public:
            // Regular
            static inline std::string black() { return "\u001b[30m"; } //!< ANSI foreground color "black"
            static inline std::string red() { return "\u001b[31m"; } //!< ANSI foreground color "red"
            static inline std::string green() { return "\u001b[32m"; } //!< ANSI foreground color "green"
            static inline std::string yellow() { return "\u001b[33m"; } //!< ANSI foreground color "yellow"
            static inline std::string blue() { return "\u001b[34m"; } //!< ANSI foreground color "blue"
            static inline std::string magenta() { return "\u001b[35m"; } //!< ANSI foreground color "magenta"
            static inline std::string cyan() { return "\u001b[36m"; } //!< ANSI foreground color "cyan"
            static inline std::string white() { return "\u001b[37m"; } //!< ANSI foreground color "white"

            // Bright
            static inline std::string brightBlack() { return "\u001b[30;1m"; } //!< ANSI foreground color "black" (bright)
            static inline std::string brightRed() { return "\u001b[31;1m"; } //!< ANSI foreground color "red" (bright)
            static inline std::string brightGreen() { return "\u001b[32;1m"; } //!< ANSI foreground color "green" (bright)
            static inline std::string brightYellow() { return "\u001b[33;1m"; } //!< ANSI foreground color "yellow" (bright)
            static inline std::string brightBlue() { return "\u001b[34;1m"; } //!< ANSI foreground color "blue" (bright)
            static inline std::string brightMagenta() { return "\u001b[35;1m"; } //!< ANSI foreground color "magenta" (bright)
            static inline std::string brightCyan() { return "\u001b[36;1m"; } //!< ANSI foreground color "cyan" (bright)
            static inline std::string brightWhite() { return "\u001b[37;1m"; } //!< ANSI foreground color "white" (bright)
        };

        // Background color
        class BackGround {
        public:
            // Regular
            static inline std::string black() { return "\u001b[40m"; } //!< ANSI background color "black"
            static inline std::string red() { return "\u001b[41m"; } //!< ANSI background color "red"
            static inline std::string green() { return "\u001b[42m"; } //!< ANSI background color "green"
            static inline std::string yellow() { return "\u001b[43m"; } //!< ANSI background color "yellow"
            static inline std::string blue() { return "\u001b[44m"; } //!< ANSI background color "blue"
            static inline std::string magenta() { return "\u001b[45m"; } //!< ANSI background color "magenta"
            static inline std::string cyan() { return "\u001b[46m"; } //!< ANSI background color "cyan"
            static inline std::string white() { return "\u001b[47m"; } //!< ANSI background color "white"

            // Bright
            static inline std::string brightBlack() { return "\u001b[40;1m"; } //!< ANSI background color "black" (bright)
            static inline std::string brightRed() { return "\u001b[41;1m"; } //!< ANSI background color "red" (bright)
            static inline std::string brightGreen() { return "\u001b[42;1m"; } //!< ANSI background color "green" (bright)
            static inline std::string brightYellow() { return "\u001b[43;1m"; } //!< ANSI background color "yellow" (bright)
            static inline std::string brightBlue() { return "\u001b[44;1m"; } //!< ANSI background color "blue" (bright)
            static inline std::string brightMagenta() { return "\u001b[45;1m"; } //!< ANSI background color "magenta" (bright)
            static inline std::string brightCyan() { return "\u001b[46;1m"; } //!< ANSI background color "cyan" (bright)
            static inline std::string brightWhite() { return "\u001b[47;1m"; } //!< ANSI background color "white" (bright)
        };

        // Decorations
        static inline std::string bold() { return "\u001b[1m"; } //!< ANSI "bold"
        static inline std::string underline() { return "\u001b[4m"; } //!< ANSI "underline"
        static inline std::string reversed() { return "\u001b[7m"; } //!< ANSI "reversed"
    };
} // namespace io
} // namespace broccoli
