/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Specification of file formats
    class PLYFormat {
    public:
        //! Specification of possible file formats of the Polygon File Format (PLY)
        enum class Type : uint8_t {
            UNKNOWN = 0, //!< Unknown format (this should **never** happen)
            ASCII, //!< Ascii ("text" format)
            BINARY_LITTLE_ENDIAN, //!< Binary format (little endian)
            BINARY_BIG_ENDIAN, //!< Binary format (big endian)
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given type
        static inline std::string toString(const Type& type)
        {
            // Check result
            switch (type) {
            case Type::UNKNOWN:
                return "UNKNOWN";
            case Type::ASCII:
                return "ascii";
            case Type::BINARY_LITTLE_ENDIAN:
                return "binary_little_endian";
            case Type::BINARY_BIG_ENDIAN:
                return "binary_big_endian";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }

        //! Returns the format from its string representation
        static inline Type fromString(const std::string& string)
        {
            if (string == "UNKNOWN")
                return Type::UNKNOWN;
            else if (string == "ascii")
                return Type::ASCII;
            else if (string == "binary_little_endian")
                return Type::BINARY_LITTLE_ENDIAN;
            else if (string == "binary_big_endian")
                return Type::BINARY_BIG_ENDIAN;
            else {
                assert(false);
                return Type::UNKNOWN;
            }
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
