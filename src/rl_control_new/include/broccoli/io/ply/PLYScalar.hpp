/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../encoding.hpp"
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    // Type definitions
    using CHAR_t = int8_t;
    using UCHAR_t = uint8_t;
    using SHORT_t = int16_t;
    using USHORT_t = uint16_t;
    using INT_t = int32_t;
    using UINT_t = uint32_t;
    using FLOAT_t = float;
    using DOUBLE_t = double;

    //! Helper class for dealing with single scalar values (numbers) stored in the Polygon File Format (PLY)
    class PLYScalar {
    public:
        //! Specification of scalar types
        enum class Type : uint8_t {
            UNKNOWN = 0, //!< Unknown type (this should **never** happen)
            CHAR, //!< Character (1 byte)
            UCHAR, //!< Unsigned character (1 byte)
            SHORT, //!< Short integer (2 bytes)
            USHORT, //!< Unsigned short integer (2 bytes)
            INT, //!< Integer (4 bytes)
            UINT, //!< Unsigned integer (4 bytes)
            FLOAT, //!< Single-precision float (4 bytes)
            DOUBLE, //!< Double-precision float (8 bytes)
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given scalar type
        static inline std::string typeToString(const Type& type)
        {
            // Check type
            switch (type) {
            case Type::UNKNOWN:
                return "unknown";
            case Type::CHAR:
                return "char";
            case Type::UCHAR:
                return "uchar";
            case Type::SHORT:
                return "short";
            case Type::USHORT:
                return "ushort";
            case Type::INT:
                return "int";
            case Type::UINT:
                return "uint";
            case Type::FLOAT:
                return "float";
            case Type::DOUBLE:
                return "double";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "unknown";
        }

        //! Returns the scalar type from its string representation
        static inline Type typeFromString(const std::string& string)
        {
            if (string == "unknown")
                return Type::UNKNOWN;
            else if (string == "char")
                return Type::CHAR;
            else if (string == "uchar")
                return Type::UCHAR;
            else if (string == "short")
                return Type::SHORT;
            else if (string == "ushort")
                return Type::USHORT;
            else if (string == "int")
                return Type::INT;
            else if (string == "uint")
                return Type::UINT;
            else if (string == "float")
                return Type::FLOAT;
            else if (string == "double")
                return Type::DOUBLE;
            else {
                assert(false);
                return Type::UNKNOWN;
            }
        }

        //! Returns the count of bytes related to the given scalar type
        static inline uint8_t byteCount(const Type& type)
        {
            // Check type
            switch (type) {
            case Type::CHAR:
            case Type::UCHAR:
                return 1;
            case Type::SHORT:
            case Type::USHORT:
                return 2;
            case Type::INT:
            case Type::UINT:
                return 4;
            case Type::FLOAT:
                return 4;
            case Type::DOUBLE:
                return 8;
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return 0;
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
