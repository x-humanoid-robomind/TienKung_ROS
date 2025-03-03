/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_pnm
     * \{
     */

    //! Specification of file encodings for the Portable Anymap (PNM) format
    class PNMFileEncoding {
    public:
        //! Specification of file encodings for the Portable Anymap (PNM) format
        enum class Type : uint8_t {
            UNKNOWN = 0, //!< Unknown encoding (this should **never** happen)
            ASCII, //!< Ascii encoding
            BINARY, //!< Binary encoding (big endian)
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
                return "ASCII";
            case Type::BINARY:
                return "BINARY";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
