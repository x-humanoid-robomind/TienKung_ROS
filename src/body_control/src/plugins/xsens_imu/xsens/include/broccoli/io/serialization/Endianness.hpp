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
    namespace serialization {
        /*!
         * \addtogroup broccoli_io_serialization
         * \{
         */

        //! Specification of possible byte-orders
        enum class Endianness : uint8_t {
            UNKNOWN = 0, //!< Not specified byte order
            LITTLE, //!< **Little-endian** byte order: store least-significant byte first
            BIG, //!< **Big-endian** byte order: store most-significant byte first
            ENDIANNESS_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given endianness
        static inline std::string endiannessToString(const Endianness& endianness)
        {
            // Check endianness
            switch (endianness) {
            case Endianness::UNKNOWN:
                return "UNKNOWN";
            case Endianness::LITTLE:
                return "LITTLE";
            case Endianness::BIG:
                return "BIG";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }

        //! \}
    } // namespace serialization
} // namespace io
} // namespace broccoli
