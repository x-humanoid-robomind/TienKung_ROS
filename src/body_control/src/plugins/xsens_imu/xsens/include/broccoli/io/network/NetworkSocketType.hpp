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
     * \addtogroup broccoli_io_network
     * \{
     */

    //! Specification of network socket types
    enum class NetworkSocketType : uint8_t {
        TCP_SERVER = 0, //!< **TCP server** socket
        TCP_CLIENT, //!< **TCP client** socket
        UDP, //!< **UDP** socket (combined server (for receiving) and client (for sending))
        NETWORKSOCKETTYPE_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given socket type
    static inline std::string networkSocketTypeString(const NetworkSocketType& type)
    {
        // Check type
        switch (type) {
        case NetworkSocketType::TCP_SERVER:
            return "TCP_SERVER";
        case NetworkSocketType::TCP_CLIENT:
            return "TCP_CLIENT";
        case NetworkSocketType::UDP:
            return "UDP";
        default:
            break;
        }

        // Unknown selection
        assert(false);
        return "UNKNOWN";
    }

    //! \}
} // namespace io
} // namespace broccoli
