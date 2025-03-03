/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "broccoli/core/EnumerableState.hpp"
#include <iomanip>
#include <sstream>
#include <string>

namespace broccoli {
namespace hwl {

    /*!
     * \brief CANopen bus description type
     * \ingroup broccoli_hwl_bus_types
     */
    struct CANopen {

        /*!
         * \brief Identifier for process objects
         *
         * This object can be hashed via std::hash
         */
        struct ObjectIdentifierType {
            uint16_t index;

            //! Returns a random identifier (for testing)
            static ObjectIdentifierType random()
            {
                return { 0x5001 };
            }

            //! Returns a string representation
            std::string toString() const
            {
                std::stringstream stream;
                stream << "idx 0x" << std::setbase(16) << index;
                return stream.str();
            }

            bool operator==(const ObjectIdentifierType& rhs) const
            {
                return index == rhs.index;
            }
            bool operator!=(const ObjectIdentifierType& rhs) const
            {
                return !(rhs == *this);
            }
        };

        //! Identifier for input PDOs
        using InputObjectIdentifierType = ObjectIdentifierType;

        //! Identifier for output PDOs
        using OutputObjectIdentifierType = ObjectIdentifierType;

        /*!
         * \brief Identifier for service data objects
         *
         * This object can be hashed via std::hash
         */
        struct AsyncObjectIdentifierType {
            uint16_t index;
            uint8_t subIndex;

            //! Returns a random identifier (for testing)
            static AsyncObjectIdentifierType random()
            {
                return { 0x500, 0x1 };
            }

            //! Returns a string representation
            std::string toString() const
            {
                std::stringstream stream;
                stream << std::hex
                       << std::setfill('0')
                       << "idx 0x" << std::setw(4) << index << ", subidx 0x" << std::setw(2) << (int)subIndex;
                return stream.str();
            }

            bool operator==(const AsyncObjectIdentifierType& rhs) const
            {
                return index == rhs.index && subIndex == rhs.subIndex;
            }
            bool operator!=(const AsyncObjectIdentifierType& rhs) const
            {
                return !(rhs == *this);
            }
        };

        //! Identifier for input SDOs
        using AsyncInputObjectIdentifierType = AsyncObjectIdentifierType;

        //! Identifier for output SDOs
        using AsyncOutputObjectIdentifierType = AsyncObjectIdentifierType;

        //! State type for CANopen
        struct StateType : public core::EnumerableState<uint8_t> {
        public:
            StateType()
                : StateType(unknown())
            {
            }

            //! Unknown state
            static constexpr StateType unknown() { return StateType(0); }

            //! Initialization state
            static constexpr StateType init() { return StateType(1); }

            //! Pre-Operational state. Only Service Data Objects are processed
            static constexpr StateType preOp() { return StateType(2); }

            //! Safe-Operational state. Process data is exchanged, but not applied on the devices (safe state)
            static constexpr StateType safeOp() { return StateType(3); }

            //! Operational state
            static constexpr StateType op() { return StateType(4); }

            //! Returns a string representation of the state
            std::string toString() const
            {
                if (*this == init()) {
                    return "Init";
                } else if (*this == preOp()) {
                    return "PreOp";
                } else if (*this == op()) {
                    return "Op";
                } else if (*this == safeOp()) {
                    return "SafeOp";
                }
                return "Unknown";
            }

            using core::EnumerableState<uint8_t>::EnumerableState;
        };

        //! Emergency Message type
#pragma pack(push, 1)
        struct EmergencyObject {
            //! Error Code
            uint16_t errorCode = 0;

            //! Error register
            uint8_t errorRegister = 0;

            //! Additional error data
            uint8_t errorData[5]{};
        };
#pragma pack(pop)

        //! Emergency Message Identifier
        struct EmergencyObjectIdentifier {
            //! Base emergency header id (node id must be added by BusDriver)
            uint16_t index = 0x80;
        };
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <>
struct hash<broccoli::hwl::CANopen::ObjectIdentifierType> : std::hash<uint16_t> {
    std::size_t operator()(const broccoli::hwl::CANopen::ObjectIdentifierType& id) const noexcept
    {
        return std::hash<uint16_t>()(id.index);
    }
};

template <>
struct hash<broccoli::hwl::CANopen::AsyncObjectIdentifierType> : std::hash<uint32_t> {
    std::size_t operator()(const broccoli::hwl::CANopen::AsyncObjectIdentifierType& id) const noexcept
    {
        uint32_t idx = id.index;
        return std::hash<uint32_t>()((idx << 8) + id.subIndex);
    }
};
} // namespace std
