/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "CANopen.hpp"

namespace broccoli {
namespace hwl {

    /*!
     * \brief Raw CAN bus description type
     * \ingroup broccoli_hwl_bus_types
     */
    struct CAN {

        //! Message ID type
        using MessageIDType = uint32_t;

        /*!
         * \brief Tests an identifier for 11bit range
         * \param identifier CAN message identifier
         * \return True if the CAN message identifier uses a maximum of 11 bits
         */
        static bool is11bit(const MessageIDType& identifier)
        {
            return (identifier >> 11) == 0;
        }

        /*!
         * \brief Identifier for output messages
         *
         * This object can be hashed via std::hash
         */
        struct OutputMessageIdentifierType {

            /*!
             * \brief CAN message ID
             * \note 11bit for base frames and 29 bit for extended frames
             */
            MessageIDType messageIdentifier;

            //! Returns a random identifier (for testing)
            static OutputMessageIdentifierType random()
            {
                return { 0x0312 };
            }

            //! Returns a string representation
            std::string toString() const
            {
                std::stringstream stream;
                stream << std::hex
                       << std::setiosflags(std::ios::showbase)
                       << "id " << messageIdentifier;
                return stream.str();
            }

            bool operator==(const OutputMessageIdentifierType& rhs) const
            {
                return messageIdentifier == rhs.messageIdentifier;
            }
            bool operator!=(const OutputMessageIdentifierType& rhs) const
            {
                return !(rhs == *this);
            }
        };

        /*!
         * \brief Identifier for input messages
         *
         * This object can be hashed via std::hash
         */
        struct InputMessageIdentifierType {

            /*!
             * \brief CAN message ID
             * \note 11bit for base frames and 29 bit for extended frames
             */
            MessageIDType messageIdentifier;

            //! Offset of object in CAN message (in bytes)
            uint8_t dataOffset;

            //! Returns a random identifier (for testing)
            static InputMessageIdentifierType random()
            {
                return { 0x0314, 0x1 };
            }

            //! Returns a string representation
            std::string toString() const
            {
                std::stringstream stream;
                stream << std::hex
                       << std::setiosflags(std::ios::showbase)
                       << "id " << messageIdentifier
                       << ", offset " << (int)dataOffset;
                return stream.str();
            }

            bool operator==(const InputMessageIdentifierType& rhs) const
            {
                return messageIdentifier == rhs.messageIdentifier && dataOffset == rhs.dataOffset;
            }
            bool operator!=(const InputMessageIdentifierType& rhs) const
            {
                return !(rhs == *this);
            }
        };

        //! Identifier for sync input messages
        using InputObjectIdentifierType = InputMessageIdentifierType;

        //! Identifier for sync output messages
        using OutputObjectIdentifierType = OutputMessageIdentifierType;

        //! Identifier for async input messages
        using AsyncInputObjectIdentifierType = InputMessageIdentifierType;

        //! Identifier for async output messages
        using AsyncOutputObjectIdentifierType = OutputMessageIdentifierType;

        /*!
         * \brief Identifier for Remote Frame Requests
         *
         * This object can be hashed via std::hash
         */
        struct RemoteFrameIdentifierType {

            /*!
             * \brief Request/Response message ID for remote frame and the answer
             * \note 11bit for base frames and 29 bit for extended frames
             */
            MessageIDType messageIdentifier;

            //! Returns a random identifier (for testing)
            static RemoteFrameIdentifierType random()
            {
                return { 0x0316 };
            }

            //! Returns a string representation
            std::string toString() const
            {
                std::stringstream stream;
                stream << std::hex
                       << std::setiosflags(std::ios::showbase)
                       << "id " << messageIdentifier;
                return stream.str();
            }

            bool operator==(const RemoteFrameIdentifierType& rhs) const
            {
                return messageIdentifier == rhs.messageIdentifier;
            }
            bool operator!=(const RemoteFrameIdentifierType& rhs) const
            {
                return !(rhs == *this);
            }
        };

        //! State type for plain CAN (uses CANopen states)
        using StateType = CANopen::StateType;

        //! Stale emergency object (not supported)
        struct EmergencyObject {
        };

        //! Stale emergency object identifier (not supported)
        struct EmergencyObjectIdentifier {
        };
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <>
struct hash<broccoli::hwl::CAN::OutputMessageIdentifierType> : std::hash<uint32_t> {
    std::size_t operator()(const broccoli::hwl::CAN::OutputMessageIdentifierType& id) const noexcept
    {
        return std::hash<uint32_t>()(id.messageIdentifier);
    }
};

template <>
struct hash<broccoli::hwl::CAN::InputMessageIdentifierType> : std::hash<uint64_t> {
    std::size_t operator()(const broccoli::hwl::CAN::InputMessageIdentifierType& id) const noexcept
    {
        return std::hash<uint64_t>()((id.messageIdentifier << 8) + id.dataOffset);
    }
};

template <>
struct hash<broccoli::hwl::CAN::RemoteFrameIdentifierType> : std::hash<uint32_t> {
    std::size_t operator()(const broccoli::hwl::CAN::RemoteFrameIdentifierType& id) const noexcept
    {
        // request is already unique
        return std::hash<uint32_t>()(id.messageIdentifier);
    }
};
} // namespace std
