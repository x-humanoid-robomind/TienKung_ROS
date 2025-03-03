/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "CANopen.hpp"
#include <functional>

namespace broccoli {
namespace hwl {

    /*!
     * \brief EtherCAT bus description type
     * \ingroup broccoli_hwl_bus_types
     */
    struct EtherCAT {
        /*!
         * \brief Identifier for process objects
         *
         * This object can be hashed via std::hash
         */
        struct ObjectIdentifierType {

            //! Object name
            std::string name;

            //! Returns a random identifier (for testing)
            static ObjectIdentifierType random()
            {
                return { "superFastSubaru" };
            }

            //! Returns a string representation
            std::string toString() const
            {
                return name;
            }

            bool operator==(const ObjectIdentifierType& rhs) const
            {
                return name == rhs.name;
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

        //! Identifier for service data objects
        using AsyncObjectIdentifierType = CANopen::AsyncObjectIdentifierType;

        //! Identifier for input SDOs
        using AsyncInputObjectIdentifierType = AsyncObjectIdentifierType;

        //! Identifier for output SDOs
        using AsyncOutputObjectIdentifierType = AsyncObjectIdentifierType;

        //! State type for EtherCAT
        using StateType = CANopen::StateType;

        //! CoE Emergency Object
        using EmergencyObject = CANopen::EmergencyObject;

        //! CoE Emergency Object Identifier
        struct EmergencyObjectIdentifier {
        };
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <>
struct hash<broccoli::hwl::EtherCAT::ObjectIdentifierType> : std::hash<std::string> {
    std::size_t operator()(const broccoli::hwl::EtherCAT::ObjectIdentifierType& id) const noexcept
    {
        return std::hash<std::string>()(id.name);
    }
};
} // namespace std
