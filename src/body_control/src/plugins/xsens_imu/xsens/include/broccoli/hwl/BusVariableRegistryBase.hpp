/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

namespace broccoli {
namespace hwl {

    /*!
     * \brief Base class for a bus variable registry
     * \ingroup broccoli_hwl
     *
     * A bus variable registry is used on BusDriver side to keep track of
     * all bus variables linked to objects. A BusDriver implementation must
     * provide its own BusVariableRegistry, which is passed to all BusDevice
     * instances when added to a BusDriver, allowing the devices to register their variables.
     *
     * \tparam Derived The CRTP derived type
     */
    template <typename Derived>
    class BusVariableRegistryBase {
    public:
        //! Returns a reference to the derived instance
        Derived& derived()
        {
            return *static_cast<Derived*>(this);
        }

        //! Returns a const reference to the derived instance
        const Derived& derived() const
        {
            return *static_cast<const Derived*>(this);
        }

        /*!
         * \brief Register a bus variable
         * \tparam VariableType Type of bus variable
         * \tparam IdentifierType Object identifier type
         * \param variable The bus variable to register
         * \param objectId The object id the variable should be linked to
         */
        template <typename VariableType, typename IdentifierType>
        void registerVariable(VariableType& variable,
            const IdentifierType& objectId) { derived().registerVariable(variable, objectId); }

        /*!
         * \brief Register a bus variable with specified endianness
         * \tparam VariableType Type of bus variable
         * \tparam IdentifierType Object identifier type
         * \param variable The bus variable to register
         * \param objectId The object id the variable should be linked to
         * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
         */
        template <typename VariableType, typename IdentifierType>
        void registerVariable(VariableType& variable,
            const IdentifierType& objectId, bool isLittleEndian) { derived().registerVariable(variable, objectId, isLittleEndian); }
    };

} // namespace hwl
} // namespace broccoli
