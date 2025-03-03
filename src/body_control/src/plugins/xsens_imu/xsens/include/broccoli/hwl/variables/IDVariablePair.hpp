/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <type_traits>

namespace broccoli {
namespace hwl {

    /*!
     * \brief A structure for storing an object identifier and a corresponding bus variable
     * \ingroup broccoli_hwl_variables
     * \tparam ObjectIdentifierType Type of the object identifier
     * \tparam VariableType Type of the bus variable
     */
    template <typename ObjectIdentifierType, typename VariableType>
    struct IDVariablePair {
        //! Identifier for variable
        ObjectIdentifierType id;

        //! The bus variable or a pointer to it
        VariableType variable;
    };

    /*!
     * \brief Make an IDVariablePair object from given arguments
     * \ingroup broccoli_hwl_variables
     * \param id Identifier
     * \param variable Variable
     */
    template <typename ObjectIdentifierType, typename VariableType>
    auto makeIDVariablePair(ObjectIdentifierType&& id, VariableType&& variable)
    {
        return IDVariablePair<std::decay_t<ObjectIdentifierType>, std::decay_t<VariableType>>{ id, variable };
    }
} // namespace hwl
} // namespace broccoli
