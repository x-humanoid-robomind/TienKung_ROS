/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "ForwardDeclarations.hpp"
#include <type_traits>

namespace broccoli {
namespace hwl {
    /*!
     * \addtogroup broccoli_hwl_variables
     * \{
     */

    //! If T is an asynchronous bus variable, this provides the member constant is_async_bus_variable::value equal true. For any other type, is_async_bus_variable::value is false.
    template <typename T>
    struct is_async_bus_variable : std::is_same<typename T::ContainerType, AsyncBusVariableValue<typename T::DataType>> {
    };

    //! If T is a synchronous bus variable, this provides the member constant is_sync_bus_variable::value equal true. For any other type, is_sync_bus_variable::value is false.
    template <typename T>
    struct is_sync_bus_variable : std::is_same<typename T::ContainerType, BusVariableValue<typename T::DataType>> {
    };

    //! If T is an output bus variable (writeable), this provides the member constant is_output_bus_variable::value equal true. For any other type, is_output_bus_variable::value is false.
    template <typename>
    struct is_output_bus_variable;
    //! \}

    template <typename ContainerType_, template <typename> class ImplType_>
    struct is_output_bus_variable<ReadableBusVariable<ContainerType_, ImplType_>> : std::false_type {
    };

    template <typename ContainerType_, template <typename> class ImplType_>
    struct is_output_bus_variable<WriteableBusVariable<ContainerType_, ImplType_>> : std::true_type {
    };

} // namespace hwl
} // namespace broccoli
