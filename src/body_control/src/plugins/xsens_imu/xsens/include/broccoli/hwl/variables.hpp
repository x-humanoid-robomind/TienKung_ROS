/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "ForwardDeclarations.hpp"
#include "variables/AsyncBusVariableValue.hpp"
#include "variables/BusVariableImpl.hpp"
#include "variables/ReadableBusVariable.hpp"
#include "variables/WriteableBusVariable.hpp"

//! Defines bus variable types

namespace broccoli {
namespace hwl {

    /*!
     * \addtogroup broccoli_hwl_variables
     * \{
     */

    //! Input (read-only) synchronous bus variable for variable data type T
    template <typename T>
    using InputBusVariable = ReadableBusVariable<BusVariableValue<T>, internal::BusVariableImpl>;

    //! Output (read-writeable) synchronous bus variable for variable data type T
    template <typename T>
    using OutputBusVariable = WriteableBusVariable<BusVariableValue<T>, internal::BusVariableImpl>;

    //! Input (read-only) asynchronous bus variable for variable data type T
    template <typename T>
    using AsyncInputBusVariable = ReadableBusVariable<AsyncBusVariableValue<T>, internal::BusVariableImpl>;

    //! Output (read-writeable) asynchronous bus variable for variable data type T
    template <typename T>
    using AsyncOutputBusVariable = WriteableBusVariable<AsyncBusVariableValue<T>, internal::BusVariableImpl>;

    //! \}
} // namespace hwl
} // namespace broccoli
