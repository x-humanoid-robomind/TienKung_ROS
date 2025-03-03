/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "AsyncBusVariableValue.hpp"
#include "BusVariablePointerBase.hpp"
#include "BusVariableValue.hpp"

//! Defines pointer types to bus variables

namespace broccoli {
namespace hwl {
    /*!
     * \addtogroup broccoli_hwl_variables
     * \{
     */

    //! Pointer to a synchronous bus variable (input or output)
    using BusVariablePointer = BusVariablePointerBase<BusVariableValue>;

    //! Pointer to an asynchronous bus variable (input or output)
    using AsyncBusVariablePointer = BusVariablePointerBase<AsyncBusVariableValue>;

    //! \}

} // namespace hwl
} // namespace broccoli
