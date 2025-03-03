/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "AsyncTransferState.hpp"
#include "AsyncTransferStateHandle.hpp"
#include "BusVariableValue.hpp"
#include <utility>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Value container for an asynchronous bus variable
     * \ingroup broccoli_hwl_variables
     *
     * This contains the variable data itself and additional states
     * for the transfer of asynchronous data on a data bus.
     * \tparam Type Data type of the bus variable
     */
    template <typename Type>
    class AsyncBusVariableValue : public BusVariableValue<Type>, public AsyncTransferState {
    public:
        using DataType = Type;
        using BusVariableValue<Type>::BusVariableValue;

        template <typename T>
        AsyncBusVariableValue& operator=(T&& data) noexcept
        {
            if (this->value() != data) {
                this->markUncertain();
            }
            BusVariableValue<Type>::operator=(std::forward<T>(data));
            return *this;
        }
    };

    /*!
     * \brief Specialization of AsyncBusVariableValue for use in an AsyncBusVariablePointer
     *
     * See also \ref BusVariablePointerBase for more details on this container type.
     * \ingroup broccoli_hwl_variables
     */
    template <>
    class AsyncBusVariableValue<void*> : public BusVariableValue<void*>, public AsyncTransferStateHandle {
    public:
        using DataType = void*;

        /*!
         * \brief Construct AsyncBusVariableValue<void*> from reference object
         *
         * This stores a pointer to the passed object and is internally used to reference
         * bus variable data on BusDriver side.
         * \tparam T Data type of the variable to reference to
         * \param ref Reference to an AsyncBusVariableValue container
         */
        template <typename T>
        AsyncBusVariableValue(AsyncBusVariableValue<T>& ref)
            : BusVariableValue<void*>(ref)
            , AsyncTransferStateHandle(ref)
        {
        }
    };

} // namespace hwl
} // namespace broccoli
