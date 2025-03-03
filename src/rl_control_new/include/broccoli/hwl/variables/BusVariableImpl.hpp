/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../../memory/ThreadSafeData.hpp"
#include "../ForwardDeclarations.hpp"

namespace broccoli {
namespace hwl {

    namespace internal {
        /*!
         * \brief Bus variable implementation type
         *
         * This implements the actual storing of guarded data using memory::ThreadSafeData.
         * All methods are thread-safe.
         * \tparam ContainerType_ Type of the container to store
         */
        template <typename ContainerType_>
        class BusVariableImpl : public memory::ThreadSafeData<ContainerType_, BROCCOLI_HWL_MUTEX_TYPE, BROCCOLI_HWL_READ_LOCK_TYPE, BROCCOLI_HWL_WRITE_LOCK_TYPE> {
        public:
            //! Container data type
            using ContainerType = ContainerType_;

            //! Variable data type
            using DataType = typename ContainerType_::DataType;

            BusVariableImpl() noexcept
                : memory::ThreadSafeData<ContainerType>(m_mutex)
            {
            }

            /*!
             * \brief Construct from variable value
             * \param value Initial value
             */
            explicit BusVariableImpl(const DataType& value) noexcept
                : memory::ThreadSafeData<ContainerType>(m_mutex, ContainerType(value))
            {
            }

            /*!
             * \brief Construct from variable value
             * \param value Initial value
             */
            explicit BusVariableImpl(DataType&& value) noexcept
                : memory::ThreadSafeData<ContainerType>(m_mutex, ContainerType(std::move(value)))
            {
            }

            /*!
             * \brief Copy constructor
             * \warning When copied, links are lost and a separate memory protection is used
             * \param other Instance to copy from
             */
            BusVariableImpl(const BusVariableImpl& other) noexcept
                : memory::ThreadSafeData<ContainerType>(m_mutex, other.value())
            {
            }

            BusVariableImpl(BusVariableImpl&& other) noexcept
                : m_mutex(std::move(other.m_mutex))
                , memory::ThreadSafeData<ContainerType>(std::move(other))
            {
            }

            BusVariableImpl& operator=(const BusVariableImpl& other) noexcept = default;

            BusVariableImpl& operator=(const DataType& data)
            {
                auto guard = this->lockWithGuard();
                *guard = data;
                return *this;
            }

            BusVariableImpl& operator=(DataType&& data) noexcept
            {
                auto guard = this->lockWithGuard();
                *guard = std::move(data);
                return *this;
            }

            //! Returns the current value of the bus variable
            DataType value() const noexcept
            {
                auto guard = this->lockConstWithGuard();
                DataType copy(guard->value());
                return copy;
            }

        protected:
            mutable typename memory::ThreadSafeData<ContainerType>::MutexType m_mutex;
        };
    } // namespace internal

} // namespace hwl
} // namespace broccoli
