/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <utility>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Value container for a synchronous bus variable
     * \ingroup broccoli_hwl_variables
     *
     * \note This type is implicitly convertable to the variable data type.
     * \tparam Type Data type of the bus variable
     */
    template <typename Type>
    class BusVariableValue {
    public:
        //! The variable data type
        using DataType = Type;

        BusVariableValue() = default;

        template <typename T>
        explicit BusVariableValue(T&& value) noexcept
            : m_value(std::forward<T>(value))
        {
        }

        template <typename T>
        BusVariableValue& operator=(T&& data) noexcept
        {
            m_value = std::forward<T>(data);
            return *this;
        }

        operator Type() const noexcept
        {
            return m_value;
        }

        //! Returns a const reference to the value
        const Type& value() const noexcept { return m_value; }

        //! Returns a reference to the value
        Type& value() noexcept { return m_value; }

    protected:
        Type m_value{};
    };

    /*!
     * \brief Specialization of BusVariableValue for use in a BusVariablePointer
     * \ingroup broccoli_hwl_variables
     *
     * See also \ref BusVariablePointerBase for more details on this container type.
     */
    template <>
    class BusVariableValue<void*> {
    public:
        using DataType = void*;

        /*!
         * \brief Construct BusVariableValue<void*> from reference object
         *
         * This stores a pointer to the passed object and is internally used to reference
         * bus variable data on BusDriver side.
         * \tparam T Data type of the variable to reference to
         * \param ref Reference to a BusVariableValue container
         */
        template <typename T>
        BusVariableValue(BusVariableValue<T>& ref)
            : m_pointer(static_cast<void*>(&ref))
        {
        }

        explicit operator void*() noexcept
        {
            return m_pointer;
        }

        explicit operator const void*() const noexcept
        {
            return m_pointer;
        }

        /*!
         * \brief Applies pointer indirection with specified type
         * \tparam T The target data type
         * \return A reference to the typed object
         */
        template <typename T>
        T& indirect()
        {
            return *static_cast<T*>(ptr());
        }

        /*!
         * \brief Applies pointer indirection with specified type
         * \tparam T The target data type
         * \return A const reference to the typed object
         */
        template <typename T>
        const T& indirect() const
        {
            return *static_cast<const T*>(ptr());
        }

        //! Returns a const pointer to the bus variable memory region
        const void* ptr() const noexcept { return m_pointer; }

        //! Returns a pointer to the bus variable memory region
        void* ptr() noexcept { return m_pointer; }

    protected:
        void* m_pointer;
    };

} // namespace hwl
} // namespace broccoli
