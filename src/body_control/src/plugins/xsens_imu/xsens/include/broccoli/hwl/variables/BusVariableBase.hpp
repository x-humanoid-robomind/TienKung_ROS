/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../ForwardDeclarations.hpp"
#include <functional>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Base class for all bus variables
     * \ingroup broccoli_hwl_variables
     *
     * This base and all bus variables are implicitly convertible to
     * the underlying data type.
     *
     * A bus variable can be hashed via std::hash. The hash of a bus variable
     * and a pointer to its memory area (see \ref BusVariablePointerBase, \ref pointers.hpp)
     * is identical.
     *
     * \note Methods are thread-safe unless noted differently.
     *
     * \tparam Derived CRTP derived type
     */
    template <typename Derived>
    class BusVariableBase {
    public:
        //! Derived bus variable implementation type
        using ImplType = typename internal::BusVariableTypes<Derived>::ImplType;

        //! Derived bus variable data type
        using DataType = typename internal::BusVariableTypes<Derived>::DataType;

        BusVariableBase() = default;

        /*!
         * \brief Construct from data type value
         * \param value Initial value
         */
        explicit BusVariableBase(const DataType& value) noexcept(std::is_nothrow_copy_constructible<ImplType>::value)
            : m_impl(value)
        {
        }

        /*!
         * \brief Construct from data type value
         * \param value Initial value
         */
        explicit BusVariableBase(DataType&& value) noexcept
            : m_impl(std::move(value))
        {
        }

        operator DataType() const noexcept
        {
            return this->value();
        }

        //! Returns the current variable value
        DataType value() const noexcept
        {
            return this->impl().value();
        }

        /*!
         * \brief Implementation instance access
         *
         * \warning This is an **internal** interface which may be subject to change.
         * \return A reference to the underlying implementation type
         */
        ImplType& impl() noexcept { return m_impl; }

        /*!
         * \brief Implementation instance access
         *
         * \warning This is an **internal** interface which may be subject to change.
         * \return A const reference to the underlying implementation type
         */
        const ImplType& impl() const noexcept { return m_impl; }

        //! \copydoc broccoli::memory::ThreadSafeData::lockWithGuard()
        template <typename... Args>
        auto lockWithGuard(Args... args) noexcept -> decltype(impl().lockWithGuard(args...))
        {
            // Note: qcc has problems without trailing type deduction
            return impl().lockWithGuard(args...);
        }

        //! \copydoc broccoli::memory::ThreadSafeData::lockWithGuard()
        template <typename... Args>
        auto lockWithGuard(Args... args) const noexcept -> decltype(impl().lockWithGuard(args...))
        {
            // Note: qcc has problems without trailing type deduction
            return impl().lockWithGuard(args...);
        }

        //! \copydoc broccoli::memory::ThreadSafeData::lockConstWithGuard()
        template <typename... Args>
        auto lockConstWithGuard(Args... args) const noexcept -> decltype(impl().lockWithGuard(args...))
        {
            // Note: qcc has problems without trailing type deduction
            return impl().lockConstWithGuard(args...);
        }

        //! Returns an unsafe (no lock) reference to the bus variable container
        typename internal::BusVariableTypes<Derived>::ContainerType& unsafeContainerRef() noexcept { return impl().unsafeRef(); }

        //! Returns an unsafe (no lock) const reference to the bus variable container
        const typename internal::BusVariableTypes<Derived>::ContainerType& unsafeContainerRef() const noexcept { return impl().unsafeRef(); }

        //! Returns a copy of the bus variable container that represents the current state of the variable
        typename internal::BusVariableTypes<Derived>::ContainerType containerCopy() const noexcept
        {
            return impl().copy();
        }

        //! Returns the size of the bus variable data in bytes
        std::size_t size() const noexcept { return sizeof(DataType); }

        //! Returns a reference to the Derived type
        Derived& derived() noexcept
        {
            return *static_cast<Derived*>(this);
        }

        //! Returns a const reference to the Derived type
        const Derived& derived() const noexcept
        {
            return *static_cast<const Derived*>(this);
        }

        template <typename OtherDerived>
        bool operator==(const BusVariableBase<OtherDerived>& rhs) const
        {
            return this->value() == rhs.value();
        }
        template <typename OtherDerived>
        bool operator!=(const BusVariableBase<OtherDerived>& rhs) const
        {
            return !(rhs == *this);
        }

    private:
        ImplType m_impl;
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <typename Derived>
struct hash<broccoli::hwl::BusVariableBase<Derived>> : std::hash<const void*> {
    std::size_t operator()(const broccoli::hwl::BusVariableBase<Derived>& var) const noexcept
    {
        // use pointer to shared data area
        return std::hash<const void*>()(static_cast<const void*>(&var.unsafeContainerRef()));
    }
};
} // namespace std
