/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <string>
#include <utility>

namespace broccoli {
namespace core {

    /*!
     * \brief Extendable type-safe enum implementation
     * This is a type-safe enumerable state implementation, which
     * may be used to replace unscoped C enums. In comparison to C++11
     * scoped enums, this implementation can be extended by user-defined methods
     * operating on the value of the enum (state).
     * \ingroup broccoli_core
     *
     * \note To create a type-safe enum class, refer to the following example:
     * \code{.cpp}
     * class Color : public EnumerableState<uint8_t> {
     * public:
     *     static constexpr Color red() { return Color(0); }
     *     static constexpr Color green() { return Color(1); }
     *     static constexpr Color yellow() { return Color(2); }
     *     static constexpr Type count() { return 3; }
     *
     *     //! Explicit contructor from given integer value
     *     //! You may want to make this private for even more type-safety
     *     constexpr explicit Color(Type state)
     *         : EnumerableState<Type>(state)
     *     {
     *     }
     * };
     * \endcode
     *
     * \tparam Type Integer type for the enum
     */
    template <typename IntegerType>
    class EnumerableState {
    public:
        using Type = IntegerType;
        static_assert(std::is_integral<IntegerType>::value, "Integral type required.");

        //! Copy constructor
        constexpr EnumerableState(const EnumerableState<IntegerType>& other)
            : m_state(other.m_state)
        {
        }

        //! Move constructor
        constexpr EnumerableState(EnumerableState<IntegerType>&& other)
            : m_state(std::move(other.m_state))
        {
        }

        //! Copy assignment operator
        EnumerableState<IntegerType>& operator=(const EnumerableState<IntegerType>& other)
        {
            this->m_state = other.m_state;
            return *this;
        }

        //! Move assignment operator
        EnumerableState<IntegerType>& operator=(EnumerableState<IntegerType>&& other)
        {
            this->m_state = std::move(other.m_state);
            return *this;
        }

        //! Returns the state of this enumeration as integer type
        constexpr const IntegerType& integer() const
        {
            return m_state;
        }

        //! Comparison operator
        bool operator==(const EnumerableState& other) const
        {
            return m_state == other.m_state;
        }

        //! Comparison operator
        bool operator!=(const EnumerableState& other) const
        {
            return m_state != other.m_state;
        }

        bool operator<(const EnumerableState& rhs) const
        {
            return m_state < rhs.m_state;
        }
        bool operator>(const EnumerableState& rhs) const
        {
            return rhs < *this;
        }
        bool operator<=(const EnumerableState& rhs) const
        {
            return !(rhs < *this);
        }
        bool operator>=(const EnumerableState& rhs) const
        {
            return !(*this < rhs);
        }

        EnumerableState<IntegerType>& operator++()
        {
            m_state++;
            return *this;
        }

        EnumerableState<IntegerType>& operator--()
        {
            m_state--;
            return *this;
        }

        EnumerableState<IntegerType> operator++(int)
        {
            EnumerableState<IntegerType> result = *this;
            m_state++;
            return result;
        }

        EnumerableState<IntegerType> operator--(int)
        {
            EnumerableState<IntegerType> result = *this;
            m_state--;
            return result;
        }

    protected:
        //! Explicit constructor from integral type
        constexpr explicit EnumerableState(const IntegerType& state)
            : m_state(state)
        {
        }

        //! Explicit move constructor from integral type
        constexpr explicit EnumerableState(IntegerType&& state)
            : m_state(std::move(state))
        {
        }

        //! Internal state of this enumeration
        IntegerType m_state = 0;
    };

} // namespace core
} // namespace broccoli
