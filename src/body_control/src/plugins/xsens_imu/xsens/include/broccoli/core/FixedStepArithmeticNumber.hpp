/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "floats.hpp"
#include <cassert>
#include <cmath>

namespace broccoli {
namespace core {
    /*!
     * \brief Represents a floating-point value in form of fixed-step integer arithmetic
     * \ingroup broccoli_core
     * A floating-point number \f$n\f$ is represented by \f$n = i \cdot s\f$,
     * with the integer state \f$i \in \mathcal{Z}\f$, and the step length \f$s \in \mathcal{R}\f$.
     * With standard floating-point arithmetic the numerical error (caused by the inexact representation of floats)
     * increases if small numbers like a step size are added to a variable. This representation has numerical advantages
     * if the number \f$n\f$ is only increased/decreased by the step length \f$s\f$ as it generates its value by multiplication.
     *
     * \note This class implements comparison operands for a safe comparison with floating-point variables.
     * For comparison the implementations in broccoli::core::math with the default options are used.
     * If you want to compare the integer part of the variable use getInteger() or the explicit conversion to an integer type.
     *
     * \tparam T An integer type used to store \f$i\f$
     */
    template <typename T, typename = std::enable_if<std::numeric_limits<T>::is_integer, T>>
    class FixedStepArithmeticNumber {
    public:
        //! Construct a fixed-step arithmetic number from a given floating-point value
        /*!
         * Note: The stepSize is set to the given value and the integer value to 1.
         * This way the constructor may serve as an implicit constructor from a double
         * as well as a stepSize initializer. If value is zero, the integer value is set to zero
         * and the stepSize to 1.
         *
         * \param value The floating-point value the number should be initialized with.
         */
        FixedStepArithmeticNumber(const double& value)
        {
            if (value == 0.0) {
                m_integerValue = 0;
                m_stepSize = 1.0;
                return;
            }

            m_integerValue = 1;
            m_stepSize = value;
        }

        //! Construct a fixed-step arithmetic number with specified step size from a given floating-point value
        /*!
         * \param stepSize The step size used for this number representation
         * \param value The floating-point value the number should be initialized with.
         */
        explicit FixedStepArithmeticNumber(const double& value, const double& stepSize)
        {
            assert(stepSize != 0);
            m_stepSize = stepSize;
            fromDouble(value);
        }

        //! Construct a fixed-step arithmetic number from an initial integer value \f$i\f$ and a stepSize \f$s\f$i.
        /*!
         * \param value Integer part of the number representation
         * \param stepSize Step size for this number representation
         */
        explicit FixedStepArithmeticNumber(const T& value, const double& stepSize)
            : m_integerValue(value)
            , m_stepSize(stepSize)
        {
            assert(stepSize != 0);
        }

        //! Copy-construct from a FixedStepArithmeticNumber with different data type
        /*!
         * \tparam U Integer type of other
         * \param other The FixedStepArithmeticNumber instance to copy
         */
        template <typename U>
        FixedStepArithmeticNumber(const FixedStepArithmeticNumber<U>& other)
        {
            assert(other.m_integerValue < std::numeric_limits<T>::max());
            m_stepSize = other.m_stepSize;
            m_integerValue = (T)other.m_integerValue;
        }

        //! Copy-construct from a FixedStepArithmeticNumber with equal data type
        /*!
         * \param other The FixedStepArithmeticNumber instance to copy
         */
        FixedStepArithmeticNumber(const FixedStepArithmeticNumber<T>& other)
        {
            m_stepSize = other.m_stepSize;
            m_integerValue = other.m_integerValue;
        }

        //! Returns the value \f$n\f$ of the number as a double.
        /*!
         * \attention Precision may be lost.
         * \return Current value of the number as double
         */
        double toDouble() const
        {
            return m_stepSize * m_integerValue;
        }

        //! Returns the value \f$n\f$ of the number as a float.
        /*!
         * \attention Precision may be lost.
         * \return Current value of the number as float
         */
        double toFloat() const
        {
            return (float)toDouble();
        }

        //! Returns the stored integer value \f$i\f$
        const T& integer() const
        {
            return m_integerValue;
        }

        //! Returns the used step size \f$s\f$
        const double& stepSize() const
        {
            return m_stepSize;
        }

        //! Explicit conversion to T
        explicit operator T const()
        {
            return m_integerValue;
        }

        //! Explicit conversion to double
        explicit operator double const()
        {
            return toDouble();
        }

        //! Explicit conversion to float
        explicit operator float const()
        {
            return toFloat();
        }

        //! Update the number representation to match the given double.
        /*!
         * \note This uses core::getThresholdFor() to calculate an acceptable boundary for
         * error caused by the floating-point representation.
         *
         * \attention Precision may be lost.
         * \param value Floating-point value to initialize the number from.
         * \return The residual which could not be represented with the current m_stepSize.
         */
        double fromDouble(const double& value)
        {
            assert(std::abs(value) < m_stepSize * std::numeric_limits<T>::max());
            m_integerValue = (T)((core::comparisonThresholdFor(value) + value) / m_stepSize);
            return (value - m_stepSize * m_integerValue);
        }

        //! Returns a copy representing the next discrete value (+ m_stepSize)
        FixedStepArithmeticNumber<T> next() const
        {
            FixedStepArithmeticNumber<T> result(*this);
            return ++result;
        }

        //! Returns a copy representing the previous discrete value (- m_stepSize)
        FixedStepArithmeticNumber<T> previous() const
        {
            FixedStepArithmeticNumber<T> result(*this);
            return --result;
        }

        //! Prefix increment operator
        FixedStepArithmeticNumber<T>& operator++()
        {
            m_integerValue++;
            return *this;
        }

        //! Prefix decrement operator
        FixedStepArithmeticNumber<T>& operator--()
        {
            m_integerValue--;
            return *this;
        }

        //! Postfix increment operator
        FixedStepArithmeticNumber<T> operator++(int)
        {
            FixedStepArithmeticNumber<T> result = *this;
            m_integerValue++;
            return result;
        }

        //! Postfix decrement operator
        FixedStepArithmeticNumber<T> operator--(int)
        {
            FixedStepArithmeticNumber<T> result = *this;
            m_integerValue--;
            return result;
        }

        template <typename U>
        bool operator<(const FixedStepArithmeticNumber<U>& rhs)
        {
            return isLess(this->toDouble(), rhs.toDouble());
        }

        template <typename U>
        bool operator<=(const FixedStepArithmeticNumber<U>& rhs)
        {
            return isLessOrEqual(this->toDouble(), rhs.toDouble());
        }

        template <typename U>
        bool operator>(const FixedStepArithmeticNumber<U>& rhs)
        {
            return isGreater(this->toDouble(), rhs.toDouble());
        }

        template <typename U>
        bool operator>=(const FixedStepArithmeticNumber<U>& rhs)
        {
            return isGreaterOrEqual(this->toDouble(), rhs.toDouble());
        }

        template <typename U>
        bool operator==(const FixedStepArithmeticNumber<U>& rhs)
        {
            return isEqual(this->toDouble(), rhs.toDouble());
        }

        template <typename U>
        bool operator!=(const FixedStepArithmeticNumber<U>& rhs)
        {
            return !isEqual(this->toDouble(), rhs.toDouble());
        }

        bool operator<(const FixedStepArithmeticNumber<T>& rhs)
        {
            return isLess(this->toDouble(), rhs.toDouble());
        }

        bool operator<=(const FixedStepArithmeticNumber<T>& rhs)
        {
            return isLessOrEqual(this->toDouble(), rhs.toDouble());
        }

        bool operator>(const FixedStepArithmeticNumber<T>& rhs)
        {
            return isGreater(this->toDouble(), rhs.toDouble());
        }

        bool operator>=(const FixedStepArithmeticNumber<T>& rhs)
        {
            return isGreaterOrEqual(this->toDouble(), rhs.toDouble());
        }

        bool operator==(const FixedStepArithmeticNumber<T>& rhs)
        {
            return isEqual(this->toDouble(), rhs.toDouble());
        }

        bool operator!=(const FixedStepArithmeticNumber<T>& rhs)
        {
            return !isEqual(this->toDouble(), rhs.toDouble());
        }

    protected:
        //! The integer storing the integer part \f$i\f$
        T m_integerValue;

        //! The floating-point step size \f$s\f$ for this variable
        double m_stepSize;
    };
} // namespace core
} // namespace broccoli
