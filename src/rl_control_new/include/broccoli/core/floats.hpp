/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "broccoli/core/type_traits.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>

namespace broccoli {
namespace core {
    namespace internal {

        //! Template for a union to hold an instance of type T and a signed integer of given size.
        template <typename T, size_t size>
        union IntegerUnion {
            // This generalization should NOT be used.
            T value;
        };

        //! Specialization of IntegerUnion for 4 byte variables
        template <typename T>
        union IntegerUnion<T, 4> {
            IntegerUnion(T value)
                : value(value)
            {
            }
            T value;
            int32_t asInteger;
        };

        //! Specialization of IntegerUnion for 8 byte variables
        template <typename T>
        union IntegerUnion<T, 8> {
            IntegerUnion(T value)
                : value(value)
            {
            }
            T value;
            int64_t asInteger;
        };

        //! Template type for a union holding a signed integer of same size as T
        template <typename T>
        using IntegerUnionMatchingType = IntegerUnion<T, sizeof(T)>;
    }

    /*!
    * \addtogroup broccoli_core_floats
    * \{
    */

    //! Return a safe threshold for the comparison of floating-point types
    /*!
     *
     * \tparam T A floating-point type
     * \param value The floating-point value for which a safe comparison threshold shall be generated.
     * \param maxULPs Maximum number of units in the last place considered for threshold computation.
     * \param maxEpsilonDiff Multiples of the machine precision considered for threshold computation.
     * \return The threshold for floating-point comparison based on the maximum of both ULPs and absolute epsilon based approaches.
     */
    template <typename T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, double>::type comparisonThresholdFor(const T& value, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        internal::IntegerUnionMatchingType<T> acceptable(value);
        acceptable.asInteger += maxULPs;

        return std::max<T>(std::abs(acceptable.value - value), std::numeric_limits<T>::epsilon() * maxEpsilonDiff);
    }

    //! Safe equality comparison for floating-point types
    /*!
     * Compares two floating point numbers based on an absolute threshold and a maximum difference in units in the last place (ULP).
     * The absolute threshold is defined as a multiple of the machine precision std::numeric_limits<T>::epsilon().
     *
     * Learn why using just epsilon is in general a bad idea:
     * https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
     *
     * \tparam T A floating-point type
     * \param a Value a
     * \param b Value b
     * \param maxULPs Maximum number of units in the last place the two numbers may differ to be treated as equal.
     * \param maxEpsilonDiff Multiples of the machine precision two numbers may differ to be treated as equal.
     * \return True if the floating-point numbers are almost equal
     */
    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type isEqual(const T& a, const T& b, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        // NaN comparisons should always return false
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }

        // Near zero the absolute integer representations can be quite high in terms of ULPs
        // Therefore we use an additional epsilon-based threshold.
        if (std::abs(a - b) <= std::numeric_limits<T>::epsilon() * maxEpsilonDiff) {
            return true;
        }

        internal::IntegerUnionMatchingType<T> uA(a);
        internal::IntegerUnionMatchingType<T> uB(b);

        if (uA.asInteger < 0 && uB.asInteger >= 0) {

            if (a == b) {
                return true; // In the case -0 == +0
            }

            return false;
        }

        // The difference in ULPs is the absolute of the difference in integer
        // representation
        size_t ULPsBetweenNumbers = std::abs(uA.asInteger - uB.asInteger);

        if (ULPsBetweenNumbers <= maxULPs)
            return true;

        return false;
    }

    /*!
     * \brief Safe comparison to zero (floating-point types)
     *
     * This is a specialization of isEqual() for comparison to zero.
     * \tparam T A floating-point type
     * \param a Value a
     * \param maxEpsilonDiff Multiples of the machine precision the number may be away from zero.
     * \return True if the floating-point number a is (almost) zero
     */
    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type isZero(const T& a, size_t maxEpsilonDiff = 2)
    {
        // NaN comparisons should always return false
        if (std::isnan(a)) {
            return false;
        }

        // Near zero the use of an epsilon-based threshold is sufficient
        return std::abs(a) <= std::numeric_limits<T>::epsilon() * maxEpsilonDiff;
    }

    //! Safe equality comparison for Eigen types
    /*!
     * Compares two Eigen matrices / vector's elements based on an absolute threshold and a maximum difference in units in the last place (ULP).
     * The absolute threshold is defined as a multiple of the machine precision std::numeric_limits<T>::epsilon().
     *
     * Learn why using just epsilon is in general a bad idea:
     * https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
     *
     * \tparam T An Eigen matrix type
     * \param a Value a
     * \param b Value b
     * \param maxULPs Maximum number of units in the last place two numbers may differ to be treated as equal.
     * \param maxEpsilonDiff Multiples of the machine precision two numbers may differ to be treated as equal.
     * \return True if the eigen matrix's elements are almost equal
     */
    template <typename T>
    typename std::enable_if<is_eigen_matrix<T>::value, bool>::type isEqual(const T& a, const T& b, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        assert(a.size() == b.size() && "Matrices have the same dimensions!");

        for (long i = 0; i < a.size(); i++) {

            if (!isEqual(a(i), b(i), maxULPs, maxEpsilonDiff)) {
                return false;
            }
        }

        return true;
    }

    //! Safe less-than comparison for floating-point types
    /*!
     * Compares two floating point numbers based on an absolute threshold and a maximum difference in units in the last place (ULP).
     * The absolute threshold is defined as a multiple of the machine precision std::numeric_limits<T>::epsilon().
     *
     * \tparam T A floating-point type
     * \param a Value a
     * \param b Value b
     * \param maxULPs Maximum number of units in the last place the two numbers may differ to be treated as equal.
     * \param maxEpsilonDiff Multiples of the machine precision two numbers may differ to be treated as equal.
     * \return True if a < b where the difference a-b fulfills the specified thresholds.
     */
    template <typename T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type isLess(const T& a, const T& b, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        // NaN comparisons should always return false
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }

        if (isEqual(a, b, maxULPs, maxEpsilonDiff))
            return false;

        return a < b;
    }

    //! Safe less or equal comparison for floating-point types
    /*!
     * Compares two floating point numbers based on an absolute threshold and a maximum difference in units in the last place (ULP).
     * The absolute threshold is defined as a multiple of the machine precision std::numeric_limits<T>::epsilon().
     *
     * \tparam T A floating-point type
     * \param a Value a
     * \param b Value b
     * \param maxULPs Maximum number of units in the last place the two numbers may differ to be treated as equal.
     * \param maxEpsilonDiff Multiples of the machine precision two numbers may differ to be treated as equal.
     * \return True if a <= b where b may be greater than a by the specified thresholds.
     */
    template <typename T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type isLessOrEqual(const T& a, const T& b, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        // NaN comparisons should always return false
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }

        if (isEqual(a, b, maxULPs, maxEpsilonDiff))
            return true;

        return a < b;
    }

    //! Safe greater-than comparison for floating-point types
    /*!
     * Compares two floating point numbers based on an absolute threshold and a maximum difference in units in the last place (ULP).
     * The absolute threshold is defined as a multiple of the machine precision std::numeric_limits<T>::epsilon().
     *
     * \tparam T A floating-point type
     * \param a Value a
     * \param b Value b
     * \param maxULPs Maximum number of units in the last place the two numbers may differ to be treated as equal.
     * \param maxEpsilonDiff Multiples of the machine precision two numbers may differ to be treated as equal.
     * \return True if a > b while the difference a-b fulfills the specified thresholds.
     */
    template <typename T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type isGreater(const T& a, const T& b, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        // NaN comparisons should always return false
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }

        if (isEqual(a, b, maxULPs, maxEpsilonDiff))
            return false;

        return a > b;
    }

    //! Safe greater or equal comparison for floating-point types
    /*!
     * Compares two floating point numbers based on an absolute threshold and a maximum difference in units in the last place (ULP).
     * The absolute threshold is defined as a multiple of the machine precision std::numeric_limits<T>::epsilon().
     *
     * \tparam T A floating-point type
     * \param a Value a
     * \param b Value b
     * \param maxULPs Maximum number of units in the last place the two numbers may differ to be treated as equal.
     * \param maxEpsilonDiff Multiples of the machine precision two numbers may differ to be treated as equal.
     * \return True if a >= b where b may be less than a by the specified thresholds.
     */
    template <typename T>
    typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type isGreaterOrEqual(const T& a, const T& b, size_t maxULPs = 4, size_t maxEpsilonDiff = 2)
    {
        // NaN comparisons should always return false
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }

        if (isEqual(a, b, maxULPs, maxEpsilonDiff))
            return true;

        return a > b;
    }
    //! \}
} // namespace core
} // namespace broccoli
