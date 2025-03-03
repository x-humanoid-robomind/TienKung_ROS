/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../core/type_traits.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace control {
    /*!
     * \brief Aggregates value limits for a signal of data type T
     * \ingroup broccoli_control
     * \tparam T The signal data type
     */
    template <typename T>
    class SignalLimits {
    public:
        //! Default constructor. Init with zeros.
        SignalLimits()
            : m_minValue(core::Traits<T>::zero())
            , m_maxValue(core::Traits<T>::zero())
        {
        }

        /*!
         * \brief Copy Constructor
         * \param minValue The min value for the signal
         * \param maxValue The max value for the signal
         */
        SignalLimits(const T& minValue, const T& maxValue)
            : m_minValue(minValue)
            , m_maxValue(maxValue)
        {
        }

        /*!
         * \brief Move constructor
         * \param minValue The min value for the signal
         * \param maxValue The max value for the signal
         */
        SignalLimits(T&& minValue, T&& maxValue)
            : m_minValue(std::move(minValue))
            , m_maxValue(std::move(maxValue))
        {
        }

        //! Returns a const reference to the maximum value of the signal
        const T& max() const
        {
            return m_maxValue;
        }

        //! Returns a reference to the maximum value of the signal
        T& max()
        {
            return m_maxValue;
        }

        //! Returns a const reference to the minimum value of the signal
        const T& min() const
        {
            return m_minValue;
        }

        //! Returns a reference to the minimum value of the signal
        T& min()
        {
            return m_minValue;
        }

    private:
        //! The minimum value
        T m_minValue;

        //! The maximum value
        T m_maxValue;

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
    };

    /*!
     * \brief Create a SignalLimits<T> instance from min, max values
     * \ingroup broccoli_control
     * \tparam T The signal data type
     * \param min The min value
     * \param max The max value
     * \return A SignalLimits instance storing min and max values
     */
    template <typename T>
    auto makeSignalLimits(T&& min, T&& max)
    {
        return SignalLimits<T>(std::forward<T>(min), std::forward<T>(max));
    }
} // namespace control
} // namespace broccoli
