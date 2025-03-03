/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/math.hpp"
#include "../SignalLimits.hpp"
#include "ConstrainedIntegrator.hpp"
#include "Saturation.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace control {
    /*!
     * \brief Abstract base class for a ConstrainedIntegrator with saturated output.
     * \ingroup broccoli_control_operators
     *
     * The integrator signal output is saturated based on the values in a specified SignalLimits object.
     * Uses clamping-based anti windup, i.e. integration is stopped when driven into the limits.
     * \tparam T The integrand type
     */
    template <typename T>
    class BaseSaturatedIntegrator : public ConstrainedIntegrator<T> {
    public:
        /*!
         * \brief A constructor
         *
         * Uses zero as initial value
         * \param limitContainer Container, which defines the limitations of the output signal
         * \param unconstrainedInitialValue Initial value for the unconstrained directions variable
         */
        BaseSaturatedIntegrator(const SignalLimits<T>& limitContainer, const T& unconstrainedInitialValue)
            : ConstrainedIntegrator<T>()
            , m_limits(limitContainer)
            , m_isUnconstrained(unconstrainedInitialValue)
        {
        }

        /*!
         * \brief A constructor
         * \param initialValue The initial output value
         * \param limitContainer Container, which defines the limitations of the output signal
         * \param unconstrainedInitialValue Initial value for the unconstrained directions variable
         */
        BaseSaturatedIntegrator(const std::decay_t<T>& initialValue, const SignalLimits<std::decay_t<T>>& limitContainer, const std::decay_t<T>& unconstrainedInitialValue)
            : ConstrainedIntegrator<T>(initialValue)
            , m_limits(limitContainer)
            , m_isUnconstrained(unconstrainedInitialValue)
        {
        }

        /*!
         * \brief A move constructor
         *
         * This can be used to integrate on an existing storage.
         * \param value The value to move
         * \param limitContainer Container, which defines the limitations of the output signal
         * \param unconstrainedInitialValue Initial value for the unconstrained directions variable
         */
        BaseSaturatedIntegrator(T&& value, const SignalLimits<std::decay_t<T>>& limitContainer, const std::decay_t<T>& unconstrainedInitialValue)
            : ConstrainedIntegrator<T>(std::forward<T>(value))
            , m_limits(limitContainer)
            , m_isUnconstrained(unconstrainedInitialValue)
        {
        }

        /*!
         * \brief Returns the unconstrained binary selection vector
         *
         * For elements of the output signal, which are currently not saturated (limited), this contains a 1.
         * For constrained directions, this contains a 0.
         */
        const std::decay_t<T>& binaryVectorUnconstrainedOutput() const
        {
            return m_isUnconstrained;
        }

    protected:
        /*!
         * \brief Clamps the given output value and sets unconstrained accordingly
         * \tparam U The type of the integrand
         * \param [in] integrand The integrand
         * \param [in,out] value The current output value
         * \param [in,out] unconstrained The unconstrained directions data type to modify
         * \param [in] minValue The minimum for value
         * \param [in] maxValue The maximum for value
         */
        template <typename U>
        static void clampOutputValue(const U& integrand, U& value, U& unconstrained, const U& minValue, const U& maxValue)
        {
            if (value >= maxValue && integrand > 0) {
                unconstrained = 0;
                value = maxValue;

            } else if (value <= minValue && integrand < 0) {
                unconstrained = 0;
                value = minValue;

            } else {
                unconstrained = 1;
            }
        }

        //! The limitations for the output signal value
        SignalLimits<std::decay_t<T>> m_limits;

        //! Is the integrator currently unconstrained (running)
        std::decay_t<T> m_isUnconstrained;
    };

    /*!
     * \brief A ConstrainedIntegrator with saturated output for non-Eigen types
     * \tparam T A non-Eigen arithmetic integrand type
     * \tparam isEigenVector Internal use only - defaults to true if T is an Eigen type
     * \ingroup broccoli_control_operators
     */
    template <typename T, bool isEigenVector = core::is_eigen_matrix<T>::value>
    class SaturatedIntegrator : public BaseSaturatedIntegrator<T> {
    public:
        /*!
         * \brief A constructor. Uses zero as initial output value.
         * \param limitContainer Container, which defines the limitations of the output signal
         */
        explicit SaturatedIntegrator(const SignalLimits<T>& limitContainer)
            : BaseSaturatedIntegrator<T>(limitContainer, 1)
        {
        }

        /*!
         * \brief A constructor for complex data types
         * \param initialValue The initial output value
         * \param limitContainer Container, which defines the limitations of the output signal
         */
        SaturatedIntegrator(const std::decay_t<T>& initialValue, const SignalLimits<std::decay_t<T>>& limitContainer)
            : BaseSaturatedIntegrator<T>(initialValue, limitContainer, 1)
        {
        }

        /*!
         * \brief A move constructor
         *
         * This can be used to integrate on an existing storage.
         * \param value The value to move
         * \param limitContainer Container, which defines the limitations of the output signal
         */
        SaturatedIntegrator(T&& value, const SignalLimits<std::decay_t<T>>& limitContainer)
            : BaseSaturatedIntegrator<T>(std::forward<T>(value), limitContainer, 1)
        {
        }

        /*!
         * \brief Computes the saturated integral of the given integrand
         *
         * The integral's output value is saturated based on a given SignalLimits object.
         * \tparam U The integrand data type
         * \param integrand The integrand
         * \return The integrator's output value
         */
        auto process(const Signal<std::decay_t<T>>& integrand)
        {
            ConstrainedIntegrator<T>::process(integrand, mapSignal(m_isUnconstrained));
            BaseSaturatedIntegrator<T>::clampOutputValue(integrand.value(), this->value(), m_isUnconstrained, m_limits.min(), m_limits.max());

            return *this;
        }

    protected:
        using BaseSaturatedIntegrator<T>::m_limits;
        using BaseSaturatedIntegrator<T>::m_isUnconstrained;
    };

#ifdef HAVE_EIGEN3
    /*!
     * \brief A ConstrainedIntegrator with saturated output for Eigen types
     * \tparam T An Eigen (matrix / vector) arithmetic integrand type
     * \ingroup broccoli_control_operators
     */
    template <typename T>
    class SaturatedIntegrator<T, true> : public BaseSaturatedIntegrator<T> {
    public:
        /*!
         * \brief A constructor. Uses zero as initial output value.
         * \param limitContainer Container, which defines the limitations of the output signal
         */
        explicit SaturatedIntegrator(const SignalLimits<T>& limitContainer)
            : BaseSaturatedIntegrator<T>(limitContainer, T::Ones())
        {
        }

        /*!
         * \brief A constructor
         * \param initialValue The initial output value
         * \param limitContainer Container, which defines the limitations of the output signal
         */
        SaturatedIntegrator(const T& initialValue, const SignalLimits<T>& limitContainer)
            : BaseSaturatedIntegrator<T>(initialValue, limitContainer, T::Ones())
        {
        }

        /*!
         * \brief A move constructor
         *
         * This can be used to integrate on an existing storage.
         * \param value The value to move
         * \param limitContainer Container, which defines the limitations of the output signal
         */
        SaturatedIntegrator(T&& value, const SignalLimits<std::decay_t<T>>& limitContainer)
            : BaseSaturatedIntegrator<T>(std::forward<T>(value), limitContainer, T::Ones())
        {
        }

        /*!
         * \brief Computes the saturated integral of the given integrand
         *
         * The integral's output value is saturated based on a given SignalLimits object.
         * \param integrand The integrand
         * \return The integrator's output value
         */
        auto process(const Signal<std::decay_t<T>>& integrand)
        {
            ConstrainedIntegrator<T>::process(integrand, mapSignal(m_isUnconstrained));

            for (int i = 0; i < this->value().size(); i++) {
                BaseSaturatedIntegrator<T>::clampOutputValue(integrand.value()(i), this->value()(i), m_isUnconstrained(i), m_limits.min()(i), m_limits.max()(i));
            }

            return *this;
        }

    protected:
        using BaseSaturatedIntegrator<T>::m_limits;
        using BaseSaturatedIntegrator<T>::m_isUnconstrained;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
#endif

} // namespace control
} // namespace broccoli
