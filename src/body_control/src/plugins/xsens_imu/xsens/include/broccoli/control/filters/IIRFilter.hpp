/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/floats.hpp"
#include "../../core/type_traits.hpp"
#include "../../memory/CircularAccessor.hpp"
#include "SignalFilter.hpp"
#include <array>
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif
#include <iostream>

namespace broccoli {
namespace control {

    /*!
     * \brief Time-Discrete Infinite Impulse Response Filter
     *
     * This digital IIR filter implements the following discrete-time transfer function:
     * \f[
     * H(z) = \frac{\beta_0 + \beta_1 z^{-1} + \beta_2 z^{-2} + ... + \beta_M z^{-M}}{ \alpha_0 + \alpha_1 z^{-1} + \alpha_2 z^{-2} + ... + \alpha_N z^{-N}},
     * \f]
     * where \f$\beta_i, i = 0 ... M\f$ are the numerator coefficients and \f$\alpha_j, j = 0 ...N\f$ are the denominator coefficients. The filter must be a causal recursive system with
     * \f$M \ge N\f$. The filter uses additional information on the sample time to provide derivatives of the filter output signal.
     *
     * \ingroup broccoli_control_filters
     * \tparam T The filter signal type
     * \tparam numeratorOrder The order of discrete-time transfer function numerator (\f$M-1\f$). Must be greater than zero.
     * \tparam denominatorOrder The order of the discrete-time transfer function denominator (\f$N-1\f$). Must be greater than zero.
     * \tparam Scalar The coefficient data type
     */
    template <typename T, std::size_t numeratorOrder, std::size_t denominatorOrder, typename Scalar = double>
    class IIRFilter : public SignalFilter<IIRFilter<T, numeratorOrder, denominatorOrder>> {
    public:
        //! The number of numerator coefficients \f$M\f$
        static constexpr std::size_t NrOfNumeratorCoefficients = numeratorOrder + 1;

        //! The number of denominator coefficients \f$N\f$
        static constexpr std::size_t NrOfDenominatorCoefficients = denominatorOrder + 1;

        static_assert(denominatorOrder >= numeratorOrder, "Filter orders must match a causal recursive system.");
        static_assert(denominatorOrder > 0, "Denominator order must be at least 1");
        static_assert(numeratorOrder > 0, "Numerator order must be at least 1");

        /*!
         * \brief Default constructor
         *
         * Initializes state with zero.
         */
        IIRFilter()
        {
            reset(core::Traits<T>::zero());
            setupForPassThru();
        }

        /*!
         * \brief Construct with given state
         * \param state The initial filter state
         */
        explicit IIRFilter(const T& state)
        {
            reset(state);
            setupForPassThru();
        }

        /*!
         * \brief Construct from state and coefficients
         * \param state The initial filter state
         * \param numeratorCoefficients The numerator coefficients
         * \param denominatorCoefficients The denominator coefficients
         * \param sampleTime The sample time in seconds
         */
        IIRFilter(const T& state, const std::initializer_list<Scalar>& numeratorCoefficients, const std::initializer_list<Scalar>& denominatorCoefficients, const double& sampleTime)
            : IIRFilter(state)
        {
            init(numeratorCoefficients, denominatorCoefficients, sampleTime);
        }

        /*!
         * \brief (Re-) initialize the filter
         * \param numeratorCoefficients The numerator coefficients
         * \param denominatorCoefficients The denominator coefficients
         * \param sampleTime The sample time in seconds
         * \returns A reference to this instance
         */
        auto& init(const std::initializer_list<Scalar>& numeratorCoefficients, const std::initializer_list<Scalar>& denominatorCoefficients, const double& sampleTime)
        {
            assert(numeratorCoefficients.size() == NrOfNumeratorCoefficients && "Number of supplied numerator coefficients must match filter order!");
            assert(denominatorCoefficients.size() == NrOfDenominatorCoefficients && "Number of supplied denominator coefficients must match filter order!");

            m_sampleTime = sampleTime;
            std::copy(numeratorCoefficients.begin(), numeratorCoefficients.end(), m_numeratorCoefficients.begin());
            std::copy(denominatorCoefficients.begin(), denominatorCoefficients.end(), m_denominatorCoefficients.begin());

            if (core::isZero(m_denominatorCoefficients[0])) {
                assert(false && "First denominator coefficient may not be zero or close to zero!");
                m_denominatorCoefficients[0] = 1e-06;
            }
            return *this;
        }

        /*!
         * \brief Setup the filter parameters for pass-thru of the input signal
         * \returns A reference to this instance
         */
        auto& setupForPassThru()
        {
            m_numeratorCoefficients.fill(0.0);
            m_denominatorCoefficients.fill(0.0);

            m_numeratorCoefficients[0] = 1.0;
            m_denominatorCoefficients[0] = 1.0;

            return *this;
        }

        /*!
         * \brief Resets the filter state and output to the given value
         * \param value The desired value for the filter
         * \returns A reference to this instance
         */
        auto& reset(const T& value)
        {
            setState(value, value, value);
            return *this;
        }

        /*!
         * \brief Reset filter value to given value and signal rate
         * \param value The desired filter output signal value
         * \param rate The desired filter output signal rate
         * \returns A reference to this instance
         */
        auto& reset(const T& value, const T& rate)
        {
            setState(value, value, value - rate * sampleTime());
            return *this;
        }

        /*!
         * \brief Computes and returns the filter output signal
         * \param input The input signal for the current time step
         * \returns A reference to this instance (the filter output signal)
         */
        auto& process(const Signal<T>& input)
        {
            assert((input.sampleTime() < 0 || core::isEqual(input.sampleTime(), m_sampleTime)) && "Sample time of filter parameters must match sample time of signal");

            auto stateHistory = m_stateCirculator.access(m_stateHistory);
            auto inputHistory = m_inputCirculator.access(m_inputHistory);

            // Store previous state in history
            stateHistory.pushFront(m_value);

            // There is at least one numerator coefficient
            m_value = m_numeratorCoefficients[0] * input.value();

            for (std::size_t i = 1; i < NrOfNumeratorCoefficients; i++) {
                m_value = m_value + m_numeratorCoefficients[i] * inputHistory[i - 1];
            }
            for (std::size_t i = 1; i < NrOfDenominatorCoefficients; i++) {
                m_value = m_value - m_denominatorCoefficients[i] * stateHistory[i - 1];
            }

            m_value = m_value / m_denominatorCoefficients[0];

            inputHistory.pushFront(input.value());
            return *this;
        }

        //! Returns the value of the filter output
        const T& value() const
        {
            return m_value;
        }

        //! Returns the sample time of the signal in seconds
        const double& sampleTime() const
        {
            return m_sampleTime;
        }

        /*!
         * \brief Returns the nth derivative of the filter output as signal
         *
         * The derivative at time \f$t = t_{now} - k \cdot \f$ sampleTime() is returned, with
         * k defaulting to 0. The zero-order derivative (n = 0) is defined as the filter output at
         * time \f$t\f$.
         *
         * \note The implementation can not calculate non-causal derivatives of the filter output,
         * i.e. \f$n + k \le \f$ denominatorOrder must hold and is asserted at compile time.
         *
         * \tparam n The order of the derivative to compute, n >= 0
         * \tparam k Optional argument specifying the time steps in the past to calculate the derivative at, k >= 0
         * \return The n th derivative of the filter output as signal
         */
        template <std::size_t n = 1, std::size_t k = 0, typename std::enable_if_t<(n > 0), int> = 0>
        auto derivative()
        {
            assert(m_sampleTime > 0.0 && "Filter derivatives require a valid sample time / initialized filter");
            static_assert(denominatorOrder >= (n + k), "Unable to calculate non-causal derivatives with higher order than filter denominator order!");

            return (derivative<n - 1, k>() - derivative<n - 1, k + 1>()) * (1.0 / m_sampleTime);
        }

        /*!
         * \private
         * \brief special case for zero-order derivative (value) at k
         */
        template <std::size_t n, std::size_t k = 0, typename std::enable_if_t<(n == 0), int> = 0>
        auto derivative()
        {
            static_assert(denominatorOrder >= k, "Unable to calculate non-causal derivatives with higher order than filter denominator order!");

            if (k == 0)
                return mapSignal(m_value, m_sampleTime);

            return mapSignal(m_stateCirculator.access(m_stateHistory)[k - 1], m_sampleTime);
        }

    protected:
        /*!
         * \brief Set the current filter output state and input/output history
         * \param currentValue The current state
         * \param inputHistory The input history state to use
         * \param outputHistory The output history state to use
         */
        void setState(const T& currentValue, const T& inputHistory, const T& outputHistory)
        {
            m_value = currentValue;
            m_inputHistory.fill(inputHistory);
            m_stateHistory.fill(outputHistory);
        }

    private:
        //! The current output value
        T m_value;

        //! The output sample time
        double m_sampleTime = -1.0;

        //! The numerator coefficients
        std::array<Scalar, NrOfNumeratorCoefficients> m_numeratorCoefficients;

        //! The denominator coefficients
        std::array<Scalar, NrOfDenominatorCoefficients> m_denominatorCoefficients;

        //! Storage for input history
        std::array<T, numeratorOrder> m_inputHistory;

        //! Storage for state history
        std::array<T, denominatorOrder> m_stateHistory;

        //! Accessor for historical input data
        memory::CircularAccessor m_inputCirculator{ 0, numeratorOrder };

        //! Accessor for historical state data
        memory::CircularAccessor m_stateCirculator{ 0, denominatorOrder };

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
    };

} // namespace control
} // namespace broccoli
