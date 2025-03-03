/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#ifdef HAVE_EIGEN3

#include "../../core/floats.hpp"
#include "../Signal.hpp"
#include "DiscreteStateSpaceParameters.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace control {

    /*!
     * \brief Discrete-time state space model
     * \ingroup broccoli_control_lti
     *
     * Describes and simulates a discrete-time state space model of the form
     * \f[
     * x[n+1] = A \, x[n] + B \, u[n] \\
     * y[n] = C \, x[n] + D \, u[n],
     * \f]
     * where \f$x\f$, \f$y\f$, and \f$u\f$ represent the system states, outputs, and inputs respectively and
     * \f$ n \f$ represents the current time step. Given the number of system states N, number of inputs M, and number of outputs P,
     * the system matrices have the following dimensionality:
     * \f[
     * A \in \mathbb{R}^{N \times N}\\
     * B \in \mathbb{R}^{N \times M}\\
     * C \in \mathbb{R}^{P \times N}\\
     * D \in \mathbb{R}^{P \times M}.
     * \f]
     * A StateSpaceModel is a Signal and its value() represents the output of the system at the current time step.
     *
     * \tparam N The number of states
     * \tparam M The number of system inputs
     * \tparam P The number of system outputs
     * \tparam Scalar The type to represent a scalar
     */
    template <int N = Eigen::Dynamic, int M = Eigen::Dynamic, int P = Eigen::Dynamic, typename Scalar = double>
    class StateSpaceModel : public Signal<Eigen::Matrix<Scalar, P, 1>> {
    public:
        using StateType = Eigen::Matrix<Scalar, N, 1>; //!< System state vector type
        using InputType = Eigen::Matrix<Scalar, M, 1>; //!< System input vector type
        using OutputType = Eigen::Matrix<Scalar, P, 1>; //!< System output vector type
        using ParameterType = DiscreteStateSpaceParameters<N, M, P, Scalar>; //!< System parameter container type

        /*!
         * \brief Constructs with zeroed initial state and output value
         * \param parameters Discrete-time state-space parameters (A,B,C,D and sample time)
         */
        explicit StateSpaceModel(const ParameterType& parameters)
            : Signal<Eigen::Matrix<Scalar, P, 1>>(core::Traits<OutputType>::zero(), parameters.sampleTime)
            , m_parameters(parameters)
            , m_x(core::Traits<StateType>::zero())
        {
            this->sampleTime() = parameters.sampleTime;
        }

        /*!
         * \brief Constructs with given initial system state
         * \param x0 Initial system state vector
         * \param parameters Discrete-time state-space parameters (A,B,C,D and sample time)
         */
        template <typename DerivedX0>
        StateSpaceModel(const Eigen::MatrixBase<DerivedX0>& x0,
            const ParameterType& parameters)
            : StateSpaceModel(parameters)
        {
            m_x.value() = x0;
            outputEquation();
        }

        /*!
         * \brief Constructs with given initial system state and input vector
         * \param x0 Initial system state vector
         * \param u0 Initial input vector
         * \param parameters Discrete-time state-space parameters (A,B,C,D and sample time)
         */
        template <typename DerivedX0, typename DerivedU0>
        StateSpaceModel(const Eigen::MatrixBase<DerivedX0>& x0, const Eigen::MatrixBase<DerivedU0>& u0,
            const ParameterType& parameters)
            : StateSpaceModel(parameters)
        {
            m_x.value() = x0;
            outputEquation(mapSignal(u0));
        }

        /*!
         * \brief Calculates system response for current time step
         * \tparam Derived Signal input derived type
         * \param input Input signal \f$u[n]\f$
         * \return The current output signal \f$y[n]\f$ of the state space model
         */
        template <typename Derived>
        const auto& process(const SignalBase<Derived>& input)
        {
            assert((input.sampleTime() < 0 || core::isEqual(input.sampleTime(), this->sampleTime()))
                && "Sample time of state space model must match sample time of input signal");

            outputEquation(input);
            m_x = m_parameters.A * m_x.value() + m_parameters.B * input.derived();
            return *this;
        }

        //! Returns the current system state \f$x[n]\f$ as Signal
        const auto& state() const
        {
            return m_x;
        }

        /*!
         * \brief Sets the system state\f$x[n]\f$ **without** updating \f$y[n]\f$
         * \tparam Derived Derived Type for state parameter
         * \param x The desired state
         * \return This instance
         */
        template <typename Derived>
        const auto& setState(const Eigen::MatrixBase<Derived>& x)
        {
            m_x.value() = x;
            return *this;
        }

        //! Returns the system matrices A, B, C, D
        const ParameterType& parameters() const { return m_parameters; }

    protected:
        //! Updates the output value based on the current state alone (assumes input is zero)
        void outputEquation()
        {
            this->value() = m_parameters.C * m_x.value();
        }

        /*!
         * \brief Updates the output value based on current state and input
         * \tparam Derived Input derived type
         * \param input Input value \f$u[n]\f$
         */
        template <typename Derived>
        void outputEquation(const SignalBase<Derived>& input)
        {
            this->value() = (m_parameters.C * m_x + m_parameters.D * input.derived()).value();
        }

        //! Parameters
        ParameterType m_parameters;

        //! Current system state
        Signal<StateType> m_x;
    };

} // namespace control
} // namespace broccoli
#endif
