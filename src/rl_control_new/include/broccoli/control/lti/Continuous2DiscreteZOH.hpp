/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#ifdef HAVE_EIGEN3

#include "ContinuousStateSpaceParameters.hpp"
#include "DiscreteStateSpaceParameters.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace broccoli {
namespace control {

    /*!
     * \brief Implements a Zero-order hold continuous to discrete state space converter
     * \ingroup broccoli_control_lti
     *
     * An instance of this class represents a time-discrete version of the continuous
     * state space model parameters passed to the constructor. It uses a zero-order hold on
     * the system's inputs with a specified sample time.
     *
     * \tparam N The number of states
     * \tparam M The number of system inputs
     * \tparam P The number of system outputs
     * \tparam Scalar The type to represent a scalar
     */
    template <int N = Eigen::Dynamic, int M = Eigen::Dynamic, int P = Eigen::Dynamic, typename Scalar = double>
    class Continuous2DiscreteZOH : public DiscreteStateSpaceParameters<N, M, P, Scalar> {
    public:
        /*!
         * \brief Construct converted system from continuous-time state space parameters and sample time
         *
         * \warning The system matrix A of the continuous state space system must be invertible!
         * \param continuous Continuous-time state space parameters
         * \param sampleTime The sample time in seconds
         */
        Continuous2DiscreteZOH(const ContinuousStateSpaceParameters<N, M, P, Scalar>& continuous, double sampleTime)
        {
            this->A = (continuous.A * sampleTime).exp();
            this->C = continuous.C;
            this->D = continuous.D;
            this->sampleTime = sampleTime;

            // A * B_d = (A_d - I) * B
            Eigen::Matrix<Scalar, N, M> rightHandSide = (this->A - DiscreteStateSpaceParameters<N, M, P, Scalar>::AType::Identity(this->A.rows(), this->A.cols())) * continuous.B;
            this->B = continuous.A.colPivHouseholderQr().solve(rightHandSide);
            assert((continuous.A * this->B - rightHandSide).norm() / rightHandSide.norm() < 0.01 && "Relative solution error is large! System matrix must be invertible");
        }
    };

    /*!
     * \brief Calculates time-discrete parameters from continuous-time parameters using Zero-order hold
     * \ingroup broccoli_control_lti
     * \tparam N The number of states
     * \tparam M The number of system inputs
     * \tparam P The number of system outputs
     * \tparam Scalar The type to represent a scalar
     * \param continuous Continuous-time input state space parameters
     * \param sampleTime The sample time in seconds
     * \return Time-discrete state space parameters
     */
    template <int N, int M, int P, typename Scalar>
    auto continuous2DiscreteZOH(const ContinuousStateSpaceParameters<N, M, P, Scalar>& continuous, double sampleTime)
    {
        return Continuous2DiscreteZOH<N, M, P, Scalar>(continuous, sampleTime);
    }

} // namespace control
} // namespace broccoli
#endif
