/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#ifdef HAVE_EIGEN3

#include "StateSpaceParametersBase.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace control {

    /*!
     * \brief Discrete-time state space parameters
     * \ingroup broccoli_control_lti
     * \tparam N The number of states
     * \tparam M The number of system inputs
     * \tparam P The number of system outputs
     * \tparam Scalar The type to represent a scalar
     */
    template <int N = Eigen::Dynamic, int M = Eigen::Dynamic, int P = Eigen::Dynamic, typename Scalar = double>
    struct DiscreteStateSpaceParameters : public internal::StateSpaceParametersBase<N, M, P, Scalar> {

        /*!
         * \brief Default Constructor
         *
         * Initializes parameters and sample time with zero
         */
        DiscreteStateSpaceParameters()
            : internal::StateSpaceParametersBase<N, M, P, Scalar>()
            , sampleTime(0.0)
        {
        }

        /*!
         * \brief Parameter Constructor
         *
         * \param A The system matrix
         * \param B The input matrix
         * \param C The output matrix
         * \param D The feedthrough matrix
         * \param sampleTime The sample time in seconds
         */
        template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
        DiscreteStateSpaceParameters(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B,
            const Eigen::MatrixBase<DerivedC>& C, const Eigen::MatrixBase<DerivedD>& D, double sampleTime)
            : internal::StateSpaceParametersBase<N, M, P, Scalar>(A, B, C, D)
            , sampleTime(sampleTime)
        {
        }

        /*!
         * \brief Copy constructor
         * \param other Other discrete-time state space parameters
         */
        template <int NOther, int MOther, int POther, typename ScalarOther>
        DiscreteStateSpaceParameters(const DiscreteStateSpaceParameters<NOther, MOther, POther, ScalarOther>& other)
            : internal::StateSpaceParametersBase<N, M, P, Scalar>(other.A, other.B, other.C, other.D)
            , sampleTime(other.sampleTime)
        {
        }

        /*!
         * \brief Assignment operator
         * \param other Other discrete-time state space parameters
         */
        template <int NOther, int MOther, int POther, typename ScalarOther>
        DiscreteStateSpaceParameters& operator=(const DiscreteStateSpaceParameters<NOther, MOther, POther, ScalarOther>& other)
        {
            this->A = other.A;
            this->B = other.B;
            this->C = other.C;
            this->D = other.D;
            this->sampleTime = other.sampleTime;
            return *this;
        }

        //! Outputs the parameters to the given stream
        friend std::ostream& operator<<(std::ostream& os, const DiscreteStateSpaceParameters<N, M, P, Scalar>& params)
        {
            os << "<broccoli discrete state space parameters>" << std::endl;
            os << "A: " << params.A << std::endl;
            os << "B: " << params.B << std::endl;
            os << "C: " << params.C << std::endl;
            os << "D: " << params.D << std::endl;
            os << "dt: " << params.sampleTime << std::endl;
            os << "</broccoli discrete state space parameters>";
            return os;
        }

        //! The sample time in seconds
        double sampleTime;
    };

} // namespace control
} // namespace broccoli
#endif
