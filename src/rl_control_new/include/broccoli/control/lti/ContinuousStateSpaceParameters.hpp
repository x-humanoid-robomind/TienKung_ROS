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
     * \brief Continuous-time state space parameters
     * \ingroup broccoli_control_lti
     * \tparam N The number of states
     * \tparam M The number of system inputs
     * \tparam P The number of system outputs
     * \tparam Scalar The type to represent a scalar
     */
    template <int N = Eigen::Dynamic, int M = Eigen::Dynamic, int P = Eigen::Dynamic, typename Scalar = double>
    struct ContinuousStateSpaceParameters : public internal::StateSpaceParametersBase<N, M, P, Scalar> {
        /*!
         * \brief Default Constructor
         *
         * Initializes parameters with zero
         */
        ContinuousStateSpaceParameters()
            : internal::StateSpaceParametersBase<N, M, P, Scalar>()
        {
        }

        /*!
         * \brief Parameter constructor
         * \param A The system matrix
         * \param B The input matrix
         * \param C The output matrix
         * \param D The feedthrough matrix
         */
        template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
        ContinuousStateSpaceParameters(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B,
            const Eigen::MatrixBase<DerivedC>& C, const Eigen::MatrixBase<DerivedD>& D)
            : internal::StateSpaceParametersBase<N, M, P, Scalar>(A, B, C, D)
        {
        }

        /*!
         * \brief Copy constructor
         * \param other Other continuous-time state space parameters
         */
        template <int NOther, int MOther, int POther, typename ScalarOther>
        ContinuousStateSpaceParameters(const ContinuousStateSpaceParameters<NOther, MOther, POther, ScalarOther>& other)
            : internal::StateSpaceParametersBase<N, M, P, Scalar>(other.A, other.B, other.C, other.D)
        {
        }

        /*!
         * \brief Assignment operator
         * \param other Other continuous-time state space parameters
         */
        template <int NOther, int MOther, int POther, typename ScalarOther>
        ContinuousStateSpaceParameters& operator=(const ContinuousStateSpaceParameters<NOther, MOther, POther, ScalarOther>& other)
        {
            this->A = other.A;
            this->B = other.B;
            this->C = other.C;
            this->D = other.D;
            return *this;
        }

        //! Outputs the parameters to the given stream
        friend std::ostream& operator<<(std::ostream& os, const ContinuousStateSpaceParameters<N, M, P, Scalar>& params)
        {
            os << "<broccoli continuous state space parameters>" << std::endl;
            os << "A: " << params.A << std::endl;
            os << "B: " << params.B << std::endl;
            os << "C: " << params.C << std::endl;
            os << "D: " << params.D << std::endl;
            os << "</broccoli continuous state space parameters>";
            return os;
        }
    };

} // namespace control
} // namespace broccoli
#endif
