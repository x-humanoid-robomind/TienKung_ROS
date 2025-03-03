/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#ifdef HAVE_EIGEN3

#include "../../core/type_traits.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace control {

    namespace internal {
        /*!
         * \brief State space parameter base class
         * \tparam N The number of states
         * \tparam M The number of system inputs
         * \tparam P The number of system outputs
         * \tparam Scalar The type to represent a scalar
         */
        template <int N = Eigen::Dynamic, int M = Eigen::Dynamic, int P = Eigen::Dynamic, typename Scalar = double>
        struct StateSpaceParametersBase {
            using AType = Eigen::Matrix<Scalar, N, N>; //!< System matrix type
            using BType = Eigen::Matrix<Scalar, N, M>; //!< Input matrix type
            using CType = Eigen::Matrix<Scalar, P, N>; //!< Output matrix type
            using DType = Eigen::Matrix<Scalar, P, M>; //!< Feedthrough matrix type

            /*!
             * \brief Default Constructor
             *
             * Initializes parameters with zero
             */
            StateSpaceParametersBase()
                : A(core::Traits<AType>::zero())
                , B(core::Traits<BType>::zero())
                , C(core::Traits<CType>::zero())
                , D(core::Traits<DType>::zero())
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
            StateSpaceParametersBase(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B,
                const Eigen::MatrixBase<DerivedC>& C, const Eigen::MatrixBase<DerivedD>& D)
                : A(A)
                , B(B)
                , C(C)
                , D(D)
            {
            }

            //! System matrix A
            AType A;

            //! Input matrix B
            BType B;

            //! Output matrix C
            CType C;

            //! Feedthrough matrix D
            DType D;

        protected:
            StateSpaceParametersBase(const StateSpaceParametersBase& other) = default;
            StateSpaceParametersBase(StateSpaceParametersBase&& other) noexcept = default;
            StateSpaceParametersBase& operator=(const StateSpaceParametersBase& other) = default;
            StateSpaceParametersBase& operator=(StateSpaceParametersBase&& other) noexcept = default;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }

} // namespace control
} // namespace broccoli
#endif
