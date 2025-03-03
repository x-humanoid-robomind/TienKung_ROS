/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../splines/QuaternionBSplineSpline.hpp"
#include "QuaternionTrajectory.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a quaternion trajectory with underlying quaternion BSpline spline
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * See \ref QuaternionTrajectory and \ref QuaternionBSplineSpline for details.
     *
     * \tparam ParameterSplineType Type of parameter spline passed to \ref QuaternionTrajectory()
     * \tparam Degree Passed through to \ref QuaternionBSplineSpline.
     * \tparam MaximumBSplineCount Passed through to \ref QuaternionBSplineSpline.
     */
    template <class ParameterSplineType, unsigned int Degree, unsigned int MaximumBSplineCount = Degree + 1>
    class QuaternionBSplineTrajectory : public QuaternionTrajectory<QuaternionBSplineSpline<Degree, MaximumBSplineCount>, ParameterSplineType> {
    public:
        //! Constructor
        QuaternionBSplineTrajectory()
        {
        }

        //! Destructor
        virtual ~QuaternionBSplineTrajectory()
        {
        }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of underlying B-spline basis
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of underlying B-spline basis
        static unsigned int order() { return Degree + 1; }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
