/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../splines/QuaternionLERPSpline.hpp"
#include "QuaternionTrajectory.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a quaternion trajectory with underlying quaternion LERP spline
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * See \ref QuaternionTrajectory and \ref QuaternionLERPSpline for details.
     *
     * \tparam ParameterSplineType Type of parameter spline passed to \ref QuaternionTrajectory()
     */
    template <class ParameterSplineType>
    class QuaternionLERPTrajectory : public QuaternionTrajectory<QuaternionLERPSpline, ParameterSplineType> {
    public:
        //! Constructor
        QuaternionLERPTrajectory()
        {
        }

        //! Destructor
        virtual ~QuaternionLERPTrajectory()
        {
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
