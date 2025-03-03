/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../splines/PolynomialSpline.hpp"
#include "Trajectory.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a trajectory with underlying polynomial splines
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * See \ref Trajectory and \ref PolynomialSpline for details.
     *
     * \tparam Degree Passed through to \ref PolynomialSpline.
     * \tparam Dimension Passed through to \ref Trajectory.
     */
    template <unsigned int Degree, unsigned int Dimension>
    class PolynomialTrajectory : public Trajectory<PolynomialSpline<Degree>, Dimension> {
    public:
        //! Constructor
        PolynomialTrajectory()
        {
        }

        //! Destructor
        virtual ~PolynomialTrajectory()
        {
        }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of underlying polynomial spline segments
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of underlying polynomial spline segments
        static unsigned int order() { return Degree + 1; }
    };
} // namespace curve
} // namespace broccoli
