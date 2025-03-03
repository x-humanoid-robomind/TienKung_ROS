/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../splines/ExponentialSpline.hpp"
#include "Trajectory.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a trajectory with underlying exponential splines
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * See \ref Trajectory and \ref ExponentialSpline for details.
     *
     * \tparam Dimension Passed through to \ref Trajectory.
     */
    template <unsigned int Dimension>
    class ExponentialTrajectory : public Trajectory<ExponentialSpline, Dimension> {
    public:
        //! Constructor
        ExponentialTrajectory()
        {
        }

        //! Destructor
        virtual ~ExponentialTrajectory()
        {
        }
    };
} // namespace curve
} // namespace broccoli
