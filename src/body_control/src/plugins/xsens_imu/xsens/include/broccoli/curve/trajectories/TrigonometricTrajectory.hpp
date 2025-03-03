/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../splines/TrigonometricSpline.hpp"
#include "Trajectory.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a trajectory with underlying trigonometric splines
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * See \ref Trajectory and \ref TrigonometricSpline for details.
     *
     * \tparam Dimension Passed through to \ref Trajectory.
     */
    template <unsigned int Dimension>
    class TrigonometricTrajectory : public Trajectory<TrigonometricSpline, Dimension> {
    public:
        //! Constructor
        TrigonometricTrajectory()
        {
        }

        //! Destructor
        virtual ~TrigonometricTrajectory()
        {
        }
    };
} // namespace curve
} // namespace broccoli
