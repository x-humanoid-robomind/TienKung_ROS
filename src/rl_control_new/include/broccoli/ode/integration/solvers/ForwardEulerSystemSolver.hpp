/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

// This class requires Eigen library
#ifdef HAVE_EIGEN3

#include "../AbstractSolvableEntity.hpp"
#include "AbstractFixedStepSystemSolver.hpp"
#include <Eigen/Dense>
#include <limits>

namespace broccoli {
namespace ode {
    //! Forward-Euler based fixed-step system solver
    /*!
     * \ingroup broccoli_ode_integration
     *
     * This uses simple forward-euler integration with a fixed time-step \f$dt\f$:
     * \f$y(t+dt)\f$ = \f$y(t) + \dot y \cdot dt\f$
     */
    class ForwardEulerSystemSolver : public AbstractFixedStepSystemSolver {
    public:
        void init(AbstractSolvable*, const double&) override {}

        void atomicStep(AbstractSolvableEntity* entity, const double& dt) override
        {
            // calculate right hand side
            entity->evaluateRHS();

            // explicit forward integration
            entity->state() += entity->stateGradient() * dt;
        }
    };
} // namespace ode
} // namespace broccoli

#endif // HAVE_EIGEN3
