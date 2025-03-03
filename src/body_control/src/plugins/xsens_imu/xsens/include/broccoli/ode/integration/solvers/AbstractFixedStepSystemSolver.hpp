/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include "../../../core/floats.hpp"
#include "AbstractSystemSolver.hpp"

namespace broccoli {
namespace ode {
    /*!
     * \brief Abstract base class for system solvers with fixed time steps
     * \ingroup broccoli_ode_integration
     */
    class AbstractFixedStepSystemSolver : public AbstractSystemSolver {
    public:
        void integrateEntity(AbstractSolvableEntity* entity, const double& t0, const double& t1) override
        {
            assert(timeStep() != 0.0);
            entity->setCurrentTime(t0);

            if (core::isLess(t1 - t0, timeStep())) {
                atomicStep(entity, t1 - t0);
                entity->setCurrentTime(t1);
                return;
            }

            // We use fixed-point arithmetic to loop through the integration block.
            // This avoids the aggregation of numerical errors in the time, which may lead to an incorrect
            // number of integration steps for a larger time interval.
            // Note: The last step might not be a full step
            core::FixedStepArithmeticNumber<uint64_t> time(t0, timeStep());
            while (true) {

                atomicStep(entity, timeStep());
                time++;
                entity->setCurrentTime(time.toDouble());

                if (time == t1) {
                    break;
                } else if (time.next() > t1) {

                    // Adjust last step
                    atomicStep(entity, t1 - time.toDouble());
                    entity->setCurrentTime(t1);
                    break;
                }
            }
        }

        //! Sets the fixed time step \f$dt\f$ used for integration
        /*!
         * \note If this step size is bigger than the smallest time-discrete
         * sample time of the system to be integrated, the solver automatically chooses a
         * smaller step size.
         *
         * \param dt Integration time step in seconds.
         */
        void setTimeStep(const double& dt)
        {
            m_timeStep = dt;
        }

        double timeStep() const override
        {
            return m_timeStep;
        }

        //! Performs an atomic integration step with step-size \f$dt\f$
        /*!
         * \note An implementation of this method must call AbstractSolvableEntity::evaluateRHS()
         * before performing the actual integration step.
         *
         * \param entity Entity, whose state is integrated
         * \param dt Actual integration step length in seconds
         */
        virtual void atomicStep(AbstractSolvableEntity* entity, const double& dt) = 0;

    protected:
        //! Time step in seconds used for integration
        double m_timeStep = 0.0;
    };
} // namespace ode
} // namespace broccoli
