/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include "../AbstractSolvable.hpp"
#include <limits>
#include <stdexcept>

namespace broccoli {
namespace ode {

    class AbstractSolvableEntity;

    //! Abstract base class for integration-based solvers of AbstractSolvable objects.
    /*!
     * \ingroup broccoli_ode_integration
     *
     * By passing a solvable instance to AbstractSystemSolver::solve(), the system is integrated (solved).
     * The solver automatically considers the nature of the system to be solved (continuous and
     * discrete parts or only discrete parts) for the best performance.
     *
     * Every solver has a public AbstractSystemSolver::integrateEntity() method, which is designed to
     * be called by the actually solvable entities of the system (double-dispatch).
     */
    class AbstractSystemSolver {
    public:
        virtual ~AbstractSystemSolver() {}

        //! Solves the given system with this solver.
        /*!
         * This first initializes the system via AbstractSolvable::init(), determines the
         * length of the continuous-time integration blocks, initializes the solver via init(),
         * and then executes integration by calling AbstractSolvable::integrateWith().
         *
         * When integration is done, AbstractSolvable::finally() is called.
         *
         * @param solvable Solvable system
         * @param endTime End time for integration
         */
        void solve(AbstractSolvable* solvable, const double& endTime)
        {

            // Init system
            solvable->init(endTime);

            /*! Time interval for which the system is considered continuous
             * (solvable without calling discrete update functions)
             * OR zero if the system has no time-discrete parts
             */
            core::IntegerNanoSeconds<uint64_t> continuousStepTime = solvable->propagatedSampleTime();

            if (continuousStepTime.integer() == 0 || continuousStepTime.toDouble() > endTime) {
                continuousStepTime.fromDouble(endTime);
            }

            // init the solver
            this->init(solvable, continuousStepTime.toDouble());

            // We use fixed-point arithmetic to loop through the integration blocks (the time).
            // This avoids the aggregation of numerical errors in the time, which may lead to an incorrect
            // number of integration steps for a larger time interval.
            // Note: The last block length might not be a full continuous step length
            core::FixedStepArithmeticNumber<uint64_t> time(0.0, continuousStepTime.toDouble());

            while (true) {

                solvable->integrateWith(this, time.toDouble(), (++time).toDouble());

                if (time == endTime) {
                    break;
                } else if (time.next() > endTime) {

                    // Adjust last step
                    solvable->integrateWith(this, time.toDouble(), endTime);
                    break;
                }
            }

            // done
            solvable->finally();
        }

        //! Perform integration from t0 to t1 on given entity
        /*!
         * This function is usually called via double-dispatch from the system to be solved.
         * However, it may also be used directly to execute a continuous-time integration from t0 to t1.
         *
         * \note An implementation of this method must first update the entity's simulation time via
         * AbstractSolvableEntity::setCurrentTime(), then call AbstractSolvableEntity::evaluateRHS(), and perform the actual integration.
         *
         * \param entity Entity to be solved
         * \param t0 Start time for the integration
         * \param t1 End time for the integration
         */
        virtual void integrateEntity(AbstractSolvableEntity* entity, const double& t0, const double& t1) = 0;

        //! Returns the current time-step size \f$dt\f$
        /*!
         * \note The actual time-step size might be different for the last integration step
         * to exactly reach the desired integration end time.
         * \return The current time-step size in seconds.
         */
        virtual double timeStep() const = 0;

        //! Initializes the solver for the given system
        /*!
         * \note This is automatically called within solve()
         * \param solvable The system to be solved
         * \param continuousStepTime Step interval for the continuous parts
         */
        virtual void init(AbstractSolvable* solvable, const double& continuousStepTime) = 0;
    };
} // namespace ode
} // namespace broccoli
