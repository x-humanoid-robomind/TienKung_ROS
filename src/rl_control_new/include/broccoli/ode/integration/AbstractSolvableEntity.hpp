/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

// This class requires Eigen library
#ifdef HAVE_EIGEN3

#include "AbstractSolvable.hpp"
#include "solvers/AbstractSystemSolver.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace broccoli {
namespace ode {
    //! Abstract base class for solvable entities based on ODEs and time-discrete parts.
    /*!
     * \ingroup broccoli_ode_integration
     *
     * This class defines interfaces for solvable entities described by
     * \f[
     * \dot y = f(t, y)
     * \f]
     * with the state vector \f$y\f$ and the righthand side function \f$f(t,y)\f$.
     * This system is solved by integration of the gradient \f$\dot y\f$ to an
     * updated system state \f$y^*\f$.
     */
    class AbstractSolvableEntity : public AbstractSolvable {
    public:
        void init(const double& endTime) override
        {
            AbstractSolvable::init(endTime);
            m_hasContinuousParts = (stateVectorSize() != 0);
        }

        //! Returns a reference to the entity's state vector \f$y\f$
        /*!
         * \returns A reference to the state vector.
         */
        virtual Eigen::VectorXd& state() = 0;

        //! Returns the system's state gradient vector \f$\dot y\f$
        /*!
         * \returns Const reference to the state gradient vector
         */
        virtual const Eigen::VectorXd& stateGradient() const = 0;

        //! Evaluates the righthand side of the differential equation.
        /*!
         * Evaluates the righthand side \f$f(t, y)\f$ of the corresponding
         * first-order differential equation \f$\dot y = f(t, y)\f$.
         * This is called by the solver before each integration step.
         *
         * \note The time for the evaluation is stored in AbstractSolvableEntity::m_time.
         * The state used for the evaluation is the systems's current state as represented by state().
         * All implementations of this method must store the computed gradient \f$\dot y\f$ and make sure stateGradient() returns it.
         */
        virtual void evaluateRHS() = 0;

        //! Returns the size of the state vector \f$y\f$
        /*!
         * \remark If zero is returned, the system is considered purely time-discrete and
         * the methods state(), stateGradient() and evaluateRHS() are not called at all.
         */
        virtual std::size_t stateVectorSize() const = 0;

        void integrateWith(AbstractSystemSolver* solver, const double& t0, const double& t1) override
        {

            if (hitsDiscreteSampleTime(t0))
                before(t0, t1);

            if (m_hasContinuousParts) {
                solver->integrateEntity(this, t0, t1);
            }

            if (hitsDiscreteSampleTime(t1))
                after(t0, t1);
        }

        //! Returns the current simulation time for this entity
        /*!
         * \return The current simulation time in seconds
         */
        const double& currentTime() const
        {
            return m_time;
        }

        //! Updates the current simulation time for this entity
        /*!
         * This must be called by the solver before AbstractSolvableEntity::evaluateRHS() is called.
         * \param time Desired state of the simulation time in seconds
         */
        virtual void setCurrentTime(const double& time)
        {
            m_time = time;
        }

    protected:
        //! The current simulation time \f$t\f$ for this entity (seconds).
        double m_time = 0.0;

    private:
        //! Does this entity have continuous parts?
        bool m_hasContinuousParts = true;
    };
} // namespace ode
} // namespace broccoli

#endif // HAVE_EIGEN3
