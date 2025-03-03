/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../../core/IntegerTimeTypes.hpp"
#include "../../core/floats.hpp"
#include "AbstractSolvableListener.hpp"
#include <limits>
#include <vector>

namespace broccoli {
namespace ode {

    class AbstractSystemSolver;

    //! Abstract base class for solvable objects.
    /*!
     * \ingroup broccoli_ode_integration
     *
     * This class defines interfaces for a general object, which may be solved
     * with an AbstractSystemSolver. In general, such objects may contain discrete-time and continuous-time parts.
     * Between subsequent discrete-time events, continuous-time integration blocks are executed by the solver.
     *
     * **Example:**
     * You have a system, which has two different time-discrete parts with events (e.g. computations running in a loop)
     * occurring every 0.1 ms and 1 ms respectively. In addition, time-continuous parts (i.e. a state-space model) need to
     * be solved and the time-discrete parts may modify the behavior of the time-continuous parts. The diagram below shows
     * the timing between time-discrete parts (before(), after()) and block-wise integration of time-continuous parts:
     *
     * \verbatim
     * Time-Discrete Entity A:           *                     *                     *                 *
     * 0.1ms sample time                                after(0.0, 1e-04)      after(1e-04, 2e-04)    after(9e-04, 1e-03)
     *                            before(0.0, 1e-04)   before(1e-04, 2e-04)    before(2e-04, 3e-04)   before(1e-03, 1.1e-03)
     *
     * Time-Discrete Entity B:           *                                                             *
     * 1ms sample time                                                                          after(0.0, 1e-03)
     *                            before(0.0, 1e-04)                                            before(1e-03, 1.1e-03)
     *
     * Continuous-time blocks:            xxxxxxxxxxxxxxxxxxxxx xxxxxxxxxxxxxxxxxxxxx xxxxxx ~~~ xxxxxx xxxx
     * (pure Integration)
     *                                   |---------------------|---------------------|------ ~~~ ------|---- ***
     * time in s:                       0.0                  0.0001                0.0002            0.001
     * \endverbatim
     *
     */
    class AbstractSolvable : public AbstractSolvableListener {
    public:
        virtual ~AbstractSolvable() {}

        //! Adds an event listener to the solvable object.
        void addListener(AbstractSolvableListener* listener)
        {
            m_listeners.push_back(listener);
        }

        virtual void init(const double& endTime) override
        {
            for (auto listener : m_listeners) {
                listener->init(endTime);
            }
        }

        virtual void finally() override
        {
            for (auto listener : m_listeners) {
                listener->finally();
            }
        }

        //! Returns the sample time of this objects's time-discrete internals in nanoseconds.
        /*!
         * This defines the sample time in which before() and after() are called.
         * If the object is fully continuous, zero should be returned. In contrast to propagatedSampleTime()
         * this method returns the sample time of the time-discrete internals directly associated with this object.
         * It does not take into account any time-discrete parts of subsystems linked to this object.
         *
         * \note If zero is returned and propagatedSampleTime() returns a non-zero value, before() and after()
         * are called based on propagatedSampleTime(). If both methods return zero, before() and after()
         * are called in the interval the solver selected for the continuous-time integration blocks.
         *
         * \returns The sample-time of this object's time-discrete internals in nanoseconds.
         */
        virtual core::IntegerNanoSeconds<uint64_t> sampleTime() const = 0;

        //! Returns the **propagated** sample time of the object itself and all associated objects in nanoseconds.
        /*!
         * This should return the greatest common divisor of all time-discrete sample times within the object and its
         * children. If the system is fully continuous, zero should be returned. The default implementation returns
         * the value of sampleTime().
         *
         * \returns The sample-time of the time-discrete internals of the object itself and all linked subsystems in nanoseconds.
         */
        virtual core::IntegerNanoSeconds<uint64_t> propagatedSampleTime() const
        {
            return sampleTime();
        }

        //! Integrates the solvable object with the specified solver.
        /*!
         * This method is usually directly called by the solver as part of a double-dispatch step.
         * However, it may also be used directly to execute an integration from t0 to t1.
         *
         * \note Implementations of this method are responsible for calling the before() and after() methods as well as handing over integration
         * to the solver by calling AbstractSystemSolver::integrateEntity() and passing itself. before() and after() must be called not at all or exactly once
         * within this integration block. before() must only be called if the time t0 is a multiple
         * of the output of sampleTime(). after() must only be called if the time t1 is a multiple of the output of sampleTime().
         *
         * @param solver The system solver object
         * @param t0 Start time of integration
         * @param t1 End time of integration
         */
        virtual void integrateWith(AbstractSystemSolver* solver, const double& t0, const double& t1) = 0;

    protected:
        //! Returns true if the given time is a multiple of the object's sample time.
        /*!
         * If sampleTime() returns zero, propagatedSampleTime() is used instead.
         * If this also returns zero, true is returned.
         *
         * \param t Time in seconds to compare
         * \return True, if t is a multiple of the sample time of the system. If the sample time is zero, this always returns true.
         */
        bool hitsDiscreteSampleTime(const double& t) const
        {
            if (sampleTime().integer() == 0) {
                return hitsDiscreteSampleTime(t, propagatedSampleTime().toDouble());
            }

            return hitsDiscreteSampleTime(t, sampleTime().toDouble());
        }

        //! Returns true if the given time t is a multiple of the specified sample time.
        /*!
         * \param t Time in seconds to compare
         * \param sampleTime Sample-Time in seconds used for comparison
         * \returns True, if t is a multiple of sampleTime. If sampleTime is zero, this always returns true.
         */
        static bool hitsDiscreteSampleTime(const double& t, const double& sampleTime)
        {
            if (sampleTime == 0) {
                return true;
            }

            uint64_t numberOfWholeSteps = (uint64_t)((t + core::comparisonThresholdFor(t)) / sampleTime);
            return core::isEqual((double)numberOfWholeSteps * sampleTime, t);
        }

        /*!
         * \copydoc broccoli::ode::AbstractSolvableListener::before()
         * \remark The difference t1-t0 is **not** automatically the sample time of the system as
         * returned by sampleTime()!
         */
        virtual void before(const double& t0, const double& t1) override
        {
            for (auto listener : m_listeners) {
                listener->before(t0, t1);
            }
        }

        /*!
         * \copydoc broccoli::ode::AbstractSolvableListener::after()
         * \remark The difference t1-t0 is **not** automatically the sample time of the system as
         * returned by sampleTime()!
         */
        virtual void after(const double& t0, const double& t1) override
        {
            for (auto listener : m_listeners) {
                listener->after(t0, t1);
            }
        }

    private:
        //! List of event listeners
        std::vector<AbstractSolvableListener*> m_listeners;
    };
} // namespace ode
} // namespace broccoli
