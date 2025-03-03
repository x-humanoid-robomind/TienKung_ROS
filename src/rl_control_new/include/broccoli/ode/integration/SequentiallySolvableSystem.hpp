/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include "../../core/math.hpp"
#include "AbstractSolvable.hpp"
#include "solvers/AbstractSystemSolver.hpp"
#include <vector>

namespace broccoli {
namespace ode {
    //! Represents a system of sequentially sorted AbstractSolvable objects.
    /*!
     * \ingroup broccoli_ode_integration
     *
     * This system contains AbstractSolvable children in a specified order.
     * The children are solved in a sequential manner, i.e. first subsystem 1 is integrated, than
     * subsystem 2 and so on. The order in which subsystems are solved is specified by the order addChild()
     * is called to add these objects to the tree. The system usually inherits its sample time from its children.
     * A specified sample-time for before() and after() may be set by overloading sampleTime(). This system does not allow parallel solving of objects.
     *
     * **Example:**By using cascaded structures of SequentiallySolvableSystem instances, complex arrangements of subsystems can be represented.
     * Image you want to simulate a robot and have implementations for the control software, the hardware abstraction layer,
     * the joint controllers, and a multi-body simulation model of the mechanical parts. You can use the following composite structure - childs are
     * added with addChild():
     *
     * - ControlSoftware : SequentiallySolvableSystem (sampleTime() overridden and returns 1ms)
     *   - HardwareAbstractionLayer: SequentiallySolvableSystem (sampleTime() overridden and returns 0.1ms)
     *     - JointController : SequentiallySolvableSystem (sampleTime() overridden and returns 0.1ms)
     *       - Multi-Body Simulation : AbstractSolvableEntity (continuous-time model)
     *
     * This structure has the advantage that first all before() methods are called in the order of the hierarchy, then
     * the multi-body system is integrated, and then all after() methods are called in inverse order of the hierarchy.
     * This way before() methods can be used to process data flow from the control software to the hardware, while the after()
     * methods process the information from the hardware back to the control software. All elements may have different sample times.
     */
    class SequentiallySolvableSystem : public AbstractSolvable {

    public:
        void init(const double& endTime) override
        {
            AbstractSolvable::init(endTime);
            for (auto sys : m_subsystems) {
                sys->init(endTime);
            }
        }

        void finally() override
        {
            AbstractSolvable::finally();
            for (auto sys : m_subsystems) {
                sys->finally();
            }
        }

        //! Add a subsystem or entity.
        /*!
         * The subsystem's calling order is based on the order in which they are added
         * to their parent system.
         *
         * \param object Solvable object to be added as child of this system.
         */
        void addChild(AbstractSolvable* object)
        {
            if (object) {
                m_subsystems.push_back(object);
            }
        }

        //! Removes all children.
        void removeChildren()
        {
            m_subsystems.clear();
        }

        void integrateWith(AbstractSystemSolver* solver, const double& t0, const double& t1) override
        {
            if (hitsDiscreteSampleTime(t0))
                before(t0, t1);

            for (auto sys : m_subsystems) {

                // step thru all subsystems
                sys->integrateWith(solver, t0, t1);
            }

            if (hitsDiscreteSampleTime(t1))
                after(t0, t1);
        }

        /*!
         * \copydoc AbstractSolvable::sampleTime()
         * \remark The default implementation returns zero, i.e. this system has no own time-discrete parts, but computes
         * its sample time from its children (see propagatedSampleTime()).
         */
        core::IntegerNanoSeconds<uint64_t> sampleTime() const override
        {
            return (uint64_t)0;
        }

        /*!
         * \copydoc AbstractSolvable::propagatedSampleTime()
         * \remark For a system this is by default the greatest common divisor of the sample times of all objects added to the system.
         */
        core::IntegerNanoSeconds<uint64_t> propagatedSampleTime() const override
        {
            core::IntegerNanoSeconds<uint64_t> commonSampleTime(sampleTime());
            for (auto sys : m_subsystems) {
                commonSampleTime = core::math::greatestCommonDivisor(commonSampleTime.integer(), sys->propagatedSampleTime().integer());
            }

            return commonSampleTime;
        }

    protected:
        /*! \copydoc AbstractSolvable::before()
         * This method is called before all subsystems are solved for the current integration block t0 to t1.
         */
        void before(const double& t0, const double& t1) override
        {
            AbstractSolvable::before(t0, t1);
        }

        /*! \copydoc AbstractSolvable::after()
         * This method is called after all subsystems have been solved for the current integration block t0 to t1.
         */
        void after(const double& t0, const double& t1) override
        {
            AbstractSolvable::after(t0, t1);
        }

    private:
        //! Vector to store the subsystems
        std::vector<AbstractSolvable*> m_subsystems;
    };
} // namespace ode
} // namespace broccoli
