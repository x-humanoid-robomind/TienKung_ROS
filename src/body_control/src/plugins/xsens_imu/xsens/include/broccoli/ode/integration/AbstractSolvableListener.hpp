/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma once

namespace broccoli {
namespace ode {
    /*!
     * \brief Defines an abstract interface for events on an AbstractSolvable object.
     * \ingroup broccoli_ode_integration
     */
    class AbstractSolvableListener {
    public:
        virtual ~AbstractSolvableListener() {}

        //! This method is called once by the solver before integration begins.
        /*!
         * \param endTime The end-time for the integration.
         */
        virtual void init(const double& endTime) = 0;

        //! This method is called once by the solver after the solvable object was solved (end time reached).
        virtual void finally() = 0;

        //! This method is called before an continuous-time integration block from t0 to t1 if t0 is a multiple of AbstractSolvable::sampleTime().
        /*!
         * When a system consists of multiple time-discrete parts with different sample times, the before() method of the corresponding
         * AbstractSolvableEntity is always called in the frequency 1 / AbstractSolvable::sampleTime(). This means that not all continuous-time integration
         * blocks from t0 to t1 are automatically discrete-time steps. Only if t0 is a multiple of AbstractSolvable::sampleTime(), before() is called.
         * \note The systems left- and righthand sides are evaluated after before() is called.
         * @param t0 Start time of last step
         * @param t1 End time of last step
         */
        virtual void before(const double& t0, const double& t1) = 0;

        //! This method is called after an continuous-time integration block from t0 to t1 if t1 is a multiple of AbstractSolvable::sampleTime().
        /*!
         * When a system consists of multiple time-discrete parts with different sample times, the after() method of the corresponding
         * AbstractSolvableEntity is always called in the frequency 1 / AbstractSolvable::sampleTime(). This means that not all continuous-time integration
         * blocks from t0 to t1 are automatically discrete-time steps. Only if t1 is a multiple of AbstractSolvable::sampleTime(), after() is called.
         * \note The systems left- and righthand sides are evaluated after before() is called
         * @param t0 Start time of last step
         * @param t1 End time of last step
         */
        virtual void after(const double& t0, const double& t1) = 0;
    };
} // namespace ode
} // namespace broccoli

#pragma clang diagnostic pop
