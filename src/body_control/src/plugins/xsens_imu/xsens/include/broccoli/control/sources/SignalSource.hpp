/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../Signal.hpp"

namespace broccoli {
namespace control {
    /*!
     * \brief Base class for a Signal source
     * \ingroup broccoli_control_sources
     */
    template <typename Derived>
    class SignalSource : public SignalBase<Derived> {
    public:
        /*!
         * \brief Construct a signal source.
         *
         * This uses 0 for initialization of the output signal.
         *
         * \param autoRun If true, the source starts to output its signal automatically.
         * Otherwise, the signal must be started with start()
         */
        explicit SignalSource(const bool& autoRun = false)
            : m_running(autoRun)
        {
        }

        /*!
         * \brief (Re-)starts the signal source
         * \returns A reference to this instance
         */
        Derived& start()
        {
            m_running = true;
            m_time = 0.0;
            return this->derived();
        }

        /*!
         * \brief (Re-)starts the signal source on change of a boolean input signal
         *
         * \param signal Boolean signal to trigger start. start() is called on change from false to true.
         * \returns A reference to this instance
         */
        Derived& startOnChange(const bool& signal)
        {
            if (!m_lastBooleanTriggerState && signal) {
                start();
                m_lastBooleanTriggerState = true;
            }
            return this->derived();
        }

        /*!
         * \brief Stops the signal source.
         *
         * \note A stopped signal source returns the
         * prototype instance given in the constructor (or 0).
         * \returns A reference to this instance
         */
        Derived& stop()
        {
            m_running = false;
            m_lastBooleanTriggerState = false;
            return this->derived();
        }

        /*!
         * \brief Calculates the signal value for the current time step and returns it
         * \return Output signal for the current time step
         */
        auto& process()
        {
            computeOutput(m_time);

            if (m_running)
                m_time += this->derived().sampleTime();

            return *this;
        }

        //! Returns true when the signal source is active (started)
        const bool& isActive() const
        {
            return m_running;
        }

        /*!
         * \brief Compute the source signal value
         * \param time The time since last start of the source
         * \returns A reference to this instance
         */
        Derived& computeOutput(const double& time)
        {
            return this->derived().computeOutput(time);
        }

    private:
        //! Was the source started?
        bool m_running = false;

        //! Last state of the trigger signal
        bool m_lastBooleanTriggerState = false;

        //! The time since the last start() of the source
        double m_time = 0.0;
    };
} // namespace control
} // namespace broccoli
