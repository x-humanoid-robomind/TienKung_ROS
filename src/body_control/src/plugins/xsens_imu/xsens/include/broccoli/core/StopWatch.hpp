/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "StatisticsEstimator.hpp"
#include "Time.hpp"

namespace broccoli {
namespace core {

    /*!
     * \brief A stop watch implementing comfortable high precision time period measurements using core::Time
     * \ingroup broccoli_core
     */
    class StopWatch {
    public:
        virtual ~StopWatch() = default;

        //! Start measuring a time period. Subsequent calls of start() restart the time measurement.
        virtual void start()
        {
            m_lastStartTime = Time::currentTime();
        }

        /*!
         * \brief Stop a measurement which was started with start().
         * \returns The time period [s] since the last call to start() / the initialization of this object.
         */
        virtual const double& stop()
        {
            m_lastTimePeriod = (Time::currentTime() - m_lastStartTime).toDouble();
            return m_lastTimePeriod;
        }

        //! An alias for start()
        void tic()
        {
            start();
        }

        //! An alias for stop()
        const double& toc()
        {
            return stop();
        }

        //! The time period [s] since the last call to start() / the initialization of this object.
        const double& elapsedTime() const
        {
            return m_lastTimePeriod;
        }

    private:
        //! The last stopped time period in seconds
        double m_lastTimePeriod = 0.0;

        //! Last recorded start() timestamp
        Time m_lastStartTime = Time::currentTime();
    };

    /*!
     * \brief A specialization of StopWatch calculating statistical data on subsequent time measurements
     * \ingroup broccoli_core
     */
    class StopWatchStatistical : public StopWatch {
    public:
        //! Reset the watch's statistics
        void resetStatistics()
        {
            m_estimator.reset();
        }

        //! Returns the statistics estimator, which gathers data on all seen samples since the last reset()
        const StatisticsEstimator<double>& statistics() const
        {
            return m_estimator;
        }

        const double& stop() override
        {
            StopWatch::stop();
            m_estimator.process(elapsedTime());
            return elapsedTime();
        }

    private:
        //! Running statistics estimator
        StatisticsEstimator<double> m_estimator;
    };

} // namespace core
} // namespace broccoli
