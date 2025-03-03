/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "SynchronizedWorker.hpp"

namespace broccoli {
namespace parallel {
    //! A helper class for dealing with pools of \ref SynchronizedWorker instances
    /*! \ingroup broccoli_parallel */
    class SynchronizedWorkerPool {
    public:
        //! Waits for all workers of the given pool the get ready
        /*!
         * \param [in] workers The worker pool in consideration
         * \param [in] timeout Optional timeout (use 0 to disable)
         * \return `true`, if all workers are ready, `false` on timeout
         */
        template <typename WorkerPool>
        static inline bool waitForReady(const WorkerPool& workers, const core::Time& timeout = 0)
        {
            const core::Time startTime = core::Time::currentTime();
            bool ready = false;
            while (!ready) {
                if (timeout > 0 && core::Time::currentTime() > startTime + timeout)
                    return false; // Timeout!
                ready = true;
                for (const auto& worker : workers) {
                    if (worker.ready() == false) {
                        ready = false;
                        break;
                    }
                }
            }
            return true; // Clean exit
        }

        //! Waits for all workers of the given pool to finish
        /*!
         * \param [in] workers The worker pool in consideration
         * \param [in] timeout Optional timeout (use 0 to disable)
         * \return `true`, if all workers are finished, `false` on timeout
         */
        template <typename WorkerPool>
        static inline bool waitForFinish(const WorkerPool& workers, const core::Time& timeout = 0)
        {
            const core::Time startTime = core::Time::currentTime();
            bool finished = false;
            while (!finished) {
                if (timeout > 0 && core::Time::currentTime() > startTime + timeout)
                    return false; // Timeout!
                finished = true;
                for (const auto& worker : workers) {
                    if (worker.finished() == false) {
                        finished = false;
                        break;
                    }
                }
            }
            return true; // Clean exit
        }
    };
} // namespace parallel
} // namespace broccoli
