/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#ifdef __QNXNTO__

#include "PlatformHelperPOSIX.hpp"
#include <algorithm>
#include <inttypes.h>
#include <limits>
#include <pthread.h>
#include <sys/netmgr.h>
#include <sys/neutrino.h>
#include <sys/procmgr.h>
#include <sys/syspage.h>
#include <unistd.h>

namespace broccoli {
namespace core {
    /*!
     * \brief PlatformHelper implementation for QNX Neutrino RTOS
     * \ingroup broccoli_core_platform
     */
    class PlatformHelperQNX : public PlatformHelperPOSIX {
    public:
        /*!
         * \copybrief PlatformHelper::setThreadPriority()
         * \param priority Scheduling priority in the range 1 - 255 (the highest priority)
         */
        bool setThreadPriority(const int& priority) const override
        {
#if _NTO_VERSION >= 700
            // Enable procmgr ability for high-priority scheduling: PROCMGR_AID_PRIORITY
            if ((procmgr_ability(0, PROCMGR_ADN_ROOT | PROCMGR_AOP_ALLOW | PROCMGR_AID_PRIORITY, PROCMGR_AID_EOL)) != 0) {
                return false;
            }
#endif

            struct sched_param param;
            int policy;
            if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
                return false;
            }
            param.sched_priority = priority;
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
                return false;
            }
            return true;
        }

        bool setCpuAffinity(const int& cpuIndex) const override
        {
            if (cpuIndex < 0)
                return true;

            unsigned long cpuSet = 0;
            cpuSet = 1 << cpuIndex;

            return (ThreadCtl(_NTO_TCTL_RUNMASK, (void*)cpuSet) != -1);
        }

        /*!
         * \copydoc PlatformHelperPOSIX::currentTime() 
         *
         * \warning This method is only available for CPUs with a core frequency of less than 18GHz. Otherwise
         * an integer overflow may occur!
         */
        void currentTime(int64_t& seconds, int64_t& nanoSeconds) const override
        {
            // Create static variables (initialized at first call of this method)
            static int64_t referenceSeconds = 0; // The reference value used for the passed seconds since Unix Epoch
            static int64_t referenceNanoSeconds = 0; // The reference value used for the passed nanoseconds since Unix Epoch
            static uint64_t referenceClockCycles = 0; // The reference value used for the clock-cycle counter
            static bool referenceInitialized = false; // Flag indicating, if the reference variables have already been initialized
            static const uint64_t cyclesPerSecond = SYSPAGE_ENTRY(qtime)->cycles_per_sec; // Count of clock cycles per second

            // Initialize reference variables at first call of this method
            if (referenceInitialized == false) {
                // Initialize system time and clock cycles
                timespec timeSpecification;
                clock_gettime(CLOCK_REALTIME, &timeSpecification);
                referenceClockCycles = ClockCycles();
                referenceSeconds = timeSpecification.tv_sec;
                referenceNanoSeconds = timeSpecification.tv_nsec;

                // Remember that we already initialized reference values
                referenceInitialized = true;

                // Pass back reference values
                seconds = referenceSeconds;
                nanoSeconds = referenceNanoSeconds;
            } else {
                // Get current clock cycles
                const uint64_t currentClockCycles = ClockCycles();

                // Compute passed clock cycles
                uint64_t passedClockCycles;
                if (currentClockCycles >= referenceClockCycles) // ...free running counter did not run through its limit
                    passedClockCycles = currentClockCycles - referenceClockCycles;
                else // ...free running counter hit the limit -> compute delta
                    passedClockCycles = (std::numeric_limits<uint64_t>::max() - referenceClockCycles) + currentClockCycles;

                // Compute passed seconds and nanoseconds since reference
                const uint64_t passedSeconds = passedClockCycles / cyclesPerSecond;
                const uint64_t remainingClockCycles = passedClockCycles % cyclesPerSecond;
                uint64_t passedNanoSeconds = 0;
                passedNanoSeconds = (1000000000ULL * remainingClockCycles) / cyclesPerSecond;

                // Compute absolute seconds and nanoseconds
                seconds = referenceSeconds + passedSeconds;
                nanoSeconds = referenceNanoSeconds + passedNanoSeconds;
                while (nanoSeconds >= 1000000000ULL) {
                    nanoSeconds -= 1000000000ULL;
                    seconds += 1;
                }
            }
        }

    protected:
        PlatformHelperQNX() {}
        friend class PlatformHelperFactory;
    };
} // namespace core
} // namespace broccoli
#endif // __QNXNTO__
