/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "PlatformHelper.hpp"
#include <assert.h>
#include <pthread.h>

namespace broccoli {
namespace core {
    /*!
     * \brief Generic PlatformHelper implementation for POSIX compliant systems
     * \ingroup broccoli_core_platform
     */
    class PlatformHelperPOSIX : public PlatformHelper {
    public:
        bool setThreadName(const std::string& name) const override
        {
#ifndef __APPLE__
            return pthread_setname_np(pthread_self(), name.c_str()) == 0;
#else
            (void)name;
            assert(false && "this should never happen");
            return false;
#endif
        }

        int threadPriority() const override
        {
            struct sched_param param;
            int policy;
            if (pthread_getschedparam(pthread_self(), &policy, &param) != 0)
                return 0;
            else
                return param.sched_priority;
        }

        bool setThreadPriority(const int& priority) const override
        {
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
#ifdef _GNU_SOURCE // Requires GNU Source-based POSIX
            if (cpuIndex < 0)
                return true;

            cpu_set_t cpuSet;
            CPU_ZERO(&cpuSet);
            CPU_SET(cpuIndex, &cpuSet);

            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet) == 0)
                return true;
#else
            (void)cpuIndex;
#endif
            return false;
        }

        void waitFor(const timespec& duration) const override
        {
            nanosleep(&duration, nullptr);
        }

        void waitFor(const uint64_t& seconds, const uint64_t& nanoSeconds) const override
        {
            struct timespec duration;
            duration.tv_sec = seconds;
            duration.tv_nsec = nanoSeconds;
            nanosleep(&duration, nullptr);
        }

        void currentTime(int64_t& seconds, int64_t& nanoSeconds) const override
        {
            // Get time specification from real-time clock
            timespec timeSpecification;
            clock_gettime(CLOCK_MONOTONIC, &timeSpecification);
            seconds = timeSpecification.tv_sec;
            nanoSeconds = timeSpecification.tv_nsec;
        }

    protected:
        PlatformHelperPOSIX() {}
        friend class PlatformHelperFactory;
    };
} // namespace core
} // namespace broccoli
