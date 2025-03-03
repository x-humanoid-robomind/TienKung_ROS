/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <stdint.h>
#include <string>
#include <time.h>

namespace broccoli {
namespace core {
    /*!
     * \brief Describes an interface for a set of platform-dependent functions
     * \ingroup broccoli_core_platform
     * To create an instance of a suitable PlatformHelper, use the PlatformHelperFactory.
     */
    class PlatformHelper {
    public:
        virtual ~PlatformHelper() {}

        /*!
         * \brief Sets the name of the current thread
         * \param name A string with the desired name of the thread
         * \returns true on success
         */
        virtual bool setThreadName(const std::string& name) const = 0;

        //! Returns the scheduling priority of the current thread
        virtual int threadPriority() const = 0;

        //! Sets the scheduling priority of the current thread
        /*!
         * \param priority Scheduling priority (see documentation of the implementations)
         * \return True on success
         */
        virtual bool setThreadPriority(const int& priority) const = 0;

        //! Sets the CPU affinity for the current thread
        /*!
         * \param cpuIndex CPU index (0 = first CPU). If an index < 0 is supplied, the CPU affinity is reset.
         * \return True on success
         */
        virtual bool setCpuAffinity(const int& cpuIndex) const = 0;

        /*!
         * \brief Pauses the current thread for the given duration.
         * \param duration The time to wait as time specification object
         *
         * \remark The time spent may be longer (but not shorter) than the requested time.
         * Timer quantization errors are not corrected. This is a non-blocking (yielding) wait.
         */
        virtual void waitFor(const timespec& duration) const = 0;

        /*!
         * \brief Pauses the current thread for the given duration.
         * \param seconds The amount of seconds to wait
         * \param nanoSeconds The amount of nanoseconds to wait
         *
         * \remark The time spent may be longer (but not shorter) than the requested time.
         * Timer quantization errors are not corrected. This is a non-blocking (yielding) wait.
         */
        virtual void waitFor(const uint64_t& seconds, const uint64_t& nanoSeconds) const = 0;

        //! Gets the current system time (as Unix Epoch time) using the highest possible precision for this platform
        /*!
         * The time is described relative to 00:00:00 UTC on 1 January 1970 (Unix Epoch). Uses `CLOCK_REALTIME`.
         * \param [out] seconds Seconds passed since Unix Epoch.
         * \param [out] nanoSeconds Nano-seconds passed (additionally to seconds) since Unix Epoch
         */
        virtual void currentTime(int64_t& seconds, int64_t& nanoSeconds) const = 0;

        //! Detects, if the platform uses **little-endian** byte-order
        /*!
         * On the very first run, the endianness of the platform is evaluated. After the first run, the already evaluated flag is reused for maximum performance.
         * \return `true` if platform uses **little-endian**, `false` if it uses **big-endian**.
         */
        virtual bool usesLittleEndian() const
        {
            // Initialize flags
            static bool littleEndian = true; // Flag indicating, if platform uses little-endian byte-order
            static bool alreadyDetected = false; // Flag indicating, if the endianness has already been evaluated

            // Check, if we need to detect endianness
            if (alreadyDetected == false) {
                static const uint16_t testNumber = 0x00FF;
                littleEndian = (*((uint8_t*)&testNumber) == 0xFF);
                alreadyDetected = true;
            }

            return littleEndian;
        }
    };
} // namespace core
} // namespace broccoli
