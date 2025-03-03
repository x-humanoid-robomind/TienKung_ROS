/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "platform/PlatformHelperFactory.hpp"
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <string>

namespace broccoli {
namespace core {
    /*!
     * \addtogroup broccoli_core_time
     * \{
     */

    // Class declaration
    // -----------------
    // See section "class definition" for details
    class Time;

    // Operator declarations
    // ---------------------
    // See section "operator definitions" for details
    // Declared and defined outside of class to also allow left-hand-side operations
    static inline Time operator+(const Time& first, const Time& second);
    static inline Time operator+(const Time& time, const double& scalar);
    static inline Time operator+(const double& scalar, const Time& time);
    static inline Time& operator+=(Time& first, const Time& second);
    static inline Time& operator+=(Time& time, const double& scalar);
    static inline Time operator-(const Time& first, const Time& second);
    static inline Time operator-(const Time& time, const double& scalar);
    static inline Time& operator-=(Time& first, const Time& second);
    static inline Time& operator-=(Time& time, const double& scalar);
    static inline Time operator*(const Time& time, const double& scalar);
    static inline Time operator*(const double& scalar, const Time& time);
    static inline Time& operator*=(Time& time, const double& scalar);
    static inline Time operator/(const Time& time, const double& scalar);
    static inline Time operator/(const double& scalar, const Time& time);
    static inline Time& operator/=(Time& time, const double& scalar);
    static inline bool operator==(const Time& first, const Time& second);
    static inline bool operator==(const Time& time, const double& scalar);
    static inline bool operator==(const double& scalar, const Time& time);
    static inline bool operator!=(const Time& first, const Time& second);
    static inline bool operator!=(const Time& time, const double& scalar);
    static inline bool operator!=(const double& scalar, const Time& time);
    static inline bool operator>(const Time& first, const Time& second);
    static inline bool operator>(const Time& time, const double& scalar);
    static inline bool operator>(const double& scalar, const Time& time);
    static inline bool operator>=(const Time& first, const Time& second);
    static inline bool operator>=(const Time& time, const double& scalar);
    static inline bool operator>=(const double& scalar, const Time& time);
    static inline bool operator<(const Time& first, const Time& second);
    static inline bool operator<(const Time& time, const double& scalar);
    static inline bool operator<(const double& scalar, const Time& time);
    static inline bool operator<=(const Time& first, const Time& second);
    static inline bool operator<=(const Time& time, const double& scalar);
    static inline bool operator<=(const double& scalar, const Time& time);

    // Class definition
    // ----------------
    //! Abstraction class for high-precision timing
    /*!
     * This container can be used for dealing with high precision time information when a simple double is not precise enough.
     *
     * By just using a simple double for (date-)time specification relative to Unix Epoch (00:00:00 UTC on 1 January 1970) one
     * is restricted to an accuracy of about 1-10 microseconds (as of today - 2018). This may not be accurate enough for some
     * purposes (especially for runtime measurements). This class extends time arithmetics to an accuracy on nanosecond level.
     * Note that the actual accuracy depends on the operating system and chosen clock type.
     *
     * The represented time is the sum of \ref m_seconds and \ref m_nanoSeconds. Note that these members may be negative (either
     * one or both). Note that special attention has to be paid to arithmetics to avoid loss of accuracy. For this reason
     * various arithmetic operators are implemented (see \ref Time.hpp).
     *
     * Futhermore the class implements convenience functions for time measurements, conversions and encoding.
     */
    class Time {
    public:
        // Constructors
        // ------------
        //! Specialized constructor (from seconds and nanoseconds)
        /*!
         * \param [in] seconds Sets \ref m_seconds - \copybrief m_seconds
         * \param [in] nanoSeconds  Sets \ref m_nanoSeconds - \copybrief m_nanoSeconds
         */
        Time(const int64_t& seconds, const int64_t& nanoSeconds)
            : m_seconds(seconds)
            , m_nanoSeconds(nanoSeconds)
        {
        }

        //! Specialized constructor (from `timespec` struct of standard library)
        /*!
         * Direct mapping of members of `timespec` to own members.
         * \param [in] timeSpecification Time specification from standard library.
         */
        Time(const timespec& timeSpecification)
            : m_seconds(timeSpecification.tv_sec)
            , m_nanoSeconds(timeSpecification.tv_nsec)
        {
        }

        //! Specialized constructor (from double)
        /*!
         * Seconds and nanoseconds are obtained from the given double represenation.
         * \warning Low accuracy, since the provided double may not contain enough accuracy!
         * \param [in] time Time [s] specified as double which will be split up into seconds and nanoseconds.
         */
        Time(const double& time)
        {
            // Assume positive time (to be able to round to zero)
            const double positiveTime = fabs(time);
            const double seconds = ::floor(positiveTime); // Extract seconds
            m_seconds = seconds;
            m_nanoSeconds = ::floor((double)1e9 * (positiveTime - seconds)); // Extract nanoseconds

            // Correct sign in case of negative time
            if (time < 0) {
                m_seconds = -m_seconds;
                m_nanoSeconds = -m_nanoSeconds;
            }
        }

        //! Default constructor (initializes to zero)
        Time()
            : Time(0, 0)
        {
        }

        // Members
        // -------
        int64_t m_seconds; //!< Seconds (may be negative and/or have a different sign than \ref m_nanoSeconds!)
        int64_t m_nanoSeconds; //!< Nanoseconds (may be negative and/or have a different sign than \ref m_seconds!)

        // Helpers
        // -------
        //! Simplify representation by seconds and nanoseconds
        /*!
         * Checks, if \ref m_nanoSeconds exceeds its limit and fixes the issue by adapting
         * \ref m_seconds accordingly such that \ref m_nanoSeconds stays within the limits.
         *
         * Furthermore modifies \ref m_seconds and \ref m_nanoSeconds, such that both
         * have the same sign.
         *
         * \attention This function does **not** change the acutal time, just the way it is represented.
         */
        inline void simplify()
        {
            // Check, if nanoseconds hits its limits
            if (m_nanoSeconds >= 1000000000LL || m_nanoSeconds <= -1000000000LL) {
                // Project to limits
                m_seconds += m_nanoSeconds / 1000000000LL;
                m_nanoSeconds = m_nanoSeconds % 1000000000LL;
            }

            // Check, if the sign of nanoseconds complies with seconds
            if (m_seconds > 0 && m_nanoSeconds < 0) {
                m_seconds -= 1;
                m_nanoSeconds += 1000000000LL;
            } else if (m_seconds < 0 && m_nanoSeconds > 0) {
                m_seconds += 1;
                m_nanoSeconds -= 1000000000LL;
            }
        }

        //! Conversion to `timespec` struct of standard library
        inline timespec toTimeSpec() const
        {
            // Create struct and copy members
            timespec timeSpecification;
            timeSpecification.tv_sec = m_seconds;
            timeSpecification.tv_nsec = m_nanoSeconds;

            // Pass back struct
            return timeSpecification;
        }

        //! Conversion to simple double
        /*!
         * \warning Loss of accuracy, since a double may not contain enough precision!
         *
         * \return Double representation of the time.
         */
        inline double toDouble() const
        {
            return (double)m_seconds + (double)1e-9 * m_nanoSeconds;
        }

        //! Determine, if time is zero
        /*!
         * Convenience method to check both members at once.
         * \return `true` if time is zero, `false` otherwise.
         */
        inline bool isZero() const
        {
            return (m_seconds == 0 && m_nanoSeconds == 0);
        }

        //! Determine, if time is positive
        /*!
         * Checks sum of seconds and nanoseconds to determine the "overall" sign.
         *
         * \remark Zero is considered as "positive".
         *
         * \return `true` if time is positive (or zero), `false` if time is negative
         */
        inline bool isPositive() const
        {
            // Get simplified version of this time
            // (to ensure, that seconds and nanoseconds have the same sign and nanoseconds does not hit its limits)
            Time simplifiedTime(m_seconds, m_nanoSeconds);
            simplifiedTime.simplify();

            // Check if seconds or nanoseconds are representative
            if (simplifiedTime.m_seconds != 0) {
                // ...seconds are representative (dominating)!
                if (simplifiedTime.m_seconds < 0)
                    return false; // Negative!
                else
                    return true; // Positive!
            } else {
                // ...nanoseconds are representative (dominating)!
                if (simplifiedTime.m_nanoSeconds < 0)
                    return false; // Negative!
                else
                    return true; // Positive!
            }
        }

        //! Determine, if time is negative
        /*!
         * Complement of \ref isPositive().
         * \return `true` if time is negative, `false` if time is positive (or zero)
         */
        inline bool isNegative() const { return !isPositive(); }

        //! Get current system time as Unix Epoch time
        /*!
         * The time is described relative to 00:00:00 UTC on 1 January 1970 (Unix Epoch). Calls \ref PlatformHelper::currentTime().
         * \return Time instance corresponding to current system time
         */
        static inline Time currentTime()
        {
            // Initialize helpers
            const auto platform = PlatformHelperFactory::create();

            // Get system time with highest possible precision
            Time systemTime;
            platform->currentTime(systemTime.m_seconds, systemTime.m_nanoSeconds);

            // Pass back time instance
            return systemTime;
        }

        //! Get resolution of specified clock (theoretic)
        /*!
         * \warning The resolution is reported by the clock and may not coincide with the real resolution!
         *
         * \param [in] sourceClock The clock to get the resolution from.
         * \return Time instance containing the resolution of the specified clock.
         */
        static inline Time getClockResolution(const clockid_t& sourceClock = CLOCK_REALTIME)
        {
            // Get time specification from clock
            timespec timeSpecification;
            clock_getres(sourceClock, &timeSpecification);

            // Create new time instance from time specification and return it
            return Time(timeSpecification);
        }

        //! Evaluate resolution of clock (real)
        /*!
         * Performs various time measurements to identify the real resolution of time measurement.
         *
         * \warning In case a zero resolution is measured this indicates the the real resolution of the clock is lower than the execution time to get the current time. In this case evaluating the clock resolution is not possible.
         *
         * \param [in] samples Count of evaluated samples to find the minimal resolution.
         * \return Time instance containing the resolution.
         */
        static inline Time evaluateClockResolution(const uint64_t& samples = 10)
        {
            // Initialize helpers
            Time minimumSample; // Minimal measured resolution

            // Loop over samples
            for (uint64_t i = 0; i < samples; i++) {
                // Perform measurement
                const Time startTime = currentTime();
                const Time stopTime = currentTime();
                const Time currentSample = stopTime - startTime;

                // Check, if this is the first measurement
                if (i == 0) {
                    // ...yes -> use this as current minimum
                    minimumSample = currentSample;
                } else {
                    // Compare to current minimum
                    if (currentSample < minimumSample)
                        minimumSample = currentSample;
                }
            }

            // Pass back minimal sample
            return minimumSample;
        }

        //! Tell the calling thread to sleep for the specified time
        /*!
         * \warning In case of \p blockingWaiting = `true` **nothing** can abort the process of waiting!
         *
         * \param [in] delay The desired duration to sleep
         * \param [in] blockingWaiting If `true` a blocking loop is called (high accuracy, high CPU usage), if `false` the system call `nanosleep` is called (low accuracy, low CPU usage)
         */
        static inline void sleep(const Time& delay, const bool& blockingWaiting = false)
        {
            // Initialize helpers
            const auto platform = PlatformHelperFactory::create();

            // Check waiting type
            if (blockingWaiting == true) {
                // "High-performance waiting"
                core::Time beginTime = currentTime();
                while (currentTime() - beginTime < delay) {
                    // Do nothing
                }
            } else {
                // "Low-performance waiting"
                platform->waitFor(delay.toTimeSpec());
            }
        }

        //! Tell the calling thread to sleep for the specified time
        /*!
         * \warning In case of \p blockingWaiting = `true` **nothing** can abort the process of waiting!
         *
         * \param [in] delay The desired duration [s] to sleep
         * \param [in] blockingWaiting If `true` a blocking loop is called (high accuracy, high CPU usage), if `false` the system call `nanosleep` is called (low accuracy, low CPU usage)
         */
        static inline void sleep(const double& delay, const bool& blockingWaiting = false) { sleep(Time(delay), blockingWaiting); }

        // Encoding
        // --------
        //! Encode as string (interpreted as duration)
        /*!
         * Encodes the time as a high-precision floating point number (difference between two time points).
         * \param [in] precision The precision for the output of nanoseconds (count of decimal digits, maximum=9).
         * \param [in] decimalDelimiter The delimiter character used between seconds and decimals. Only used for \p precision > 0.
         * \return String respresentation of the time (interpreted as duration)
         */
        std::string encodeToDurationString(const uint8_t& precision = 9, const char& decimalDelimiter = '.') const
        {
            // Initialize helpers
            static const size_t bufferSize = 50; // Maximum size of character buffers (including null-termination)
            char resultBuffer[bufferSize]; // Buffer containing the resulting char array
            size_t resultBufferLength = 0; // Current length of the contents in the buffer

            // Get simplified time representation
            Time simplifiedTime(m_seconds, m_nanoSeconds);
            simplifiedTime.simplify();

            // Get unsigned version (and remember sign)
            bool positive = simplifiedTime.isPositive();
            if (simplifiedTime.m_seconds < 0)
                simplifiedTime.m_seconds = -simplifiedTime.m_seconds;
            if (simplifiedTime.m_nanoSeconds < 0)
                simplifiedTime.m_nanoSeconds = -simplifiedTime.m_nanoSeconds;

            // Print sign to result buffer
            if (positive == true)
                resultBuffer[0] = '+';
            else
                resultBuffer[0] = '-';
            resultBufferLength++;

            // Print "seconds" to result buffer
            int printedCharacters = ::snprintf(&(resultBuffer[1]), bufferSize - 1, "%u", (unsigned int)simplifiedTime.m_seconds);
            if (printedCharacters < 0) {
                assert(false);
                return ""; // An error occured -> abort
            }
            resultBufferLength += printedCharacters;

            // Check if output of decimals is requested and result buffer is big enough
            if (precision > 0 && (bufferSize - resultBufferLength - (1 + 1) /* <-- null-termination + decimal delimiter */) >= precision) {
                // Print decimal delimiter
                resultBuffer[resultBufferLength] = decimalDelimiter;
                resultBufferLength++;

                // Create buffer for decimals and write decimals to it
                char decimalBuffer[bufferSize];
                int decimalBufferLength = ::snprintf(decimalBuffer, bufferSize, "%u", (unsigned int)simplifiedTime.m_nanoSeconds);
                if (decimalBufferLength < 0) {
                    assert(false);
                    return ""; // An error occured -> abort
                }
                if (decimalBufferLength > precision)
                    decimalBufferLength = precision; // Cut off unneccessary digits

                // Get count of leading zeros to add
                int leadingDecimalZeros = precision - decimalBufferLength;

                // Add leading zeros to result buffer
                for (int i = 0; i < leadingDecimalZeros; i++)
                    resultBuffer[resultBufferLength + i] = '0';
                resultBufferLength += leadingDecimalZeros;

                // Write decimal buffer to result buffer
                for (int i = 0; i < decimalBufferLength; i++)
                    resultBuffer[resultBufferLength + i] = decimalBuffer[i];
                resultBufferLength += decimalBufferLength;

                // Add null-termination
                resultBuffer[resultBufferLength] = '\0';
            }

            // Convert to string and pass back
            return std::string(resultBuffer);
        }

        //! Encode as string (interpreted as datetime relative to Unix Epoch)
        /*!
         * Uses internally the `strftime` function provided by the standard libraries. Unfortunately `strftime` only supports
         * a precision on the level of seconds. However the functionality is expanded by passing a precision specification to this
         * function (\p precision).
         *
         * \attention The additional parameter \p precision only works as expected in case seconds are at the end of \p format. Furthermore
         * it only works for positive times (i.e. for time points **not before** Unix Epoch). In this case it is possible to append
         * decimals just at the end of the string. One may also change the default decimal delimiter by \p decimalDelimiter.
         *
         * \param [in] useLocalTimeZone If `true` the local time zone is used as reference. If `false` the UTC time zone is used as reference.
         * \param [in] format Output format (specifies how the time should be printed). See documentation of [`strftime`](http://www.cplusplus.com/reference/ctime/strftime/) for details.
         * \param [in] precision The precision for the output of nanoseconds (count of decimal digits).
         * \param [in] decimalDelimiter The delimiter character used between seconds and decimals. Only used for \p precision > 0.
         * \return String respresentation of the time (interpreted as datetime relative to Unix Epoch)
         */
        std::string encodeToDateTimeString(const bool& useLocalTimeZone, const std::string& format = "%Y-%m-%d_%H-%M-%S", const uint8_t& precision = 0, const char& decimalDelimiter = '.') const
        {
            // Initialize helpers
            static const size_t bufferSize = 50; // Maximum size of character buffers (including null-termination)
            char resultBuffer[bufferSize]; // Buffer containing the resulting char array
            size_t resultBufferLength = 0; // Current length of the contents in the buffer

            // Get simplified time representation
            Time simplifiedTime(m_seconds, m_nanoSeconds);
            simplifiedTime.simplify();

            // Create low resolution time (precision only on level of seconds)
            time_t lowResolutionTime = static_cast<time_t>(simplifiedTime.m_seconds);

            // Print low resolution time to result buffer
            size_t printedCharacters = 0;
            if (useLocalTimeZone == true)
                printedCharacters = strftime(resultBuffer, bufferSize, format.c_str(), localtime(&lowResolutionTime));
            else
                printedCharacters = strftime(resultBuffer, bufferSize, format.c_str(), gmtime(&lowResolutionTime));
            if (printedCharacters == 0) {
                assert(false);
                return ""; // An error occured -> abort
            }
            resultBufferLength += printedCharacters;

            // Check if output of decimals is requested and result buffer is big enough and time is positive
            if (precision > 0 && (bufferSize - resultBufferLength - (1 + 1) /* <-- null-termination + decimal delimiter */) >= precision && simplifiedTime.isPositive() == true) {
                // Print decimal delimiter
                resultBuffer[resultBufferLength] = decimalDelimiter;
                resultBufferLength++;

                // Create buffer for decimals and write decimals to it
                char decimalBuffer[bufferSize];
                int decimalBufferLength = ::snprintf(decimalBuffer, bufferSize, "%u", (unsigned int)simplifiedTime.m_nanoSeconds);
                if (decimalBufferLength < 0) {
                    assert(false);
                    return ""; // An error occured -> abort
                }
                if (decimalBufferLength > precision)
                    decimalBufferLength = precision; // Cut off unneccessary digits

                // Get count of leading zeros to add
                int leadingDecimalZeros = precision - decimalBufferLength;

                // Add leading zeros to result buffer
                for (int i = 0; i < leadingDecimalZeros; i++)
                    resultBuffer[resultBufferLength + i] = '0';
                resultBufferLength += leadingDecimalZeros;

                // Write decimal buffer to result buffer
                for (int i = 0; i < decimalBufferLength; i++)
                    resultBuffer[resultBufferLength + i] = decimalBuffer[i];
                resultBufferLength += decimalBufferLength;

                // Add null-termination
                resultBuffer[resultBufferLength] = '\0';
            }

            // Convert to string and pass back
            return std::string(resultBuffer);
        }
    };

    // Operator definitions
    // --------------------
    //! Addition operator (Time + Time)
    /*! Performs a simplification after evaluation. */
    static inline Time operator+(const Time& first, const Time& second)
    {
        // Create new time object for the result
        Time result(first.m_seconds + second.m_seconds, first.m_nanoSeconds + second.m_nanoSeconds);

        // Simplify result
        result.simplify();

        // Pass back result
        return result;
    }

    //! Addition operator (Time + double)
    /*! Performs a simplification after evaluation. */
    static inline Time operator+(const Time& time, const double& scalar) { return time + Time(scalar); }

    //! Addition operator (double + Time)
    /*! Performs a simplification after evaluation. */
    static inline Time operator+(const double& scalar, const Time& time) { return Time(scalar) + time; }

    //! Addition operator (Time += Time)
    /*! Performs a simplification after evaluation. */
    static inline Time& operator+=(Time& first, const Time& second)
    {
        // Evaluate
        first = first + second;

        // Pass back result
        return first;
    }

    //! Addition operator (Time += double)
    /*! Performs a simplification after evaluation. */
    static inline Time& operator+=(Time& time, const double& scalar)
    {
        // Evaluate
        time = time + Time(scalar);

        // Pass back result
        return time;
    }

    //! Subtraction operator (Time - Time)
    /*! Performs a simplification after evaluation. */
    static inline Time operator-(const Time& first, const Time& second)
    {
        // Create new time object for the result
        Time result(first.m_seconds - second.m_seconds, first.m_nanoSeconds - second.m_nanoSeconds);

        // Simplify result
        result.simplify();

        // Pass back result
        return result;
    }

    //! Subtraction operator (Time - double)
    /*! Performs a simplification after evaluation. */
    static inline Time operator-(const Time& time, const double& scalar) { return time - Time(scalar); }

    //! Subtraction operator (double - Time)
    /*! Performs a simplification after evaluation. */
    static inline Time operator-(const double& scalar, const Time& time) { return Time(scalar) - time; }

    //! Subtraction operator (Time -= Time)
    /*! Performs a simplification after evaluation. */
    static inline Time& operator-=(Time& first, const Time& second)
    {
        // Evaluate
        first = first - second;

        // Pass back result
        return first;
    }

    //! Subtraction operator (Time -= double)
    /*! Performs a simplification after evaluation. */
    static inline Time& operator-=(Time& time, const double& scalar)
    {
        // Evaluate
        time = time - Time(scalar);

        // Pass back result
        return time;
    }

    //! Multiplication operator (Time * double)
    /*! Performs a simplification after evaluation. */
    static inline Time operator*(const Time& time, const double& scalar)
    {
        // Multiply components
        double seconds = time.m_seconds * scalar;
        double nanoSeconds = time.m_nanoSeconds * scalar;

        // Cut off decimals of seconds
        const double roundedSeconds = ::round(seconds);

        // Get residuum of seconds which will be cut off
        const double residuum = seconds - roundedSeconds;

        // Round seconds to cut off residuum
        seconds = roundedSeconds;

        // Add residuum to nanoseconds
        nanoSeconds += residuum * 1e9;

        // Round nanoseconds to avoid numerical error
        nanoSeconds = ::round(nanoSeconds);

        // Create new time object for the result
        Time result(seconds, nanoSeconds);

        // Simplify result
        result.simplify();

        // Pass back result
        return result;
    }

    //! Multiplication operator (double * Time)
    /*! Performs a simplification after evaluation. */
    static inline Time operator*(const double& scalar, const Time& time) { return time * scalar; }

    //! Multiplication operator (Time *= double)
    /*! Performs a simplification after evaluation. */
    static inline Time& operator*=(Time& time, const double& scalar)
    {
        // Evaluate
        time = time * scalar;

        // Pass back result
        return time;
    }

    //! Division operator (Time / double)
    /*! Performs a simplification after evaluation. */
    static inline Time operator/(const Time& time, const double& scalar) { return time * (1.0 / scalar); }

    //! Division operator (double / Time)
    /*! Performs a simplification after evaluation. */
    static inline Time operator/(const double& scalar, const Time& time) { return Time(scalar / time.toDouble()); }

    //! Division operator (Time /= double)
    /*! Performs a simplification after evaluation. */
    static inline Time& operator/=(Time& time, const double& scalar)
    {
        // Evaluate
        time = time * (1.0 / scalar);

        // Pass back result
        return time;
    }

    //! Equality operator (Time == Time)
    /*! Compares simplified versions */
    static inline bool operator==(const Time& first, const Time& second)
    {
        // Create simplified versions of both times
        Time simplifiedFirst(first);
        simplifiedFirst.simplify();
        Time simplifiedSecond(second);
        simplifiedSecond.simplify();

        // Compare members
        if (simplifiedFirst.m_seconds == simplifiedSecond.m_seconds && simplifiedFirst.m_nanoSeconds == simplifiedSecond.m_nanoSeconds)
            return true;
        else
            return false;
    }

    //! Equality operator (Time == double)
    /*! Compares simplified versions */
    static inline bool operator==(const Time& time, const double& scalar) { return time == Time(scalar); }

    //! Equality operator (double == Time)
    /*! Compares simplified versions */
    static inline bool operator==(const double& scalar, const Time& time) { return time == scalar; }

    //! Inequality operator (Time != Time)
    /*! Compares simplified versions */
    static inline bool operator!=(const Time& first, const Time& second)
    {
        return !(first == second);
    }

    //! Inequality operator (Time != double)
    /*! Compares simplified versions */
    static inline bool operator!=(const Time& time, const double& scalar) { return time != Time(scalar); }

    //! Inequality operator (double != Time)
    /*! Compares simplified versions */
    static inline bool operator!=(const double& scalar, const Time& time) { return time != scalar; }

    //! Greater-than operator (Time > Time)
    /*! Compares simplified versions */
    static inline bool operator>(const Time& first, const Time& second)
    {
        // Create simplified versions of both times
        Time simplifiedFirst(first);
        simplifiedFirst.simplify();
        Time simplifiedSecond(second);
        simplifiedSecond.simplify();

        // first level: compare seconds...
        if (simplifiedFirst.m_seconds > simplifiedSecond.m_seconds)
            return true;
        else if (simplifiedFirst.m_seconds == simplifiedSecond.m_seconds) {
            // second level: compare nanoseconds...
            if (simplifiedFirst.m_nanoSeconds > simplifiedSecond.m_nanoSeconds)
                return true;
            else
                return false;
        } else
            return false;
    }

    //! Greater-than operator (Time > double)
    /*! Compares simplified versions */
    static inline bool operator>(const Time& time, const double& scalar) { return time > Time(scalar); }

    //! Greater-than operator (double > Time)
    /*! Compares simplified versions */
    static inline bool operator>(const double& scalar, const Time& time) { return Time(scalar) > time; }

    //! Greater-or-equal operator (Time >= Time)
    /*! Compares simplified versions */
    static inline bool operator>=(const Time& first, const Time& second)
    {
        // Create simplified versions of both times
        Time simplifiedFirst(first);
        simplifiedFirst.simplify();
        Time simplifiedSecond(second);
        simplifiedSecond.simplify();

        // first level: compare seconds...
        if (simplifiedFirst.m_seconds > simplifiedSecond.m_seconds)
            return true;
        else if (simplifiedFirst.m_seconds == simplifiedSecond.m_seconds) {
            // second level: compare nanoseconds...
            if (simplifiedFirst.m_nanoSeconds >= simplifiedSecond.m_nanoSeconds)
                return true;
            else
                return false;
        } else
            return false;
    }

    //! Greater-or-equal operator (Time > double)
    /*! Compares simplified versions */
    static inline bool operator>=(const Time& time, const double& scalar) { return time >= Time(scalar); }

    //! Greater-or-equal operator (double > Time)
    /*! Compares simplified versions */
    static inline bool operator>=(const double& scalar, const Time& time) { return Time(scalar) >= time; }

    //! Lesser-than operator (Time < Time)
    /*! Compares simplified versions */
    static inline bool operator<(const Time& first, const Time& second)
    {
        // Create simplified versions of both times
        Time simplifiedFirst(first);
        simplifiedFirst.simplify();
        Time simplifiedSecond(second);
        simplifiedSecond.simplify();

        // first level: compare seconds...
        if (simplifiedFirst.m_seconds < simplifiedSecond.m_seconds)
            return true;
        else if (simplifiedFirst.m_seconds == simplifiedSecond.m_seconds) {
            // second level: compare nanoseconds...
            if (simplifiedFirst.m_nanoSeconds < simplifiedSecond.m_nanoSeconds)
                return true;
            else
                return false;
        } else
            return false;
    }

    //! Lesser-than operator (Time < double)
    /*! Compares simplified versions */
    static inline bool operator<(const Time& time, const double& scalar) { return time < Time(scalar); }

    //! Lesser-than operator (double < Time)
    /*! Compares simplified versions */
    static inline bool operator<(const double& scalar, const Time& time) { return Time(scalar) < time; }

    //! Lesser-or-equal operator (Time <= Time)
    /*! Compares simplified versions */
    static inline bool operator<=(const Time& first, const Time& second)
    {
        // Create simplified versions of both times
        Time simplifiedFirst(first);
        simplifiedFirst.simplify();
        Time simplifiedSecond(second);
        simplifiedSecond.simplify();

        // first level: compare seconds...
        if (simplifiedFirst.m_seconds < simplifiedSecond.m_seconds)
            return true;
        else if (simplifiedFirst.m_seconds == simplifiedSecond.m_seconds) {
            // second level: compare nanoseconds...
            if (simplifiedFirst.m_nanoSeconds <= simplifiedSecond.m_nanoSeconds)
                return true;
            else
                return false;
        } else
            return false;
    }

    //! Lesser-or-equal operator (Time <= double)
    /*! Compares simplified versions */
    static inline bool operator<=(const Time& time, const double& scalar) { return time <= Time(scalar); }

    //! Lesser-or-equal operator (double <= Time)
    /*! Compares simplified versions */
    static inline bool operator<=(const double& scalar, const Time& time) { return Time(scalar) <= time; }

    //! \}
} // namespace core
} // namespace broccoli
