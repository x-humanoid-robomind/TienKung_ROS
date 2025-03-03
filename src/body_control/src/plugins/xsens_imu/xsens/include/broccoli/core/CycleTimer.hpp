/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <algorithm>
#include <cassert>

namespace broccoli {
namespace core {

    /*!
     * \brief Timer for executing certain operations only every xth discrete update cycle
     * \ingroup broccoli_core
     */
    class CycleTimer {
    public:
        /*!
         * \brief Default Constructor. The timer will trigger every cycle.
         */
        CycleTimer()
            : m_cyclesToGo(1)
        {
        }

        /*!
         * \brief Construct from n where every nth cycle triggers the timer
         * \param n The number of cycles until the timer triggers
         */
        explicit CycleTimer(std::size_t n)
            : m_cyclesToGo(n)
        {
        }

        /*!
         * \brief Construct from given duration between timer events and the sample time
         * \param durationInUs The timer duration in microseconds
         * \param sampleTimeInUs The sample time of cycle() calls in microseconds
         */
        CycleTimer(std::size_t durationInUs, std::size_t sampleTimeInUs)
            : m_cyclesToGo(cyclesForDuration(durationInUs, sampleTimeInUs))
        {
        }

        /*!
         * \brief Re-set duration between timer events and the sample time
         * \param durationInUs The timer duration in microseconds
         * \param sampleTimeInUs The sample time of cycle() calls in microseconds
         */
        void setDuration(std::size_t durationInUs, std::size_t sampleTimeInUs) noexcept
        {
            m_cyclesToGo = cyclesForDuration(durationInUs, sampleTimeInUs);
        }

        /*!
         * \brief Cycle the timer
         * \return True if the specified duration elapsed.
         */
        bool cycle() noexcept
        {
            if (m_cycleCounter % m_cyclesToGo == 0) {
                m_cycleCounter = 1;
                return true;
            }
            m_cycleCounter++;
            return false;
        }

        //! Returns the number of cycles for the configured duration
        std::size_t cyclesForDuration() const noexcept { return m_cyclesToGo; }

        //! Reset the timers internal counter and start duration from zero
        void reset() noexcept
        {
            m_cycleCounter = 1;
        }

    protected:
        /*!
         * \brief Internal method to calculate the number of cycles for the duration
         * \param durationInUs The timer duration in microseconds
         * \param sampleTimeInUs The sample time of cycle() calls in microseconds
         * \return The number of cycles equivalent to the specified duration
         */
        static std::size_t cyclesForDuration(std::size_t durationInUs, std::size_t sampleTimeInUs) noexcept
        {
            assert(sampleTimeInUs != 0 && "Sample time may not be zero!");
            assert(durationInUs == 0 || durationInUs >= sampleTimeInUs && "Duration can not be smaller than sample time!");
            return std::max(static_cast<std::size_t>(durationInUs) / sampleTimeInUs,
                std::size_t{ 1 });
        };

        //! The cycles equivalent to the duration
        std::size_t m_cyclesToGo;

        //! The internal cycles counter
        std::size_t m_cycleCounter = 1;
    };
} // namespace core
} // namespace broccoli
