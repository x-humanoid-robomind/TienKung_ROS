/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "SignalSource.hpp"
#include <cmath>
#include <vector>

namespace broccoli {
namespace control {
    /*!
     * \brief Multi-sine signal source with minimal Crest factor
     * \ingroup broccoli_control_sources
     *
     * The sine waves are superposed using Schroeder-Phases, which minimizes the maximum amplitude of the
     * resulting signal (Crest-Factor). For more details refer to 
     * "M. Schroeder, Synthesis of low-peak-factor signals and binary sequences with low autocorrelation (Corresp.), in IEEE Transactions on Information Theory, vol. 16, no. 1, pp. 85-89, January 1970.".
     * This signal is a good excitation source and may be used for system identification tasks.
     * \tparam Scalar The scalar output data type
     */
    template <typename Scalar = double>
    class MultiSine : public SignalSource<MultiSine<Scalar>> {
    public:
        /*!
         * \brief Constructor
         * \param dt The sample time in seconds
         */
        explicit MultiSine(const double& dt)
            : SignalSource<MultiSine<Scalar>>(false)
            , m_dt(dt)
            , m_value(Scalar(0))
        {
        }

        /*!
         * \brief Setup single-sine output for given frequency
         * \param freq The sine frequency in Hz
         * \returns A reference to this instance
         */
        MultiSine<Scalar>& singleSine(const double& freq)
        {
            m_numberOfSines = 1;
            m_minFreq = freq;
            m_freqStep = 0.0;
            return *this;
        }

        /*!
         * \brief Setup multi-sine output with uniform distribution between a minimum and a maximum frequency.
         *
         * \param minFreq Minimum frequency in Hz
         * \param maxFreq Maximum frequency in Hz
         * \param numberOfSines Number of sines between min and max frequency
         * \returns A reference to this instance
         */
        MultiSine<Scalar>& uniformSineDistribution(const double& minFreq, const double& maxFreq, const std::size_t& numberOfSines)
        {
            m_numberOfSines = numberOfSines;
            m_minFreq = minFreq;

            if (m_numberOfSines > 1) {
                m_freqStep = (maxFreq - minFreq) / (m_numberOfSines - 1);
            } else {
                m_freqStep = 0;
            }
            return *this;
        }

        /*!
         * \brief Set the amplitude of the sines
         * \param amplitude Amplitude (max/min value) of the sines.
         * \returns A reference to this instance
         */
        MultiSine<Scalar>& setAmplitude(const Scalar& amplitude)
        {
            m_amplitude = amplitude;
            return *this;
        }

        //! Returns the number of sines currently used in the signal
        const std::size_t& numberOfSines() const
        {
            return m_numberOfSines;
        }

        //! Returns a list of all sine frequencies in Hz
        std::vector<double> frequencyList() const
        {
            std::vector<double> list;
            list.reserve(m_numberOfSines);

            for (std::size_t i = 0; i < m_numberOfSines; i++) {
                list.push_back(m_minFreq + m_freqStep * i);
            }

            return list;
        }

        //! \copydoc SignalSource::computeOutput()
        MultiSine<Scalar>& computeOutput(const double& time)
        {
            m_value = Scalar(0);

            if (!this->isActive())
                return *this;

            for (std::size_t i = 0; i < m_numberOfSines; i++) {
                double phase = (-M_PI * (i + 1) * i) / m_numberOfSines;

                m_value += m_amplitude * cos(2 * M_PI * (m_minFreq + m_freqStep * i) * time + phase);
            }

            return *this;
        }

        //! \copydoc SignalBase::value()
        const Scalar& value() const
        {
            return m_value;
        }

        //! \copydoc SignalBase::sampleTime()
        const double& sampleTime() const
        {
            return m_dt;
        }

    private:
        //! The sample time
        const double m_dt;

        //! The resulting signal value
        Scalar m_value;

        //! Amplitude of the sine waves
        Scalar m_amplitude = Scalar(0);

        //! Minimum frequency in Hz (uniform distribution)
        double m_minFreq = 0.0;

        //! Distance between two sines (uniform distribution)
        double m_freqStep = 0.0;

        //! The number of sines used
        std::size_t m_numberOfSines = 0;
    };
} // namespace control
} // namespace broccoli
