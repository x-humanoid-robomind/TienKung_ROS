/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "IIRFilter.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace control {
    /*!
     * \brief Implements a second order digital low-pass filter
     * \ingroup broccoli_control_filters
     *
     * The filter implements a time-discretized version of
     * the following transfer function
     * \f[
     * G(s) = \frac{1}{T^2s^2 + 2dTs + 1},
     * \f]
     * with the filter time constant \f$T\f$, and damping ratio \f$d\f$. The filter is
     * discretized via tustin transformation and does NOT use frequency pre-warping. The digital filter will match
     * \f$G(s)\f$ for low frequencies and may deviate in phase and amplitude for higher frequencies when \f$T\f$ is
     * close to the sample time \f$dt\f$. For best results ensure \f$T \ge 10 dt\f$.
     * For \f$T = 0\f$ the filter operates in passthru mode, i.e. the input value is directly used as output.
     *
     * \tparam T The signal data type
     */
    template <typename T>
    class SecondOrderLowPass : public IIRFilter<T, 2, 2> {
    public:
        using Base = IIRFilter<T, 2, 2>;

        //! Default constructor
        SecondOrderLowPass()
            : Base()
        {
        }

        /*!
         * \brief Prototype initialization constructor for complex data types
         * \param prototype Data type instance used for (re-)initialization of the filter output value
         */
        explicit SecondOrderLowPass(const T& prototype)
            : Base(prototype)
        {
        }

        /*!
         * \brief Full initialization constructor
         * \param prototype Data type instance used for (re-)initialization of the filter output value
         * \param dt The sample time in seconds
         * \param cutOffFrequencyInHz The filter cutoff frequency in Hertz. If zero is passed, the filter is deactivated (passthru mode)
         * \param dampingRatio The damping ratio \f$d \ge 0\f$.
         */
        SecondOrderLowPass(const T& prototype, const double& dt, const double& cutOffFrequencyInHz, const double& dampingRatio)
            : Base(prototype)
        {
            init(dt, cutOffFrequencyInHz, dampingRatio);
        }

        /*!
         * \brief (Re-)initialize the filter parameters from cutoff frequency and damping ratio
         * \param dt Sample time in seconds
         * \param cutOffFrequencyInHz The filter cutoff frequency in Hertz. If zero is passed, the filter is deactivated (passthru mode)
         * \param dampingRatio The damping ratio \f$d \ge 0\f$.
         * \returns A reference to this instance
         */
        auto& init(const double& dt, const double& cutOffFrequencyInHz, const double& dampingRatio = 1.0)
        {
            assert(cutOffFrequencyInHz >= 0.0 && "The filter cut-off frequency must be greater than or equal to zero!");

            if (cutOffFrequencyInHz <= 0.0) {
                return initTimeConstant(dt, 0.0, dampingRatio);
            }

            return initTimeConstant(dt, 1.0 / (2.0 * M_PI * cutOffFrequencyInHz), dampingRatio);
        }

        /*!
         * \brief (Re-)initialize the filter parameters from time constant and damping ratio
         * \param dt Sample time in seconds
         * \param timeConstant The filter time constant \f$T\f$ in seconds. If zero is passed, the filter is deactivated (passthru mode)
         * \param dampingRatio The damping ratio \f$d \ge 0\f$.
         * \returns A reference to this instance
         */
        auto& initTimeConstant(const double& dt, double timeConstant, double dampingRatio = 1.0)
        {
            assert(dt > 0.0 && "The filter sample time must be greater than zero!");
            assert(dampingRatio >= 0.0 && "The filter damping ratio must be greater or equal to zero");

            if (dampingRatio < 0.0) {
                dampingRatio = 0.0;
            }

            if (dt <= 0.0 || timeConstant <= 0) {
                Base::setupForPassThru();
                return *this;
            }

            // Stability margin
            if (timeConstant < dt / 2.0) {
                assert(false && "The filter time constant must be greater or equal to half the sample time");
                timeConstant = dt / 2.0;
            }

            // calculate filter parameters
            Base::init({ dt * dt,
                           2 * dt * dt, dt * dt },
                { 4 * timeConstant * timeConstant + 4 * dampingRatio * timeConstant * dt + dt * dt,
                    2 * dt * dt - 8 * timeConstant * timeConstant,
                    4 * timeConstant * timeConstant - 4 * dampingRatio * timeConstant * dt + dt * dt },
                dt);
            return *this;
        }
    };
} // namespace control
} // namespace broccoli
