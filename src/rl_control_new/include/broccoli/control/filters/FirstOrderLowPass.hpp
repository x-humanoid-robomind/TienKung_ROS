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
     * \brief Implements a first order digital low-pass filter
     * \ingroup broccoli_control_filters
     *
     * The filter implements a time-discretized version of
     * the following transfer function
     * \f[
     * G(s) = \frac{1}{Ts + 1},
     * \f]
     * with the filter time constant \f$T\f$. The filter is
     * discretized via tustin transformation and does NOT use frequency pre-warping. The digital filter will match
     * \f$G(s)\f$ for low frequencies and may deviate in phase and amplitude for higher frequencies when \f$T\f$ is
     * close to the sample time \f$dt\f$. For best results ensure \f$T \ge 10 dt\f$.
     * For \f$T = 0\f$ the filter operates in passthru mode, i.e. the input value is directly used as output.
     *
     * \tparam T The signal data type
     */
    template <typename T>
    class FirstOrderLowPass : public IIRFilter<T, 1, 1> {
    public:
        using Base = IIRFilter<T, 1, 1>;

        //! Default constructor
        FirstOrderLowPass()
            : Base()
        {
        }

        /*!
         * \brief Prototype initialization constructor for complex data types
         * \param prototype Data type instance used for (re-)initialization of the filter output value
         */
        explicit FirstOrderLowPass(const T& prototype)
            : Base(prototype)
        {
        }

        /*!
         * \brief Full initialization constructor
         * \param prototype Data type instance used for (re-)initialization of the filter output value
         * \param dt Sample time in seconds
         * \param cutOffFrequencyInHz The filter cutoff frequency in Hertz. If zero is passed, the filter is deactivated (passthru mode)
         */
        FirstOrderLowPass(const T& prototype, const double& dt, const double& cutOffFrequencyInHz)
            : Base(prototype)
        {
            init(dt, cutOffFrequencyInHz);
        }

        /*!
         * \brief (Re-)initialize the filter parameters from cutoff frequency
         * \param dt Sample time in seconds
         * \param cutOffFrequencyInHz The filter cutoff frequency in Hertz. If zero is passed, the filter is deactivated (passthru mode)
         * \returns A reference to this instance
         */
        auto& init(const double& dt, const double& cutOffFrequencyInHz)
        {
            assert(cutOffFrequencyInHz >= 0.0 && "The filter cut-off frequency must be greater than or equal to zero!");

            if (cutOffFrequencyInHz <= 0.0) {
                return initTimeConstant(dt, 0.0);
            }

            return initTimeConstant(dt, 1.0 / (2.0 * M_PI * cutOffFrequencyInHz));
        }

        /*!
         * \brief (Re-)initialize the filter parameters from time constant
         * \param dt Sample time in seconds
         * \param timeConstant The filter time constant \f$T\f$ in seconds. If zero is passed, the filter is deactivated (passthru mode)
         * \returns A reference to this instance
         */
        auto& initTimeConstant(const double& dt, double timeConstant)
        {
            assert(dt > 0.0 && "The filter sample time must be greater than zero!");

            if (dt <= 0.0 || timeConstant <= 0.0) {
                Base::setupForPassThru();
                return *this;
            }

            // Stability margin
            if (timeConstant < dt / 2) {
                assert(false && "The filter time constant must be higher than half the sample time");
                timeConstant = dt / 2;
            }

            Base::init({ dt,
                           dt },
                { 2 * timeConstant + dt,
                    -2 * timeConstant + dt },
                dt);

            return *this;
        }
    };
} // namespace control
} // namespace broccoli
