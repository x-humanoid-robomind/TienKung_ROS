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
#include <cassert>
#include <cmath>

namespace broccoli {
namespace control {
    /*!
     * \brief Digital Notch Filter Implementation
     * \ingroup broccoli_control_filters
     *
     * Implements a standard digital notch filter based on tustin transformation of the following continuous-time transfer function
     * \f[
     * G(s) = \frac{T^2 s^2 + 2 d T s + 1}{T_p^2 s^2 + T_p \frac{1}{Q} s + 1} \\
     * T = \frac{1}{2 \pi f_{\text{stop}}} \\
     * T_p = \frac{1}{2 \pi f_{\text{poles}}},
     * \f]
     * with \f$Q\f$ being the sharpness factor, \f$f_{\text{stop}}\f$ the stop frequency,
     * \f$f_{\text{poles}}\f$ the optional pole frequency (by default \f$f_{\text{poles}} = f_{\text{stop}}\f$ ),
     * and \f$d\f$ the damping of the filter. For \f$f_{\text{stop}} = 0\f$ the filter operates in passthru mode, i.e. the input value is directly used as output.
     */
    template <typename T>
    class NotchFilter : public IIRFilter<T, 2, 2> {
    public:
        using Base = IIRFilter<T, 2, 2>;

        //! Default constructor
        NotchFilter()
            : Base()
        {
        }

        /*!
         * \brief Prototype initialization constructor for complex data types
         * \param prototype Data type instance used for (re-)initialization of the filter. This must be a representation of "zero".
         */
        explicit NotchFilter(const T& prototype)
            : Base(prototype)
        {
        }

        /*!
         * \brief (Re)initialize the notch filter parameters.
         * \note This method does not change the filter output.
         *
         * \param dt Sample Time in seconds.
         * \param stopFrequencyHz Stop Frequency \f$f_{\text{stop}}\f$ in Hz. If zero is passed, the filter is deactivated (passthru mode)
         * \param Q Sharpness-factor \f$Q\f$ of the notch. If you increase this value, the notch gets sharper but the impulse response also gets longer.
         * \param d Damping of the filter zeros \f$d\f$. This determines the depth of the filter notch.
         * \param poleFrequencyHz Frequency of the poles \f$f_{\text{poles}}\f$ in Hz. This determines the type of notch filter. For \f$f_{\text{poles}} > f_{\text{stop}}\f$ it is a high-pass notch.
         *        For \f$f_{\text{stop}} = f_{\text{poles}}\f$ (default) the standard notch. \f$f_{\text{poles}} < f_{\text{stop}}\f$ defines a low-pass notch.
         * \returns A reference to this instance
         */
        auto& init(const double& dt, const double& stopFrequencyHz, double Q, double d = 0.0, const double& poleFrequencyHz = 0.0)
        {
            assert(dt > 0.0 && "The filter sample time must be greater than zero!");
            assert(Q > 0 && "The filter sharpness must be greater than zero!");
            assert(d >= 0 && "The filter damping must be greater than or equal to zero!");
            assert(poleFrequencyHz >= 0.0 && "The pole frequency must be greater than or equal to zero!");

            if (Q <= 0) {
                Q = 1.0;
            }
            if (d < 0.0) {
                d = 0.0;
            }

            if (dt <= 0.0 || stopFrequencyHz <= 0.0) {
                Base::setupForPassThru();
                return *this;
            }

            // calc the time constants for the filter
            double T0 = 1.0 / (2.0 * M_PI * stopFrequencyHz);
            double Tp = T0;

            if (poleFrequencyHz > 0.0) {
                Tp = 1.0 / (2.0 * M_PI * poleFrequencyHz);
            }

            Base::init({ Q * (4 * T0 * T0 + 4 * d * T0 * dt + dt * dt),
                           -Q * (8 * T0 * T0 - 2 * dt * dt),
                           Q * (4 * T0 * T0 - 4 * d * T0 * dt + dt * dt) },
                { 4 * Q * Tp * Tp + 2 * dt * Tp + Q * dt * dt,
                    2 * Q * dt * dt - 8 * Q * Tp * Tp,
                    -2 * Tp * dt + Q * dt * dt + 4 * Q * Tp * Tp },
                dt);

            return *this;
        }
    };
} // namespace control
} // namespace broccoli
