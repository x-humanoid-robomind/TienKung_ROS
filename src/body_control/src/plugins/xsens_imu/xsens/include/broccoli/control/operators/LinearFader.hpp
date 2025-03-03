/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/math.hpp"
#include "Integrator.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif
#include <cassert>

namespace broccoli {
namespace control {
    /*!
     * \brief Implements a linear fader to fade-in or -out a value-based signal
     * \ingroup broccoli_control_operators
     *
     * The output of the fader is given by
     * \f[
     * out(t) = f(t) \cdot in(t),
     * \f]
     * with \f$f(t) = \frac{1.0}{T} t\f$ or \f$f(t) = 1.0 - \frac{1.0}{T} t\f$, where \f$ f(t)\f$ is clamped to \f$[0.0,1.0]\f$,
     * and \f$T\f$ being a timespan.
     *
     */
    class LinearFader {
    public:
        /*!
         * \brief Default constructor
         *
         * Sets initial fading value to zero.
         */
        LinearFader() = default;

        /*!
         * \brief Construct based on given initial fading value
         *
         * \param initialFadeValue The initial fading value \f$f(0)\f$ (0.0 to 1.0)
         */
        LinearFader(const double& initialFadeValue)
            : m_state(broccoli::core::math::clamp(initialFadeValue, 0.0, 1.0))
        {
            assert(initialFadeValue >= 0.0);
            assert(initialFadeValue <= 1.0);
        }

        /*!
         * \brief (Re-) initialize the fader
         * \param initialFadeValue The initial fading value \f$f(0)\f$ (0.0 to 1.0)
         */
        void init(const double& initialFadeValue)
        {
            assert(initialFadeValue >= 0.0);
            assert(initialFadeValue <= 1.0);

            m_state.value() = broccoli::core::math::clamp(initialFadeValue, 0.0, 1.0);
            m_mode = FaderMode::IDLE;
        }

        /*!
         * \brief (Re-) initialize the fader and the time span.
         * \param initialFadeValue The initial fading value \f$f(0)\f$ (0.0 to 1.0)
         * \param timespan The fading timespan \f$T\f$ in seconds
         */
        void init(const double& initialFadeValue, const double& timespan)
        {
            init(initialFadeValue);
            setTimespan(timespan);
        }

        /*!
         * \brief Set the timespan for the fade-in/-out process
         * \param timespan The fading timespan \f$T\f$ in seconds
         */
        void setTimespan(const double& timespan)
        {
            assert(timespan > 0.0);
            if (timespan > 0.0)
                m_timespan = timespan;
        }

        /*!
         * \brief Computes the internal fading value
         *
         * \attention Must only be called once per time step.
         * \param sampleTime The sample time in seconds
         * \returns The fading value \f$f(t)\f$ (0...1)
         */
        double process(const double& sampleTime)
        {
            assert(sampleTime > 0.0 && "filter input must provide valid sample time!");

            if (m_mode == FaderMode::FADEIN) {

                m_state.process(mapSignal(1.0 / m_timespan, sampleTime));

                if (m_state.value() >= 1.0) {
                    m_mode = FaderMode::IDLE;
                    m_state.value() = 1.0;
                }

            } else if (m_mode == FaderMode::FADEOUT) {

                m_state.process(mapSignal(-1.0 / m_timespan, sampleTime));

                if (m_state.value() <= 0.0) {
                    m_mode = FaderMode::IDLE;
                    m_state.value() = 0.0;
                }
            }

            return m_state.value();
        }

        /*!
         * \brief Returns the faded output \f$f(t) \cdot in(t)\f$ for the given input signal
         * \param input The input Signal \f$in(t)\f$
         * \return The output Signal expression \f$out(t)\f$
         */
        template <typename TypeIn>
        auto fadeSignal(TypeIn&& input)
        {
            return m_state * std::forward<TypeIn>(input);
        }

        /*!
         * \brief Computes the internal fading value and returns the faded output for the given input signal
         *
         * \attention Must only be called once per time step.
         * \param input The input Signal \f$in(t)\f$
         * \return The output Signal expression \f$out(t)\f$
         */
        template <typename TypeIn>
        auto processSignal(TypeIn&& input)
        {
            process(input.sampleTime());
            return fadeSignal(std::forward<TypeIn>(input));
        }

        /*!
         * \brief Start (or continue) fade-in.
         *
         * The fader switches to idle mode (keep last fade value) once
         * \f$f(t) = 1.0\f$ is reached.
         */
        void fadeIn()
        {
            m_mode = FaderMode::FADEIN;
        }

        /*!
         * \brief Start (or continue) fade-out.
         *
         * The fader switches to idle mode (keep last fade value) once
         * \f$f(t) = 0.0\f$ is reached.
         */
        void fadeOut()
        {
            m_mode = FaderMode::FADEOUT;
        }

        //! Switch to idle mode (keep current fading value)
        void idle()
        {
            m_mode = FaderMode::IDLE;
        }

        //! Return true if the fader is idling
        bool isIdling() const
        {
            return m_mode == FaderMode::IDLE;
        }

        /*!
         * \brief Sets the fading value
         *
         * \note This does not change the state of the fader.
         * For example, if fadeIn() was called the fader will still continue to fade in from the
         * new fading value.
         *
         * \param fading The fading value \f$f(t)\f$ (0...1)
         */
        void setFadingValue(const double& fading)
        {
            m_state.value() = core::math::clamp(fading, 0.0, 1.0);
        }

        //! Returns the current fading value \f$f(t)\f$ (0...1)
        const double& fadingValue() const
        {
            return m_state.value();
        }

        //! Set fading value to zero immediately and switches to idle
        void switchOff()
        {
            m_state.value() = 0.0;
            idle();
        }

        //! Set fading value to one immediately and switches to idle
        void switchOn()
        {
            m_state.value() = 1.0;
            idle();
        }

    private:
        //! Stores the timespan, with which the fading is done
        double m_timespan = 1.0;

        //! Fading state
        Integrator<double> m_state{ 0.0 };

        //! Type enum for fade modes
        enum class FaderMode {
            IDLE,
            FADEIN,
            FADEOUT
        };

        //! current mode of the fader
        FaderMode m_mode = FaderMode::IDLE;

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
    };
} // namespace control
} // namespace broccoli
