/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/type_traits.hpp"
#include "../Signal.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace control {
    /*!
     * \brief Implements a simple euler-forward integrator
     * \ingroup broccoli_control_operators
     *
     * Calculates \f$\int_0^t x \; d\tau\f$.
     *
     * \tparam Type The arithmetic type of the output signal
     */
    template <typename Type>
    class Integrator : public SignalBase<Integrator<Type>> {
    public:
        //! Default constructor
        Integrator()
            : m_value(core::Traits<Type>::zero())
        {
        }

        /*!
         * \brief Prototype initialization constructor for complex data types
         * \param prototype Data type instance used for initial value of integrator.
         */
        explicit Integrator(const std::decay_t<Type>& prototype)
            : m_value(prototype)
        {
        }

        /*!
         * \brief Move Constructor.
         *
         * This can be used to integrate on an existing storage.
         * \param value The value to construct from
         */
        explicit Integrator(Type&& value)
            : m_value(std::forward<Type>(value))
        {
        }

        /*!
         * \brief Performs an integration step
         * \param integrand The integrand signal to integrate
         * \return The integrator signal after integration
         */
        auto process(const Signal<std::decay_t<Type>>& integrand)
        {
            assert(integrand.sampleTime() > 0.0 && "Integrand signal must provide valid sample time!");

            if (integrand.sampleTime() > 0.0) {
                m_value += integrand.value() * integrand.sampleTime();
            }
            m_sampleTime = integrand.sampleTime();
            return *this;
        }

        //! Returns the output value of the integrator
        const std::decay_t<Type>& value() const
        {
            return m_value;
        }

        //! Returns the output value of the integrator
        std::decay_t<Type>& value()
        {
            return m_value;
        }

        //! Returns the sample time of the signal in seconds
        const double& sampleTime() const
        {
            return m_sampleTime;
        }

    private:
        //! The current signal output value
        Type m_value;

        //! The sample time of the output signal
        double m_sampleTime = -1.0;
    };
} // namespace control
} // namespace broccoli
