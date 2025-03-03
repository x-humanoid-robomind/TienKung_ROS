/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "Integrator.hpp"

namespace broccoli {
namespace control {
    /*!
     * \brief Implements a constrained signal integrator
     * \ingroup broccoli_control_operators
     *
     * Calculates \f$\int_0^t u \, x \; d\tau\f$, where x is the integrand and u is
     * an additional input flag (0.0 or 1.0) to stop integration when the integrator output is constrained.
     * In the case of vectors or matrices, the product between u and x is performed element wise.
     *
     * \tparam T The type of the integrand
     */
    template <typename T>
    class ConstrainedIntegrator : public Integrator<T> {
    public:
        /*!
         * \brief Default constructor
         *
         * Use zero as initial value
         */
        ConstrainedIntegrator()
            : Integrator<T>()
        {
        }

        /*!
         * \brief Prototype initialization constructor for complex data types
         * \param prototype Data type instance used for initial value of the integrator.
         */
        explicit ConstrainedIntegrator(const std::decay_t<T>& prototype)
            : Integrator<T>(prototype)
        {
        }

        /*!
         * \brief Move Constructor.
         *
         * This can be used to integrate on an existing storage.
         * \param value The value to construct from
         */
        explicit ConstrainedIntegrator(T&& value)
            : Integrator<T>(std::forward<T>(value))
        {
        }

        /*!
         * \brief Performs a constrained integration step and returns the result
         *
         * \param integrand The integrand signal to integrate
         * \param isUnconstrained Indicates if the integrand is unconstrained (1.0) or constrained (0.0). Vectors are multiplied element-wise with the integrand.
         * \return The integrator signal after integration
         */
        auto process(const Signal<std::decay_t<T>>& integrand, const Signal<std::decay_t<T>>& isUnconstrained)
        {
            auto constrainedIntegrand = integrand.elementWiseProduct(isUnconstrained.derived());
            return Integrator<T>::process(constrainedIntegrand);
        }

    private:
        using Integrator<T>::process;
    };
} // namespace control
} // namespace broccoli
