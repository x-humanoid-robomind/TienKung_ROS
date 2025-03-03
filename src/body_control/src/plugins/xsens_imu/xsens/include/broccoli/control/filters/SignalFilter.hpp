/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../Signal.hpp"

namespace broccoli {
namespace control {
    /*!
     * \brief Base class for Signal filters
     * \ingroup broccoli_control_filters
     */
    template <typename Derived>
    class SignalFilter : public SignalBase<Derived> {
    public:
        /*!
         * \brief Computes and returns the filter output signal
         * \param input The input signal for the current time step
         * \returns A reference to this instance (the filter output signal)
         */
        template <typename T>
        auto& process(const Signal<T>& input)
        {
            return this->derived().process(input);
        }
    };
} // namespace control
} // namespace broccoli
