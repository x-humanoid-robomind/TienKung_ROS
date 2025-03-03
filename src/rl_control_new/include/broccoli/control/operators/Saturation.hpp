/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../Signal.hpp"
#include "../SignalLimits.hpp"

namespace broccoli {
namespace control {
    /*!
     * \brief Limits a signal between minimum and maximum
     * \ingroup broccoli_control_operators
     *
     * The result is a signal expression for max(min(input, max), min). Eigen types are processed
     * coefficient-wise.
     *
     * \param input The input Signal to limit
     * \param min The minimum limit Signal
     * \param max The maximum limit Signal
     * \return A Signal for max(min(input, max), min)
     */
    template <typename TypeIn, typename TypeMin, typename TypeMax>
    auto Saturation(TypeIn&& input, TypeMin&& min, TypeMax&& max)
    {
        return std::forward<TypeIn>(input).min(std::forward<TypeMax>(max)).max(std::forward<TypeMin>(min));
    }

    /*!
     * \brief Limits a signal between minimum and maximum
     * \ingroup broccoli_control_operators
     *
     * The result is a signal expression for max(min(input, max), min). Eigen types are processed
     * coefficient-wise.
     *
     * \param input The input Signal to limit
     * \param limits The signal limits (min, max)
     * \return A Signal for max(min(input, max), min)
     */
    template <typename TypeIn, typename ValueType>
    auto Saturation(TypeIn&& input, const SignalLimits<ValueType>& limits)
    {
        return Saturation(std::forward<TypeIn>(input), mapSignal(limits.min()), mapSignal(limits.max()));
    }

} // namespace control
} // namespace broccoli
