/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../operators/BinaryOperator.hpp"
#include <utility>

namespace broccoli {
namespace control {

    /*!
     * \brief A general P control formulation
     * \ingroup broccoli_control_feedback
     * \param gain The control loop gain
     * \param error The control loop tracking error
     * \return The controller output
     */
    template <typename GainType, typename ErrorType>
    auto PControl(GainType&& gain, ErrorType&& error)
    {
        return mapSignal(std::forward<GainType>(gain)) * mapSignal(std::forward<ErrorType>(error));
    }

    /*!
     * \brief A general P control formulation
     * \ingroup broccoli_control_feedback
     * \param gain The control loop gain
     * \param desired The desired state
     * \param actual The actual state
     * \return The controller output
     */
    template <typename GainType, typename DesiredType, typename ActualType>
    auto PControl(GainType&& gain, DesiredType&& desired, ActualType&& actual)
    {
        return PControl(std::forward<GainType>(gain), mapSignal(std::forward<DesiredType>(desired)) - mapSignal(std::forward<ActualType>(actual)));
    }

    /*!
     * \brief An elementwise-operating P control formulation
     * \ingroup broccoli_control_feedback
     * \param gain The control loop gain
     * \param error The control loop tracking error
     * \return The controller output
     */
    template <typename GainType, typename ErrorType>
    auto PControlElementWise(GainType&& gain, ErrorType&& error)
    {
        return internal::binaryOpHelper<internal::ElementwiseProductFunctor>(mapSignal(std::forward<GainType>(gain)), mapSignal(std::forward<ErrorType>(error)));
    }

    /*!
     * \brief An elementwise-operating P control formulation
     * \ingroup broccoli_control_feedback
     * \param gain The control loop gain
     * \param desired The desired state
     * \param actual The actual state
     * \return The controller output
     */
    template <typename GainType, typename DesiredType, typename ActualType>
    auto PControlElementWise(GainType&& gain, DesiredType&& desired, ActualType&& actual)
    {
        return PControlElementWise(std::forward<GainType>(gain), mapSignal(std::forward<DesiredType>(desired)) - mapSignal(std::forward<ActualType>(actual)));
    }

} // namespace control
} // namespace broccoli
