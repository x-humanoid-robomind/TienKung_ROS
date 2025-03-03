/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

namespace broccoli {
namespace control {
    /*!
     * \brief Contains internal stuff of the control module
     */
    namespace internal {
        template <typename>
        class SignalNoSampleTime;

        struct SignalBaseTrait;
    }

    template <typename>
    class SignalBase;

    template <typename>
    class Signal;

} // namespace control
} // namespace broccoli
