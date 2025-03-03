/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

namespace broccoli {
namespace hwl {
    /*!
     * \brief Describes objects, which may be in a fault state
     * \ingroup broccoli_hwl_components
     */
    class Faultable {
    public:
        virtual ~Faultable() = default;

        /*!
         * \brief Returns true if a (fatal) fault occurred on this entity.
         * Safety reactions need to be taken in this case
         */
        virtual bool onFault() const noexcept = 0;

        /*!
         * \brief Reset fault state of this entity.
         * This resets any fault flags and enables continued operation after a fault.
         */
        virtual void resetFault() noexcept = 0;
    };

} // namespace hwl
} // namespace broccoli
