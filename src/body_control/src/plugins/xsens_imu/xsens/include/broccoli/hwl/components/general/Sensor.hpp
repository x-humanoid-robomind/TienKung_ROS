/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

namespace broccoli {
namespace hwl {
    /*!
     * \brief An interface for general sensor objects
     * \ingroup broccoli_hwl_components
     */
    class Sensor {
    public:
        virtual ~Sensor() = default;

        /*!
         * \brief Evaluates if a sensor is ready (initialized) / returns valid data
         * \return True if sensor output is valid
         */
        virtual bool ready() const noexcept = 0;

        /*!
         * \brief Zeros the data reading of a sensor
         *
         * This must not be necessarily supported by a Sensor object.
         * If unsupported, the method should return true immediately.
         * \return True on success, false on failure.
         */
        virtual bool zeroState() noexcept { return true; }
    };

} // namespace hwl
} // namespace broccoli
