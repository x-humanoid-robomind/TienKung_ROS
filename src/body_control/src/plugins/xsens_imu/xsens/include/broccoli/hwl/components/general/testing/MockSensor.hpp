/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../Sensor.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Mocks Sensor
         */
        class MockSensor : public Sensor {
        public:
            MOCK_METHOD(bool, ready, (), (const, noexcept, override));
            MOCK_METHOD(bool, zeroState, (), (noexcept, override));
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
