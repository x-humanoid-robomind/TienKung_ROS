/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../BusDevice.hpp"
#include "../ForwardDeclarations.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Mocks BusDevice
         * \tparam BusType
         */
        template <typename BusType>
        class MockBusDevice : public BusDevice<BusType> {
        public:
            using BusDevice<BusType>::BusDevice;

            MOCK_METHOD(void, processDevice, (), (override));
            MOCK_METHOD(void, onStateChange, (), (override));

            template <typename Derived>
            void linkVariables(BusVariableRegistryBase<Derived>&)
            {
            }
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
