/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../ForwardDeclarations.hpp"
#include "LoopbackBusDevice.hpp"
#include "MockBusDriverControl.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Test Suite for classes derived from BusVariableRegistryBase
         * \tparam RegistryType RegistryType under test
         * \tparam BusType Bus description type
         */
        template <typename RegistryType_, typename BusType, typename FundamentalType = uint8_t>
        class BusVariableRegistryTest : public ::testing::Test {
        public:
            using RegistryType = RegistryType_;

        protected:
            RegistryType m_registry;
            ::testing::NiceMock<MockBusDriverControl<BusType>> m_busDriverControl;
            LoopbackBusDevice<BusType, FundamentalType> m_device{ m_busDriverControl };
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
