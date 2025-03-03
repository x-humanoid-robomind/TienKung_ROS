/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../BusDriver.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Mocks BusDriver
         * \tparam BusType
         */
        template <typename BusType>
        class MockBusDriver : public BusDriver<BusType> {
        public:
            MOCK_METHOD(typename BusType::StateType, state, (), (const, noexcept, override));
            MOCK_METHOD(void, requestState, (const typename BusType::StateType&), (override));
            MOCK_METHOD(std::size_t, cycleTimeInUs, (), (const, noexcept, override));
            MOCK_METHOD(bool, delegateTransfer, (std::size_t), (override));
            MOCK_METHOD(void, processBus, (), (override));
            MOCK_METHOD(typename BusDriver<BusType>::DeviceContainerType&, devices, (), (override));
            MOCK_METHOD(const typename BusDriver<BusType>::DeviceContainerType&, devices, (), (const, override));
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
