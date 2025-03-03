/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../BusDriverControl.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Mocks BusDriverControl
         * \tparam BusType Bus description type
         */
        template <typename BusType>
        class MockBusDriverControl : public BusDriverControl<BusType> {
        public:
            MOCK_METHOD(typename BusType::StateType, state, (), (const, noexcept, override));
            MOCK_METHOD(typename BusType::StateType, previousState, (), (const, noexcept, override));
            MOCK_METHOD(void, requestState, (const typename BusType::StateType&), (override));
            MOCK_METHOD(bool, delegateTransfer, (std::size_t), (override));
            MOCK_METHOD(std::size_t, cycleTimeInUs, (), (const, noexcept, override));
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
