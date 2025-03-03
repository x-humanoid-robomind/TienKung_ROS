/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../Faultable.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Mocks Faultable
         * \tparam BusType
         */
        class MockFaultable : public Faultable {
        public:
            MOCK_METHOD(bool, onFault, (), (const, noexcept, override));
            MOCK_METHOD(void, resetFault, (), (noexcept, override));
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
