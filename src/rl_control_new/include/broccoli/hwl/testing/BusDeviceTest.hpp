/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../ForwardDeclarations.hpp"
#include "FakeBusVariableRegistry.hpp"
#include "MockBusDriverControl.hpp"
#include "gmock/gmock.h"
#include <memory>

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Test Suite for BusDevice instances
         * \tparam DUTType Device under Test Type
         * \tparam BusType Explicitly specified BusType
         * \tparam RegistryTemplate Bus Variable Registry template
         */
        template <typename DUTType, typename BusType = typename DUTType::BusType, template <typename> class RegistryTemplate = FakeBusVariableRegistry>
        class BusDeviceTest : public ::testing::Test {
        public:
            using DeviceType = DUTType;

        protected:
            void SetUp() override
            {
                m_device = std::make_shared<DeviceType>(m_busControl);
                m_device->linkVariables(m_registry);
                m_registry.testIfValid();

                ON_CALL(this->m_busControl, cycleTimeInUs()).WillByDefault(::testing::Return(1000));
                ON_CALL(this->m_busControl, previousState()).WillByDefault(::testing::ReturnPointee(&m_previousState));
            }

            //! Returns a reference to the device under test
            DeviceType& dut() { return *m_device; }

            //! Test the device's process() method. Passes m_busControl as parameter
            void process()
            {
                if (m_busControl.stateChanged()) {
                    m_device->onStateChange();
                }

                m_device->processDevice();
                m_previousState = m_busControl.state();
            }

            std::shared_ptr<DeviceType> m_device;
            RegistryTemplate<BusType> m_registry;
            ::testing::NiceMock<MockBusDriverControl<BusType>> m_busControl;
            typename BusType::StateType m_previousState;
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
