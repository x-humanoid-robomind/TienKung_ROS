/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "BusDeviceTest.hpp"
#include "LoopbackBusDevice.hpp"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Test Suite for an object both being a BusDevice and BusDriver
         * \tparam DUTType Device under Test Type
         * \tparam UpstreamBusType Upstream bus type (BusDevice)
         * \tparam DownStreamBusType Downstream bus type (BusDriver)
         * \tparam RegistryTemplate Bus Variable Registry template
         */
        template <typename DUTType, typename UpstreamBusType, typename DownStreamBusType, template <typename> class RegistryTemplate = FakeBusVariableRegistry>
        class BusGatewayTest : public BusDeviceTest<DUTType, UpstreamBusType, RegistryTemplate> {
        public:
            using Base = BusDeviceTest<DUTType, UpstreamBusType, RegistryTemplate>;
            using DeviceType = DUTType;

            void SetUp() override
            {
                Base::SetUp();
                m_downStreamDevice = std::make_shared<LoopbackBusDevice<DownStreamBusType>>(this->dut());
                this->dut().addDevice(*m_downStreamDevice, 0);
            }

            //! Returns a reference to the loopback downstream device
            LoopbackBusDevice<DownStreamBusType>& loopback() { return *m_downStreamDevice; }

        protected:
            //! Fake device for the downstream side
            std::shared_ptr<LoopbackBusDevice<DownStreamBusType>> m_downStreamDevice;
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
