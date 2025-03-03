/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../BusDevice.hpp"
#include "../ForwardDeclarations.hpp"
#include "../variables.hpp"
#include "gmock/gmock.h"

namespace broccoli {
namespace hwl {
    namespace testing {

        /*!
         * \brief Base LoopbackBusDevice implementation
         *
         * This device loops back input to its output, both
         * synchronously and asynchronously.
         * \tparam BusType Bus description type
         * \tparam FundamentalType Data Type for the variables
         */
        template <typename BusType, typename FundamentalType = uint8_t>
        class LoopbackBusDeviceBase : public BusDevice<BusType> {
        public:
            using BusDevice<BusType>::BusDevice;

            InputBusVariable<FundamentalType> m_syncInput;
            OutputBusVariable<FundamentalType> m_syncOutput;
            AsyncInputBusVariable<FundamentalType> m_asyncInput;
            AsyncOutputBusVariable<FundamentalType> m_asyncOutput;

            /*!
             * \brief Link all variables
             * \param registry The target registry
             */
            template <typename Derived>
            void linkVariables(BusVariableRegistryBase<Derived>& registry)
            {
                linkVariablesASync(registry);
                linkVariablesSync(registry);
            }

            /*!
             * \brief Link synchronous variables only
             * \param registry The target registry
             */
            template <typename Derived>
            void linkVariablesSync(BusVariableRegistryBase<Derived>& registry)
            {
                registry.template registerVariable(m_syncInput, BusType::InputObjectIdentifierType::random());
                registry.template registerVariable(m_syncOutput, BusType::OutputObjectIdentifierType::random());
            }

            /*!
             * \brief Link asynchronous variables only
             * \param registry The target registry
             */
            template <typename Derived>
            void linkVariablesASync(BusVariableRegistryBase<Derived>& registry)
            {
                registry.template registerVariable(m_asyncInput, BusType::AsyncInputObjectIdentifierType::random());
                registry.template registerVariable(m_asyncOutput, BusType::AsyncOutputObjectIdentifierType::random());
            }

        protected:
            void processDevice() override
            {
                m_syncOutput = m_syncInput;

                if (!m_asyncInput.template lockConstWithGuard()->isLatched() && !m_asyncInput.template lockConstWithGuard()->hasFailed())
                    this->bus().template transfer(m_asyncInput);

                m_asyncOutput = m_asyncInput;
                if (!m_asyncOutput.template lockConstWithGuard()->isLatched() && !m_asyncOutput.template lockConstWithGuard()->hasFailed())
                    this->bus().template transfer(m_asyncOutput);
            }
            void onStateChange() override
            {
            }
        };

        /*!
         * \brief Fakes a BusDevice for testing bus variable registries
         * \tparam BusType Bus description type
         * \tparam FundamentalType Data Type for the variables
         */
        template <typename BusType, typename FundamentalType = uint8_t>
        class LoopbackBusDevice : public LoopbackBusDeviceBase<BusType, FundamentalType> {
            using LoopbackBusDeviceBase<BusType, FundamentalType>::LoopbackBusDeviceBase;
        };

        /*!
         * \brief LoopbackBusDevice specialization for CANopen
         * \tparam FundamentalType Data Type for the variables
         */
        template <typename FundamentalType>
        class LoopbackBusDevice<CANopen, FundamentalType> : public LoopbackBusDeviceBase<CANopen, FundamentalType> {
        public:
            using LoopbackBusDeviceBase<CANopen, FundamentalType>::LoopbackBusDeviceBase;
            AsyncInputBusVariable<CANopen::EmergencyObject> m_emergencyInput;

            template <typename Derived>
            void linkVariables(BusVariableRegistryBase<Derived>& registry)
            {
                LoopbackBusDeviceBase<CANopen, FundamentalType>::linkVariables(registry);
                linkEmergencyObject(registry);
            }

            /*!
             * \brief Link emergency object variables only
             * \param registry The target registry
             */
            template <typename Derived>
            void linkEmergencyObject(BusVariableRegistryBase<Derived>& registry)
            {
                registry.template registerVariable(m_emergencyInput, CANopen::EmergencyObjectIdentifier{});
            }
        };

        /*!
         * \brief LoopbackBusDevice specialization for EtherCAT
         * \tparam FundamentalType Data Type for the variables
         */
        template <typename FundamentalType>
        class LoopbackBusDevice<EtherCAT, FundamentalType> : public LoopbackBusDeviceBase<EtherCAT, FundamentalType> {
        public:
            using LoopbackBusDeviceBase<EtherCAT, FundamentalType>::LoopbackBusDeviceBase;
            AsyncInputBusVariable<EtherCAT::EmergencyObject> m_emergencyInput;

            template <typename Derived>
            void linkVariables(BusVariableRegistryBase<Derived>& registry)
            {
                LoopbackBusDeviceBase<EtherCAT, FundamentalType>::linkVariables(registry);
                linkEmergencyObject(registry);
            }

            /*!
             * \brief Link emergency object variables only
             * \param registry The target registry
             */
            template <typename Derived>
            void linkEmergencyObject(BusVariableRegistryBase<Derived>& registry)
            {
                registry.template registerVariable(m_emergencyInput, EtherCAT::EmergencyObjectIdentifier{});
            }
        };

    } // namespace testing
} // namespace hwl
} // namespace broccoli
