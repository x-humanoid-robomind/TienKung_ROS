/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../../core/CycleTimer.hpp"
#include "../../../core/utils.hpp"
#include "../../../io/console/Console.hpp"
#include "../../BusDevice.hpp"
#include "../../BusDriver.hpp"
#include "../../bus_types/CAN.hpp"
#include "../../bus_types/EtherCAT.hpp"
#include "../../variables.hpp"
#include "../general/Faultable.hpp"
#include "ESDCANRegistry.hpp"
#include "ESDCANRxQueue.hpp"
#include "ESDCANTxQueue.hpp"

namespace broccoli {
namespace hwl {

    /*!
     * \brief Implements the esd CAN-EtherCAT Gateway device
     *
     * The class provides a BusDriver interface for a CAN bus and
     * acts as a BusDevice for an EtherCAT bus to bridge technologies. A lower cycle
     * time may be chosen for the CAN bus, affecting the time between BusDevice::process() calls
     * on the CAN devices.
     *
     * The CAN bus implementation supports synchronous and asynchronous
     * messages. For synchronous messages, the attaching device itself must
     * take care of missing messages / a timeout. For asynchronous messages, a
     * timeout can be set via setAsyncCommunicationTimeout(). Note that incoming messages
     * linked to an asynchronous bus variable are always processed (regardless if a transfer
     * was requested for this message). All outgoing messages are fed through a local queue before being sent to the
     * limited-size device queue.
     *
     * All communication errors (timeouts, working counter mismatches) by default trigger
     * a fault of the device. This "strict mode" may be turned off via beStrict().
     *
     * \note The implementation is limited to standard 11bit CAN identifiers
     * \note Remote transmission requests are currently not supported
     * \ingroup broccoli_hwl_components
     */
    template <typename RegistryType, typename RXQueueType, typename TXQueueType>
    class ESDCANEtherCATGatewayImpl : public BusDevice<EtherCAT>, public BusDriver<CAN>, public Faultable {
    public:
        /*!
         * \brief Construct Gateway device
         *
         * Inherits bus cycle time from EtherCAT bus
         */
        explicit ESDCANEtherCATGatewayImpl(BusDriverControl<EtherCAT>& bus)
            : ESDCANEtherCATGatewayImpl(bus, 0)
        {
        }

        /*!
         * \brief Construct Gateway device
         * \param cycleTimeInUs The cycle time of child-CAN Bus (CAN device update rate)
         */
        ESDCANEtherCATGatewayImpl(BusDriverControl<EtherCAT>& bus, std::size_t cycleTimeInUs)
            : BusDevice<EtherCAT>(bus)
            , m_cycleTimeInUs(cycleTimeInUs)
            , m_registry(*this)
            , m_txQueue(*this)
        {
        }

        /*!
         * \brief Add a device to the CAN bus
         * \tparam DeviceType
         * \param device The device to add to the bus
         * \param baseCANId Base CAN identifier added to the device-specific CAN-ID offsets. This must be zero when a device does not use offsets for linking.
         */
        template <typename DeviceType>
        void addDevice(DeviceType& device, CAN::MessageIDType baseCANId)
        {
            m_registry.setBaseCANId(baseCANId);
            device.linkVariables(m_registry);
            m_devices.push_back(device);
        }

        /*!
         * \brief Make and add a device the the CAN bus
         * \tparam DeviceType Type of device to make
         * \tparam Args Parameter pack types for constructor arguments
         * \param baseCANId Base CAN identifier added to the device-specific CAN-ID offsets. This must be zero when a device does not use offsets for linking.
         * \param args Optional arguments passed to the constructor of DeviceType
         */
        template <typename DeviceType, typename... Args>
        core::implicit_construct_and_call<DeviceType> makeDevice(CAN::MessageIDType baseCANId, Args&&... args)
        {
            return { std::bind(&ESDCANEtherCATGatewayImpl::addDevice<DeviceType>, this, std::placeholders::_1, baseCANId),
                *this, std::forward<Args>(args)... };
        }

        /*!
         * \brief Turn on/off strict mode
         *
         * In strict mode, every communication failure leads to a fault.
         * When strict mode is off, failures are tolerated and operation continues for
         * newly incoming data.
         * \param strict True to set strict mode
         */
        void beStrict(bool strict) noexcept { m_beStrict = strict; }

        /*!
         * \brief Set size of the local queue for outgoing messages
         * \param txQueueSize Number of messages to buffer. Four times the device-queue size by default.
         */
        void setTXQueueSize(std::size_t txQueueSize) noexcept
        {
            assert(txQueueSize != 0 && "Maximum queue size must be nonzero!");
            m_txQueue.setSize(txQueueSize);
        }

        /*!
         * \brief Set the timeout for asynchronous communication
         * \param timeoutInMs Timeout in milliseconds. The default value is 100 milliseconds.
         */
        void setAsyncCommunicationTimeout(std::size_t timeoutInMs) noexcept { m_asyncTimeOutInMs = timeoutInMs; }

        std::size_t cycleTimeInUs() const noexcept override { return m_cycleTimeInUs; }
        bool onFault() const noexcept override { return m_fault; }
        void resetFault() noexcept override { m_fault = false; }
        void requestState(const CAN::StateType&) override {}
        CAN::StateType state() const noexcept override
        {
            if (onFault()) {
                // In fault, safeop is the maximum state
                return m_state > CAN::StateType::safeOp() ? CAN::StateType::safeOp() : m_state;
            }

            return m_state;
        }

        void processDevice() override
        {
            m_state = bus().state();
            process();
        }

        void onStateChange() override
        {
            if (m_cycleTimeInUs == 0) {
                m_cycleTimeInUs = bus().cycleTimeInUs();
            }
            m_cycleTimer.setDuration(m_cycleTimeInUs, bus().cycleTimeInUs());
            m_registry.setAsyncMessageTimeout(m_asyncTimeOutInMs * 1000, bus().cycleTimeInUs());

            if (bus().state() == EtherCAT::StateType::safeOp()) {
                // changed to safeOp
                m_txQueue.reset();
                m_registry.pendingTransfersFailed();
                m_rxQueue.reset();
            }
        }

        /*!
         * \brief Register the EtherCAT bus variables of the gateway
         * \param registry The BusVariableRegistry to register to
         */
        template <typename Derived>
        void linkVariables(BusVariableRegistryBase<Derived>& registry)
        {
            registry.registerVariable(m_lastTXTransactionNumber, EtherCAT::ObjectIdentifierType{ "CAN TxPDO-Map.TX Transaction Number" });
            registry.registerVariable(m_rxQueueOverflow, EtherCAT::ObjectIdentifierType{ "CAN Status PDO.RX Overflow" });
            m_rxQueue.template linkVariables(registry);
            m_txQueue.template linkVariables(registry);
        }

        const DeviceContainerType& devices() const override { return m_devices; }

    protected:
        void process() override
        {
            if (m_state == CAN::StateType::op()) {
                if (m_rxQueueOverflow) {
                    io::Console::error().print(prefixName("ESDCANEtherCATGateway: Device RX queue overflow. Lost messages...\n"));
                    if (m_beStrict)
                        m_fault = true;
                }
                if (!m_rxQueue.process(m_registry)) {
                    if (m_beStrict)
                        m_fault = true;
                }
                m_registry.updatePendingTransfers(m_lastTXTransactionNumber);
            }

            // The devices of the CAN bus are executed slower (at CAN update rate)
            // Still TX/RX processing takes place at the potentially faster EtherCAT cycle
            if (m_cycleTimer.cycle()) {
                BusDriver::process();
                m_registry.enqueueSyncOutputVariables(m_txQueue);
            }

            if (m_state == CAN::StateType::op()) {
                if (!m_txQueue.process()) {
                    m_registry.pendingTransfersFailed();
                    if (m_beStrict)
                        m_fault = true;
                }
            }
        }

        void processBus() override
        {
            // Administrative stuff at slower CAN bus rate would happen here
        }
        bool delegateTransfer(std::size_t hash) override
        {
            // The gateway only processes messages in operational mode
            if (state() != CAN::StateType::op())
                return false;

            return m_registry.triggerAsyncTransfer(hash, m_txQueue);
        }

        DeviceContainerType& devices() override { return m_devices; }

    private:
        //! Fault state?
        bool m_fault = false;

        //! Strict mode?
        bool m_beStrict = true;

        //! CAN bus cycle time in us
        std::size_t m_cycleTimeInUs{};

        //! Timer for CAN bus update rate
        core::CycleTimer m_cycleTimer;

        //! CAN bus state (inherited from EtherCAT Bus)
        broccoli::hwl::CAN::StateType m_state;

        //! Timeout for asynchronous communication
        std::size_t m_asyncTimeOutInMs = 100;

        //! The registered devices
        DeviceContainerType m_devices;

        //! Variable registry
        RegistryType m_registry;

        //! TX queue (local + on device)
        TXQueueType m_txQueue;

        //! RX queue
        RXQueueType m_rxQueue;

        //! Rx overflow?
        InputBusVariable<bool> m_rxQueueOverflow{ false };

        //! TX transaction number of last sent CAN message
        InputBusVariable<uint16_t> m_lastTXTransactionNumber{ 0 };
    };

    /*!
     * \brief ESD EtherCAT to CAN gateway implementation with FIFO transmit queue
     *
     * \copydoc ESDCANEtherCATGatewayImpl
     * \note This variant uses a simple FIFO queue to buffer outgoing messages. Asynchronous messages are enqueued before synchronous ones.
     * Asynchronous messages are sent in the order their transfers are requested, synchronous messages are sent in the order of registration to the ESDCANRegistry.
     * \ingroup broccoli_hwl_components
     */
    using ESDCANEtherCATGateway = ESDCANEtherCATGatewayImpl<ESDCANRegistry, ESDCANRxQueue<>, ESDCANTxQueue<>>;

    /*!
     * \brief ESD EtherCAT to CAN gateway implementation with priority-based transmit queue
     *
     * \copydoc ESDCANEtherCATGatewayImpl
     * \note This variant uses a priority queue to buffer outgoing messages. High-priority messages (i.e. low CAN identifier) are sent first.
     * Sorting the messages takes additional time.
     * \ingroup broccoli_hwl_components
     */
    using ESDCANEtherCATGatewayPQ = ESDCANEtherCATGatewayImpl<ESDCANRegistry, ESDCANRxQueue<>, ESDCANTxQueue<ESDTxQueueEntry, std::priority_queue<ESDTxQueueEntry>>>;

} // namespace hwl
} // namespace broccoli
