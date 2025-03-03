/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../../core/Nameable.hpp"
#include "../../variables/IDVariablePair.hpp"
#include "internal/helpers.hpp"
#include <queue>

namespace broccoli {
namespace hwl {

    /*!
     * \brief ESD queue entry for outgoing messages
     *
     * This entry supports 11bit base frames only.
     */
    struct ESDTxQueueEntry {

        ESDTxQueueEntry() = default;

        /*!
         * \brief Constructs a queue entry for an outgoing CAN message
         * \param lockGuard Locked guard for the bus variable
         * \param transactionNumber Transaction number to identify the transmission.
         * The device reports the last transmission number on successful send operation,
         * which is used for state updating of asynchronous variables.
         * \param RTRBit True for a remote transmission request frame
         * \param idDataPair Pair of identifier and bus variable to read payload from
         */
        template <typename GuardType, template <typename> class ContainerType>
        ESDTxQueueEntry(GuardType& lockGuard, uint16_t transactionNumber, bool RTRBit, const IDVariablePair<CAN::OutputObjectIdentifierType, BusVariablePointerBase<ContainerType>>& idDataPair)
        {
            uint8_t payloadSize = idDataPair.variable.size();
            rawBytes[0] = transactionNumber & 0xff;
            rawBytes[1] = (transactionNumber & 0xff00) >> 8;

            uint16_t payLoadSizeRTRId = (payloadSize & 0xf) | (RTRBit ? (1 << 4) : 0) | ((idDataPair.id.messageIdentifier & 0x7ff) << 5);
            rawBytes[2] = payLoadSizeRTRId & 0xff;
            rawBytes[3] = (payLoadSizeRTRId & 0xff00) >> 8;

            idDataPair.variable.copyTo(lockGuard, payload(), idDataPair.variable.size(), 0);
        }

        //! Returns the CAN identifier (11bit)
        uint16_t identifier() const noexcept
        {
            uint16_t result = rawBytes[2] >> 5;
            result |= (rawBytes[3]) << 3;
            return result;
        }

        //! Returns a pointer to the payload data area
        void* payload() noexcept { return &rawBytes[4]; };

        //! Returns the transaction number for this message
        uint16_t transactionNumber() const noexcept
        {
            return rawBytes[0] | (rawBytes[1] << 8);
        }

        bool operator<(const ESDTxQueueEntry& rhs) const noexcept
        {
            // For priority queue sorting:
            // use only the message identifier.
            // a lower identifier has a higher priority
            return identifier() > rhs.identifier();
        }
        bool operator>(const ESDTxQueueEntry& rhs) const noexcept
        {
            return rhs < *this;
        }
        bool operator<=(const ESDTxQueueEntry& rhs) const noexcept
        {
            return !(rhs < *this);
        }
        bool operator>=(const ESDTxQueueEntry& rhs) const noexcept
        {
            return !(*this < rhs);
        }

        //! Raw byte aggregate
        std::array<uint8_t, 12> rawBytes;
    };

    /*!
     * \brief ESD message queue for outgoing data
     *
     * This component consists of a local queue and the on-device queue.
     * \tparam EntryType Type for a queue element (must support the ESD device queue entry structure)
     * \tparam QueueType Type for the local queue (output buffer)
     * \tparam DeviceQueueSize Size of the on-device queue
     */
    template <typename EntryType = ESDTxQueueEntry, typename QueueType = std::queue<EntryType>, std::size_t DeviceQueueSize = 16>
    class ESDCANTxQueue {
    public:
        /*!
         * \brief Constructor
         * \param nameable Nameable parent device, used for logging
         */
        ESDCANTxQueue(const core::Nameable& nameable)
            : m_nameable(nameable)
        {
        }

        /*!
         * \brief Adds a message to the transmission queues
         *
         * The message is first pushed to the local queue. The local queue
         * entries are then passed to the on-device queue every bus cycle.
         * \param lockGuard Locked guard for the bus variable
         * \param RTRBit True for a remote transmission request frame
         * \param idDataPair Pair of identifier and bus variable to read payload from
         */
        template <typename GuardType, template <typename> class ContainerType>
        void addMessage(GuardType& lockGuard, bool RTRBit, const IDVariablePair<CAN::OutputObjectIdentifierType, BusVariablePointerBase<ContainerType>>& idDataPair)
        {
            m_localTxQueue.emplace(lockGuard, nextTransactionNumber()++, RTRBit, idDataPair);
        }

        /*!
         * \brief Sets the size of the local transmission queue
         * \param maxQueueSize Maximum size of the queue (messages)
         */
        void setSize(std::size_t maxQueueSize) noexcept { m_maxTxQueueSize = maxQueueSize; }

        /*!
         * \brief Dispatch outgoing messages in the transmission queues
         * \return True on success
         */
        bool process() noexcept
        {
            if (m_txWorkingCounter != m_txWorkingCounterAck) {
                io::Console::error().print(m_nameable.get().prefixName("ESDCANEtherCATGateway: Working counter mismatch on TX data. Is the gateway offline?\n"));
                clearLocalQueue();
                m_txWorkingCounter = m_txWorkingCounterAck;
                return false;
            }

            if (m_localTxQueue.empty())
                return true;

            bool success = true;
            if (m_localTxQueue.size() > m_maxTxQueueSize) {
                io::Console::warning().print(m_nameable.get().prefixName("ESDCANEtherCATGateway: Local TX queue overloaded!\n"));
                onOverload();
                success = false;
            }

            fillDeviceQueue();
            return success;
        }

        //! Reset local and on-device queues; drops all enqueued messages
        void reset() noexcept
        {
            clearLocalQueue();
            m_txWorkingCounter = m_txWorkingCounterAck;
        }

        //! Returns the transmission number used for the next message enqueued via addMessage()
        static uint16_t& nextTransactionNumber() noexcept
        {
            static uint16_t transactionNumberGenerator = 0;
            return transactionNumberGenerator;
        }

        /*!
         * \brief Register the bus variable of the gateway's TX queue
         * \param registry The BusVariableRegistry to register to
         */
        template <typename Derived>
        void linkVariables(BusVariableRegistryBase<Derived>& registry)
        {
            registry.registerVariable(m_txMessagesInQueue, EtherCAT::ObjectIdentifierType{ "CAN RxPDO-Map.Number of TX Messages" });
            registry.registerVariable(m_txWorkingCounterAck, EtherCAT::ObjectIdentifierType{ "CAN TxPDO-Map.TX Counter" });
            registry.registerVariable(m_txWorkingCounter, EtherCAT::ObjectIdentifierType{ "CAN RxPDO-Map.TX Counter" });
            internal::registerArray(registry, "CAN RxPDO-Map.TX Message ", m_txMessageQueue);
        }

    protected:
        //! Fills the on-device queue from local queue
        void fillDeviceQueue()
        {
            m_txMessagesInQueue = std::min(m_txMessageQueue.size(), m_localTxQueue.size());
            for (std::size_t i = 0; i < m_txMessagesInQueue; i++) {
                m_txMessageQueue[i] = internal::frontOrTop(m_localTxQueue);
                m_localTxQueue.pop();
            }
            m_txWorkingCounter = m_txWorkingCounter + 1;
        }

        //! Handler for overflow of the local transmission queue
        void onOverload()
        {
            // Drop the oldest messages
            while (m_localTxQueue.size() > m_maxTxQueueSize) {
                m_localTxQueue.pop();
            }
        }

        //! Clears the local transmission queue
        void clearLocalQueue() noexcept
        {
            while (!m_localTxQueue.empty()) {
                m_localTxQueue.pop();
            }
        }

        //! Reference to nameable object
        std::reference_wrapper<const core::Nameable> m_nameable;

        //! Maximum number of messages in the local TX queue
        std::size_t m_maxTxQueueSize = DeviceQueueSize * 4; // The gateway is known to take up to 4 EC cycles for delivery of a CAN message

        //! Local TX queue
        QueueType m_localTxQueue;

        //! Output message queue
        std::array<OutputBusVariable<EntryType>, DeviceQueueSize> m_txMessageQueue{};

        //! Number of Tx Messages in on-device queue
        OutputBusVariable<uint16_t> m_txMessagesInQueue{ 0 };

        //! TX working counter
        OutputBusVariable<uint16_t> m_txWorkingCounter{ 0 };

        //! TX working counter acknowledgment
        InputBusVariable<uint16_t> m_txWorkingCounterAck{ 0 };
    };
} // namespace hwl
} // namespace broccoli
