/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "internal/helpers.hpp"

namespace broccoli {
namespace hwl {

    /*!
     * \brief ESD queue entry for incoming messages
     *
     * This entry supports 11bit base frames only.
     */
    struct ESDRxQueueEntry {
        //! Raw byte aggregate
        std::array<uint8_t, 10> rawBytes;

        //! Returns the payload size in bytes
        std::size_t payloadSize() const
        {
            return rawBytes[0] & 0xf;
        }

        //! Returns true for a remote transmission request message
        bool RTRBit() const
        {
            return rawBytes[0] & (1 << 4);
        }

        //! Returns the CAN identifier (11bit)
        uint16_t identifier() const
        {
            uint16_t result = rawBytes[0] >> 5;
            result |= (rawBytes[1]) << 3;
            return result;
        }

        //! Returns a pointer to the payload data area
        const void* payload() const { return &rawBytes[2]; }
    };

    /*!
     * \brief ESD message queue for incoming data
     *
     * This component links to the on-device queue entries.
     * \tparam EntryType Type for a queue element (must support the ESD device queue entry structure)
     * \tparam DeviceQueueSize Size of the queue on the device
     */
    template <typename EntryType = ESDRxQueueEntry, std::size_t DeviceQueueSize = 16>
    class ESDCANRxQueue {
    public:
        /*!
         * \brief Process incoming messages using the given registry
         * \tparam Registry Variable registry type
         * \param registry The registry to look up matching bus variables for incoming data
         * \return True on success
         */
        template <typename Registry>
        bool process(Registry& registry)
        {
            // new messages?
            if (m_rxWorkingCounter == m_rxWorkingCounterAck + 1) {

                for (std::size_t i = 0; i < m_rxMessagesInQueue; i++) {
                    auto guard = m_rxMessageQueue[i].lockConstWithGuard();
                    if (!registry.handleInputMessage(guard->value().identifier(), guard->value().payload(), guard->value().payloadSize())) {
                        return false;
                    }
                }
                m_rxWorkingCounterAck = m_rxWorkingCounterAck + 1;
            }
            return true;
        }

        /*!
        * \brief Register the bus variables of the rx queue
        * \param registry The BusVariableRegistry to register to
        */
        template <typename Derived>
        void linkVariables(BusVariableRegistryBase<Derived>& registry)
        {
            registry.registerVariable(m_rxMessagesInQueue, EtherCAT::ObjectIdentifierType{ "CAN TxPDO-Map.Number of RX Messages" });
            registry.registerVariable(m_rxWorkingCounter, EtherCAT::ObjectIdentifierType{ "CAN TxPDO-Map.RX Counter" });

            registry.registerVariable(m_rxWorkingCounterAck, EtherCAT::ObjectIdentifierType{ "CAN RxPDO-Map.RX Counter" });

            internal::registerArray(registry, "CAN TxPDO-Map.RX Message ", m_rxMessageQueue);
        }

        //! Reset queue working counter
        void reset()
        {
            m_rxWorkingCounterAck = m_rxWorkingCounter;
        }

    protected:
        //! Number of Rx Messages in Queue
        InputBusVariable<uint16_t> m_rxMessagesInQueue{ 0 };

        //! RX working counter
        InputBusVariable<uint16_t> m_rxWorkingCounter{ 0 };

        //! RX working counter acknowledgment
        OutputBusVariable<uint16_t> m_rxWorkingCounterAck{ 0 };

        //! Input message queue
        std::array<InputBusVariable<EntryType>, DeviceQueueSize> m_rxMessageQueue{};
    };

} // namespace hwl
} // namespace broccoli
