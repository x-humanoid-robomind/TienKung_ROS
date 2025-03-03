/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../../../io/console/Console.hpp"
#include "../../BusVariableRegistryBase.hpp"
#include "../../bus_types/CAN.hpp"
#include "../../variables.hpp"
#include "../../variables/IDVariablePair.hpp"
#include "../../variables/pointers.hpp"
#include "ESDCANRxQueue.hpp"
#include "ESDCANTxQueue.hpp"
#include <map>
#include <queue>
#include <unordered_map>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Default ESD CAN bus registry
     */
    class ESDCANRegistry : public BusVariableRegistryBase<ESDCANRegistry> {
    public:
        using BusType = CAN;

        /*!
         * \brief Construct from nameable device
         * \param nameable Nameable parent device, used for logging
         */
        explicit ESDCANRegistry(const core::Nameable& nameable)
            : m_nameable(nameable)
        {
        }

        /*!
         * \brief Register the following synchronously-updated output BusVariable
         * \param variable The variable to register
         * \param id The object identifier to register to
         * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
         */
        template <typename T>
        void registerVariable(OutputBusVariable<T>& variable,
            CAN::OutputMessageIdentifierType id, bool isLittleEndian = true)
        {
            id.messageIdentifier += m_baseCANId;
            if (isValid(variable, id))
                m_syncOutputVariables.push_back({ id, BusVariablePointer(variable, isLittleEndian) });
        }

        /*!
         * \brief Register the following synchronously-updated input BusVariable
         * \param variable The variable to register
         * \param id The object identifier to register to
         * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
         */
        template <typename T>
        void registerVariable(InputBusVariable<T>& variable,
            CAN::InputMessageIdentifierType id, bool isLittleEndian = true)
        {
            id.messageIdentifier += m_baseCANId;
            if (isValid(variable, id))
                m_syncInputVariables[id.messageIdentifier].push_back({ id, BusVariablePointer(variable, isLittleEndian) });
        }

        /*!
         * \brief Register the following asynchronously-updated input BusVariable
         * \param variable The variable to register
         * \param id The object identifier to register to
         * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
         */
        template <typename T>
        void registerVariable(AsyncInputBusVariable<T>& variable,
            CAN::InputMessageIdentifierType id, bool isLittleEndian = true)
        {
            id.messageIdentifier += m_baseCANId;
            if (isValid(variable, id)) {
                m_asyncInputVariables.emplace(std::make_pair(std::hash<AsyncInputBusVariable<T>>{}(variable), AsyncBusVariablePointer(variable, isLittleEndian)));
                m_asyncInputVariablesById[id.messageIdentifier].push_back({ id, AsyncBusVariablePointer(variable, isLittleEndian) });
            }
        }

        /*!
         * \brief Register the following asynchronously-updated output BusVariable
         * \param variable The variable to register
         * \param id The object identifier to register to
         * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
         */
        template <typename T>
        void registerVariable(AsyncOutputBusVariable<T>& variable,
            CAN::OutputMessageIdentifierType id, bool isLittleEndian = true)
        {
            id.messageIdentifier += m_baseCANId;
            if (isValid(variable, id))
                m_asyncOutputVariables.emplace(std::make_pair(std::hash<AsyncOutputBusVariable<T>>{}(variable), makeIDVariablePair(id, AsyncBusVariablePointer(variable, isLittleEndian))));
        }

        /*!
         * \brief Set the timeout for asynchronous messages
         * \param timeOutInMs The timeout in milliseconds. Defaults to 1 cycle for updatePendingTransfers().
         * \param updateCycleTimeInUs The time between consecutive calls to updatePendingTransfers()
         */
        void setAsyncMessageTimeout(std::size_t timeOutInMs, std::size_t updateCycleTimeInUs) noexcept
        {
            m_cycleTimerAsyncTimeout.setDuration(timeOutInMs, updateCycleTimeInUs);
        }

        /*!
         * \brief Set the CAN base identifier used for all upcoming variable registrations
         * \param baseCANId CAN base identifier (11bit)
         */
        void setBaseCANId(CAN::MessageIDType baseCANId) { m_baseCANId = baseCANId; }

        /*!
         * \brief Handler for incoming CAN messages
         *
         * This method looks for the respective bus variable (synchronous or asynchronous) and updates its contents.
         * \param messageID The CAN identifier for the message
         * \param payload Pointer to the raw CAN payload
         * \param payloadSize Size of the raw CAN payload in bytes
         * \return True on success
         */
        bool handleInputMessage(CAN::MessageIDType messageID, const void* const payload, std::size_t payloadSize) noexcept
        {
            bool success = true;
            if (!updateSyncInputVariables(messageID, payload, payloadSize))
                success = false;

            if (!updateAsyncInputVariables(messageID, payload, payloadSize))
                success = false;
            return success;
        }

        /*!
         * \brief Triggers an asynchronous transfer for a variable identified via its hash
         * \tparam TxQueueType Type of the used transmission queue (see ESDCANTxQueue)
         * \param variableHash Hash of the bus variable to trigger transfer for
         * \param txQueue The transmission queue
         * \return True on success
         */
        template <typename TxQueueType>
        bool triggerAsyncTransfer(std::size_t variableHash, TxQueueType& txQueue) noexcept
        {
            auto it = m_asyncOutputVariables.find(variableHash);
            if (it == m_asyncOutputVariables.end()) {
                return triggerAsyncInputTransfer(variableHash);
            }

            auto guard = it->second.variable.lockWithGuard();
            if (guard->isTransferPending())
                return true; // already in progress

            m_pendingAsyncOutputVariables.insert({ TxQueueType::nextTransactionNumber(), std::ref(it->second.variable) });
            guard->indicatePendingTransfer();
            txQueue.addMessage(guard, false, it->second);
            return true;
        }

        /*!
         * \brief Updates the state of pending asynchronous transfers
         *
         * This uses the transaction number last known to have been processed by the gateway
         * to find transfers which therefore must have been processed. Pending input
         * transfers are checked for timeout.
         * If a timeout occurs, the asynchronous variable states are set to failed.
         *
         * \param lastTransactionNumber Transaction number for last outgoing message
         */
        void updatePendingTransfers(uint16_t lastTransactionNumber) noexcept
        {
            for (auto it = m_pendingAsyncInputVariables.begin(); it != m_pendingAsyncInputVariables.end();) {
                if (it->second.second.cycle()) {
                    {
                        auto guard = it->second.first.lockWithGuard();
                        guard->indicateFailedTransfer();
                    }
                    m_pendingAsyncInputVariables.erase(it++);
                } else {
                    it++;
                }
            }

            for (auto it = m_pendingAsyncOutputVariables.begin(); it != m_pendingAsyncOutputVariables.end(); it++) {
                if (it->first <= lastTransactionNumber) {
                    m_cycleTimerAsyncTimeout.reset();
                    auto guard = it->second.get().lockWithGuard();
                    guard->indicateSuccessfulTransfer();
                } else {
                    // Erase all completed transfers
                    if (it != m_pendingAsyncOutputVariables.begin())
                        m_pendingAsyncOutputVariables.erase(m_pendingAsyncOutputVariables.begin(), --it);

                    if (m_cycleTimerAsyncTimeout.cycle()) {
                        pendingTransfersFailed();
                    }
                    return;
                }
            }
            m_pendingAsyncOutputVariables.clear();
        }

        //! Sets the state of all pending asynchronous transactions to failed
        void pendingTransfersFailed() noexcept
        {
            for (auto& pair : m_pendingAsyncInputVariables) {
                auto guard = pair.second.first.lockWithGuard();
                guard->indicateFailedTransfer();
            }
            m_pendingAsyncInputVariables.clear();

            for (auto& pair : m_pendingAsyncOutputVariables) {
                auto guard = pair.second.get().lockWithGuard();
                guard->indicateFailedTransfer();
            }
            m_pendingAsyncOutputVariables.clear();
        }

        /*!
         * \brief Enqueues all synchronous output variables to a transmission queue
         * \tparam TxQueueType Type of the transmission queue (see ESDCANTxQueue)
         * \param txQueue Transmission queue
         */
        template <typename TxQueueType>
        void enqueueSyncOutputVariables(TxQueueType& txQueue) const noexcept
        {
            for (const auto& idVariablePair : m_syncOutputVariables) {
                auto guard = idVariablePair.variable.lockWithGuard();
                txQueue.addMessage(guard, false, idVariablePair);
            }
        }

    protected:
        /*!
         * \brief Triggers asynchronous transfer of an input variable
         * \param variableHash Hash of the variable
         * \return True on success
         */
        bool triggerAsyncInputTransfer(std::size_t variableHash) noexcept
        {
            // just set pending state until this CAN frame arrives
            auto it = m_asyncInputVariables.find(variableHash);
            if (it == m_asyncInputVariables.end())
                return false;

            m_pendingAsyncInputVariables.insert({ variableHash, { it->second, core::CycleTimer(m_cycleTimerAsyncTimeout.cyclesForDuration()) } });

            auto guard = it->second.lockWithGuard();
            guard->indicatePendingTransfer();
            return true;
        }

        /*!
         * \brief Updates all synchronous bus variables for incoming CAN message
         * \param messageID CAN identifier
         * \param payload Pointer to the raw CAN payload
         * \param payloadSize Size of the raw CAN payload in bytes
         * \return True when successful or no variable is registered for this message
         */
        bool updateSyncInputVariables(CAN::MessageIDType messageID, const void* const payload, std::size_t payloadSize) noexcept
        {
            auto it = m_syncInputVariables.find(messageID);
            if (it == m_syncInputVariables.end())
                return true;

            bool copySuccessful = true;
            for (auto& pair : it->second) {
                if (!pair.variable.copyFrom(payload, payloadSize, pair.id.dataOffset)) {
                    copySuccessful = false;
                    io::Console::error().print(m_nameable.get().prefixName("ESDCANGateway: Sync variable size mismatch for ") + pair.id.toString() + "\n");
                }
            }
            return copySuccessful;
        }

        /*!
         * \brief Updates all asynchronous bus variables for incoming CAN message
         * \param messageID CAN identifier
         * \param payload Pointer to the raw CAN payload
         * \param payloadSize Size of the raw CAN payload in bytes
         * \return True when successful or no variable is registered for this message
         */
        bool updateAsyncInputVariables(CAN::MessageIDType messageID, const void* const payload, std::size_t payloadSize) noexcept
        {
            auto it = m_asyncInputVariablesById.find(messageID);
            if (it == m_asyncInputVariablesById.end())
                return true;

            std::hash<AsyncBusVariablePointer> hashFor;
            bool copySuccessful = true;
            for (auto& pair : it->second) {
                m_pendingAsyncInputVariables.erase(hashFor(pair.variable));

                auto guard = pair.variable.lockWithGuard();
                guard->indicatePendingTransfer();
                if (!pair.variable.copyFrom(guard, payload, payloadSize, pair.id.dataOffset)) {
                    copySuccessful = false;
                    guard->indicateFailedTransfer();
                    io::Console::error().print(m_nameable.get().prefixName("ESDCANGateway: Async variable size mismatch for ") + pair.id.toString() + "\n");
                } else {
                    guard->indicateSuccessfulTransfer();
                }
            }

            return copySuccessful;
        }

        /*!
         * \brief Validates bus variable size and identifier upon registration
         * \tparam IdentifierType Type of the identifier
         * \param variable The bus variable to register to
         * \param id The identifier
         * \return True if identifier and variable properties are compatible with this device
         */
        template <typename Derived, typename IdentifierType>
        bool isValid(const BusVariableBase<Derived>& variable, const IdentifierType& id)
        {
            if (variable.size() > 8 || !CAN::is11bit(id.messageIdentifier)) {
                assert(false && "Invalid variable size or message ID. Only 11bit message IDs are supported by this implementation!");
                io::Console::critical().print(m_nameable.get().prefixName("ESDCANGateway: Invalid variable size or message identifier for identifier ") + id.toString() + ". Ignoring variable...\n");
                return false;
            }
            return true;
        }

        //! Reference to nameable object
        std::reference_wrapper<const core::Nameable> m_nameable;

        //! The current base CAN identifier added to registered variables' identifier
        CAN::MessageIDType m_baseCANId = 0;

        //! Timer for async variable timeouts
        core::CycleTimer m_cycleTimerAsyncTimeout;

        //! List of all synchronous output variables as an identifier/pointer pair
        std::vector<IDVariablePair<CAN::OutputMessageIdentifierType, BusVariablePointer>> m_syncOutputVariables;

        //! Maps a CAN message id to synchronous input variables
        std::unordered_map<CAN::MessageIDType, std::vector<IDVariablePair<CAN::InputMessageIdentifierType, BusVariablePointer>>> m_syncInputVariables;

        //! Maps a bus variable hash to an asynchronous input variable
        std::unordered_map<std::size_t, AsyncBusVariablePointer> m_asyncInputVariables;

        //! Maps a CAN message id to an asynchronous input variable
        std::unordered_map<CAN::MessageIDType, std::vector<IDVariablePair<CAN::InputMessageIdentifierType, AsyncBusVariablePointer>>> m_asyncInputVariablesById;

        //! Map of all pending asynchronous input transfers, based on bus variable hash
        std::unordered_map<std::size_t, std::pair<AsyncBusVariablePointer, core::CycleTimer>> m_pendingAsyncInputVariables;

        //! Maps a bus variable hash to an asynchronous output variable
        std::unordered_map<std::size_t, IDVariablePair<CAN::OutputMessageIdentifierType, AsyncBusVariablePointer>> m_asyncOutputVariables;

        //! Map of all pending asynchronous output transfers, based on bus variable hash
        std::map<uint16_t, std::reference_wrapper<AsyncBusVariablePointer>> m_pendingAsyncOutputVariables;
    };

} // namespace hwl
} // namespace broccoli
