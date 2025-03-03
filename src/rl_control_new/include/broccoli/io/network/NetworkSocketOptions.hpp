/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include "../serialization/Endianness.hpp"
#include "NetworkSocketType.hpp"
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_network
     * \{
     */

    //! Container for options of the \ref NetworkSocket class
    class NetworkSocketOptions {
    public:
        //! Constructor
        /*!
         * \param [in] type Initializes \ref m_type - \copybrief m_type
         * \param [in] name Initializes \ref m_name - \copybrief m_name
         * \param [in] remoteIP Initializes \ref m_remoteIP - \copybrief m_remoteIP
         * \param [in] remotePort Initializes \ref m_remotePort - \copybrief m_remotePort
         * \param [in] ownPort Initializes \ref m_ownPort - \copybrief m_ownPort
         * \param [in] multiThreaded Initializes \ref m_multiThreaded - \copybrief m_multiThreaded
         * \param [in] threadPriority Initializes \ref m_threadPriority - \copybrief m_threadPriority
         * \param [in] minimumCycleTime Initializes \ref m_minimumCycleTime - \copybrief m_minimumCycleTime
         * \param [in] blockingWaiting Initializes \ref m_blockingWaiting - \copybrief m_blockingWaiting
         * \param [in] receivedObjectBufferSize Initializes \ref m_receivedObjectBufferSize - \copybrief m_receivedObjectBufferSize
         * \param [in] sendObjectBufferSize Initializes \ref m_sendObjectBufferSize - \copybrief m_sendObjectBufferSize
         * \param [in] endianness Initializes \ref m_endianness - \copybrief m_endianness
         */
        NetworkSocketOptions(const NetworkSocketType& type, const std::string& name, const std::string& remoteIP, const int& remotePort, const int& ownPort, const bool& multiThreaded, const int& threadPriority, const core::Time& minimumCycleTime, const bool& blockingWaiting, const size_t& receivedObjectBufferSize, const size_t& sendObjectBufferSize, const serialization::Endianness& endianness = serialization::Endianness::LITTLE)
            : m_type(type)
            , m_name(name)
            , m_remoteIP(remoteIP)
            , m_remotePort(remotePort)
            , m_ownPort(ownPort)
            , m_maximumMessageSize(4096)
            , m_multiThreaded(multiThreaded)
            , m_threadPriority(threadPriority)
            , m_minimumCycleTime(minimumCycleTime)
            , m_blockingWaiting(blockingWaiting)
            , m_receivedObjectBufferSize(receivedObjectBufferSize)
            , m_sendObjectBufferSize(sendObjectBufferSize)
            , m_endianness(endianness)
        {
        }

        // General options
        NetworkSocketType m_type; //!< Socket type (TCP/UDP, server/client)
        std::string m_name; //!< Identifying name (also used for name of background thread)

        // Network options
        std::string m_remoteIP; //!< IP address of the remote socket (unused in case of `TCP_SERVER`)
        int m_remotePort; //!< Port of the remote socket (unused in case of `TCP_SERVER`)
        int m_ownPort; //!< Port of the own socket
        unsigned long long m_maximumMessageSize; //!< Maximum size of single message (TCP and UDP) in bytes (Note that for TCP the serialized object may exceed this limit)

        // Threading options
        bool m_multiThreaded; //!< \copybrief parallel::BackgroundWorker::multiThreaded()
        int m_threadPriority; //!< \copybrief parallel::BackgroundWorker::threadPriority()
        core::Time m_minimumCycleTime; //!< \copybrief parallel::BackgroundWorker::minimumCycleTime()
        bool m_blockingWaiting; //!< \copybrief parallel::BackgroundWorker::blockingWaiting()

        // Buffer options
        size_t m_receivedObjectBufferSize; //!< Size of buffer for received objects
        size_t m_sendObjectBufferSize; //!< Size of buffer for sending objects

        // Serialization options
        serialization::Endianness m_endianness; //! Specifies the used byte-order for serialization and deserialization
    };

    //! \}
} // namespace io
} // namespace broccoli
