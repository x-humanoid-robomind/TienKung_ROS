/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "NetworkSocketOptions.hpp"

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_network
     * \{
     */

    //! Container for options of \ref NetworkSocket of type \ref NetworkSocketType::TCP_SERVER
    class TCPServerOptions : public NetworkSocketOptions {
    public:
        //! Constructor
        /*!
         * \param [in] name Initializes \ref m_name - \copybrief m_name
         * \param [in] ownPort Initializes \ref m_ownPort - \copybrief m_ownPort
         * \param [in] multiThreaded Initializes \ref m_multiThreaded - \copybrief m_multiThreaded
         * \param [in] threadPriority Initializes \ref m_threadPriority - \copybrief m_threadPriority
         * \param [in] minimumCycleTime Initializes \ref m_minimumCycleTime - \copybrief m_minimumCycleTime
         * \param [in] blockingWaiting Initializes \ref m_blockingWaiting - \copybrief m_blockingWaiting
         * \param [in] receivedObjectBufferSize Initializes \ref m_receivedObjectBufferSize - \copybrief m_receivedObjectBufferSize
         * \param [in] sendObjectBufferSize Initializes \ref m_sendObjectBufferSize - \copybrief m_sendObjectBufferSize
         * \param [in] endianness Initializes \ref m_endianness - \copybrief m_endianness
         */
        TCPServerOptions(const std::string& name, const int& ownPort, const bool& multiThreaded, const int& threadPriority, const core::Time& minimumCycleTime, const bool& blockingWaiting, const size_t& receivedObjectBufferSize, const size_t& sendObjectBufferSize, const serialization::Endianness& endianness = serialization::Endianness::LITTLE)
            : NetworkSocketOptions(NetworkSocketType::TCP_SERVER, name, "", 0, ownPort, multiThreaded, threadPriority, minimumCycleTime, blockingWaiting, receivedObjectBufferSize, sendObjectBufferSize, endianness)
        {
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
