/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../ForwardDeclarations.hpp"
#include <functional>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Handle for AsyncTransferState on BusDriver side
     * \ingroup broccoli_hwl_variables
     *
     * Ties to a reference of an AsyncTransferState with interface
     * for BusDriver manipulation of the state.
     */
    class AsyncTransferStateHandle {
    public:
        /*!
         * \brief Constructs a BusDriver-side handle for an AsyncTransferState
         * \param state Reference to the AsyncTransferState
         */
        explicit AsyncTransferStateHandle(AsyncTransferState& state)
            : m_state(std::ref(state))
        {
        }

        //! Has a transfer been started already?
        bool isTransferPending() const { return m_state.get().m_transferPending; }

        //! Indicate a transfer is pending now
        void indicatePendingTransfer()
        {
            m_state.get().m_transferPending = true;
            m_state.get().m_possibleChange = false;
            m_state.get().m_transferFailed = false;
            m_state.get().m_transferFinished = false;
        }

        //! Indicate a transfer finished but failed
        void indicateFailedTransfer()
        {
            m_state.get().m_transferPending = false;
            m_state.get().m_transferFailed = true;
            m_state.get().m_transferFinished = true;
        }

        //! Indicate a transfer finished successfully
        void indicateSuccessfulTransfer()
        {
            m_state.get().m_transferPending = false;
            m_state.get().m_transferFailed = false;
            m_state.get().m_transferFinished = true;
        }

    protected:
        //! Reference to the AsyncState object
        std::reference_wrapper<AsyncTransferState> m_state;
    };

} // namespace hwl
} // namespace broccoli
