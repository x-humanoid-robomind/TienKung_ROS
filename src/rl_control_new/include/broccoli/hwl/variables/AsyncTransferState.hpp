/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../ForwardDeclarations.hpp"

namespace broccoli {
namespace hwl {

    /*!
     * \brief Transfer state container for asynchronous bus variables
     * \ingroup broccoli_hwl_variables
     */
    class AsyncTransferState {
    public:
        friend class AsyncTransferStateHandle;
        /*!
         * \brief Indicates the last transfer has finished successfully and the value of the variable is certain.
         * \return True if bus variable is up-to-date (no known uncertainty on its value)
         */
        bool isLatched() const noexcept { return !m_transferPending && !m_possibleChange && !m_transferFailed && m_transferFinished; }

        //! Is a transfer already in progress?
        bool isPending() const noexcept { return m_transferPending; }

        /*!
         * \brief Indicates a transfer has finished
         * \warning The transfer may also have failed (timed out), check failed()
         */
        bool hasFinished() const noexcept { return m_transferFinished; }

        //! True if the last transfer failed
        bool hasFailed() const noexcept { return m_transferFailed; }

        //! True if it is known that the value of the object has changed since start of the last transfer
        bool isUncertain() const noexcept { return m_possibleChange; }

        //! Marks current value as uncertain
        /*!
         * \brief Marks current value as uncertain
         *
         * This is used to keep track of changes to a bus variable when a transfer
         * is currently in progress.
         */
        void markUncertain() noexcept { m_possibleChange = true; }

        /*!
         * \brief Marks current value as known
         *
         * This may be used to indicate the current value of a bus variable
         * is known from other sources. When no transfer is currently pending,
         * isLatched() will be true immediately.
         */
        void markKnown() noexcept
        {
            m_possibleChange = false;

            if (!isPending()) {
                m_transferFinished = true;
                m_transferFailed = false;
            }
        }

    protected:
        //! Indicates the last transfer finished (not necessarily successfully)
        bool m_transferFinished = false;

        //! Indicates if the last transfer failed
        bool m_transferFailed = false;

        //! Indicates a transfer on the bus is pending
        bool m_transferPending = false;

        //! Indicates the value possibly changed since last start of a transfer
        bool m_possibleChange = false;
    };

} // namespace hwl
} // namespace broccoli
