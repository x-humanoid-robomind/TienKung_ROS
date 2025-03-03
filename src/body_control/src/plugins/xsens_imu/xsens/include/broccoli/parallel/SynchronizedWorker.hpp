/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "BackgroundWorker.hpp"

namespace broccoli {
namespace parallel {
    //! A background worker which can be used for synchronous processing of multiple operations
    /*!
     * \ingroup broccoli_parallel
     *
     * This class is meant to build a pool of synchronized background workers which process a given list of operations in parallel.
     * Exemplary usage:
     *
     *   * Preparation:
     *     * Define `Operation` class which has to provide a method `process()` defining the actual work to be done (will be triggerd by worker).
     *   * Initialization:
     *     * Create and initialize pool of synchronized workers (see \ref initialize()).
     *   * May be run in a loop:
     *     * Wait for all workers to be ready (can be checked with \ref ready())
     *     * Update global list of operations to be processed (remove finished operations and add new ones)
     *     * Trigger workers and specify the "range" of operations for each workers from the list (see \ref trigger())
     *     * Wait for all workers to be finished (can be checked with \ref finished())
     *   * Deinitialization:
     *     * Worker object might just be destroyed (wake-up and de-initialization is called internally)
     *
     * \tparam OperationIterator Type of the iterator to a specific operation in the list to process (e.g. std::vector<MyOperation>::iterator).
     */
    template <typename OperationIterator>
    class SynchronizedWorker : protected parallel::BackgroundWorker {
        // Construction
        // ------------
    public:
        //! Constructor
        /*!
         * \param [in] name Initializes \ref name() - \copybrief name()
         * \param [in] threadPriority Initializes \ref threadPriority() - \copybrief threadPriority()
         */
        SynchronizedWorker(const std::string& name, const int& threadPriority)
            : BackgroundWorker(name, true, threadPriority, 0, false)
        {
        }

        //! Destructor
        ~SynchronizedWorker()
        {
            // De-initialize (if not done already)
            deInitialize();
        }

        // Members
        // -------
    protected:
        // Start protected data (internal mutex)
        OperationIterator m_firstOperation; //!< Iterator pointing to the first operation to be processed by this worker
        size_t m_operationCount = 0; //!< Specifies count of operations to be processed by this worker (starting from the first operation) (0 indicates, that there are no operations to process)
        bool m_finished = true; //!< \copybrief finished()
        // End protected data (internal mutex)

        // Main interface
        // --------------
    public:
        //! Initializes the worker
        /*! \return `true` on success, `false` on failure */
        inline bool initialize() { return start(); }

        //! Flag indicating, if the worker is ready to be triggered
        inline bool ready() const { return syncWaiting(); }

        //! Triggers execution of worker
        /*!
         * \param [in] firstOperation \copybrief m_firstOperation
         * \param [in] operationCount \copybrief m_operationCount
         */
        inline void trigger(const OperationIterator& firstOperation, const size_t& operationCount)
        {
            lockForWrite();
            m_firstOperation = firstOperation;
            m_operationCount = operationCount;
            m_finished = false;
            unlock();
            triggerSynchronization();
        }

        //! Flag indicating, if the worker finished its work \details **Thread-safe getter**
        inline bool finished() const { return getProtectedData(m_finished); }

        //! Deinitializes the worker
        /*!
         * \param [in] timeout Timeout for waiting until thread has finished. ("=0": do not wait (kill immediately), "<0": no timeout (wait forever), ">0": timeout in [s])
         * \return `true` if thread finished before timeout or has already been joined, `false` if thread hit timeout and had to be canceled
         */
        inline bool deInitialize(const core::Time& timeout = core::Time(1, 0)) { return join(timeout); }

        // Main functionality
        // ------------------
    protected:
        //! Execution cycle
        void execute()
        {
            // Wait until new data is ready to be processed
            waitForSynchronization();
            if (stopTriggered() == true)
                return;

            // Get input data
            lockForRead();
            OperationIterator currentOperation = m_firstOperation;
            const size_t operationCount = m_operationCount;
            unlock();
            if (operationCount == 0)
                return; // Nothing to do

            // Process
            for (size_t i = 0; i < operationCount; i++) {
                currentOperation->process();
                currentOperation++;
            }

            // Finish
            lockForWrite();
            m_operationCount = 0;
            m_finished = true;
            unlock();
        }
    };
} // namespace parallel
} // namespace broccoli
