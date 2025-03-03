/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../core/Time.hpp"
#include "../core/platform/PlatformHelperFactory.hpp"
#include "ThreadSafeContainer.hpp"
#include <pthread.h>
#include <string>

namespace broccoli {
namespace parallel {
    //! Multi- or single-threaded background worker abstraction
    /*!
     * \ingroup broccoli_parallel
     *
     * A background worker is an abstract class which continuously calls a function
     * \ref execute(). The worker optionally runs in a background thread (\p multiThreaded).
     * Calling \ref start() and \ref stop() triggers the start and stop of the execution
     * loop. While \ref start() and \ref stop() can only be called once, one can pause and
     * resume the execution loop with \ref pause() and \ref resume(). Moreover, enhanced
     * synchronization with the parent thread is possible with \ref triggerSynchronization()
     * and \ref waitForSynchronization().
     *
     * In the **multi-threaded** case you may want to wait until the background
     * thread finishes its work (i.e. exits the execution loop). This can be
     * done by calling \ref join().
     *
     * In **single-threaded** mode (no background thread) one has to trigger the
     * execution loop manually by \ref triggerManualExecution(). Note that you first
     * have to call \ref start(), otherwise execution will be skipped.
     *
     * The class is intended to be used as base class for derivation. You have at least to
     * implement the \ref execute() function. You may add possibilities for emergency stops
     * at appropriate places inside your \ref execute() function (e.g. in loops with long
     * runtimes):
     * - Example A for emergency stop:
     * \code
     * if(stopTriggered() == true)
     *    return;
     * \endcode
     * - Example B for emergency stop:
     * \code
     * while(... && stopTriggered() == false) {
     *    ...
     * }
     * \endcode
     *
     * \par Waiting Types
     * The class supports two waiting types (only in multi-threaded case):
     * -# "Low-performance" waiting: (\p blockWaiting = `false`)
     *  - calls `sleep` for active background thread
     *  - Advantage: low CPU usage
     *  - Disadvantage: inaccurate
     * -# "High-performance" waiting: (\p blockWaiting = `true`)
     *  - run dummy loop and wait for time to expire
     *  - Advantage: very accurate cycle time
     *  - Disadvantage: high CPU usage ("waisting" CPU)
     *
     * \par Advanced Synchronization
     * The class supports advanced synchronization between the parent and the own thread (based on
     * condition variables). A typical synchronization may look like this:
     *  - call \ref waitForSynchronization() in **own thread** (e.g. in \ref execute())
     *  - call \ref triggerSynchronization() in **parent thread**.
     *
     * The call \ref waitForSynchronization() blocks the own thread and waits (high precision, low CPU usage) for the parent
     * thread to call \ref triggerSynchronization().
     *
     * Note that \ref stop() also automatically calls \ref triggerSynchronization() to wake up the thread.
     *
     * \par Analytics
     * The class features automatic (and thread-safe) runtime statistics:
     *  - Cylce time measurement with \ref lastCycleTime()
     *  - Runtime measurements of last \ref execute() call with \ref lastExecutionRunTime()
     *  - Total runtime (accumulated) of all \ref execute() calls so far with \ref totalExecutionRunTime()
     *  - Count of calls of \ref execute() with \ref executionCalls()
     *  - ...
     *
     * \par Thread-safe Member Access
     * The class implements a generic thread-safe access to member variables. For this to
     * work for the members of your derived class you have to define setters and getters
     * for your additional member variables. Use \ref setProtectedData() and
     * \ref getProtectedData() as templates to use the common mutex.
     */
    class BackgroundWorker : public ThreadSafeContainer {
    public:
        //! Default constructor (runs in parent thread)
        /*!
         * \param [in] name Initializes: \copybrief name()
         * \param [in] multiThreaded Initializes: \copybrief multiThreaded()
         * \param [in] threadPriority Initializes: \copybrief threadPriority()
         * \param [in] minimumCycleTime Initializes: \copybrief minimumCycleTime()
         * \param [in] blockingWaiting Initializes: \copybrief blockingWaiting()
         */
        BackgroundWorker(const std::string& name, const bool& multiThreaded, const int& threadPriority, const core::Time& minimumCycleTime, const bool& blockingWaiting)
            : ThreadSafeContainer()
            , m_thread(0)
            , m_name(name)
            , m_multiThreaded(multiThreaded)
            , m_threadPriority(threadPriority)
            , m_minimumCycleTime(minimumCycleTime)
            , m_blockingWaiting(blockingWaiting)
            , m_threadRunning(false)
            , m_startTriggered(false)
            , m_stopTriggered(false)
            , m_joinTriggered(false)
            , m_pauseActive(false)
            , m_lastCycleTime(-1)
            , m_lastExecutionCallTime(0)
            , m_lastExecutionRunTime(-1)
            , m_totalExecutionRunTime(0)
            , m_executionCalls(0)
            , m_syncMutex(PTHREAD_MUTEX_INITIALIZER)
            , m_syncConditionalVariable(PTHREAD_COND_INITIALIZER)
            , m_syncWaiting(false)
            , m_syncTriggered(false)
        {
        }

        //! Copy constructor (**not** thread-safe, should only be called by thread-safe wrapper (to be implemented by derived class))
        /*! \param [in] original Reference to original object. */
        BackgroundWorker(const BackgroundWorker& original)
            : ThreadSafeContainer(original)
            , m_thread(0) // m_thread = DO NOT COPY FROM ORIGINAL (use own thread handle)
            , m_name(original.m_name)
            , m_multiThreaded(original.m_multiThreaded)
            , m_threadPriority(original.m_threadPriority)
            , m_minimumCycleTime(original.m_minimumCycleTime)
            , m_blockingWaiting(original.m_blockingWaiting)
            , m_threadRunning(original.m_threadRunning)
            , m_startTriggered(original.m_startTriggered)
            , m_stopTriggered(original.m_stopTriggered)
            , m_joinTriggered(original.m_joinTriggered)
            , m_pauseActive(original.m_pauseActive)
            , m_lastCycleTime(original.m_lastCycleTime)
            , m_lastExecutionCallTime(original.m_lastExecutionCallTime)
            , m_lastExecutionRunTime(original.m_lastExecutionRunTime)
            , m_totalExecutionRunTime(original.m_totalExecutionRunTime)
            , m_executionCalls(original.m_executionCalls)
            , m_syncMutex(PTHREAD_MUTEX_INITIALIZER) // m_syncMutex = DO NOT COPY FROM ORIGINAL (use own mutex)
            , m_syncConditionalVariable(PTHREAD_COND_INITIALIZER) // m_syncConditionalVariable = DO NOT COPY FROM ORIGINAL (use own conditional variable)
            , m_syncWaiting(false) // m_syncWaiting = DO NOT COPY FROM ORIGINAL (use own flag)
            , m_syncTriggered(false) // m_syncTriggered = DO NOT COPY FROM ORIGINAL (use own flag)
        {
        }

        //! Copy assignment operator (**not** thread-safe, should be protected by derived class)
        /*!
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        BackgroundWorker& operator=(const BackgroundWorker& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Copy data
            ThreadSafeContainer::operator=(reference);
            // m_thread = DO NOT COPY FROM ORIGINAL (use own thread handle)
            m_name = reference.m_name;
            m_multiThreaded = reference.m_multiThreaded;
            m_threadPriority = reference.m_threadPriority;
            m_minimumCycleTime = reference.m_minimumCycleTime;
            m_blockingWaiting = reference.m_blockingWaiting;
            m_threadRunning = reference.m_threadRunning;
            m_startTriggered = reference.m_startTriggered;
            m_stopTriggered = reference.m_stopTriggered;
            m_joinTriggered = reference.m_joinTriggered;
            m_pauseActive = reference.m_pauseActive;
            m_lastCycleTime = reference.m_lastCycleTime;
            m_lastExecutionCallTime = reference.m_lastExecutionCallTime;
            m_lastExecutionRunTime = reference.m_lastExecutionRunTime;
            m_executionCalls = reference.m_executionCalls;
            // m_syncMutex = DO NOT COPY FROM ORIGINAL (use own mutex)
            // m_syncConditionalVariable = DO NOT COPY FROM ORIGINAL (use own conditional variable)
            // m_syncWaiting = DO NOT COPY FROM ORIGINAL (use own flag)
            // m_syncTriggered = DO NOT COPY FROM ORIGINAL (use own flag)

            return *this;
        }

        //! Destructor
        virtual ~BackgroundWorker()
        {
            // Join thread (if not already triggered)
            join();
        }

        //! Function to start the execution (runs in parent thread)
        /*!
         * Can only be run **once** (calling it multiple times has no effect)
         * - Single-threaded case: enables manual execution via \ref triggerManualExecution()
         * - Multi-threaded case: initializes the background thread and starts its execution loop.
         *
         * \remark In single-threaded case \ref triggerManualExecution() is called automatically with \ref start().
         *
         * \return `true` on success, `false` on failure
         */
        bool start()
        {
            // Abort, if we started already
            if (startTriggered() == true)
                return false;

            setStartTriggered(true);

            // Check, if we should launch a background thread
            if (multiThreaded() == true) {
                // ...with background thread -> launch thread
                pthread_attr_t threadAttribute;
                pthread_attr_init(&threadAttribute);
                pthread_attr_setdetachstate(&threadAttribute, PTHREAD_CREATE_JOINABLE);
                if (pthread_create(&m_thread, &threadAttribute, triggerLoop, this) == 0) {
                    // Remember, that we successfully launched the thread
                    setThreadRunning(true);
                    return true;
                } else {
                    setThreadRunning(false);
                    return false;
                }
            }

            return true;
        }

        //! Function to pause the execution (runs in parent thread)
        /*!
         * Can be run multiple times. Pausing an already paused execution has no effect.
         * - Single-threaded case: disables manual execution via \ref triggerManualExecution()
         * - Multi-threaded case: waits within execution loop until \ref resume() or \ref stop() is triggered.
         *
         * \return `true` if worker was not paused, `false` if worker was already in pause state
         */
        bool pause()
        {
            // Skip, if we paused already
            if (pauseActive() == true)
                return false;
            setPauseActive(true);
            return true;
        }

        //! Function to resume a paused execution (runs in parent thread)
        /*!
         * Can be run multiple times. Resuming an already running execution has no effect.
         * - Single-threaded case: enables manual execution via \ref triggerManualExecution()
         * - Multi-threaded case: tells background thread to resume execution loop
         *
         * \return `true` if worker was paused, `false` if worker was not paused
         */
        bool resume()
        {
            // Skip, if we are not in paused state
            if (pauseActive() == false)
                return false;
            setPauseActive(false);
            return true;
        }

        //! Function to stop the execution (runs in parent OR own thread)
        /*!
         * Can only be run **once** (calling it multiple times has no effect)
         * - Single-threaded case: disables manual execution via \ref triggerManualExecution()
         * - Multi-threaded case: tells the background thread to exit the execution loop and finish ihts work
         *
         * \return `true`, if \ref stop() has not been triggered yet, false if \ref stop() has been triggered already
         */
        bool stop()
        {
            // Skip, if we triggered stop already
            if (stopTriggered() == true)
                return false;
            setStopTriggered(true);
            triggerSynchronization(); // Wake up own thread if it waits for synchronization
            return true;
        }

        //! Function to join the thread (**blocking** wait for exit) (runs in parent thread)
        /*!
         * - Single-threaded case: triggers \ref stop()
         * - Multi-threaded case: triggers \ref stop() and waits until background thread has finished its work
         *
         * \warning This function may **block** since it waits for the background thread to finish (in multi-threaded case).
         * However you can specify a \p timeout to set a maximum waiting time after which the thread is killed.
         *
         * \param [in] timeout Timeout for waiting until thread has finished. ("=0": do not wait (kill immediately), "<0": no timeout (wait forever), ">0": timeout in [s])
         * \return `true` if thread finished before timeout or has already been joined, `false` if thread hit timeout and had to be canceled
         */
        bool join(const core::Time& timeout = core::Time(1, 0))
        {
            // Skip, if we triggered join already
            if (joinTriggered() == true)
                return true;
            setJoinTriggered(true);

            // Stop thread, if it has not already
            stop();

            // Single-threaded case
            // --------------------
            // We are finished here...
            if (multiThreaded() == false)
                return true;

            // Multi-threaded case
            // -------------------
            // Check, if background thread is still running
            if (threadRunning() == true) {
                // Check timout specification
                if (timeout.isNegative()) {
                    // ...no timeout -> regular join with blocking waiting
                    pthread_join(m_thread, nullptr); // BLOCKING!
                    setThreadRunning(false);
                    return true;
                } else {
                    // Wait unitl thread finished, or timeout is hit
                    core::Time startOfWaiting = core::Time::currentTime();
                    while (core::Time::currentTime() - startOfWaiting < timeout) {
                        // Check, if thread has finished now
                        if (threadRunning() == false) {
                            // ...yes -> regular join
                            pthread_join(m_thread, nullptr); // BLOCKING! (only minimal blocking time, since flag "threadRunning" is immediately set before thread exit)
                            setThreadRunning(false);
                            return true;
                        }
                    }

                    // Timout-limit was hit -> try to join it (BLOCKING!), then quit and report failure
                    // (Note: we do not "cancel" the thread since this is dangerous and could lead to a crash of the whole application -> instead we take the risk of a freeze of the application)
                    pthread_join(m_thread, nullptr);
                    setThreadRunning(false);
                    return false;
                }
            } else {
                // Thread already finished its work -> join it to free its memory
                pthread_join(m_thread, nullptr);
                return true;
            }
        }

        //! Manual trigger for execution (runs in parent thread)
        /*!
         * - Single-threaded case: triggers one single call of \ref execute() (only if started and not paused)
         * - Multi-threaded case: does nothing
         */
        void triggerManualExecution()
        {
            // Skip, if we are running an own background thread
            // (execution is triggered automatically in this case)
            if (multiThreaded() == true)
                return;

            // Skip, if we did not start yet or if we stopped already, or execution is paused
            if (startTriggered() == false || stopTriggered() == true || pauseActive() == true)
                return;

            // Update timing
            core::Time currentTime = core::Time::currentTime();
            core::Time currentCycleTime = currentTime - lastExecutionCallTime();

            // Update statistics
            incrementExecutionCalls();

            // Manually trigger call of execution
            core::Time startTime = core::Time::currentTime();
            execute();
            core::Time endTime = core::Time::currentTime();

            // Update timing
            setLastCycleTime(currentCycleTime);
            setLastExecutionCallTime(currentTime);
            setLastExecutionRunTime(endTime - startTime);
            updateTotalExecutionRunTime();
        }

        //! Triggers synchronization of parent thread with own thread (runs in parent thread)
        /*!
         * - Single-threaded case: does nothing and returns `false`
         * - Multi-threaded case: Wakes up own thread if it waits for synchronization. In own thread the function \ref waitForSynchronization() has to be called in advance (e.g. in \ref execute()).
         *
         * \warning This function has to be called from the **parent thread**. Calling it from own thread results in undefined behaviour.
         *
         * \return `true` if own thread was waiting for synchronization, `false` otherwise or in case of an error
         */
        bool triggerSynchronization()
        {
            // Initialize helpers
            bool returnValue = false;

            // Skip, if we are running in single-threaded mode
            // (synchronization has to be done manually by triggerManualExecution)
            if (multiThreaded() == false)
                return returnValue;

            // Trigger conditional variable
            pthread_mutex_lock(&m_syncMutex); // Get lock for synchronization
            if (m_syncWaiting == true)
                returnValue = true;
            m_syncTriggered = true; // Tell own thread that we triggered synchronization
            pthread_cond_signal(&m_syncConditionalVariable); // Send signal to wake up own thread
            pthread_mutex_unlock(&m_syncMutex); // Release lock for synchronization

            // Pass back result
            return returnValue;
        }

    protected:
        //! Starts the execution loop (only for multi-threaded version) (runs in own thread)
        /*!
         * Calls a static function with a reference to this object. This allows us to call the \ref loop() function of the
         * object. Inside the loop function we have access to all members of the object (=shared memory).
         * \param [in] _this Pointer to current instance
         * \return null pointer
         */
        static void* triggerLoop(void* _this)
        {
            // Setup thread priority and name
            // ------------------------------
            const auto platformHelper = core::PlatformHelperFactory::create();
            platformHelper->setThreadName(((BackgroundWorker*)_this)->name());
            platformHelper->setThreadPriority(((BackgroundWorker*)_this)->threadPriority());

            // Trigger execution loop
            // ----------------------
            ((BackgroundWorker*)_this)->loop();
            return nullptr;
        }

        //! Execution loop (only for multi-threaded version) (runs in own thread)
        /*!
         * Main execution loop function which periodically triggers \ref execute(). The loop can be paused and stopped by
         * triggering \ref pause() and \ref stop() respectively.
         */
        void loop()
        {
            // Trigger execution until stop is triggered
            while (stopTriggered() == false) {
                // Check timing and pause status
                core::Time currentTime = core::Time::currentTime();
                core::Time currentCycleTime = currentTime - lastExecutionCallTime();
                if (currentCycleTime >= minimumCycleTime() && pauseActive() == false) {
                    // ... trigger Execution

                    // Update statistics
                    incrementExecutionCalls();

                    // Trigger a single execution cycle
                    core::Time startTime = core::Time::currentTime();
                    execute();
                    core::Time endTime = core::Time::currentTime();

                    // Update timing
                    setLastCycleTime(currentCycleTime);
                    setLastExecutionCallTime(currentTime);
                    setLastExecutionRunTime(endTime - startTime);
                    updateTotalExecutionRunTime();
                } else {
                    // ... wait

                    // Gather protected memory first for fast access without mutex locking
                    const core::Time currentMinimumCycleTime = minimumCycleTime();
                    const core::Time currentLastExecutionCallTime = lastExecutionCallTime();

                    // Select waiting type ...
                    if (blockingWaiting() == true) {
                        // "High-performance waiting"
                        // We do not use core::sleep here since we want to be able to abort the waiting in case "stop" has been triggered
                        while (stopTriggered() == false && (core::Time::currentTime() - currentLastExecutionCallTime >= currentMinimumCycleTime)) {
                            // Do nothing
                        }
                    } else {
                        // "Low-performance waiting"
                        core::Time::sleep(currentMinimumCycleTime - (currentTime - currentLastExecutionCallTime), false);
                    }
                }
            }

            // Clean-up afterwards: proper exit of the thread
            setThreadRunning(false);
            pthread_exit(nullptr);
        }

        //! Waits for the synchronization of parent thread with own thread (runs in own thread)
        /*!
         * - Single-threaded case: does nothing and return `false`
         * - Multi-threaded case: Blocks own thread with minimal CPU usage until \ref triggerSynchronization() is called by the parent thread.
         *
         * \warning This function has to be called from the **own thread** (e.g. from \ref execute()). Calling it from the parent thread results in undefined behavior.
         *
         * \param [in] timeout Maximum time to wait (use <=0 for no timeout)
         *
         * \return `true` on success (i.e. if a proper wakeup signal was received), `false` otherwise (i.e. in case of an error, or if timeout was hit)
         */
        bool waitForSynchronization(const core::Time& timeout = 0)
        {
            // Initialize helpers
            bool receivedSignal = false; // Flag indicating, if a proper wakeup signal was received
            const bool withTimeout = (timeout > 0); // Flag indicating, if a timeout should be used
            const timespec timeoutAbsoluteTime = (core::Time::currentTime() + timeout).toTimeSpec(); // System time at which the waiting should be aborted

            // Skip, if we are running in single-threaded mode
            // (synchronization has to be done manually by triggerManualExecution)
            if (multiThreaded() == false)
                return receivedSignal;

            // Skip, if stop has been triggered already (we do not want to block the thread in this case)
            if (stopTriggered() == true)
                return receivedSignal;

            // Wait
            pthread_mutex_lock(&m_syncMutex);
            m_syncWaiting = true;
            m_syncTriggered = false;
            while (m_syncTriggered == false && stopTriggered() == false) {
                // Trigger wait for conditional variable
                // Note: pthread_cond_timedwait() and pthread_cond_wait() unlock the mutex, wait for the wakeup signal and then lock the mutex again!
                int waitResult = 0;
                if (withTimeout == true)
                    waitResult = pthread_cond_timedwait(&m_syncConditionalVariable, &m_syncMutex, &timeoutAbsoluteTime);
                else
                    waitResult = pthread_cond_wait(&m_syncConditionalVariable, &m_syncMutex);

                // Check return value
                if (waitResult == 0)
                    receivedSignal = true; // A proper signal was received!
                else
                    break; // An error occured! -> stop waiting
            }
            m_syncWaiting = false;
            pthread_mutex_unlock(&m_syncMutex);

            // Pass back result
            return receivedSignal;
        }

    protected:
        //! Dummy implementation of one execution cycle
        /*!
         * Specifies code to be executed periodically within the execution loop.
         * If only a single execution is necessary, call \ref stop() at the end of the \ref execute() function.
         */
        virtual void execute() = 0;

    protected:
        pthread_t m_thread; //!< Own background thread

        // Constants (do not have to be protected, because they are read-only)
        // ---------
        std::string m_name; //!< \copybrief name()
        bool m_multiThreaded; //!< \copybrief multiThreaded()
        int m_threadPriority; //!< \copybrief threadPriority()
        core::Time m_minimumCycleTime; //!< \copybrief minimumCycleTime()
        bool m_blockingWaiting; //!< \copybrief blockingWaiting()

        // Protected worker data
        // ---------------------
        // Start protected area
        bool m_threadRunning; //!< \copybrief threadRunning()
        bool m_startTriggered; //!< \copybrief startTriggered()
        bool m_stopTriggered; //!< \copybrief stopTriggered()
        bool m_joinTriggered; //!< \copybrief joinTriggered()
        bool m_pauseActive; //!< \copybrief pauseActive()
        core::Time m_lastCycleTime; //!< \copybrief lastCycleTime()
        core::Time m_lastExecutionCallTime; //!< \copybrief lastExecutionCallTime()
        core::Time m_lastExecutionRunTime; //!< \copybrief lastExecutionRunTime()
        core::Time m_totalExecutionRunTime; //!< \copybrief totalExecutionRunTime()
        unsigned long long m_executionCalls; //!< \copybrief executionCalls()
        // End protected area

        // Protected data for advanced synchronization (between parent thread and own thread)
        // -------------------------------------------
        mutable pthread_mutex_t m_syncMutex; //!< Mutex used for advanced synchronization between parent thread and own thread
        mutable pthread_cond_t m_syncConditionalVariable; //!< Conditional variable for advanced synchronization between parent thread and own thread
        // Start protected area of m_syncMutex
        mutable bool m_syncWaiting = false; //!< Flag indicating, if own thread is waiting for synchronization
        mutable bool m_syncTriggered = false; //!< Flag to indicate, if the synchronization between parent thread and own thread has been triggered
        // End protected area of m_syncMutex

    protected:
        using ThreadSafeContainer::setProtectedData;

        //! Setter template for protected data (**deprecated**)
        /*!
         * \warning Deprecated! Use ThreadSafeContainer::setProtectedData() instead!
         *
         * Sets the value of a protected data member. Thread-safety is guaranteed by a mutex.
         * \param [in] newValue New value for data member.
         * \param [in] data Pointer to data member
         */
        template <typename T>
        inline void setProtectedData(const T& newValue, T* data) { setProtectedData(*data, newValue); }

        using ThreadSafeContainer::getProtectedData;

        //! Getter template for protected data (**deprecated**)
        /*!
         * \warning Deprecated! Use ThreadSafeContainer::getProtectedData() instead!
         *
         * Gets the value of a protected data member. Thread-safety is guaranteed by a mutex.
         * \param [in] data Pointer to data member
         * \return Value of data member (copy)
         */
        template <typename T>
        inline T getProtectedData(const T* data) const { return getProtectedData(*data); }

        // Getters (thread-safe)
        // ---------------------
    public:
        // Constants -> no protection necessary
        //! Name of the worker. Used as name of an optional background thread. \details **Thread-safe getter**
        inline std::string name() const { return m_name; }
        //! Flag indicating, if this worker runs on its own thread \details **Thread-safe getter**
        inline bool multiThreaded() const { return m_multiThreaded; }
        //! Priority of the background worker (only in multi-threaded case) \details **Thread-safe getter**
        inline int threadPriority() const { return m_threadPriority; }
        //! Minimum time between two execution calls (use 0 to run on maximum speed) \details **Thread-safe getter**
        inline core::Time minimumCycleTime() const { return m_minimumCycleTime; }
        //! Flag to enable waiting under hard real-time constraints (if `true`: blocking loop, accurate cycle time, if `false`: call of `nanosleep()`, varying cycle time) \details **Thread-safe getter**
        inline bool blockingWaiting() const { return m_blockingWaiting; }

        // Dynamic members
        //! Flag indicating, if the background thread is currently running \details **Thread-safe getter**
        inline bool threadRunning() const { return getProtectedData(m_threadRunning); }
        //! Flag indicating, if we triggered \ref start() \details **Thread-safe getter**
        inline bool startTriggered() const { return getProtectedData(m_startTriggered); }
        //! Flag indicating, if we triggered \ref stop() \details **Thread-safe getter**
        inline bool stopTriggered() const { return getProtectedData(m_stopTriggered); }
        //! Flag indicating, if we triggered \ref join() \details **Thread-safe getter**
        inline bool joinTriggered() const { return getProtectedData(m_joinTriggered); }
        //! Flag indicating, if pause state is activated \details **Thread-safe getter**
        inline bool pauseActive() const { return getProtectedData(m_pauseActive); }
        //! Time between last two execution calls \details **Thread-safe getter**
        inline core::Time lastCycleTime() const { return getProtectedData(m_lastCycleTime); }
        //! Time of last call of execution function (system time) \details **Thread-safe getter**
        inline core::Time lastExecutionCallTime() const { return getProtectedData(m_lastExecutionCallTime); }
        //! Runtime measurement of last execution call \details **Thread-safe getter**
        inline core::Time lastExecutionRunTime() const { return getProtectedData(m_lastExecutionRunTime); }
        //! Runtime measurement for all execution calls so far (accumulated) \details **Thread-safe getter**
        inline core::Time totalExecutionRunTime() const { return getProtectedData(m_totalExecutionRunTime); }
        //! Counter for execution calls \details **Thread-safe getter**
        inline unsigned long long executionCalls() const { return getProtectedData(m_executionCalls); }

        // Synchronization variables
        //! Flag indicating, if own thread is waiting for synchronization
        inline bool syncWaiting() const
        {
            pthread_mutex_lock(&m_syncMutex);
            const bool returnValue = m_syncWaiting;
            pthread_mutex_unlock(&m_syncMutex);
            return returnValue;
        }

        // Setters (thread-safe)
        // ---------------------
        // Dynamic members
    protected:
        //! **Thread-safe setter for:** \copybrief threadRunning()
        inline void setThreadRunning(const bool& newValue) { setProtectedData(m_threadRunning, newValue); }
        //! **Thread-safe setter for:** \copybrief startTriggered()
        inline void setStartTriggered(const bool& newValue) { setProtectedData(m_startTriggered, newValue); }
        //! **Thread-safe setter for:** \copybrief stopTriggered()
        inline void setStopTriggered(const bool& newValue) { setProtectedData(m_stopTriggered, newValue); }
        //! **Thread-safe setter for:** \copybrief joinTriggered()
        inline void setJoinTriggered(const bool& newValue) { setProtectedData(m_joinTriggered, newValue); }
        //! **Thread-safe setter for:** \copybrief pauseActive()
        inline void setPauseActive(const bool& newValue) { setProtectedData(m_pauseActive, newValue); }
        //! **Thread-safe setter for:** \copybrief lastCycleTime()
        inline void setLastCycleTime(const core::Time& newValue) { setProtectedData(m_lastCycleTime, newValue); }
        //! **Thread-safe setter for:** \copybrief lastExecutionCallTime()
        inline void setLastExecutionCallTime(const core::Time& newValue) { setProtectedData(m_lastExecutionCallTime, newValue); }
        //! **Thread-safe setter for:** \copybrief lastExecutionRunTime()
        inline void setLastExecutionRunTime(const core::Time& newValue) { setProtectedData(m_lastExecutionRunTime, newValue); }
        //! Add last execution runtime to accumulated sum of all execution calls (**thread-safe**)
        inline void updateTotalExecutionRunTime() { protectedDataAddition(m_totalExecutionRunTime, m_lastExecutionRunTime); }
        //! Increase counter of execution calls by one (**thread-safe**)
        inline void incrementExecutionCalls() { protectedDataAddition(m_executionCalls, (decltype(m_executionCalls))1); }

        // Helpers
        // -------
    public:
        //! Returns the thread priority of the thread which calls this function
        static inline int detectThreadPriority()
        {
            const auto platform = core::PlatformHelperFactory::create();
            return platform->threadPriority();
        }
    };
} // namespace parallel
} // namespace broccoli
