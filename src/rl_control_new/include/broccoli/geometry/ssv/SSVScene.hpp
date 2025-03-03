/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/Time.hpp"
#include "../../core/platform/PlatformHelper.hpp"
#include "../../parallel/SynchronizedWorker.hpp"
#include "../../parallel/SynchronizedWorkerPool.hpp"
#include "SSVDistanceEvaluator.hpp"
#include "SSVSceneData.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace geometry {
    //! Representation of a SSV **scene**
    /*!
     * \ingroup broccoli_geometry_ssv
     * A SSV scene describes the highest level in the SSV distance computation framework. It contains a collection of SSVSegment%s
     * as well as a list of pairs of SSVSegment%s for which the distance should be evaluated. The scene may be evaluated sequentially
     * (single-core) or in parallel (multi-core).
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVScene : public SSVSceneData {
        // Construction
        // ------------
    public:
        //! Default constructor
        SSVScene() = default;

        //! Specialized constructor
        /*! \param [in] name Initializes \ref name() - \copybrief name() */
        SSVScene(const std::string& name)
            : m_name(name)
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] name Initializes \ref name() - \copybrief name()
         * \param [in] threadCount Initializes \ref threadCountPreProcessing() and \ref threadCountProcessing()
         * \param [in] threadPriority Initializes \ref threadPriorityPreProcessing() and \ref threadPriorityProcessing()
         */
        SSVScene(const std::string& name, const size_t& threadCount, const int& threadPriority = parallel::BackgroundWorker::detectThreadPriority())
            : m_name(name)
            , m_threadCountPreProcessing(threadCount)
            , m_threadCountProcessing(threadCount)
            , m_threadPriorityPreProcessing(threadPriority)
            , m_threadPriorityProcessing(threadPriority)
        {
            reInitializeWorkersPreProcess();
            reInitializeWorkersProcess();
        }

        //! Copy constructor
        SSVScene(const SSVScene& original)
            : SSVSceneData(original)
            , m_name(original.m_name)
            , m_threadCountPreProcessing(original.m_threadCountPreProcessing)
            , m_threadCountProcessing(original.m_threadCountProcessing)
            , m_threadPriorityPreProcessing(original.m_threadPriorityPreProcessing)
            , m_threadPriorityProcessing(original.m_threadPriorityProcessing)
            , m_evaluatedElementPairs(original.m_evaluatedElementPairs)
            , m_runTimeRecomputeSegmentsToUpdate(original.m_runTimeRecomputeSegmentsToUpdate)
            , m_runTimeRecomputePairsToEvaluate(original.m_runTimeRecomputePairsToEvaluate)
            , m_runTimePreProcessing(original.m_runTimePreProcessing)
            , m_runTimeProcessing(original.m_runTimeProcessing)
            , m_runTime(original.m_runTime)
        {
            reInitializeWorkersPreProcess();
            reInitializeWorkersProcess();
        }

        //! Copy assignment operator
        SSVScene& operator=(const SSVScene& reference)
        {
            SSVSceneData::operator=(reference);
            m_name = reference.m_name;
            m_threadCountPreProcessing = reference.m_threadCountPreProcessing;
            m_threadCountProcessing = reference.m_threadCountProcessing;
            m_threadPriorityPreProcessing = reference.m_threadPriorityPreProcessing;
            m_threadPriorityProcessing = reference.m_threadPriorityProcessing;
            reInitializeWorkersPreProcess();
            reInitializeWorkersProcess();
            m_evaluatedElementPairs = reference.m_evaluatedElementPairs;
            m_runTimeRecomputeSegmentsToUpdate = reference.m_runTimeRecomputeSegmentsToUpdate;
            m_runTimeRecomputePairsToEvaluate = reference.m_runTimeRecomputePairsToEvaluate;
            m_runTimePreProcessing = reference.m_runTimePreProcessing;
            m_runTimeProcessing = reference.m_runTimeProcessing;
            m_runTime = reference.m_runTime;
            return *this;
        }

        // Members
        // -------
    protected:
        // Generic
        std::string m_name = "SSVScene"; //!< \copybrief name()
        // Parallel processing
        size_t m_threadCountPreProcessing = 0; //!< \copybrief threadCountPreProcessing()
        size_t m_threadCountProcessing = 0; //!< \copybrief threadCountProcessing()
        int m_threadPriorityPreProcessing = parallel::BackgroundWorker::detectThreadPriority(); //!< \copybrief threadPriorityPreProcessing()
        int m_threadPriorityProcessing = parallel::BackgroundWorker::detectThreadPriority(); //!< \copybrief threadPriorityProcessing()
        std::vector<parallel::SynchronizedWorker<SSVSceneData::SegmentToUpdateList::iterator>> m_workersPreProcessing; //!< List of workers used for preprocessing
        std::vector<parallel::SynchronizedWorker<SSVSceneData::PairToEvaluateList::iterator>> m_workersProcessing; //!< List of workers used for processing
        // Statistics
        size_t m_evaluatedElementPairs = 0; //!< \copybrief evaluatedElementPairs()
        core::Time m_runTimeReInitializeWorkersPreProcess = 0; //!< \copybrief runTimeReInitializeWorkersPreProcess()
        core::Time m_runTimeReInitializeWorkersProcess = 0; //!< \copybrief runTimeReInitializeWorkersProcess()
        core::Time m_runTimeRecomputeSegmentsToUpdate = 0; //!< \copybrief runTimeRecomputeSegmentsToUpdate()
        core::Time m_runTimeRecomputePairsToEvaluate = 0; //!< \copybrief runTimeRecomputePairsToEvaluate()
        core::Time m_runTimePreProcessing = 0; //!< \copybrief runTimePreProcessing()
        core::Time m_runTimeProcessing = 0; //!< \copybrief runTimeProcessing()
        core::Time m_runTime = 0; //!< \copybrief runTime()

        // Getters
        // -------
    public:
        //! Individual name of the scene (used to define thread names)
        inline const std::string& name() const { return m_name; }

        //! Count of threads to be used for pre-processing (updating segments) (0 and 1: single-threaded, >=2: multi-threaded)
        inline const size_t& threadCountPreProcessing() const { return m_threadCountPreProcessing; }

        //! Count of threads to be used for processing (distance calculation) (0 and 1: single-threaded, >=2: multi-threaded)
        inline const size_t& threadCountProcessing() const { return m_threadCountProcessing; }

        //! Priority of background threads for pre-processing (updating segments) (see \ref broccoli::parallel::BackgroundWorker::threadPriority())
        inline const int& threadPriorityPreProcessing() const { return m_threadPriorityPreProcessing; }

        //! Priority of background threads for processing (distance calculation) (see \ref broccoli::parallel::BackgroundWorker::threadPriority())
        inline const int& threadPriorityProcessing() const { return m_threadPriorityProcessing; }

        //! Total count of evaluated SSV element pairs in last run of \ref evaluate()
        inline const size_t& evaluatedElementPairs() const { return m_evaluatedElementPairs; }

        //! Runtime of \ref reInitializeWorkersPreProcess() in last call
        inline const core::Time& runTimeReInitializeWorkersPreProcess() const { return m_runTimeReInitializeWorkersPreProcess; }

        //! Runtime of \ref reInitializeWorkersProcess() in last call
        inline const core::Time& runTimeReInitializeWorkersProcess() const { return m_runTimeReInitializeWorkersProcess; }

        //! Runtime of \ref recomputeSegmentsToUpdate() in last run of \ref evaluate()
        inline const core::Time& runTimeRecomputeSegmentsToUpdate() const { return m_runTimeRecomputeSegmentsToUpdate; }

        //! Runtime of \ref recomputePairsToEvaluate() in last run of \ref evaluate()
        inline const core::Time& runTimeRecomputePairsToEvaluate() const { return m_runTimeRecomputePairsToEvaluate; }

        //! Runtime of pre-processing in last run of \ref evaluate()
        inline const core::Time& runTimePreProcessing() const { return m_runTimePreProcessing; }

        //! Runtime of processing in last run of \ref evaluate()
        inline const core::Time& runTimeProcessing() const { return m_runTimeProcessing; }

        //! Total runtime of \ref evaluate() in last call
        inline const core::Time& runTime() const { return m_runTime; }

        // Setters
        // -------
    public:
        //! Setter for \ref name() - \copybrief name()
        inline void setName(const std::string& name)
        {
            if (name == m_name)
                return;
            m_name = name;
            reInitializeWorkersPreProcess();
            reInitializeWorkersProcess();
        }

        //! Setter for \ref threadCountPreProcessing() - \copybrief threadCountPreProcessing()
        inline void setThreadCountPreProcessing(const size_t& threadCount)
        {
            if (threadCount == m_threadCountPreProcessing)
                return;
            m_threadCountPreProcessing = threadCount;
            reInitializeWorkersPreProcess();
        }

        //! Setter for \ref threadCountProcessing() - \copybrief threadCountProcessing()
        inline void setThreadCountProcessing(const size_t& threadCount)
        {
            if (threadCount == m_threadCountProcessing)
                return;
            m_threadCountProcessing = threadCount;
            reInitializeWorkersProcess();
        }

        //! Setter for \ref threadPriorityPreProcessing() - \copybrief threadPriorityPreProcessing()
        inline void setThreadPriorityPreProcessing(const int& threadPriority)
        {
            if (threadPriority == m_threadPriorityPreProcessing)
                return;
            m_threadPriorityPreProcessing = threadPriority;
            reInitializeWorkersPreProcess();
        }

        //! Setter for \ref threadPriorityProcessing() - \copybrief threadPriorityProcessing()
        inline void setThreadPriorityProcessing(const int& threadPriority)
        {
            if (threadPriority == m_threadPriorityProcessing)
                return;
            m_threadPriorityProcessing = threadPriority;
            reInitializeWorkersProcess();
        }

        // Evaluation
        // ----------
    public:
        //! Evaluates the distances between the SSV segments of all registered segment pairs in the scene
        /*! \return `true` if the scene was evaluated, `false` otherwise (if the output data did not change) */
        bool evaluate()
        {
            // Skip, if an update is not required
            if (distancesValid() == true)
                return false;

            // Reset statistics
            const core::Time startTime = core::Time::currentTime();
            m_evaluatedElementPairs = 0;

            // Check, if scene is properly defined
            assert(isValid());

            // Prepare list of operations
            const core::Time startTimeRecomputeSegmentsToUpdate = core::Time::currentTime();
            recomputeSegmentsToUpdate();
            m_runTimeRecomputeSegmentsToUpdate = core::Time::currentTime() - startTimeRecomputeSegmentsToUpdate;
            const core::Time startTimeRecomputePairsToEvaluate = core::Time::currentTime();
            recomputePairsToEvaluate();
            m_runTimeRecomputePairsToEvaluate = core::Time::currentTime() - startTimeRecomputePairsToEvaluate;

            // Step 1: Pre-Processing
            // ----------------------
            const core::Time startTimePreProcessing = core::Time::currentTime();
            auto& segments2Update = segmentsToUpdate();
            if (m_workersPreProcessing.size() == 0 || segments2Update.size() < m_workersPreProcessing.size() + 1 /* <- force single-threaded execution, if there are only very few operations */) {
                // Single-threaded
                for (size_t i = 0; i < segments2Update.size(); i++)
                    segments2Update[i].process();
            } else {
                // Multi-threaded
                runMultiThreaded(segments2Update, m_workersPreProcessing);
            }
            m_runTimePreProcessing = core::Time::currentTime() - startTimePreProcessing;

            // Step 2: Processing
            // ------------------
            const core::Time startTimeProcessing = core::Time::currentTime();
            auto& pairs2Evaluate = pairsToEvaluate();
            if (m_workersProcessing.size() == 0 || pairs2Evaluate.size() < m_workersProcessing.size() + 1 /* <- force single-threaded execution, if there are only very few operations */) {
                // Single-threaded
                for (size_t i = 0; i < pairs2Evaluate.size(); i++)
                    pairs2Evaluate[i].process();
            } else {
                // Multi-threaded
                runMultiThreaded(pairs2Evaluate, m_workersProcessing);
            }
            clearDistances();
            reserveDistances(pairs2Evaluate.size());
            for (size_t i = 0; i < pairs2Evaluate.size(); i++) {
                if (pairs2Evaluate[i].evaluatedElementPairs() > 0) {
                    if (maximumDistance() < 0 || pairs2Evaluate[i].distance().distance() <= maximumDistance())
                        addDistance(pairs2Evaluate[i].distance());
                    m_evaluatedElementPairs += pairs2Evaluate[i].evaluatedElementPairs();
                }
            }
            m_runTimeProcessing = core::Time::currentTime() - startTimeProcessing;

            // Finish update of distances
            finalizeDistances();

            // End runtime measurement
            m_runTime = core::Time::currentTime() - startTime;

            // The evaluation took place
            return true;
        }

        // Tuning
        // ------
    public:
        //! Finds the optimal thread count for pre-processing and processing based on the current scene status
        /*!
         * \param [in] minimumThreadCountPreProcessing The minimal allowed thread count for pre-processing
         * \param [in] maximumThreadCountPreProcessing The maximum allowed thread count for pre-processing
         * \param [in] minimumThreadCountProcessing The minimal allowed thread count for processing
         * \param [in] maximumThreadCountProcessing The maximum allowed thread count for processing
         * \param [in] sampleCount The count of samples used for computing the minimum runtime of a configuration
         */
        void autoTune(const size_t& minimumThreadCountPreProcessing, const size_t& maximumThreadCountPreProcessing, const size_t& minimumThreadCountProcessing, const size_t& maximumThreadCountProcessing, const size_t& sampleCount = 10)
        {
            assert(isValid());
            assert(maximumThreadCountPreProcessing >= minimumThreadCountPreProcessing);
            assert(maximumThreadCountProcessing >= minimumThreadCountProcessing);

            // Initialize helpers
            size_t optimalThreadCountPreProcessing = minimumThreadCountPreProcessing;
            size_t optimalThreadCountProcessing = minimumThreadCountProcessing;

            // Initial evaluation (to refresh all data which is not dependent on segment position and orientation)
            setThreadCountPreProcessing(optimalThreadCountPreProcessing);
            setThreadCountProcessing(optimalThreadCountProcessing);
            evaluate();

            // Get original segment data
            const auto segmentIDs = segmentIDList();
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> originalPositions;
            std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> originalOrientations;
            originalPositions.reserve(segmentIDs.size());
            originalOrientations.reserve(segmentIDs.size());
            for (size_t i = 0; i < segmentIDs.size(); i++) {
                originalPositions.push_back(segment(segmentIDs[i]).position());
                originalOrientations.push_back(segment(segmentIDs[i]).orientation());
            }

            // Loop over variations of possible thread counts
            core::Time shortestRunTime(0, 0);
            for (size_t i = minimumThreadCountPreProcessing; i <= maximumThreadCountPreProcessing; i++) {
                for (size_t j = minimumThreadCountProcessing; j <= maximumThreadCountProcessing; j++) {
                    // Set thread counts
                    setThreadCountPreProcessing(i);
                    setThreadCountProcessing(j);

                    // Repeat for each sample
                    for (size_t k = 0; k < sampleCount; k++) {
                        // Slightly vary segment positions and orientations
                        for (size_t l = 0; l < segmentIDs.size(); l++) {
                            const double x = 2.0 * ((double)k) / ((double)sampleCount) - 1.0; // Value between -1 and 1
                            const double y = 2.0 * ((double)l) / ((double)segmentIDs.size()) - 1.0; // Value between -1 and 1
                            const double z = (x + y) / 2.0; // Value between -1 and 1
                            const Eigen::Vector3d smallVector(1e-3 * x, 1e-3 * y, 1e-3 * z);
                            const Eigen::Vector3d rotationAxis = (Eigen::Vector3d(1.0, 1.0, 1.0) + smallVector).normalized();
                            const double rotationAngle = 2.0 * M_PI * 1e-3 * z;
                            setSegmentPosition(segmentIDs[l], originalPositions[l] + smallVector);
                            setSegmentOrientation(segmentIDs[l], originalOrientations[l] * Eigen::AngleAxisd(rotationAngle, rotationAxis).toRotationMatrix());
                        }

                        // Trigger evaluation and compare runtime
                        if (evaluate() == true) {
                            if ((i == minimumThreadCountPreProcessing && j == minimumThreadCountProcessing) || m_runTime < shortestRunTime) {
                                shortestRunTime = m_runTime;
                                optimalThreadCountPreProcessing = i;
                                optimalThreadCountProcessing = j;
                            }
                        }
                    }
                }
            }

            // Restore original segment positions and orientations
            for (size_t i = 0; i < segmentIDs.size(); i++) {
                setSegmentPosition(segmentIDs[i], originalPositions[i]);
                setSegmentOrientation(segmentIDs[i], originalOrientations[i]);
            }

            // Set optimal thread counts and run final evaluation (to get right distances)
            setThreadCountPreProcessing(optimalThreadCountPreProcessing);
            setThreadCountProcessing(optimalThreadCountProcessing);
            evaluate();
        }

        // Helpers
        // -------
    protected:
        //! Re-initializes a worker pool
        /*!
         * \param [out] workers The list of workers to re-initialize
         * \param [in] workerCount The total count of workers
         * \param [in] threadPriority The thread priority to be used
         * \param [in] namePrefix The prefix used for naming the workers
         */
        template <typename WorkerType>
        void reInitializeWorkers(std::vector<WorkerType>& workers, const size_t& workerCount, const int& threadPriority, const std::string& namePrefix)
        {
            // Skip, in single-threaded case
            if (workerCount == 0) {
                workers.clear();
                return;
            }

            // Create workers
            workers.clear();
            workers.reserve(workerCount);
            for (size_t i = 0; i < workerCount; i++) {
                workers.push_back(WorkerType(namePrefix + std::to_string(i), threadPriority));
                workers.back().initialize();
            }

            // Wait until all workers are ready
            if (parallel::SynchronizedWorkerPool::waitForReady(workers, 3) == false)
                throw std::runtime_error("SSVScene::reInitializeWorkers(): Timeout hit while waiting for workers to get ready!");
        }

        //! Re-initializes the worker pool for pre-processing
        void reInitializeWorkersPreProcess()
        {
            const core::Time startTime = core::Time::currentTime();
            size_t workerCount = 0;
            if (m_threadCountPreProcessing >= 2)
                workerCount = m_threadCountPreProcessing - 1; // One thread is the main thread
            reInitializeWorkers(m_workersPreProcessing, workerCount, m_threadPriorityPreProcessing, m_name + "_U");
            m_runTimeReInitializeWorkersPreProcess = core::Time::currentTime() - startTime;
        }

        //! Re-initializes the worker pool for processing
        void reInitializeWorkersProcess()
        {
            const core::Time startTime = core::Time::currentTime();
            size_t workerCount = 0;
            if (m_threadCountProcessing >= 2)
                workerCount = m_threadCountProcessing - 1; // One thread is the main thread
            reInitializeWorkers(m_workersProcessing, workerCount, m_threadPriorityProcessing, m_name + "_E");
            m_runTimeReInitializeWorkersProcess = core::Time::currentTime() - startTime;
        }

        //! Processes a given list of operations in parallel (making use of the given list of background workers)
        /*!
         * \tparam OperationList Type of list of operations to perform. The elements have to provide a `process()` and a `expectedCost()` method which are called internally.
         * \tparam Worker Type of the available workers.
         */
        template <typename OperationList, typename Worker>
        void runMultiThreaded(OperationList& operations, std::vector<Worker>& workers)
        {
            // Compute thread count (use all background workers + parent thread; parent thread is 1st thread)
            const size_t threadCount = workers.size() + 1;

            // Compute total and average cost per thread
            double totalCost = 0;
            for (size_t i = 0; i < operations.size(); i++)
                totalCost += operations[i].expectedCost();
            const double targetCostPerThread = totalCost / ((double)m_threadCountPreProcessing);

            // Wait for all workers to get ready (with timeout)
            if (parallel::SynchronizedWorkerPool::waitForReady(workers, 3) == false)
                throw std::runtime_error("SSVScene::runMultiThreaded(): Timeout hit while waiting for workers to get ready!");

            // Distribute operations depending on cost
            size_t parentThreadStartIndex = 0; // Start index for own (parent) thread
            size_t parentThreadEndIndex = 0; // End index for own (parent) thread
            size_t startIndex = 0; // Start index of the current thread
            size_t endIndex = 0; // End index of the current thread
            for (size_t i = 0; i < threadCount; i++) {
                const size_t remainingThreads = threadCount - i - 1; // Count of threads after this thread

                // Compute end index of current thread
                if (remainingThreads > 0) {
                    // ...not last thread -> set end index such that the current thread has workload of average cost
                    double threadCost = operations[startIndex].expectedCost();
                    endIndex = startIndex;
                    while (threadCost < targetCostPerThread && endIndex + 1 + remainingThreads < operations.size()) {
                        endIndex++;
                        threadCost += operations[endIndex].expectedCost();
                    }
                } else {
                    // ...last thread -> hard-set to last operation
                    endIndex = operations.size() - 1;
                }

                // Setup thread
                if (i == 0) {
                    // ...first thread -> use parent thread
                    parentThreadStartIndex = startIndex;
                    parentThreadEndIndex = endIndex;
                } else {
                    // ...not first thread -> use background worker
                    workers[i - 1].trigger(operations.begin() + startIndex, endIndex - startIndex + 1);
                }

                // Link start index of next thread with end index of this thread
                startIndex = endIndex + 1;
            }

            // Process workload of parent thread (while background threads are already running)
            for (size_t i = parentThreadStartIndex; i <= parentThreadEndIndex; i++)
                operations[i].process();

            // Wait for all workers to finish (no timeout)
            parallel::SynchronizedWorkerPool::waitForFinish(workers);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
