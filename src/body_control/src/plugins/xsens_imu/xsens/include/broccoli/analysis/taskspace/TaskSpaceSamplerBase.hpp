/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/logging/LogFile.hpp"
#include "../../parallel/BackgroundWorker.hpp"
#include "TaskSpaceEvaluatorInput.hpp"
#include "TaskSpaceEvaluatorOutput.hpp"
#include "TaskSpaceSample.hpp"
#include "TaskSpaceSampleEvaluator.hpp"
#include <memory>

namespace broccoli {
namespace analysis {
    //! Abstraction of a background worker for sampling the taskspace
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     *  \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L>
    class TaskSpaceSamplerBase : public parallel::BackgroundWorker {
    public:
        //! Specialized constructor
        /*!
         * \param [in] name Initializes \ref m_name - \copybrief m_name
         * \param [in] input Initializes \ref m_input - \copybrief m_input
         * \param [in] firstSampleIndex Initializes \ref m_firstSampleIndex - \copybrief m_firstSampleIndex
         * \param [in] lastSampleIndex Initializes \ref m_lastSampleIndex - \copybrief m_lastSampleIndex
         * \param [in] writeOutputFile Initializes \ref m_writeOutputFile - \copybrief m_writeOutputFile
         * \param [in] output Initializes \ref m_output - \copybrief m_output
         */
        TaskSpaceSamplerBase(const std::string& name, const TaskSpaceEvaluatorInput<L>& input, const uint64_t& firstSampleIndex, const uint64_t& lastSampleIndex, const bool& writeOutputFile, TaskSpaceEvaluatorOutput<L>& output)
            : BackgroundWorker(name, true, 0, 0, false)
            , m_input(input)
            , m_firstSampleIndex(firstSampleIndex)
            , m_lastSampleIndex(lastSampleIndex)
            , m_writeOutputFile(writeOutputFile)
            , m_output(output)
        {
        }

    protected:
        //! Copy constructor (internal)
        /*!
         * \warning **Not thread-safe** -> should only be called by the thread-safe wrapper!
         *
         * \param [in] original Reference to original object.
         */
        TaskSpaceSamplerBase(const TaskSpaceSamplerBase& original, const int& /* <- trick used for locking mutex */)
            : BackgroundWorker(original)
            , m_input(original.m_input)
            , m_firstSampleIndex(original.m_firstSampleIndex)
            , m_lastSampleIndex(original.m_lastSampleIndex)
            , m_writeOutputFile(original.m_writeOutputFile)
            , m_minimumPosition(original.m_minimumPosition)
            , m_maximumPosition(original.m_maximumPosition)
            , m_output(original.m_output)
            , m_evaluatedSamples(original.m_evaluatedSamples)
            , m_success(original.m_success)
            , m_errorMessage(original.m_errorMessage)
            , m_finished(original.m_finished)
        {
        }

    public:
        //! Copy constructor (wrapper) (thread-safe)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        TaskSpaceSamplerBase(const TaskSpaceSamplerBase& original)
            : TaskSpaceSamplerBase(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (thread-safe)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         *
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        TaskSpaceSamplerBase& operator=(const TaskSpaceSamplerBase& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            BackgroundWorker::operator=(reference);
            m_input = reference.m_input;
            m_firstSampleIndex = reference.m_firstSampleIndex;
            m_lastSampleIndex = reference.m_lastSampleIndex;
            m_writeOutputFile = reference.m_writeOutputFile;
            m_minimumPosition = reference.m_minimumPosition;
            m_maximumPosition = reference.m_maximumPosition;
            m_output = reference.m_output;
            m_evaluatedSamples = reference.m_evaluatedSamples;
            m_success = reference.m_success;
            m_errorMessage = reference.m_errorMessage;
            m_finished = reference.m_finished;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

        //! Virtual destructor
        virtual ~TaskSpaceSamplerBase()
        {
        }

        // Members
        // -------
    protected:
        // Constants (do not have to be protected, because they are read-only)
        TaskSpaceEvaluatorInput<L> m_input; //!< \copybrief input()
        uint64_t m_firstSampleIndex; //!< \copybrief firstSampleIndex()
        uint64_t m_lastSampleIndex; //!< \copybrief lastSampleIndex()
        bool m_writeOutputFile; //!< \copybrief writeOutputFile()
        // Internal data (only accessed by background thread)
        std::unique_ptr<TaskSpaceSample> m_currentSample; //!< Internal buffer for current evaluated sample
        Eigen::Vector3d m_minimumPosition = Eigen::Vector3d::Zero(); //!< Minimum position (x, y, z) of tcp frame from all evaluated samples
        Eigen::Vector3d m_maximumPosition = Eigen::Vector3d::Zero(); //!< Maximum position (x, y, z) of tcp frame from all evaluated samples
        std::unique_ptr<io::LogFile<TaskSpaceSample>> m_outputFile; //!< Output data file container
        // Output data container (thread-safe on its own)
        TaskSpaceEvaluatorOutput<L>& m_output; //!< Reference to output data storage (thread-safe on its own)
        // Start protected area
        uint64_t m_evaluatedSamples = 0; //!< \copybrief evaluatedSamples()
        bool m_success = false; //!< \copybrief success()
        std::string m_errorMessage = ""; //!< \copybrief errorMessage()
        bool m_finished = false; //!< \copybrief finished()
        // End protected area

        //! Execution loop of background thread
        void execute()
        {
            // Initialization
            // --------------
            const size_t chainLength = input().m_kinematicChain.segments().size(); // Count of segments
            std::vector<uint64_t> sampleCount; // Count of samples for each segment
            std::vector<uint64_t> subSampleCount; // Count of sub-samples for each sample of this segment
            std::vector<uint64_t> localSampleIndex; // Local index ("coordinates") of sample for each segment
            sampleCount.resize(chainLength, 0);
            subSampleCount.resize(chainLength, 0);
            localSampleIndex.resize(chainLength, 0);
            for (size_t i = 0; i < chainLength; i++) {
                sampleCount[i] = input().m_kinematicChain.segment(i).samples();
                for (int i = (int)chainLength - 1; i >= 0; i--) {
                    if (i == (int)chainLength - 1)
                        subSampleCount[i] = 1; // Lowest level -> 1 subsample
                    else
                        subSampleCount[i] = sampleCount[i + 1] * subSampleCount[i + 1];
                }
            }
            std::vector<double> stepSize(chainLength, 0.0); // Step size for each frame
            for (size_t i = 0; i < chainLength; i++) {
                const KinematicChainSegment& currentSegment = input().m_kinematicChain.segment(i);
                const double range = fabs(currentSegment.maximumPosition() - currentSegment.minimumPosition());
                if (currentSegment.samples() <= 1)
                    stepSize[i] = range;
                else
                    stepSize[i] = range / ((double)currentSegment.samples() - 1.0);
            }
            m_currentSample = std::make_unique<TaskSpaceSample>();
            TaskSpaceSampleEvaluator sampleEvaluator(input().m_kinematicChain, input().m_taskSpaceSelectionMatrix); // Helper class for evaluating samples

            // Abort, if stop has been triggered from "outside"
            if (stopTriggered() == true) {
                emergencyExit("aborted");
                return;
            }

            // Setup output file
            if (writeOutputFile() == true) {
                m_outputFile = std::make_unique<io::LogFile<TaskSpaceSample>>(name(), input().m_outputFolder, name(), ".log", false, true, false, 1);

                // Setup header and footer sample (to get the correct column count for dynamic vectors/matrices)
                sampleEvaluator.evaluate(0, Eigen::VectorXd::Zero(chainLength), *m_currentSample);
                m_outputFile->setHeaderData(*m_currentSample);
                m_outputFile->setFooterData(*m_currentSample);

                // Open file and write header
                if (m_outputFile->openLogFile() == false || m_outputFile->writeLogHeader(core::Time::currentTime()) == false) {
                    emergencyExit("could not open log file");
                    return;
                }
            }

            // Pre-processing
            // --------------
            if (preProcess() == false) {
                emergencyExit("pre-processing failed");
                return;
            }

            // Abort, if stop has been triggered from "outside"
            if (stopTriggered() == true) {
                emergencyExit("aborted");
                return;
            }

            // Processing
            // ----------
            // Iterate over sample batch
            uint64_t currentEvaluatedSamples = 0;
            setEvaluatedSamples(currentEvaluatedSamples);
            Eigen::VectorXd configuration = Eigen::VectorXd::Zero(chainLength); // Currently evaluated configuration
            for (uint64_t sampleIndex = firstSampleIndex(); sampleIndex <= lastSampleIndex(); sampleIndex++) {
                // Abort, if stop has been triggered from "outside"
                if (stopTriggered() == true) {
                    emergencyExit("aborted");
                    return;
                }

                // Compute local sample indices
                uint64_t remainder = sampleIndex;
                for (size_t i = 0; i < chainLength; i++) {
                    localSampleIndex[i] = remainder / subSampleCount[i];
                    remainder -= localSampleIndex[i] * subSampleCount[i];
                }

                // Compute configuration from local sample indices
                for (size_t i = 0; i < chainLength; i++)
                    configuration(i) = input().m_kinematicChain.segment(i).minimumPosition() + localSampleIndex[i] * stepSize[i];

                // Evaluate sample
                sampleEvaluator.evaluate(sampleIndex, configuration, *m_currentSample);

                // Update minimum/maximum position
                const Eigen::Vector3d& currentPosition = m_currentSample->position();
                if (currentEvaluatedSamples == 0) {
                    // This is the first evaluated sample -> initialize minimum and maximum
                    m_minimumPosition = currentPosition;
                    m_maximumPosition = m_minimumPosition;
                } else {
                    // This is NOT the first evaluated sample -> update minimum/maximum
                    for (int i = 0; i < 3; i++) {
                        if (m_minimumPosition(i) > currentPosition(i))
                            m_minimumPosition(i) = currentPosition(i);
                        if (m_maximumPosition(i) < currentPosition(i))
                            m_maximumPosition(i) = currentPosition(i);
                    }
                }

                // Add sample to output file
                if (writeOutputFile() == true) {
                    m_outputFile->addNewDataToBuffer(*m_currentSample);
                    if (m_outputFile->processLogBuffer() == false) {
                        emergencyExit("could not process log buffer");
                        return;
                    }
                }

                // Trigger processing
                if (process(*m_currentSample) == false) {
                    emergencyExit("processing failed");
                    return;
                }

                // Update count of processed samples
                currentEvaluatedSamples++;
                setEvaluatedSamples(currentEvaluatedSamples);
            }

            // Abort, if stop has been triggered from "outside"
            if (stopTriggered() == true) {
                emergencyExit("aborted");
                return;
            }

            // Post-processing
            // ---------------
            if (postProcess() == false) {
                emergencyExit("post-processing failed");
                return;
            }

            // Close output file
            if (writeOutputFile() == true) {
                if (m_outputFile->writeLogFooter(core::Time::currentTime()) == false || m_outputFile->closeLogFile() == false) {
                    emergencyExit("could not close output file");
                    return;
                }
            }

            // Finish execution
            setSuccess(true);
            setErrorMessage("");
            setFinished(true);
            stop();
        }

        //! Pre-processing task
        /*! \return `true` on success, `false` otherwise */
        virtual bool preProcess()
        {
            // Do nothing per default...
            return true;
        }

        //! Processing task
        /*!
         * \param [in] sample Evaluated sample
         * \return `true` on success, `false` otherwise
         */
        virtual bool process(const TaskSpaceSample& sample)
        {
            // Do nothing per default...
            (void)sample;
            return true;
        }

        //! Post-processing task
        /*! \return `true` on success, `false` otherwise */
        virtual bool postProcess()
        {
            // Do nothing per default...
            return true;
        }

        // Setters (thread-safe)
        // ---------------------
        // Dynamic members
    protected:
        //! **Thread-safe setter for:** \copybrief evaluatedSamples()
        inline void setEvaluatedSamples(const uint64_t& newValue) { setProtectedData(m_evaluatedSamples, newValue); }
        //! **Thread-safe setter for:** \copybrief success()
        inline void setSuccess(const bool& newValue) { setProtectedData(m_success, newValue); }
        //! **Thread-safe setter for:** \copybrief errorMessage()
        inline void setErrorMessage(const std::string& newValue) { setProtectedData(m_errorMessage, newValue); }
        //! **Thread-safe setter for:** \copybrief finished()
        inline void setFinished(const bool& newValue) { setProtectedData(m_finished, newValue); }

        // Getters (thread-safe)
        // ---------------------
    public:
        // Constants -> no protection necessary
        //! Input data container
        const TaskSpaceEvaluatorInput<L>& input() const { return m_input; }
        //! Index of first sample to evaluate
        const uint64_t& firstSampleIndex() const { return m_firstSampleIndex; }
        //! Index of last sample to evaluate
        const uint64_t& lastSampleIndex() const { return m_lastSampleIndex; }
        //! If `true`, the output file is written, otherwise not
        const bool& writeOutputFile() const { return m_writeOutputFile; }

        // Dynamic members
        //! Total count of evaluated samples so far \details **Thread-safe getter**
        inline uint64_t evaluatedSamples() const { return getProtectedData(m_evaluatedSamples); }
        //! Flag indicating, if the sampler was successful \details **Thread-safe getter**
        inline bool success() const { return getProtectedData(m_success); }
        //! Error message in case of an error \details **Thread-safe getter**
        inline std::string errorMessage() const { return getProtectedData(m_errorMessage); }
        //! Flag indicating, if the sampler finished his work \details **Thread-safe getter**
        inline bool finished() const { return getProtectedData(m_finished); }

        // Helpers
        // -------
    protected:
        //! Performs an emergency exit for the running background thread (**thread-safe**)
        /*! \param [in] reason The reason for the exit */
        inline void emergencyExit(const std::string& reason = "")
        {
            // Append reason to error message
            std::string currentErrorMessage = errorMessage();
            if (currentErrorMessage != "")
                currentErrorMessage += "\n";
            currentErrorMessage += reason;

            // Append signals of logfile
            if (m_outputFile) {
                io::LogFileSignal signal;
                while (m_outputFile->m_signalBuffer.pop(signal) > 0) {
                    if (signal.m_type == io::LogFileSignal::Type::ERROR) {
                        if (currentErrorMessage != "")
                            currentErrorMessage += "\n";
                        currentErrorMessage += "Error of output file '" + m_outputFile->fileName() + m_outputFile->fileExtension() + "': " + signal.messageString() + "(" + signal.m_text + ")";
                    }
                }
            }

            // Update error message
            setErrorMessage(currentErrorMessage);

            // Quit
            setSuccess(false);
            setFinished(true);
            stop();
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
