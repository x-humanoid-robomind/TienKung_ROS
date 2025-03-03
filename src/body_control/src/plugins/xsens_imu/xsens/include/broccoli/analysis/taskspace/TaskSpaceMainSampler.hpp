/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "TaskSpaceSamplerBase.hpp"

namespace broccoli {
namespace analysis {
    //! Main taskspace sampler used for evaluation
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L>
    class TaskSpaceMainSampler : public TaskSpaceSamplerBase<L> {
    public:
        using TaskSpaceSamplerBase<L>::m_minimumPosition;
        using TaskSpaceSamplerBase<L>::m_maximumPosition;
        using TaskSpaceSamplerBase<L>::m_output;

        //! Specialized constructor
        /*!
         * \param [in] name Initializes \ref m_name - \copybrief m_name
         * \param [in] input Initializes \ref m_input - \copybrief m_input
         * \param [in] firstSampleIndex Initializes \ref m_firstSampleIndex - \copybrief m_firstSampleIndex
         * \param [in] lastSampleIndex Initializes \ref m_lastSampleIndex - \copybrief m_lastSampleIndex
         * \param [in] output Initializes \ref m_output - \copybrief m_output
         */
        TaskSpaceMainSampler(const std::string& name, const TaskSpaceEvaluatorInput<L>& input, const uint64_t& firstSampleIndex, const uint64_t& lastSampleIndex, TaskSpaceEvaluatorOutput<L>& output)
            : TaskSpaceSamplerBase<L>(name, input, firstSampleIndex, lastSampleIndex, (input.m_writeOutputFiles && input.m_writeOutputFilesMainSampler), output)
        {
        }

        // Members
        // -------
    protected:
        // Internal data (only accessed by background thread)
        typename TaskSpace<L>::SampleList m_samplesBuffer; //!< Internal buffer for samples (for batch processing)
        const size_t m_samplesBufferSize = 10000; //!< Target size of sample buffer (synchronization every 10ms if evaluating a single sample lasts 1us)

        // Pre-processing task (see base class for details)
        bool preProcess()
        {
            // Pre-allocate sample buffer
            m_samplesBuffer.reserve(m_samplesBufferSize);

            // Success
            return true;
        }

        // Processing task (see base class for details)
        bool process(const TaskSpaceSample& sample)
        {
            // Add sample to buffer
            m_samplesBuffer.push_back(sample);

            // Flush samples buffer
            if (m_samplesBuffer.size() >= m_samplesBufferSize)
                flushSamplesBuffer();

            // Success
            return true;
        }

        // Post-processing task (see base class for details)
        bool postProcess()
        {
            // Initialize helpers
            bool success = true;

            // Write "local" bounding box to solution
            // (handle minimum and maximum position as separate "samples")
            m_output.boundingBoxMainSampling().addSample(m_minimumPosition);
            m_output.boundingBoxMainSampling().addSample(m_maximumPosition);

            // Flush samples buffer
            if (m_samplesBuffer.size() > 0)
                flushSamplesBuffer();

            // Return result
            return success;
        }

        // Helpers
        // -------
    protected:
        //! Sends all buffered samples to the output data structure and clears the buffer
        void flushSamplesBuffer()
        {
            m_output.taskspace().addSamples(m_samplesBuffer);
            m_samplesBuffer.clear();
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
