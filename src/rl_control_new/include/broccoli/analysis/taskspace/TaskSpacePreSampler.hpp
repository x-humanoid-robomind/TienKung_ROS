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
    //! Pre-processing taskspace sampler used for evaluation
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L>
    class TaskSpacePreSampler : public TaskSpaceSamplerBase<L> {
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
        TaskSpacePreSampler(const std::string& name, const TaskSpaceEvaluatorInput<L>& input, const uint64_t& firstSampleIndex, const uint64_t& lastSampleIndex, TaskSpaceEvaluatorOutput<L>& output)
            : TaskSpaceSamplerBase<L>(name, input, firstSampleIndex, lastSampleIndex, (input.m_writeOutputFiles && input.m_writeOutputFilesPreSampler), output)
        {
        }

        // Members
        // -------
    protected:
        // Post-processing task (see base class for details)
        bool postProcess()
        {
            // Write "local" bounding box to solution
            // (handle minimum and maximum position as separate "samples")
            m_output.boundingBoxPreSampling().addSample(m_minimumPosition);
            m_output.boundingBoxPreSampling().addSample(m_maximumPosition);

            // Success
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
