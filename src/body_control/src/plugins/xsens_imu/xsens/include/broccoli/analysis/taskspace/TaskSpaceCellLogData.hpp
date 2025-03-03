/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/logging/ColumnBasedLogFileData.hpp"
#include "TaskSpaceCell.hpp"
#include <Eigen/Dense>
#include <array>
#include <stdint.h>

namespace broccoli {
namespace analysis {
    //! Container for logged data for each cell in the taskspace discretization
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceCellLogData : public io::ColumnBasedLogFileData {
    public:
        //! Default constructor
        TaskSpaceCellLogData()
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] level Initializes \ref m_level - \copybrief m_level
         * \param [in] indices Initializes \ref m_indices - \copybrief m_indices
         * \param [in] position Initializes \ref m_position - \copybrief m_position
         * \param [in] data Initializes \ref m_data - \copybrief m_data
         */
        TaskSpaceCellLogData(const int64_t& level, const std::array<size_t, 3>& indices, const Eigen::Vector3d& position, const TaskSpaceCell& data)
            : m_level(level)
            , m_indices(indices)
            , m_position(position)
            , m_data(data)
        {
        }

        // Members
        // -------
        int64_t m_level = 0; //!< Level of the octree the cell belongs to (use < 0 for meta-cell)
        std::array<size_t, 3> m_indices{}; //!< Grid indices (x, y, z) in the corresponding level of the octree
        Eigen::Vector3d m_position = Eigen::Vector3d::Zero(); //!< Absolute position (x, y, z) of the **center** of the grid cell [m]
        TaskSpaceCell m_data; //!< Grid cell data

        // Logging
        // -------
    public:
        // Encodes the column-based log data to the log stream (see base class for details)
        void encodeColumnDataToLogStream(io::ColumnBasedLogStream& stream) const
        {
            stream.addData(m_level, "Level");
            stream.addData(m_indices, "Indices");
            stream.addData(m_position, "Position");
            stream.addData(m_data.m_totalSamples, "TotalSamples");
            stream.addData(m_data.m_conditionIndexMinimum, "ConditionIndexMinimum");
            stream.addData(m_data.m_conditionIndexMean, "ConditionIndexMean");
            stream.addData(m_data.m_conditionIndexMaximum, "ConditionIndexMaximum");
            stream.addData(m_data.m_manipulabilityMeasureMinimum, "ManipulabilityMeasureMinimum");
            stream.addData(m_data.m_manipulabilityMeasureMean, "ManipulabilityMeasureMean");
            stream.addData(m_data.m_manipulabilityMeasureMaximum, "ManipulabilityMeasureMaximum");
            stream.addData(m_data.m_jointRangeAvailabilityMinimum, "JointRangeAvailabilityMinimum");
            stream.addData(m_data.m_jointRangeAvailabilityMean, "JointRangeAvailabilityMean");
            stream.addData(m_data.m_jointRangeAvailabilityMaximum, "JointRangeAvailabilityMaximum");
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
