/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/serialization/serialization.hpp"
#include "../../memory/MultiLevelGrid.hpp"
#include "TaskSpaceCell.hpp"

namespace broccoli {
namespace analysis {
    //! Dense octree representing the discretized task space
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the octree
     */
    template <unsigned int L>
    class TaskSpaceOctree : public memory::DenseOctree<L, TaskSpaceCell>, public io::serialization::SerializableData {
    public:
        using memory::DenseOctree<L, TaskSpaceCell>::cellCount;
        using memory::DenseOctree<L, TaskSpaceCell>::totalCellCount;
        using memory::DenseOctree<L, TaskSpaceCell>::cell;

        // Members
        // -------
        Eigen::Vector3d m_center = Eigen::Vector3d::Zero(); //!< Position (x, y, z) of the center of the octree [m]
        Eigen::Vector3d m_dimensions = Eigen::Vector3d::Zero(); //!< Dimensions (x, y, z) of the octree [m]
        TaskSpaceCell m_metaCell; //!< "Meta"-cell representing the complete octree

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const TaskSpaceOctree& reference) const
        {
            // Compare base class and members
            if (memory::DenseOctree<L, TaskSpaceCell>::operator==(reference) == false || //
                m_center != reference.m_center || //
                m_dimensions != reference.m_dimensions || //
                m_metaCell != reference.m_metaCell)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const TaskSpaceOctree& reference) const { return !(*this == reference); }

        // Generic
        // -------
    public:
        //! \copydoc broccoli::memory::MultiLevelGrid::reset()
        /*! \note Additionally resets the meta-cell */
        virtual void reset()
        {
            memory::DenseOctree<L, TaskSpaceCell>::reset();
            m_metaCell.reset();
        }

        //! Updates \ref m_metaCell from the first level octree cells
        void updateMetaCell()
        {
            std::vector<TaskSpaceCell const*> firstLevelCells;
            firstLevelCells.reserve(cellCount(0));
            for (size_t i = 0; i < cellCount(0); i++)
                firstLevelCells.push_back(&cell(0, i));
            cellPropagateUp(m_metaCell, firstLevelCells, 0, std::array<size_t, 3>{});
        }

        // Cell operations
        // ---------------
    protected:
        // Resets the given cell to its initial state (see base class for details)
        virtual void cellReset(TaskSpaceCell& cell, const size_t&) { cell.reset(); }

        // Propagates the data of the lower level cells to the data of the linked upper level cell (see base class for details)
        virtual void cellPropagateUp(TaskSpaceCell& upperLevelCell, const std::vector<TaskSpaceCell const*>& lowerLevelCells, const size_t&, const std::array<size_t, 3>&)
        {
            // Reset upper level cell to reset all values
            upperLevelCell.reset();

            // Process total samples
            upperLevelCell.m_totalSamples = 0;
            for (size_t i = 0; i < lowerLevelCells.size(); i++)
                upperLevelCell.m_totalSamples += lowerLevelCells[i]->m_totalSamples;

            // Process minimum/maximum values
            for (size_t i = 0; i < lowerLevelCells.size(); i++) {
                if (i == 0) {
                    // ...first lower level cell -> initialize minimum/maximum
                    upperLevelCell.m_conditionIndexMinimum = lowerLevelCells[i]->m_conditionIndexMinimum;
                    upperLevelCell.m_conditionIndexMaximum = lowerLevelCells[i]->m_conditionIndexMaximum;
                    upperLevelCell.m_manipulabilityMeasureMinimum = lowerLevelCells[i]->m_manipulabilityMeasureMinimum;
                    upperLevelCell.m_manipulabilityMeasureMaximum = lowerLevelCells[i]->m_manipulabilityMeasureMaximum;
                    upperLevelCell.m_jointRangeAvailabilityMinimum = lowerLevelCells[i]->m_jointRangeAvailabilityMinimum;
                    upperLevelCell.m_jointRangeAvailabilityMaximum = lowerLevelCells[i]->m_jointRangeAvailabilityMaximum;
                } else {
                    // ...no-first lower level cell -> update minimum/maximum
                    upperLevelCell.m_conditionIndexMinimum = std::min(upperLevelCell.m_conditionIndexMinimum, lowerLevelCells[i]->m_conditionIndexMinimum);
                    upperLevelCell.m_conditionIndexMaximum = std::max(upperLevelCell.m_conditionIndexMaximum, lowerLevelCells[i]->m_conditionIndexMaximum);
                    upperLevelCell.m_manipulabilityMeasureMinimum = std::min(upperLevelCell.m_manipulabilityMeasureMinimum, lowerLevelCells[i]->m_manipulabilityMeasureMinimum);
                    upperLevelCell.m_manipulabilityMeasureMaximum = std::max(upperLevelCell.m_manipulabilityMeasureMaximum, lowerLevelCells[i]->m_manipulabilityMeasureMaximum);
                    upperLevelCell.m_jointRangeAvailabilityMinimum = std::min(upperLevelCell.m_jointRangeAvailabilityMinimum, lowerLevelCells[i]->m_jointRangeAvailabilityMinimum);
                    upperLevelCell.m_jointRangeAvailabilityMaximum = std::max(upperLevelCell.m_jointRangeAvailabilityMaximum, lowerLevelCells[i]->m_jointRangeAvailabilityMaximum);
                }
            }

            // Process mean values
            upperLevelCell.m_conditionIndexMean = 0;
            upperLevelCell.m_manipulabilityMeasureMean = 0;
            upperLevelCell.m_jointRangeAvailabilityMean = 0;
            if (upperLevelCell.m_totalSamples > 0) {
                for (size_t i = 0; i < lowerLevelCells.size(); i++) {
                    upperLevelCell.m_conditionIndexMean += lowerLevelCells[i]->m_conditionIndexMean * lowerLevelCells[i]->m_totalSamples;
                    upperLevelCell.m_manipulabilityMeasureMean += lowerLevelCells[i]->m_manipulabilityMeasureMean * lowerLevelCells[i]->m_totalSamples;
                    upperLevelCell.m_jointRangeAvailabilityMean += lowerLevelCells[i]->m_jointRangeAvailabilityMean * lowerLevelCells[i]->m_totalSamples;
                }
                upperLevelCell.m_conditionIndexMean /= ((double)upperLevelCell.m_totalSamples);
                upperLevelCell.m_manipulabilityMeasureMean /= ((double)upperLevelCell.m_totalSamples);
                upperLevelCell.m_jointRangeAvailabilityMean /= ((double)upperLevelCell.m_totalSamples);
            }
        }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of base class
            totalSize += 3 * sizeof(size_t); // Header
            totalSize += totalCellCount() * TaskSpaceCell::computeBinaryStreamSize(); // Data

            // Contribution of members
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double); // Contribution of m_center
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double); // Contribution of m_dimensions
            totalSize += m_metaCell.computeBinaryStreamSize(); // Contribution of m_metaCell

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            const memory::DenseOctree<L, TaskSpaceCell>& baseClass = *this;
            return io::serialization::serialize(stream, endianness, //
                baseClass, m_center, m_dimensions, m_metaCell);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            memory::DenseOctree<L, TaskSpaceCell>& baseClass = *this;
            return io::serialization::deSerialize(stream, index, endianness, //
                baseClass, m_center, m_dimensions, m_metaCell);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
