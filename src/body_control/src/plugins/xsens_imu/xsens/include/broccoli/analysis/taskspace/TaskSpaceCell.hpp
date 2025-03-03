/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/serialization/serialization.hpp"
#include "TaskSpaceMetric.hpp"
#include "TaskSpaceSample.hpp"
#include <stdint.h>

namespace broccoli {
namespace analysis {
    //! Representation of a single task-space cell alias "voxel"
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * The taskspace cell collects metric data for a single voxel. This data contains
     *  - the count of samples collected in the cell (count of sampled TCP positions lying in this voxel)
     *  - condition index (from task-space jacobian)
     *  - manipulability measure (from task-space jacobian)
     *  - joint range availability (from joint angles and joint minimum/maximum angles related to the current robot pose)
     *
     * The cell stores minimum/maximum and mean-values for the metrics (except for reachability). These values are updated
     * incrementally for each new sample through \ref addSample().
     */
    class TaskSpaceCell : public io::serialization::SerializableData {
    public:
        // Members
        // -------
        // Reachability
        uint64_t m_totalSamples = 0; //!< Total count of samples collected in this cell (sum of all lower level cells)

        // Condition index
        double m_conditionIndexMinimum = 0.0; //!< Minimum value of condition index (\ref TaskSpaceSample::conditionIndex()) from all samples collected in this cell
        double m_conditionIndexMean = 0.0; //!< Mean value of condition index (\ref TaskSpaceSample::conditionIndex()) from all samples collected in this cell
        double m_conditionIndexMaximum = 0.0; //!< Maximum value of condition index (\ref TaskSpaceSample::conditionIndex()) from all samples collected in this cell

        // Manipulability measure
        double m_manipulabilityMeasureMinimum = 0.0; //!< Minimum value of manipulability measure (\ref TaskSpaceSample::manipulabilityMeasure()) from all samples collected in this cell
        double m_manipulabilityMeasureMean = 0.0; //!< Mean value of manipulability measure (\ref TaskSpaceSample::manipulabilityMeasure()) from all samples collected in this cell
        double m_manipulabilityMeasureMaximum = 0.0; //!< Maximum value of manipulability measure (\ref TaskSpaceSample::manipulabilityMeasure()) from all samples collected in this cell

        // Joint range availability
        double m_jointRangeAvailabilityMinimum = 0.0; //!< Minimum value of joint range availability (\ref TaskSpaceSample::jointRangeAvailability()) from all samples collected in this cell
        double m_jointRangeAvailabilityMean = 0.0; //!< Mean value of joint range availability (\ref TaskSpaceSample::jointRangeAvailability()) from all samples collected in this cell
        double m_jointRangeAvailabilityMaximum = 0.0; //!< Maximum value of joint range availability (\ref TaskSpaceSample::jointRangeAvailability()) from all samples collected in this cell

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const TaskSpaceCell& reference) const
        {
            // Compare members
            if (m_totalSamples != reference.m_totalSamples || //
                m_conditionIndexMinimum != reference.m_conditionIndexMinimum || //
                m_conditionIndexMean != reference.m_conditionIndexMean || //
                m_conditionIndexMaximum != reference.m_conditionIndexMaximum || //
                m_manipulabilityMeasureMinimum != reference.m_manipulabilityMeasureMinimum || //
                m_manipulabilityMeasureMean != reference.m_manipulabilityMeasureMean || //
                m_manipulabilityMeasureMaximum != reference.m_manipulabilityMeasureMaximum || //
                m_jointRangeAvailabilityMinimum != reference.m_jointRangeAvailabilityMinimum || //
                m_jointRangeAvailabilityMean != reference.m_jointRangeAvailabilityMean || //
                m_jointRangeAvailabilityMaximum != reference.m_jointRangeAvailabilityMaximum)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const TaskSpaceCell& reference) const { return !(*this == reference); }

        // Modifiers
        // ---------
    public:
        //! Resets the cell to its default values
        void reset()
        {
            // Reachability
            m_totalSamples = 0;

            // Condition index
            m_conditionIndexMinimum = 0.0;
            m_conditionIndexMean = 0.0;
            m_conditionIndexMaximum = 0.0;

            // Manipulability measure
            m_manipulabilityMeasureMinimum = 0.0;
            m_manipulabilityMeasureMean = 0.0;
            m_manipulabilityMeasureMaximum = 0.0;

            // Joint range availability
            m_jointRangeAvailabilityMinimum = 0.0;
            m_jointRangeAvailabilityMean = 0.0;
            m_jointRangeAvailabilityMaximum = 0.0;
        }

        //! Adds the given sample to this cell ("collect")
        void addSample(const TaskSpaceSample& sample)
        {
            m_totalSamples++;
            if (m_totalSamples == 1) {
                // First sample -> initialize...

                // Condition index
                m_conditionIndexMinimum = sample.conditionIndex();
                m_conditionIndexMean = sample.conditionIndex();
                m_conditionIndexMaximum = sample.conditionIndex();

                // Manipulability measure
                m_manipulabilityMeasureMinimum = sample.manipulabilityMeasure();
                m_manipulabilityMeasureMean = sample.manipulabilityMeasure();
                m_manipulabilityMeasureMaximum = sample.manipulabilityMeasure();

                // Joint range availability
                m_jointRangeAvailabilityMinimum = sample.jointRangeAvailability();
                m_jointRangeAvailabilityMean = sample.jointRangeAvailability();
                m_jointRangeAvailabilityMaximum = sample.jointRangeAvailability();
            } else {
                // Not first sample -> update values...

                // Condition index
                m_conditionIndexMinimum = std::min(sample.conditionIndex(), m_conditionIndexMinimum);
                m_conditionIndexMean = m_conditionIndexMean + (sample.conditionIndex() - m_conditionIndexMean) / ((double)m_totalSamples);
                m_conditionIndexMaximum = std::max(sample.conditionIndex(), m_conditionIndexMaximum);

                // Manipulability measure
                m_manipulabilityMeasureMinimum = std::min(sample.manipulabilityMeasure(), m_manipulabilityMeasureMinimum);
                m_manipulabilityMeasureMean = m_manipulabilityMeasureMean + (sample.manipulabilityMeasure() - m_manipulabilityMeasureMean) / ((double)m_totalSamples);
                m_manipulabilityMeasureMaximum = std::max(sample.manipulabilityMeasure(), m_manipulabilityMeasureMaximum);

                // Joint range availability
                m_jointRangeAvailabilityMinimum = std::min(sample.jointRangeAvailability(), m_jointRangeAvailabilityMinimum);
                m_jointRangeAvailabilityMean = m_jointRangeAvailabilityMean + (sample.jointRangeAvailability() - m_jointRangeAvailabilityMean) / ((double)m_totalSamples);
                m_jointRangeAvailabilityMaximum = std::max(sample.jointRangeAvailability(), m_jointRangeAvailabilityMaximum);
            }
        }

        // Helpers
        // -------
    public:
        //! Returns the desired metric of this cell
        double metric(const TaskSpaceMetric::Type& metricType) const
        {
            // Check metric type
            switch (metricType) {
            case TaskSpaceMetric::Type::REACHABILITY:
                return m_totalSamples;
            case TaskSpaceMetric::Type::CONDITION_INDEX_MINIMUM:
                return m_conditionIndexMinimum;
            case TaskSpaceMetric::Type::CONDITION_INDEX_MEAN:
                return m_conditionIndexMean;
            case TaskSpaceMetric::Type::CONDITION_INDEX_MAXIMUM:
                return m_conditionIndexMaximum;
            case TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MINIMUM:
                return m_manipulabilityMeasureMinimum;
            case TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MEAN:
                return m_manipulabilityMeasureMean;
            case TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MAXIMUM:
                return m_manipulabilityMeasureMaximum;
            case TaskSpaceMetric::Type::JOINT_RANGE_AVAILABILITY_MINIMUM:
                return m_jointRangeAvailabilityMinimum;
            case TaskSpaceMetric::Type::JOINT_RANGE_AVAILABILITY_MEAN:
                return m_jointRangeAvailabilityMean;
            case TaskSpaceMetric::Type::JOINT_RANGE_AVAILABILITY_MAXIMUM:
                return m_jointRangeAvailabilityMaximum;
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return 0;
        }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        static inline io::serialization::BinaryStreamSize computeBinaryStreamSize()
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of members
            totalSize += sizeof(m_totalSamples); // Contribution of m_totalSamples
            totalSize += sizeof(m_conditionIndexMinimum); // Contribution of m_conditionIndexMinimum
            totalSize += sizeof(m_conditionIndexMean); // Contribution of m_conditionIndexMean
            totalSize += sizeof(m_conditionIndexMaximum); // Contribution of m_conditionIndexMaximum
            totalSize += sizeof(m_manipulabilityMeasureMinimum); // Contribution of m_manipulabilityMeasureMinimum
            totalSize += sizeof(m_manipulabilityMeasureMean); // Contribution of m_manipulabilityMeasureMean
            totalSize += sizeof(m_manipulabilityMeasureMaximum); // Contribution of m_manipulabilityMeasureMaximum
            totalSize += sizeof(m_jointRangeAvailabilityMinimum); // Contribution of m_jointRangeAvailabilityMinimum
            totalSize += sizeof(m_jointRangeAvailabilityMean); // Contribution of m_jointRangeAvailabilityMean
            totalSize += sizeof(m_jointRangeAvailabilityMaximum); // Contribution of m_jointRangeAvailabilityMaximum

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            return io::serialization::serialize(stream, endianness, //
                m_totalSamples, //
                m_conditionIndexMinimum, m_conditionIndexMean, m_conditionIndexMaximum, //
                m_manipulabilityMeasureMinimum, m_manipulabilityMeasureMean, m_manipulabilityMeasureMaximum, //
                m_jointRangeAvailabilityMinimum, m_jointRangeAvailabilityMean, m_jointRangeAvailabilityMaximum);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            return io::serialization::deSerialize(stream, index, endianness, //
                m_totalSamples, //
                m_conditionIndexMinimum, m_conditionIndexMean, m_conditionIndexMaximum, //
                m_manipulabilityMeasureMinimum, m_manipulabilityMeasureMean, m_manipulabilityMeasureMaximum, //
                m_jointRangeAvailabilityMinimum, m_jointRangeAvailabilityMean, m_jointRangeAvailabilityMaximum);
        }
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
