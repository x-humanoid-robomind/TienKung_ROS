/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "TaskSpace.hpp"
#include "TaskSpaceBoundingBox.hpp"

namespace broccoli {
namespace analysis {
    //! Container for all output-data of the task space evaluator
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L>
    class TaskSpaceEvaluatorOutput : public io::serialization::SerializableData {
    protected:
        // Members
        // -------
        TaskSpaceBoundingBox m_boundingBoxPreSampling; //!< \copybrief boundingBoxPreSampling()
        TaskSpaceBoundingBox m_boundingBoxMainSampling; //!< \copybrief boundingBoxMainSampling()
        TaskSpace<L> m_taskspace; //!< Discretized task space representation (thread-safe on its own)

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const TaskSpaceEvaluatorOutput& reference) const
        {
            // Compare members
            if (m_boundingBoxPreSampling != reference.m_boundingBoxPreSampling || //
                m_boundingBoxMainSampling != reference.m_boundingBoxMainSampling || //
                m_taskspace != reference.m_taskspace)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const TaskSpaceEvaluatorOutput& reference) const { return !(*this == reference); }

    public:
        // Setters and Getters
        // -------------------
        //! Bounding box computed during pre-sampling (**thread-safe** on its own)
        const TaskSpaceBoundingBox& boundingBoxPreSampling() const { return m_boundingBoxPreSampling; }
        //! \copydoc boundingBoxPreSampling() const
        TaskSpaceBoundingBox& boundingBoxPreSampling() { return m_boundingBoxPreSampling; }

        //! Bounding box computed during main-sampling (**thread-safe** on its own)
        const TaskSpaceBoundingBox& boundingBoxMainSampling() const { return m_boundingBoxMainSampling; }
        //! \copydoc boundingBoxMainSampling() const
        TaskSpaceBoundingBox& boundingBoxMainSampling() { return m_boundingBoxMainSampling; }

        //! Discretized task space representation (**thread-safe** on its own)
        const TaskSpace<L>& taskspace() const { return m_taskspace; }
        //! \copydoc taskspace() const
        TaskSpace<L>& taskspace() { return m_taskspace; }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of members
            totalSize += m_boundingBoxPreSampling.computeBinaryStreamSize(); // Contribution of m_boundingBoxPreSampling
            totalSize += m_boundingBoxMainSampling.computeBinaryStreamSize(); // Contribution of m_boundingBoxMainSampling
            totalSize += m_taskspace.computeBinaryStreamSize(); // Contribution of m_taskspace

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            return io::serialization::serialize(stream, endianness, //
                m_boundingBoxPreSampling, m_boundingBoxMainSampling, //
                m_taskspace);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            return io::serialization::deSerialize(stream, index, endianness, //
                m_boundingBoxPreSampling, m_boundingBoxMainSampling, //
                m_taskspace);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
