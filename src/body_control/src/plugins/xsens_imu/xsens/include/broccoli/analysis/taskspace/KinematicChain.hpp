/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "KinematicChainSegment.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace analysis {
    //! Kinematic chain representing a robotic structure
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * This class contains a (sorted) list of linked chain segments, which describe the kinematic topology to evaluate.
     * The chain segments each have a name and parent to specify their connections. To setup the chain from a set of
     * segments, one can either call
     *
     *   * \ref set() if the order of segments is already known, or
     *   * \ref assemble() to automatically setup the sorted list of segments from their parent-child relations
     *
     * After this one may call \ref autoComputeSamples() to automatically compute the chain segment samples.
     */
    class KinematicChain : public io::serialization::SerializableData {
    public:
        // Type definitions
        using SegmentList = std::vector<KinematicChainSegment, Eigen::aligned_allocator<KinematicChainSegment>>; //!< Type of the (sorted) segment list

        // Members
        // -------
    protected:
        SegmentList m_segments; //!< \copybrief segments()

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const KinematicChain& reference) const
        {
            // Compare members
            if (m_segments != reference.m_segments)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const KinematicChain& reference) const { return !(*this == reference); }

        // General
        // -------
    public:
        //! Checks, if the kinematic chain is valid
        /*!
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true`, if the chain is valid, `false` otherwise.
         */
        bool isValid(TaskSpaceEvaluatorResult::Type* const result = nullptr) const
        {
            // Check, if there are at least two segments in the list
            if (m_segments.size() < 2) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_INVALID_SEGMENT_COUNT;
                return false;
            }

            // Check, if segments are valid
            for (size_t i = 0; i < m_segments.size(); i++)
                if (m_segments[i].isValid(result) == false)
                    return false;

            // Check, if there are segments with the same name
            for (size_t i = 0; i < m_segments.size(); i++) {
                for (size_t j = i + 1; j < m_segments.size(); j++) {
                    if (m_segments[i].name() == m_segments[j].name()) {
                        if (result != nullptr)
                            *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_DUPLICATE_SEGMENT_NAME;
                        return false;
                    }
                }
            }

            // Check, if segments are linked correctly
            for (size_t i = 0; i < m_segments.size(); i++) {
                if (i == 0) {
                    // Root segment -> there must not be any parent
                    if (m_segments[0].parent() != "") {
                        if (result != nullptr)
                            *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_ROOT_HAS_PARENT;
                        return false;
                    }
                } else {
                    if (m_segments[i].parent() != m_segments[i - 1].name()) {
                        if (result != nullptr)
                            *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_PARENT_CHILD_MISMATCH;
                        return false;
                    }
                }
            }

            // No error -> valid
            if (result != nullptr)
                *result = TaskSpaceEvaluatorResult::Type::SUCCESS;
            return true;
        }

        // Encoding
        // --------
    public:
        //! Encodes this object to a string (for logging/console output)
        std::string toString() const
        {
            using namespace broccoli::io;
            encoding::CharacterStream stream;
            encoding::encode(stream, " R (ROOT)\n |\n");
            for (size_t i = 0; i < m_segments.size(); i++) {
                encoding::encode(stream, "Segment: ");
                encoding::encode(stream, i);
                encoding::encode(stream, "\n");
                encoding::encode(stream, m_segments[i].toString());
                if (i < m_segments.size() - 1)
                    encoding::encode(stream, " |\n v\n");
            }
            encoding::encode(stream, " |\n T (TCP)\n");
            encoding::encode(stream, "DoF count: ");
            encoding::encode(stream, dofCount());
            encoding::encode(stream, "\nTotal samples: ");
            encoding::encode(stream, totalSamples());
            encoding::encode(stream, '\n');
            return std::string(stream.begin(), stream.end());
        }

        // Modifiers
        // ---------
    public:
        //! Clears the list of segments
        void clear() { m_segments.clear(); }

        //! Assigns the given **sorted** list of segments
        /*!
         * \param [in] sortedSegmentList Sorted list of segments (root segment is first element, TCP segment is last element)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool set(const SegmentList& sortedSegmentList, TaskSpaceEvaluatorResult::Type* const result = nullptr)
        {
            // Simple copy
            m_segments = sortedSegmentList;

            // Check validity
            if (isValid(result) == false) {
                assert(false);
                return false;
            }

            // Success
            if (result != nullptr)
                *result = TaskSpaceEvaluatorResult::Type::SUCCESS;
            return true;
        }

        //! Generates kinematic chain from a given unordered list of segments
        /*!
         * \param [in] unsortedSegmentList (Unsorted) list containing the segments (may contain more segments)
         * \param [in] rootSegmentName Name of the desired "root" segment (start of the chain) (use "" for auto-detect by empty parent tag)
         * \param [in] tcpSegmentName Name of the desired tool-center-point "TCP" segment (end of the chain)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool assemble(const SegmentList& unsortedSegmentList, const std::string& rootSegmentName, const std::string& tcpSegmentName, TaskSpaceEvaluatorResult::Type* const result = nullptr)
        {
            // Clear chain
            clear();

            // Check input
            if (unsortedSegmentList.size() < 2) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_INVALID_SEGMENT_COUNT;
                assert(false);
                return false;
            }
            if (tcpSegmentName.size() == 0) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_INVALID_TCP_NAME;
                assert(false);
                return false;
            }
            if (rootSegmentName == tcpSegmentName) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_INVALID_ROOT_NAME;
                assert(false);
                return false;
            }

            // Check, if segments are valid
            for (size_t i = 0; i < unsortedSegmentList.size(); i++) {
                if (unsortedSegmentList[i].isValid(result) == false) {
                    assert(false);
                    return false;
                }
            }

            // Check, if there are two segments in the list with the same name
            for (size_t i = 0; i < unsortedSegmentList.size(); i++) {
                for (size_t j = i + 1; j < unsortedSegmentList.size(); j++) {
                    if (unsortedSegmentList[i].name() == unsortedSegmentList[j].name()) {
                        if (result != nullptr)
                            *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_DUPLICATE_SEGMENT_NAME;
                        assert(false);
                        return false;
                    }
                }
            }

            // Try to find root and tcp segment
            bool rootSegmentFound = false;
            bool tcpSegmentFound = false;
            size_t idxRootSegment = 0;
            size_t idxTCPSegment = 0;
            // Search for root segment
            for (size_t i = 0; i < unsortedSegmentList.size(); i++) {
                if ((rootSegmentName.size() == 0 && unsortedSegmentList[i].parent().size() == 0) || (rootSegmentName.size() > 0 && unsortedSegmentList[i].name() == rootSegmentName)) {
                    idxRootSegment = i;
                    rootSegmentFound = true;
                    break;
                }
            }
            // Search for tcp segment
            for (size_t i = 0; i < unsortedSegmentList.size(); i++) {
                if (unsortedSegmentList[i].name() == tcpSegmentName) {
                    idxTCPSegment = i;
                    tcpSegmentFound = true;
                    break;
                }
            }
            if (rootSegmentFound == false) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_ROOT_NOT_FOUND;
                assert(false);
                return false;
            }
            if (tcpSegmentFound == false) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_TCP_NOT_FOUND;
                assert(false);
                return false;
            }

            // Assemble reversed chain
            std::vector<size_t> reverseChainIndices;
            reverseChainIndices.reserve(unsortedSegmentList.size());
            reverseChainIndices.push_back(idxTCPSegment);
            while (reverseChainIndices.back() != idxRootSegment) {
                // Search for parent segment
                const size_t& idxCurrentSegment = reverseChainIndices.back();
                bool parentFound = false;
                for (size_t i = 0; i < unsortedSegmentList.size(); i++) {
                    if (i != idxCurrentSegment && unsortedSegmentList[i].name() == unsortedSegmentList[idxCurrentSegment].parent()) {
                        reverseChainIndices.push_back(i);
                        parentFound = true;
                        break;
                    }
                }
                if (parentFound == false) {
                    if (result != nullptr)
                        *result = TaskSpaceEvaluatorResult::Type::ERROR_CHAIN_PARENT_NOT_FOUND;
                    assert(false);
                    return false;
                }
            }

            // Assemble final chain
            m_segments.reserve(reverseChainIndices.size());
            for (int64_t i = reverseChainIndices.size() - 1; i >= 0; i--)
                m_segments.push_back(unsortedSegmentList[reverseChainIndices[i]]);

            // Force unlinking root segment from its parent
            if (m_segments.size() > 0)
                m_segments.front().unlinkParent();

            // Check validity
            assert(isValid(result));

            // Success
            if (result != nullptr)
                *result = TaskSpaceEvaluatorResult::Type::SUCCESS;
            return true;
        }

        //! Automatically computes appropriate sample counts for the current chain segments
        /*!
         * \param [in] maximumTranslationStepSize Maximum allowed translational step size [m] of the TCP frame in the task space between two samples (should be smaller than the voxel size)
         * \return (Over-)estimated total lever arm length [m] which has been used during the estimation
         */
        double autoComputeSamples(const double& maximumTranslationStepSize)
        {
            // Iterate through chain backwards
            double totalLeverArm = 0.0; // The (over-estimated) total lever arm to the tcp frame
            for (int64_t i = m_segments.size() - 1; i >= 0; i--) {
                // Get reference to current sample
                auto& currentSegment = m_segments[i];

                // Compute samples
                if (currentSegment.dofType() == KinematicChainSegment::DOFType::ROTATION) {
                    // Range of possible rotation in [rad]
                    const double rotationRange = std::fabs(currentSegment.maximumPosition() - currentSegment.minimumPosition());

                    // Compute range of translation from circle chord
                    double translationRange = 2.0 * totalLeverArm;
                    if (rotationRange < M_PI)
                        translationRange *= std::sin(rotationRange / 2.0);

                    // Compute samples from maximum translation stepsize
                    currentSegment.samples() = std::max((uint64_t)std::ceil(translationRange / std::fabs(maximumTranslationStepSize)), (uint64_t)1);
                } else if (currentSegment.dofType() == KinematicChainSegment::DOFType::TRANSLATION) {
                    // Range of possible translation in [m]
                    const double translationRange = std::fabs(currentSegment.maximumPosition() - currentSegment.minimumPosition());

                    // Compute samples from maximum translation stepsize
                    // Note: independent of of the following chain segments (lever arm), a motion of a translational-DOF can only generate the same translation of the tcp frame (if all other DOFs are fixed)
                    currentSegment.samples() = std::max((uint64_t)std::ceil(translationRange / std::fabs(maximumTranslationStepSize)), (uint64_t)1);
                } else {
                    // Fixed DOF -> only one sample
                    currentSegment.samples() = 1;
                }

                // Update lever arm
                totalLeverArm += currentSegment.maximumLeverArmLength();
            }

            // Pass back estimated lever arm length
            return totalLeverArm;
        }

        // Setters and getters
        // -------------------
    public:
        //! Sorted list of segments in kinematic chain (first element is root segment, last element is tool-center-point ("TCP") segment)
        const SegmentList& segments() const { return m_segments; }

        //! Reference to a specific chain segment (selected by index)
        const KinematicChainSegment& segment(const size_t& index) const { return m_segments[index]; }

        //! \copydoc segment(const size_t&) const
        KinematicChainSegment& segment(const size_t& index) { return m_segments[index]; }

        //! Total count of segments with linked degree of freedom ("movable" segments)
        size_t dofCount() const
        {
            size_t dofs = 0;
            for (size_t i = 0; i < m_segments.size(); i++)
                if (m_segments[i].dofType() != KinematicChainSegment::DOFType::FIXED)
                    dofs++;
            return dofs;
        }

        //! Total count of samples to be evaluated for this chain
        /*! \warning This is **not** the sum of samples of all segments but instead the **product** of samples of all segments! */
        uint64_t totalSamples() const
        {
            // Check, if chain contains segments
            if (m_segments.size() == 0)
                return 0;

            // Accumulate all chain elements
            uint64_t samples = 1;
            for (size_t i = 0; i < m_segments.size(); i++)
                samples *= m_segments[i].samples();
            return samples;
        }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of m_segments
            totalSize += sizeof(io::serialization::BinaryStreamSize); // Header
            for (size_t i = 0; i < m_segments.size(); i++)
                totalSize += m_segments[i].computeBinaryStreamSize();

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            return io::serialization::serialize(stream, endianness, m_segments);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            return io::serialization::deSerialize(stream, index, endianness, m_segments);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
