/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/encoding.hpp"
#include "../../io/serialization/serialization.hpp"
#include "TaskSpaceEvaluatorResult.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace analysis {
    //! Represents a single segment in a kinematic chain
    /*!
     * \ingroup broccoli_analysis_taskspace
     * A segment in a kinematic chain which may represent a fixed linkage or an actuated joint (rotational **or** translational DoF).
     * Note that for n-DoF joints one may just "serialize" the DoFs (n-segments linked with each other). However, parallel structures
     * are not supported.
     */
    class KinematicChainSegment : public io::serialization::SerializableData {
    public:
        //! Specification of DoF types
        enum class DOFType : uint8_t {
            FIXED = 0, //!< Fixed (no motion)
            ROTATION, //!< Rotation around positive local z-axis (own body frame)
            TRANSLATION, //!< Translation along positive local z-axis (own body frame)
            DOFTYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given type
        static inline std::string toString(const DOFType& type)
        {
            // Check type
            switch (type) {
            case DOFType::FIXED:
                return "FIXED";
            case DOFType::ROTATION:
                return "ROTATION";
            case DOFType::TRANSLATION:
                return "TRANSLATION";
            default:
                break;
            }

            // Unknown selection
            assert(false);
            return "UNKNOWN";
        }

        //! Default constructor
        KinematicChainSegment()
            : m_name("")
            , m_parent("")
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] name Initializes \ref m_name - \copybrief m_name
         * \param [in] parent Initializes \ref m_parent - \copybrief m_parent
         */
        KinematicChainSegment(const std::string& name, const std::string& parent = "")
            : m_name(name)
            , m_parent(parent)
        {
        }

        // Members
        // -------
    protected:
        std::string m_name; //!< \copybrief name()
        std::string m_parent; //!< \copybrief parent()
        DOFType m_dofType = DOFType::FIXED; //!< \copybrief dofType()
        Eigen::Isometry3d m_initialTransform = Eigen::Isometry3d::Identity(); //!< \copybrief initialTransform()
        double m_minimumPosition = 0.0; //!< \copybrief minimumPosition()
        double m_maximumPosition = 0.0; //!< \copybrief maximumPosition()
        uint64_t m_samples = 1; //!< \copybrief samples()

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const KinematicChainSegment& reference) const
        {
            // Compare members
            if (m_name != reference.m_name || //
                m_parent != reference.m_parent || //
                m_dofType != reference.m_dofType || //
                m_initialTransform.matrix().isApprox(reference.m_initialTransform.matrix()) == false || //
                m_minimumPosition != reference.m_minimumPosition || //
                m_maximumPosition != reference.m_maximumPosition || //
                m_samples != reference.m_samples)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const KinematicChainSegment& reference) const { return !(*this == reference); }

        // General
        // -------
    public:
        //! Checks, if the segment is properly defined
        /*!
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true`, if the segment is valid, `false` otherwise.
         */
        bool isValid(TaskSpaceEvaluatorResult::Type* const result = nullptr) const
        {
            // Check, if names are set properly
            if (m_name.size() == 0) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_NAME;
                return false;
            }
            if (m_parent == m_name) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_PARENT;
                return false;
            }

            // Differentiate between DOF types
            if (m_dofType == DOFType::FIXED) {
                if (m_minimumPosition != m_maximumPosition) {
                    if (result != nullptr)
                        *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_LIMITS;
                    return false;
                }
                if (m_samples != 1) {
                    if (result != nullptr)
                        *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_SAMPLES;
                    return false;
                }
            } else if (m_dofType == DOFType::ROTATION || m_dofType == DOFType::TRANSLATION) {
                // Check count of samples
                if (m_samples == 0) {
                    if (result != nullptr)
                        *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_SAMPLES;
                    return false;
                } else if (m_samples == 1) {
                    // Check, minimum and maximum DoF position
                    if (m_minimumPosition != m_maximumPosition) {
                        if (result != nullptr)
                            *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_LIMITS;
                        return false;
                    }
                } else {
                    // Check, minimum and maximum DoF position
                    if (m_minimumPosition >= m_maximumPosition) {
                        if (result != nullptr)
                            *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_LIMITS;
                        return false;
                    }
                }
            } else {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_SEGMENT_INVALID_DOFTYPE;
                return false;
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
            encoding::encode(stream, "Name: " + m_name);
            encoding::encode(stream, "\nParent: ");
            if (m_parent == "")
                encoding::encode(stream, "None");
            else
                encoding::encode(stream, m_parent);
            encoding::encode(stream, "\nDOFType: " + toString(m_dofType));
            encoding::encode(stream, "\nInitial-Transform: Rotation=[");
            encoding::encode(stream, m_initialTransform.linear(), "%g");
            encoding::encode(stream, "], Translation=[");
            encoding::encode(stream, m_initialTransform.translation(), "%g");
            encoding::encode(stream, "]");
            if (m_dofType != DOFType::FIXED) {
                encoding::encode(stream, "\nMinimumPosition: ");
                encoding::encode(stream, m_minimumPosition, "%g");
                if (m_dofType == DOFType::ROTATION)
                    encoding::encode(stream, "rad");
                if (m_dofType == DOFType::TRANSLATION)
                    encoding::encode(stream, 'm');
                encoding::encode(stream, "\nMaximumPosition: ");
                encoding::encode(stream, m_maximumPosition, "%g");
                if (m_dofType == DOFType::ROTATION)
                    encoding::encode(stream, "rad");
                if (m_dofType == DOFType::TRANSLATION)
                    encoding::encode(stream, 'm');
                encoding::encode(stream, "\nSamples: ");
                encoding::encode(stream, m_samples);
                encoding::encode(stream, "\nStepsize: ");
                encoding::encode(stream, stepSize(), "%g");
                if (m_dofType == DOFType::ROTATION)
                    encoding::encode(stream, "rad");
                if (m_dofType == DOFType::TRANSLATION)
                    encoding::encode(stream, 'm');
            }
            encoding::encode(stream, "\nMaximumLeverArmLength: ");
            encoding::encode(stream, maximumLeverArmLength(), "%gm");
            encoding::encode(stream, '\n');
            return std::string(stream.begin(), stream.end());
        }

        // Setters and Getters
        // -------------------
    public:
        //! Identifying name of this segment (must **not** be empty)
        const std::string& name() const { return m_name; }

        //! Name of the linked parent segment (keep empty if there is no parent)
        const std::string& parent() const { return m_parent; }

        //! Removes link to the parent
        /*! \warning Can not be undone! */
        void unlinkParent() { m_parent = ""; }

        //! Actuation type of this segment
        const DOFType& dofType() const { return m_dofType; }

        //! \copydoc dofType() const
        DOFType& dofType() { return m_dofType; }

        //! Initial transform from the own body frame to the parent frame (**without** actuated DoF)
        const Eigen::Isometry3d& initialTransform() const { return m_initialTransform; }

        //! \copydoc initialTransform() const
        Eigen::Isometry3d& initialTransform() { return m_initialTransform; }

        //! Minimum position (translation: [m], rotation: [rad]) of the DoF
        const double& minimumPosition() const { return m_minimumPosition; }

        //! \copydoc minimumPosition() const
        double& minimumPosition() { return m_minimumPosition; }

        //! Maximum position (translation: [m], rotation: [rad]) of the DoF
        const double& maximumPosition() const { return m_maximumPosition; }

        //! \copydoc maximumPosition() const
        double& maximumPosition() { return m_maximumPosition; }

        //! Count of position samples to evaluate (between minimum and maximum position)
        const uint64_t& samples() const { return m_samples; }

        //! \copydoc samples() const
        uint64_t& samples() { return m_samples; }

        //! Stepsize used for sampling (computed from limits and sample count)
        double stepSize() const
        {
            // No step size for fixed segment or only one sample
            if (m_dofType == DOFType::FIXED || m_samples == 1)
                return 0;

            // Compute from limits and sample count
            return (m_maximumPosition - m_minimumPosition) / ((double)m_samples - 1.0);
        }

        //! The maximum lever arm length [m] of this segment (between parent-frame and DOF-frame) (over-estimated)
        double maximumLeverArmLength() const
        {
            // Add contribution of parent-frame to body-frame
            double leverArm = m_initialTransform.translation().norm();

            // Add contribution of body-frame to DOF-frame
            if (m_dofType == DOFType::TRANSLATION)
                leverArm += std::max(std::fabs(m_minimumPosition), std::fabs(m_maximumPosition));

            // Pass back lever arm length
            return leverArm;
        }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of m_name
            totalSize += sizeof(io::serialization::BinaryStreamSize); // Header
            totalSize += m_name.size();

            // Contribution of m_parent
            totalSize += sizeof(io::serialization::BinaryStreamSize); // Header
            totalSize += m_parent.size();

            // Contribution of m_dofType
            totalSize += sizeof(m_dofType);

            // Contribution of m_initialTransform
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize); // Header
            totalSize += 4 * 4 * sizeof(double);

            // Contribution of m_minimumPosition
            totalSize += sizeof(m_minimumPosition);

            // Contribution of m_maximumPosition
            totalSize += sizeof(m_maximumPosition);

            // Contribution of m_samples
            totalSize += sizeof(m_samples);

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            const uint8_t dofTypeInt = static_cast<uint8_t>(m_dofType);
            return io::serialization::serialize(stream, endianness, //
                m_name, m_parent, dofTypeInt, m_initialTransform.matrix(), m_minimumPosition, m_maximumPosition, m_samples);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            uint8_t dofTypeInt = 0;
            const auto returnValue = io::serialization::deSerialize(stream, index, endianness, //
                m_name, m_parent, dofTypeInt, m_initialTransform.matrix(), m_minimumPosition, m_maximumPosition, m_samples);
            m_dofType = static_cast<DOFType>(dofTypeInt);
            return returnValue;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
