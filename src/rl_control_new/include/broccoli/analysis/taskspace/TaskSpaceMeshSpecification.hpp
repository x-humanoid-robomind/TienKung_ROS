/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../geometry/CGMesh.hpp"
#include "../../io/serialization/serialization.hpp"
#include "TaskSpaceMesh.hpp"
#include "TaskSpaceMetric.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <array>
#include <limits>

namespace broccoli {
namespace analysis {
    //! Specification of output mesh format for representing the discretized task space
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceMeshSpecification : public io::serialization::SerializableData {
        // Type definitions
        using ColorGradient = std::vector<geometry::CGMesh::ColorType, Eigen::aligned_allocator<geometry::CGMesh::ColorType>>; //!< Type of color gradients

    public:
        //! Specialized constructor
        /*!
         * \param [in] level Initializes \ref m_level - \copybrief m_level
         * \param [in] metricType Initializes \ref m_metricType - \copybrief m_metricType
         * \param [in] meshType Initializes \ref m_meshType - \copybrief m_meshType
         * \param [in] name Initializes \ref m_name - \copybrief m_name (if not set, a default name will be computed)
         */
        TaskSpaceMeshSpecification(const size_t& level, const TaskSpaceMetric::Type& metricType, const TaskSpaceMesh::Type& meshType, const std::string& name = "")
            : m_name(name)
            , m_level(level)
            , m_metricType(metricType)
            , m_meshType(meshType)
        {
            // Compute default name
            if (m_name == "")
                m_name = "L" + std::to_string(level) + "_" + TaskSpaceMetric::toString(metricType) + "_" + TaskSpaceMesh::toString(meshType);

            // Setup default color gradient
            m_colorGradient.resize(3);
            m_colorGradient[0] << 255, 0, 0, 255; // Red
            m_colorGradient[1] << 0, 255, 0, 255; // Green
            m_colorGradient[2] << 0, 0, 255, 255; // Blue
        }

        //! Default constructor
        TaskSpaceMeshSpecification()
            : TaskSpaceMeshSpecification(0, TaskSpaceMetric::Type::REACHABILITY, TaskSpaceMesh::Type::VOXEL_BOX_FLAT)
        {
        }

        // Generic
        // -------
        //! Name of this mesh (may be used to define the file name)
        std::string m_name;

        //! Level of the octree used to obtain data from
        size_t m_level;

        //! Chosen metric for visualization
        TaskSpaceMetric::Type m_metricType;

        //! Type of the mesh (geometric representation)
        TaskSpaceMesh::Type m_meshType;

        //! Minimum cell position (center of voxel, component-wise)
        /*! Only cells with a position greater or equal (component-wise) to the minimum will be drawn */
        Eigen::Vector3d m_positionMinimum = Eigen::Vector3d(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());

        //! Maximum cell position (center of voxel, component-wise)
        /*! Only cells with a position lesser or equal (component-wise) to the maximum will be drawn */
        Eigen::Vector3d m_positionMaximum = Eigen::Vector3d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

        //! Minimum value for metric
        /*! Only cells with a metric greater or equal to the minimum will be drawn */
        double m_metricMinimum = std::numeric_limits<double>::lowest();

        //! Maximum value for metric
        /*! Only cells with a metric lesser or equal to the maximum will be drawn */
        double m_metricMaximum = std::numeric_limits<double>::max();

        // Geometry
        // --------
        // Voxel meshes
        //! Scaling factor for generating voxel meshes (only for corresponding \ref m_meshType)
        Eigen::Vector3d m_voxelMeshScaling = Eigen::Vector3d(1.0, 1.0, 1.0);

        //! Count of subdivisions for icosphere representation (only for corresponding \ref m_meshType)
        uint64_t m_icoSphereSubdivisions = 2;

        // Volume meshes
        //! Threshold of metric for volume creation (every cell with a metric value below this threshold is considered as "empty") (only for corresponding \ref m_meshType)
        double m_metricVolumeThreshold = 1e-6;

        // Coloring
        // --------
        //! Color gradient for generating the mesh colors
        /*!
         * Contains a list of colors. The local mesh color is computed by linear interpolation of the metric value between the specified
         * limits (see \ref m_metricFirstColor and \ref m_metricLastColor).
         *
         * The colors are specified by their channels (Red=0...255, Green, Blue, Alpha)
         *
         * If the color gradient contains no colors, the color buffer of the resulting mesh remains empty.
         */
        ColorGradient m_colorGradient;

        //! Value of metric linked to the first color in the color gradient (if <0, the global minimum of the metric is used)
        double m_metricFirstColor = -1;

        //! Value of metric linked to the last color in the color gradient (if <0, the global maximum of the metric is used)
        double m_metricLastColor = -1;

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const TaskSpaceMeshSpecification& reference) const
        {
            // Compare members
            if (m_name != reference.m_name || //
                m_level != reference.m_level || //
                m_metricType != reference.m_metricType || //
                m_meshType != reference.m_meshType || //
                m_positionMinimum != reference.m_positionMinimum || //
                m_positionMaximum != reference.m_positionMaximum || //
                m_metricMinimum != reference.m_metricMinimum || //
                m_metricMaximum != reference.m_metricMaximum || //
                m_voxelMeshScaling.isApprox(reference.m_voxelMeshScaling) == false || //
                m_icoSphereSubdivisions != reference.m_icoSphereSubdivisions || //
                m_metricVolumeThreshold != reference.m_metricVolumeThreshold || //
                m_colorGradient.size() != reference.m_colorGradient.size() || //
                m_metricFirstColor != reference.m_metricFirstColor || //
                m_metricLastColor != reference.m_metricLastColor)
                return false;
            for (size_t i = 0; i < m_colorGradient.size(); i++)
                if (m_colorGradient[i].isApprox(reference.m_colorGradient[i]) == false)
                    return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const TaskSpaceMeshSpecification& reference) const { return !(*this == reference); }

        // Helpers
        // -------
    public:
        //! Checks, if the given position is within the bounds (\ref m_positionMinimum and \ref m_positionMaximum)
        bool positionWithinBounds(const Eigen::Vector3d& position) const
        {
            return (position(0) >= m_positionMinimum(0) && position(1) >= m_positionMinimum(1) && position(2) >= m_positionMinimum(2) && //
                position(0) <= m_positionMaximum(0) && position(1) <= m_positionMaximum(1) && position(2) <= m_positionMaximum(2));
        }

        //! Checks, if the given metric is within the bounds (\ref m_metricMinimum and \ref m_metricMaximum)
        bool metricWithinBounds(const double& metric) const
        {
            return (metric >= m_metricMinimum && metric <= m_metricMaximum);
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

            // Contribution of m_level
            totalSize += sizeof(m_level);

            // Contribution of m_metricType
            totalSize += sizeof(m_metricType);

            // Contribution of m_meshType
            totalSize += sizeof(m_meshType);

            // Contribution of m_positionMinimum
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double);

            // Contribution of m_positionMaximum
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double);

            // Contribution of m_metricMinimum
            totalSize += sizeof(m_metricMinimum);

            // Contribution of m_metricMaximum
            totalSize += sizeof(m_metricMaximum);

            // Contribution of m_voxelMeshScaling
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double);

            // Contribution of m_icoSphereSubdivisions
            totalSize += sizeof(m_icoSphereSubdivisions);

            // Contribution of m_metricVolumeThreshold
            totalSize += sizeof(m_metricVolumeThreshold);

            // Contribution of m_colorGradient
            totalSize += sizeof(io::serialization::BinaryStreamSize); // Header
            totalSize += m_colorGradient.size() * (2 * sizeof(io::serialization::BinaryStreamSize) + sizeof(geometry::CGMesh::ColorChannelType) * geometry::CGMesh::ColorType::RowsAtCompileTime * geometry::CGMesh::ColorType::ColsAtCompileTime);

            // Contribution of m_metricFirstColor
            totalSize += sizeof(m_metricFirstColor);

            // Contribution of m_metricLastColor
            totalSize += sizeof(m_metricLastColor);

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            const uint8_t metricTypeInt = static_cast<uint8_t>(m_metricType);
            const uint8_t meshTypeInt = static_cast<uint8_t>(m_meshType);
            return io::serialization::serialize(stream, endianness, //
                m_name, m_level, metricTypeInt, meshTypeInt, //
                m_positionMinimum, m_positionMaximum, m_metricMinimum, m_metricMaximum, m_voxelMeshScaling, m_icoSphereSubdivisions, m_metricVolumeThreshold, //
                m_colorGradient, m_metricFirstColor, m_metricLastColor);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            uint8_t metricTypeInt = 0;
            uint8_t meshTypeInt = 0;
            const auto returnValue = io::serialization::deSerialize(stream, index, endianness, //
                m_name, m_level, metricTypeInt, meshTypeInt, //
                m_positionMinimum, m_positionMaximum, m_metricMinimum, m_metricMaximum, m_voxelMeshScaling, m_icoSphereSubdivisions, m_metricVolumeThreshold, //
                m_colorGradient, m_metricFirstColor, m_metricLastColor);
            m_metricType = static_cast<TaskSpaceMetric::Type>(metricTypeInt);
            m_meshType = static_cast<TaskSpaceMesh::Type>(meshTypeInt);
            return returnValue;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
