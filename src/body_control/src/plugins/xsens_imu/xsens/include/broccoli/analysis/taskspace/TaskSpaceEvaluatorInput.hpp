/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/ply/PLYFormat.hpp"
#include "KinematicChain.hpp"
#include "TaskSpaceMeshSpecification.hpp"
#include <Eigen/StdVector>
#include <array>
#include <math.h>
#include <thread>

namespace broccoli {
namespace analysis {
    //! Container for all input-data of the task-space evaluator
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L>
    class TaskSpaceEvaluatorInput : public io::serialization::SerializableData {
    public:
        //! Default constructor
        TaskSpaceEvaluatorInput()
        {
            // Use position-only as default task-space setup
            setupTaskSpaceSelectionMatrix(true, true, true, false, false, false);

            // Try to detect available cpu cores
            m_maximumThreadCount = std::thread::hardware_concurrency();
            if (m_maximumThreadCount == 0)
                m_maximumThreadCount = 1; // Detection failed -> use 1 thread

            // Setup default mesh list
            setupOutputMeshesDefault();
        }

        // Members
        // =======
        // Generic
        // -------
        bool m_consoleOutput = true; //!< Flag to enable/disable console output (for debugging/inspection purposes)

        // Pre-processing (these parameters have to be set BEFORE pre-processing)
        // --------------
        // Definition of joint and task-space
        KinematicChain m_kinematicChain; //!< Kinematic chain used for sampling (defines the **joint-space**)
        Eigen::Matrix<double, Eigen::Dynamic, 6> m_taskSpaceSelectionMatrix; //!< Task space selection matrix (defines the **task-space**) - see \ref TaskSpaceSampleEvaluator::m_taskSpaceSelectionMatrix

        // Performance
        uint64_t m_maximumThreadCount = 1; //!< Maximum count of parallel threads used for computations

        // Task space specification
        Eigen::Vector3d m_minimumCellDimensions = Eigen::Vector3d(0.1, 0.1, 0.1); //!< Dimensions (x,y,z) of a single cell (voxel) in the lowest level of the octree [m]
        uint64_t m_extraCellCount = 3; //!< Count of cells to append to the borders (lowest level) as "safety margin"

        // Processing (these parameters have to be set BEFORE processing)
        // ----------
        // none...

        // Post-processing (these parameters have to be set BEFORE post-processing)
        // ---------------
        bool m_writeOutputFiles = true; //!< Flag to **globally** enable/disable writing of output files
        std::string m_outputFolder = ""; //!< Path to the folder for the output files
        io::PLYFormat::Type m_plyFileFormat = io::PLYFormat::Type::BINARY_LITTLE_ENDIAN; //!< Used file format for writing ply files

        // Evaluator
        bool m_writeOutputFilesEvaluator = true; //!< Flag to enable/disable writing the **current status** of the evaluator instance (serialized, binary) to a file (at end of pre-processing, processing and post-processing)
        std::string m_outputFilesEvaluatorPrefix = "Evaluator"; //!< File name prefix for writing output files of evaluator instance

        // Pre sampler
        bool m_writeOutputFilesPreSampler = false; //!< Flag to enable/disable writing output files of the pre-samplers (**Warning**: amount of data can be huge!)

        // Main sampler
        bool m_writeOutputFilesMainSampler = false; //!< Flag to enable/disable writing output files of the main-samplers (**Warning**: amount of data can be huge!)

        // Bounding box (pre-sampling)
        bool m_writeOutputFilesBoundingBoxPreSampling = true; //!< Flag to enable/disable writing output files of (pre-sampling) bounding box
        std::string m_outputFilesBoundingBoxPreSamplingName = "BoundingBox_PreSampling"; //!< File name for writing output files of (pre-sampling) bounding box
        bool m_outputFilesBoundingBoxPreSamplingWithColors = true; //!< Flag to enable/disable mesh colors for (pre-sampling) bounding box

        // Bounding box (main-sampling)
        bool m_writeOutputFilesBoundingBoxMainSampling = true; //!< Flag to enable/disable writing output files of (main-sampling) bounding box
        std::string m_outputFilesBoundingBoxMainSamplingName = "BoundingBox_MainSampling"; //!< File name for writing output files of (main-sampling) bounding box
        bool m_outputFilesBoundingBoxMainSamplingWithColors = true; //!< Flag to enable/disable mesh colors for (main-sampling) bounding box

        // Octree
        bool m_writeOutputFilesOctree = true; //!< Flag to enable/disable writing output files of octree
        std::string m_outputFilesOctreePrefix = "Octree"; //!< File name prefix for writing output files of octree
        std::vector<TaskSpaceMeshSpecification, Eigen::aligned_allocator<TaskSpaceMeshSpecification>> m_outputFilesOctreeMeshList; //!< List of octree meshes to output (specification of meshes)

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const TaskSpaceEvaluatorInput& reference) const
        {
            // Compare members
            if (m_consoleOutput != reference.m_consoleOutput || //
                m_kinematicChain != reference.m_kinematicChain || //
                m_taskSpaceSelectionMatrix.isApprox(reference.m_taskSpaceSelectionMatrix) == false || //
                m_maximumThreadCount != reference.m_maximumThreadCount || //
                m_minimumCellDimensions.isApprox(reference.m_minimumCellDimensions) == false || //
                m_extraCellCount != reference.m_extraCellCount || //
                m_writeOutputFiles != reference.m_writeOutputFiles || //
                m_outputFolder != reference.m_outputFolder || //
                m_plyFileFormat != reference.m_plyFileFormat || //
                m_writeOutputFilesEvaluator != reference.m_writeOutputFilesEvaluator || //
                m_outputFilesEvaluatorPrefix != reference.m_outputFilesEvaluatorPrefix || //
                m_writeOutputFilesPreSampler != reference.m_writeOutputFilesPreSampler || //
                m_writeOutputFilesMainSampler != reference.m_writeOutputFilesMainSampler || //
                m_writeOutputFilesBoundingBoxPreSampling != reference.m_writeOutputFilesBoundingBoxPreSampling || //
                m_outputFilesBoundingBoxPreSamplingName != reference.m_outputFilesBoundingBoxPreSamplingName || //
                m_outputFilesBoundingBoxPreSamplingWithColors != reference.m_outputFilesBoundingBoxPreSamplingWithColors || //
                m_writeOutputFilesBoundingBoxMainSampling != reference.m_writeOutputFilesBoundingBoxMainSampling || //
                m_outputFilesBoundingBoxMainSamplingName != reference.m_outputFilesBoundingBoxMainSamplingName || //
                m_outputFilesBoundingBoxMainSamplingWithColors != reference.m_outputFilesBoundingBoxMainSamplingWithColors || //
                m_writeOutputFilesOctree != reference.m_writeOutputFilesOctree || //
                m_outputFilesOctreePrefix != reference.m_outputFilesOctreePrefix || //
                m_outputFilesOctreeMeshList != reference.m_outputFilesOctreeMeshList)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const TaskSpaceEvaluatorInput& reference) const { return !(*this == reference); }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of members
            totalSize += sizeof(m_consoleOutput); // Contribution of m_consoleOutput
            totalSize += m_kinematicChain.computeBinaryStreamSize(); // Contribution of m_kinematicChain
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + sizeof(double) * m_taskSpaceSelectionMatrix.rows() * m_taskSpaceSelectionMatrix.cols(); // Contribution of m_taskSpaceSelectionMatrix
            totalSize += sizeof(m_maximumThreadCount); // Contribution of m_maximumThreadCount
            totalSize += 2 * sizeof(io::serialization::BinaryStreamSize) + 3 * sizeof(double); // Contribution of m_minimumCellDimensions
            totalSize += sizeof(m_extraCellCount); // Contribution of m_extraCellCount
            totalSize += sizeof(m_writeOutputFiles); // Contribution of m_writeOutputFiles
            totalSize += sizeof(io::serialization::BinaryStreamSize) + m_outputFolder.size(); // Contribution of m_outputFolder
            totalSize += sizeof(m_plyFileFormat); // Contribution of m_plyFileFormat
            totalSize += sizeof(m_writeOutputFilesEvaluator); // Contribution of m_writeOutputFilesEvaluator
            totalSize += sizeof(io::serialization::BinaryStreamSize) + m_outputFilesEvaluatorPrefix.size(); // Contribution of m_outputFilesEvaluatorPrefix
            totalSize += sizeof(m_writeOutputFilesPreSampler); // Contribution of m_writeOutputFilesPreSampler
            totalSize += sizeof(m_writeOutputFilesMainSampler); // Contribution of m_writeOutputFilesMainSampler
            totalSize += sizeof(m_writeOutputFilesBoundingBoxPreSampling); // Contribution of m_writeOutputFilesBoundingBoxPreSampling
            totalSize += sizeof(io::serialization::BinaryStreamSize) + m_outputFilesBoundingBoxPreSamplingName.size(); // Contribution of m_outputFilesBoundingBoxPreSamplingName
            totalSize += sizeof(m_outputFilesBoundingBoxPreSamplingWithColors); // Contribution of m_outputFilesBoundingBoxPreSamplingWithColors
            totalSize += sizeof(m_writeOutputFilesBoundingBoxMainSampling); // Contribution of m_writeOutputFilesBoundingBoxMainSampling
            totalSize += sizeof(io::serialization::BinaryStreamSize) + m_outputFilesBoundingBoxMainSamplingName.size(); // Contribution of m_outputFilesBoundingBoxMainSamplingName
            totalSize += sizeof(m_outputFilesBoundingBoxMainSamplingWithColors); // Contribution of m_outputFilesBoundingBoxMainSamplingWithColors
            totalSize += sizeof(m_writeOutputFilesOctree); // Contribution of m_writeOutputFilesOctree
            totalSize += sizeof(io::serialization::BinaryStreamSize) + m_outputFilesOctreePrefix.size(); // Contribution of m_outputFilesOctreePrefix
            totalSize += sizeof(io::serialization::BinaryStreamSize); // Contribution of m_outputFilesOctreeMeshList (Header)
            for (size_t i = 0; i < m_outputFilesOctreeMeshList.size(); i++)
                totalSize += m_outputFilesOctreeMeshList[i].computeBinaryStreamSize(); // Contribution of m_outputFilesOctreeMeshList (Data)

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            const uint8_t plyFileFormatInt = static_cast<uint8_t>(m_plyFileFormat);
            return io::serialization::serialize(stream, endianness, //
                m_consoleOutput, //
                m_kinematicChain, m_taskSpaceSelectionMatrix, m_maximumThreadCount, m_minimumCellDimensions, m_extraCellCount, //
                m_writeOutputFiles, m_outputFolder, plyFileFormatInt, //
                m_writeOutputFilesEvaluator, m_outputFilesEvaluatorPrefix, //
                m_writeOutputFilesPreSampler, m_writeOutputFilesMainSampler, //
                m_writeOutputFilesBoundingBoxPreSampling, m_outputFilesBoundingBoxPreSamplingName, m_outputFilesBoundingBoxPreSamplingWithColors, //
                m_writeOutputFilesBoundingBoxMainSampling, m_outputFilesBoundingBoxMainSamplingName, m_outputFilesBoundingBoxMainSamplingWithColors, //
                m_writeOutputFilesOctree, m_outputFilesOctreePrefix, m_outputFilesOctreeMeshList);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            uint8_t plyFileFormatInt = 0;
            const auto returnValue = io::serialization::deSerialize(stream, index, endianness, //
                m_consoleOutput, //
                m_kinematicChain, m_taskSpaceSelectionMatrix, m_maximumThreadCount, m_minimumCellDimensions, m_extraCellCount, //
                m_writeOutputFiles, m_outputFolder, plyFileFormatInt, //
                m_writeOutputFilesEvaluator, m_outputFilesEvaluatorPrefix, //
                m_writeOutputFilesPreSampler, m_writeOutputFilesMainSampler, //
                m_writeOutputFilesBoundingBoxPreSampling, m_outputFilesBoundingBoxPreSamplingName, m_outputFilesBoundingBoxPreSamplingWithColors, //
                m_writeOutputFilesBoundingBoxMainSampling, m_outputFilesBoundingBoxMainSamplingName, m_outputFilesBoundingBoxMainSamplingWithColors, //
                m_writeOutputFilesOctree, m_outputFilesOctreePrefix, m_outputFilesOctreeMeshList);
            m_plyFileFormat = static_cast<io::PLYFormat::Type>(plyFileFormatInt);
            return returnValue;
        }

        // Helpers
        // -------
    public:
        //! Generates \ref m_taskSpaceSelectionMatrix according to the chosen taskspace directions
        /*!
         * \param [in] translationX If `true` the translation in x-direction (root frame) is part of the task space
         * \param [in] translationY If `true` the translation in y-direction (root frame) is part of the task space
         * \param [in] translationZ If `true` the translation in z-direction (root frame) is part of the task space
         * \param [in] rotationX If `true` the rotation around the x-axis (root frame) is part of the task space
         * \param [in] rotationY If `true` the rotation around the y-axis (root frame) is part of the task space
         * \param [in] rotationZ If `true` the rotation around the z-axis (root frame) is part of the task space
         */
        void setupTaskSpaceSelectionMatrix(const bool& translationX, const bool& translationY, const bool& translationZ, const bool& rotationX, const bool& rotationY, const bool& rotationZ)
        {
            std::vector<Eigen::Index> m_components;
            m_components.reserve(6);
            if (translationX == true)
                m_components.push_back(0);
            if (translationY == true)
                m_components.push_back(1);
            if (translationZ == true)
                m_components.push_back(2);
            if (rotationX == true)
                m_components.push_back(3);
            if (rotationY == true)
                m_components.push_back(4);
            if (rotationZ == true)
                m_components.push_back(5);
            m_taskSpaceSelectionMatrix = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(m_components.size(), 6);
            for (size_t i = 0; i < m_components.size(); i++)
                m_taskSpaceSelectionMatrix(i, m_components[i]) = 1;
        }

        //! Generates a default selection for \ref m_outputFilesOctreeMeshList
        void setupOutputMeshesDefault()
        {
            m_outputFilesOctreeMeshList.clear();
            m_outputFilesOctreeMeshList.reserve(5 * L);
            for (unsigned int l = 0; l < L; l++) {
                const int64_t icoSphereSubdivisions = std::max(((int64_t)L) - ((int64_t)l), (int64_t)3);
                m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(l, TaskSpaceMetric::Type::REACHABILITY, TaskSpaceMesh::Type::VOXEL_BOX_FLAT));
                m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(l, TaskSpaceMetric::Type::REACHABILITY, TaskSpaceMesh::Type::VOXEL_BOX_SMOOTH));
                m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(l, TaskSpaceMetric::Type::REACHABILITY, TaskSpaceMesh::Type::VOXEL_ICOSPHERE));
                m_outputFilesOctreeMeshList.back().m_icoSphereSubdivisions = icoSphereSubdivisions;
                m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(l, TaskSpaceMetric::Type::REACHABILITY, TaskSpaceMesh::Type::VOLUME));
                m_outputFilesOctreeMeshList.back().m_metricVolumeThreshold = 0.5;
                m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(l, TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MAXIMUM, TaskSpaceMesh::Type::VOXEL_ICOSPHERE));
                m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(l, TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MAXIMUM, TaskSpaceMesh::Type::POINT_CLOUD));
            }
        }

        //! Generates all variants for \ref m_outputFilesOctreeMeshList
        void setupOutputMeshesAll()
        {
            const unsigned int levelCount = L;
            const uint8_t metricTypeCount = static_cast<uint8_t>(TaskSpaceMetric::Type::TYPE_COUNT);
            const uint8_t meshTypeCount = static_cast<uint8_t>(TaskSpaceMesh::Type::TYPE_COUNT);
            m_outputFilesOctreeMeshList.clear();
            m_outputFilesOctreeMeshList.reserve(levelCount * metricTypeCount * meshTypeCount);
            for (unsigned int i = 0; i < levelCount; i++) {
                const unsigned int& currentLevel = i;
                const int64_t icoSphereSubdivisions = ((int64_t)L) - ((int64_t)i);
                for (uint8_t j = 0; j < metricTypeCount; j++) {
                    const TaskSpaceMetric::Type currentMetricType = static_cast<TaskSpaceMetric::Type>(j);
                    for (uint8_t k = 0; k < meshTypeCount; k++) {
                        const TaskSpaceMesh::Type currentMeshType = static_cast<TaskSpaceMesh::Type>(k);
                        m_outputFilesOctreeMeshList.push_back(TaskSpaceMeshSpecification(currentLevel, currentMetricType, currentMeshType));
                        if (currentMeshType == TaskSpaceMesh::Type::VOXEL_ICOSPHERE)
                            m_outputFilesOctreeMeshList.back().m_icoSphereSubdivisions = icoSphereSubdivisions;
                        if (currentMetricType == TaskSpaceMetric::Type::REACHABILITY && currentMeshType == TaskSpaceMesh::Type::VOLUME)
                            m_outputFilesOctreeMeshList.back().m_metricVolumeThreshold = 0.5;
                    }
                }
            }
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
