/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../geometry/CGMeshFactory.hpp"
#include "../../geometry/CGMeshTools.hpp"
#include "../../io/logging/LogFile.hpp"
#include "../../parallel/ThreadSafeContainer.hpp"
#include "TaskSpaceCellLogData.hpp"
#include "TaskSpaceMeshSpecification.hpp"
#include "TaskSpaceOctree.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace analysis {
    //! Discretized task-space
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the underlying octree
     */
    template <unsigned int L>
    class TaskSpace : protected parallel::ThreadSafeContainer, public io::serialization::SerializableData {
    public:
        // Type definitions
        using SampleList = std::vector<TaskSpaceSample, Eigen::aligned_allocator<TaskSpaceSample>>; //!< Type of a list of samples

        //! Default constructor
        TaskSpace()
            : parallel::ThreadSafeContainer()
            , io::serialization::SerializableData()
        {
        }

    protected:
        //! Copy constructor (internal)
        /*!
         * \warning **Not thread-safe** -> should only be called by the thread-safe wrapper!
         *
         * \param [in] original Reference to original object.
         */
        TaskSpace(const TaskSpace& original, const int& /* <- trick used for locking mutex */)
            : ThreadSafeContainer(original)
            , m_octree(original.m_octree)
            , m_droppedSamples(original.m_droppedSamples)
        {
        }

    public:
        //! Copy constructor (wrapper) (thread-safe)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        TaskSpace(const TaskSpace& original)
            : TaskSpace(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (thread-safe)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         *
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        TaskSpace& operator=(const TaskSpace& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            ThreadSafeContainer::operator=(reference);
            m_octree = reference.m_octree;
            m_droppedSamples = reference.m_droppedSamples;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

    protected:
        // Members
        // -------
        // Start protected data
        TaskSpaceOctree<L> m_octree; //!< Underlying octree (cells = "voxels")
        uint64_t m_droppedSamples = 0; //!< \copybrief droppedSamples()
        // End protected data

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality** (**thread-safe**)
        bool operator==(const TaskSpace& reference) const
        {
            bool equality = true;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForRead(); // Lock ourselves for reading (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Compare members
            if (m_octree != reference.m_octree || //
                m_droppedSamples != reference.m_droppedSamples)
                equality = false;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return equality;
        }

        //! Comparison operator: **inequality** (**thread-safe**)
        inline bool operator!=(const TaskSpace& reference) const { return !(*this == reference); }

        // Setters (thread-safe)
        // ---------------------
    public:
        //! Setup of octree data structure (**thread-safe**)
        /*!
         * Automatically computes the cell-count in all dimensions from the given bounding box (\p minimumPosition and \p maximumPosition).
         *
         * \param [in] minimumPosition Minimum position (x, y, z) which should be representable by the octree [m]
         * \param [in] maximumPosition Maximum position (x, y, z) which should be representable by the octree [m]
         * \param [in] minimumCellDimensions Dimensions (x, y, z) of a single cell on the lowest level of the octree [m]
         * \param [in] extraCellCount Count of cells to append to the borders (lowest level) as "safety margin"
         * \return `true` on success, `false` otherwise (allocation failed)
         */
        bool setup(const Eigen::Vector3d& minimumPosition, const Eigen::Vector3d& maximumPosition, const Eigen::Vector3d& minimumCellDimensions, const size_t& extraCellCount)
        {
            // Initialize helpers
            static constexpr unsigned int cellMultiplier = 1 << (L - 1);

            // Compute desired geometry
            const Eigen::Vector3d desiredCenter = 0.5 * (minimumPosition + maximumPosition);
            const Eigen::Vector3d desiredDimensions = maximumPosition - minimumPosition;

            // Compute octree geometry
            std::array<size_t, 3> firstLevelSize{}; // Size of the first (upmost) level grid in the octree (cell counts)
            std::array<size_t, 3> lastLevelSize{}; // Size of the last (lowest) level grid in the octree (cell counts)
            Eigen::Vector3d octreeDimensions = Eigen::Vector3d::Zero(); // Total dimensions (x,y,z) of the octree (grid sizes) in [m]
            for (int i = 0; i < 3; i++) {
                const double targetLastLevelSize = ceil(desiredDimensions(i) / minimumCellDimensions(i)) + extraCellCount;
                firstLevelSize[i] = std::ceil(targetLastLevelSize / (double)cellMultiplier);
                firstLevelSize[i] = std::max(firstLevelSize[i], (size_t)1); // One cell minimum
                lastLevelSize[i] = firstLevelSize[i] * cellMultiplier;
                octreeDimensions(i) = lastLevelSize[i] * minimumCellDimensions(i);
            }

            // Update octree
            lockForWrite();
            m_octree.m_center = desiredCenter;
            m_octree.m_dimensions = octreeDimensions;
            const bool resizeResult = m_octree.resize(firstLevelSize);
            unlock();
            if (resizeResult == false)
                return false; // The resizing failed (out of memory?) -> pass back failure to caller

            // Perform an initial reset of the complete octree (initialize all cells)
            reset();

            // Success!
            return true;
        }

        //! Resets the complete taskspace (**thread-safe**)
        void reset()
        {
            lockForWrite();
            m_octree.reset();
            m_droppedSamples = 0;
            unlock();
        }

        //! Adds the given list of samples to the octree (**thread-safe**)
        /*!
         * \param [in] samples Group of samples to add to the octree
         * \return Count of "dropped" samples (samples lying outside of the octree)
         */
        size_t addSamples(const SampleList& samples)
        {
            // Initialize helpers
            size_t droppedSamples = 0;

            // Get octree parameters
            lockForRead(); // Lock protected memory for read
            const Eigen::Vector3d octreeCenter = m_octree.m_center;
            const Eigen::Vector3d octreeDimensions = m_octree.m_dimensions;
            const std::array<size_t, 3> lowestLevelSize = m_octree.size(L - 1);
            unlock(); // Unlock protected memory
            const Eigen::Vector3d cellDimensions(octreeDimensions(0) / ((double)lowestLevelSize[0]), octreeDimensions(1) / ((double)lowestLevelSize[1]), octreeDimensions(2) / ((double)lowestLevelSize[2]));

            // Pre-compute extra grid parameters
            const Eigen::Vector3d halfCellDimensions = 0.5 * cellDimensions;
            const Eigen::Vector3d octreeOrigin = octreeCenter - 0.5 * octreeDimensions;

            // Pre-compute positions
            std::vector<bool> sampleWithinGrid(samples.size(), true); // Flags indicating, if the corresponding sample lies within the grid, or not ("dropped")
            std::vector<std::array<size_t, 3>> octreeIndices; // Octree indices for each sample in the sample list ("In which cell should be write the sample?")
            octreeIndices.resize(samples.size());
            for (size_t i = 0; i < samples.size(); i++) {
                // Compute position of sample relative to octree origin
                const Eigen::Vector3d relativePosition = samples[i].position() - octreeOrigin;

                // Compute indices for each dimension
                for (size_t j = 0; j < 3; j++) {
                    const int64_t index = round((relativePosition(j) - halfCellDimensions(j)) / cellDimensions(j));
                    if (index >= 0 && index < (int64_t)lowestLevelSize[j])
                        octreeIndices[i][j] = index;
                    else {
                        sampleWithinGrid[i] = false; // Outside of grid -> drop this sample
                        droppedSamples++;
                        break;
                    }
                }
            }

            // Write samples to octree
            for (size_t i = 0; i < samples.size(); i++) {
                // Only write samples within the grid
                if (sampleWithinGrid[i] == true) {
                    // Add sample to grid cell
                    lockForWrite();
                    m_octree(L - 1, octreeIndices[i]).addSample(samples[i]);
                    unlock();
                }
            }

            // Update total count of dropped samples
            protectedDataAddition(m_droppedSamples, (uint64_t)droppedSamples);

            // Pass back count of dropped samples
            return droppedSamples;
        }

        //! Triggers update of octree (propagates data from bottom level to top level) (**thread-safe**)
        void update()
        {
            lockForWrite();

            // Propagate data from lowest level of octree to upmost level of octree
            m_octree.propagateBottomUp();

            // Propagate data from upmost level of octree to meta-cell
            m_octree.updateMetaCell();

            unlock();
        }

        // Getters (thread-safe)
        // ---------------------
    public:
        //! Position (x, y, z) of the center of the octree [m] \details **Thread-safe getter**
        inline Eigen::Vector3d center() const { return getProtectedData(m_octree.m_center); }

        //! Dimensions (x, y, z) of the octree [m] \details **Thread-safe getter**
        inline Eigen::Vector3d dimensions() const { return getProtectedData(m_octree.m_dimensions); }

        //! "Meta"-cell representing the complete octree \details **Thread-safe getter**
        inline TaskSpaceCell metaCell() const { return getProtectedData(m_octree.m_metaCell); }

        //! Returns the cell at the specified location \details **Thread-safe getter**
        /*!
         * \param [in] level Level in which the desired cell is contained
         * \param [in] x Index in x-dimension of the octree
         * \param [in] y Index in y-dimension of the octree
         * \param [in] z Index in z-dimension of the octree
         */
        inline TaskSpaceCell operator()(const size_t& level, const size_t& x, const size_t& y, const size_t& z) const { return getProtectedData(m_octree(level, x, y, z)); }

        //! \copydoc TaskSpaceOctree::cell(const size_t&, const size_t&) const \note **Thread-safe getter**
        inline TaskSpaceCell cell(const size_t& level, const size_t& cellIndex) const { return getProtectedData(m_octree.cell(level, cellIndex)); }

        //! \copydoc TaskSpaceOctree::size(const size_t&) const \note **Thread-safe getter**
        inline std::array<size_t, 3> size(const size_t& level) const { return getProtectedData(m_octree.size(level)); }

        //! \copydoc TaskSpaceOctree::cellCount(const size_t&) const \note **Thread-safe getter**
        inline size_t cellCount(const size_t& level) const
        {
            lockForRead();
            const auto returnValue = m_octree.cellCount(level);
            unlock();
            return returnValue;
        }

        //! \copydoc TaskSpaceOctree::totalCellCount() const \note **Thread-safe getter**
        inline size_t totalCellCount() const
        {
            lockForRead();
            const auto returnValue = m_octree.totalCellCount;
            unlock();
            return returnValue;
        }

        //! Total count of dropped samples (=samples outside of the volume spanned by the octree) \details **Thread-safe getter**
        inline uint64_t droppedSamples() const { return getProtectedData(m_droppedSamples); }

        // Postprocessing
        // --------------
    public:
        //! Writes a certain level of the octree to an output file (**thread-safe**)
        /*!
         * \param [in] level The desired octree level to output (use <0 for the meta-cell)
         * \param [in] targetDirectory Specifies the target directory
         * \param [in] fileName Specifies the name of the logfile (without extension)
         * \param [in] compressed If `true` the file is compressed, otherwise not
         * \return `true` on success, `false` otherwise
         */
        bool writeOctreeLevelToFile(const int64_t& level, const std::string& targetDirectory, const std::string& fileName, const bool& compressed) const
        {
            // Setup the logfile
            io::LogFile<TaskSpaceCellLogData> logFile(fileName, targetDirectory, fileName, ".log", false, compressed, false, 1);

            // Open file and write header
            if (logFile.openLogFile() == false || logFile.writeLogHeader(core::Time::currentTime()) == false)
                return false;

            lockForRead();

            // Get octree parameters
            const Eigen::Vector3d octreeCenter = m_octree.m_center;
            const Eigen::Vector3d octreeDimensions = m_octree.m_dimensions;
            const std::array<size_t, 3> currentLevelSize = m_octree.size(level);
            const Eigen::Vector3d cellDimensions(octreeDimensions(0) / ((double)currentLevelSize[0]), octreeDimensions(1) / ((double)currentLevelSize[1]), octreeDimensions(2) / ((double)currentLevelSize[2]));

            // Pre-compute extra parameters
            const Eigen::Vector3d halfCellDimensions = 0.5 * cellDimensions;
            const Eigen::Vector3d octreeOrigin = octreeCenter - 0.5 * octreeDimensions;

            // Output cell data
            if (level < 0) {
                // Output meta-cell
                logFile.addNewDataToBuffer(TaskSpaceCellLogData(level, std::array<size_t, 3>{}, m_octree.m_center, m_octree.m_metaCell));
                if (logFile.processLogBuffer() == false) {
                    unlock();
                    return false;
                }
            } else {
                // Iterate through all cells in this level
                const std::array<size_t, 3>& levelSize = m_octree.size(level);
                for (size_t x = 0; x < levelSize[0]; x++) {
                    for (size_t y = 0; y < levelSize[1]; y++) {
                        for (size_t z = 0; z < levelSize[2]; z++) {
                            // Get reference to cell
                            const TaskSpaceCell& currentCell = m_octree(level, x, y, z);

                            // Compute voxel center
                            const Eigen::Vector3d voxelCenter = octreeOrigin + Eigen::Vector3d(x * cellDimensions(0), y * cellDimensions(1), z * cellDimensions(2)) + halfCellDimensions;

                            // Output cell
                            logFile.addNewDataToBuffer(TaskSpaceCellLogData(level, std::array<size_t, 3>{ x, y, z }, voxelCenter, currentCell));
                            if (logFile.processLogBuffer() == false) {
                                unlock();
                                return false;
                            }
                        }
                    }
                }
            }

            unlock();

            // Write footer and close file
            if (logFile.writeLogFooter(core::Time::currentTime()) == false || logFile.closeLogFile() == false)
                return false;

            // Otherwise: success!
            return true;
        }

        //! Creates a mesh for visualization (**thread-safe**)
        /*! \param [in] specification Specification of the mesh to create */
        geometry::CGMesh createMesh(const TaskSpaceMeshSpecification& specification) const
        {
            // Initialize helpers
            const bool withColor = (specification.m_colorGradient.size() > 0);

            lockForRead();

            // Get octree parameters
            const Eigen::Vector3d& octreeCenter = m_octree.m_center;
            const Eigen::Vector3d& octreeDimensions = m_octree.m_dimensions;
            const std::array<size_t, 3> currentLevelSize = m_octree.size(specification.m_level);
            const Eigen::Vector3d cellDimensions(octreeDimensions(0) / ((double)currentLevelSize[0]), octreeDimensions(1) / ((double)currentLevelSize[1]), octreeDimensions(2) / ((double)currentLevelSize[2]));

            // Pre-compute extra parameters
            const Eigen::Vector3d halfCellDimensions = 0.5 * cellDimensions;
            const Eigen::Vector3d octreeOrigin = octreeCenter - 0.5 * octreeDimensions;
            const size_t totalVoxelCount = m_octree.cellCount(specification.m_level);

            // Compute color distribution (assigns a color to each voxel in the octree)
            // --------------------------
            memory::MultiVector<geometry::CGMesh::ColorType, 3> colorGrid;
            memory::MultiVector<geometry::CGMesh::ColorType, 3> colorGridInterpolated; // Interpolated color grid
            if (withColor == true) {
                // Compute main color grid
                computeColorGrid(specification, colorGrid);

                // Compute interpolated color grid
                if (specification.m_meshType == TaskSpaceMesh::Type::VOXEL_BOX_SMOOTH)
                    interpolateColorGrid(specification, colorGrid, colorGridInterpolated);
            }

            // Type A: voxels
            // --------------
            if (specification.m_meshType == TaskSpaceMesh::Type::VOXEL_BOX_FLAT || specification.m_meshType == TaskSpaceMesh::Type::VOXEL_BOX_SMOOTH || specification.m_meshType == TaskSpaceMesh::Type::VOXEL_ICOSPHERE) {
                // Initialize helpers
                geometry::CGMesh voxelMesh;
                if (specification.m_meshType == TaskSpaceMesh::Type::VOXEL_BOX_FLAT || specification.m_meshType == TaskSpaceMesh::Type::VOXEL_BOX_SMOOTH)
                    voxelMesh = geometry::CGMeshFactory::createBox(1.0, 1.0, 1.0);
                if (specification.m_meshType == TaskSpaceMesh::Type::VOXEL_ICOSPHERE)
                    voxelMesh = geometry::CGMeshFactory::createIcoSphere(0.5, specification.m_icoSphereSubdivisions);
                voxelMesh.scale((cellDimensions.array() * specification.m_voxelMeshScaling.array()).matrix());

                // Iterate through voxels
                std::vector<geometry::CGMesh, Eigen::aligned_allocator<geometry::CGMesh>> meshList;
                std::vector<geometry::CGMesh const*> meshPointerList;
                meshList.reserve(totalVoxelCount);
                meshPointerList.reserve(totalVoxelCount);
                for (size_t x = 0; x < currentLevelSize[0]; x++) {
                    for (size_t y = 0; y < currentLevelSize[1]; y++) {
                        for (size_t z = 0; z < currentLevelSize[2]; z++) {
                            // Get reference to cell
                            const TaskSpaceCell& currentCell = m_octree(specification.m_level, x, y, z);

                            // Compute voxel center
                            const Eigen::Vector3d voxelCenter = octreeOrigin + Eigen::Vector3d(x * cellDimensions(0), y * cellDimensions(1), z * cellDimensions(2)) + halfCellDimensions;

                            // Check, if voxel center is within bounds and there is at least one sample in the cell (otherwise the metrics are not computed)
                            if (specification.positionWithinBounds(voxelCenter) == true && currentCell.m_totalSamples > 0) {
                                // Get local metric value
                                const double localMetric = currentCell.metric(specification.m_metricType);

                                // Check, if local metric is within desired bounds
                                if (specification.metricWithinBounds(localMetric) == true) {
                                    // Add voxel mesh to list
                                    meshList.push_back(voxelMesh);
                                    meshPointerList.push_back(&meshList.back());

                                    // Shift to voxel center position
                                    meshList.back().translate(voxelCenter);

                                    // Check, if we should set the color buffer
                                    if (withColor == true)
                                        colorizeVoxelMesh(meshList.back(), x, y, z, colorGrid, colorGridInterpolated, specification.m_meshType);
                                }
                            }
                        }
                    }
                }

                unlock();

                // Pass back mesh
                return geometry::CGMeshTools::merge(meshPointerList);
            }
            // Type B: volume
            // --------------
            else if (specification.m_meshType == TaskSpaceMesh::Type::VOLUME) {
                // Create density grid
                geometry::CGMeshFactory::MarchingCubesDensityGrid densityGrid(currentLevelSize);
                densityGrid.fill(0); // Initialize as "empty"
                for (size_t x = 0; x < currentLevelSize[0]; x++) {
                    for (size_t y = 0; y < currentLevelSize[1]; y++) {
                        for (size_t z = 0; z < currentLevelSize[2]; z++) {
                            // Get reference to cell and density
                            const TaskSpaceCell& currentCell = m_octree(specification.m_level, x, y, z);
                            double& currentDensity = densityGrid(x, y, z);

                            // Compute voxel center
                            const Eigen::Vector3d voxelCenter = octreeOrigin + Eigen::Vector3d(x * cellDimensions(0), y * cellDimensions(1), z * cellDimensions(2)) + halfCellDimensions;

                            // Check, if voxel center is within bounds and there is at least one sample in the cell (otherwise the metrics are not computed)
                            if (specification.positionWithinBounds(voxelCenter) == true && currentCell.m_totalSamples > 0) {
                                // Get local metric value
                                const double localMetric = currentCell.metric(specification.m_metricType);

                                // Check, if local metric is within desired bounds
                                if (specification.metricWithinBounds(localMetric) == true)
                                    currentDensity = localMetric;
                            }
                        }
                    }
                }

                unlock();

                // Compute volume via marching cubes algorithm
                geometry::CGMesh mesh = geometry::CGMeshFactory::createVolumeMarchingCubes(densityGrid, specification.m_metricVolumeThreshold, octreeDimensions, true, colorGrid);
                mesh.translate(octreeCenter - 0.5 * octreeDimensions);
                return mesh;
            }
            // Type C: point cloud
            // -------------------
            else if (specification.m_meshType == TaskSpaceMesh::Type::POINT_CLOUD) {
                // Iterate through voxels
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vertexBuffer;
                vertexBuffer.reserve(totalVoxelCount);
                std::vector<geometry::CGMesh::ColorType, Eigen::aligned_allocator<geometry::CGMesh::ColorType>> colorBuffer;
                if (withColor == true)
                    colorBuffer.reserve(totalVoxelCount);
                for (size_t x = 0; x < currentLevelSize[0]; x++) {
                    for (size_t y = 0; y < currentLevelSize[1]; y++) {
                        for (size_t z = 0; z < currentLevelSize[2]; z++) {
                            // Get reference to cell
                            const TaskSpaceCell& currentCell = m_octree(specification.m_level, x, y, z);

                            // Compute voxel center
                            const Eigen::Vector3d voxelCenter = octreeOrigin + Eigen::Vector3d(x * cellDimensions(0), y * cellDimensions(1), z * cellDimensions(2)) + halfCellDimensions;

                            // Add point for this cell
                            // -----------------------
                            // Check, if voxel center is within bounds and there is at least one sample in the cell (otherwise the metrics are not computed)
                            if (specification.positionWithinBounds(voxelCenter) == true && currentCell.m_totalSamples > 0) {
                                // Get local metric value
                                const double localMetric = currentCell.metric(specification.m_metricType);

                                // Check, if local metric is within desired bounds
                                if (specification.metricWithinBounds(localMetric) == true) {
                                    vertexBuffer.push_back(voxelCenter);
                                    if (withColor == true)
                                        colorBuffer.push_back(colorGrid(x, y, z));
                                }
                            }
                        }
                    }
                }

                unlock();

                // Create mesh from point cloud
                geometry::CGMesh mesh;
                mesh.m_vertexBuffer.resize(Eigen::NoChange, vertexBuffer.size());
                for (size_t i = 0; i < vertexBuffer.size(); i++)
                    mesh.m_vertexBuffer.col(i) = vertexBuffer[i];
                mesh.m_colorBuffer.resize(Eigen::NoChange, colorBuffer.size());
                for (size_t i = 0; i < colorBuffer.size(); i++)
                    mesh.m_colorBuffer.col(i) = colorBuffer[i];

                // Pass back mesh
                return mesh;
            }
            // Unknown type!
            // -------------
            else {
                unlock();

                // Pass back empty mesh
                assert(false);
                return geometry::CGMesh();
            }
        }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of members
            totalSize += m_octree.computeBinaryStreamSize(); // Contribution of m_octree
            totalSize += sizeof(m_droppedSamples); // Contribution of m_droppedSamples

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            return m_octree.serialize(stream, endianness) + io::serialization::serialize(stream, endianness, m_droppedSamples);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            const io::serialization::BinaryStreamSize octreeSize = m_octree.deSerialize(stream, index, endianness);
            if (octreeSize == 0)
                return 0;
            return octreeSize + io::serialization::deSerialize(stream, index + octreeSize, endianness, m_droppedSamples);
        }

        // Helpers
        // -------
    protected:
        //! Computes the global bounds for the metric (minimum and maximum) (**not** thread-safe)
        /*!
         * \warning The data structure has to be locked for read when calling this method!
         *
         * \param [in] level Level of the octree
         * \param [in] metricType Type of the metric
         * \param [out] minimum Global minimum for the metric on this level
         * \param [out] maximum Global maximum for the metric on this level
         */
        void getGlobalMetricBounds(const size_t& level, const TaskSpaceMetric::Type& metricType, double& minimum, double& maximum) const
        {
            // Reset values
            minimum = 0;
            maximum = 0;

            // Check type of metric
            if (metricType == TaskSpaceMetric::Type::REACHABILITY) {
                // Get minimum and maximum of per-cell sample count
                const size_t totalVoxelCount = m_octree.cellCount(level);
                for (size_t i = 0; i < totalVoxelCount; i++) {
                    const TaskSpaceCell& currentCell = m_octree.cell(level, i);
                    if (i == 0) {
                        minimum = currentCell.m_totalSamples;
                        maximum = currentCell.m_totalSamples;
                    } else {
                        minimum = std::min(minimum, (double)currentCell.m_totalSamples);
                        maximum = std::max(maximum, (double)currentCell.m_totalSamples);
                    }
                }
            } else if (metricType == TaskSpaceMetric::Type::CONDITION_INDEX_MINIMUM || metricType == TaskSpaceMetric::Type::CONDITION_INDEX_MEAN || metricType == TaskSpaceMetric::Type::CONDITION_INDEX_MAXIMUM) {
                minimum = m_octree.m_metaCell.m_conditionIndexMinimum;
                maximum = m_octree.m_metaCell.m_conditionIndexMaximum;
            } else if (metricType == TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MINIMUM || metricType == TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MEAN || metricType == TaskSpaceMetric::Type::MANIPULABILITY_MEASURE_MAXIMUM) {
                minimum = m_octree.m_metaCell.m_manipulabilityMeasureMinimum;
                maximum = m_octree.m_metaCell.m_manipulabilityMeasureMaximum;
            } else if (metricType == TaskSpaceMetric::Type::JOINT_RANGE_AVAILABILITY_MINIMUM || metricType == TaskSpaceMetric::Type::JOINT_RANGE_AVAILABILITY_MEAN || metricType == TaskSpaceMetric::Type::JOINT_RANGE_AVAILABILITY_MAXIMUM) {
                minimum = m_octree.m_metaCell.m_jointRangeAvailabilityMinimum;
                maximum = m_octree.m_metaCell.m_jointRangeAvailabilityMaximum;
            } else {
                // Unknown metric type!
                assert(false);
            }
        }

        //! Computes a grid storing the color information of the volume
        /*!
         * \warning The data structure has to be locked for read when calling this method!
         *
         * \param [in] specification Specification of the mesh used to colorize the volume
         * \param [out] colorGrid Computed color grid
         */
        void computeColorGrid(const TaskSpaceMeshSpecification& specification, memory::MultiVector<geometry::CGMesh::ColorType, 3>& colorGrid) const
        {
            // Initialize helpers
            const std::array<size_t, 3> currentLevelSize = m_octree.size(specification.m_level);
            const size_t totalVoxelCount = m_octree.cellCount(specification.m_level);

            // Get global metric values
            double globalMetricMinimum = 0.0;
            double globalMetricMaximum = 0.0;
            getGlobalMetricBounds(specification.m_level, specification.m_metricType, globalMetricMinimum, globalMetricMaximum);

            // Compute metric values linked to first and last color in color gradient
            double metricFirstColor = globalMetricMinimum;
            if (specification.m_metricFirstColor >= 0)
                metricFirstColor = specification.m_metricFirstColor;
            double metricLastColor = globalMetricMaximum;
            if (specification.m_metricLastColor >= 0)
                metricLastColor = specification.m_metricLastColor;

            // Allocate memory
            colorGrid.resize(currentLevelSize);

            // Run through voxels
            for (size_t i = 0; i < totalVoxelCount; i++) {
                geometry::CGMesh::ColorType& currentColor = colorGrid[i];
                if (specification.m_colorGradient.size() == 1) {
                    // Simplest case: constant color
                    currentColor = specification.m_colorGradient[0];
                } else {
                    // Interpolate color ...
                    const TaskSpaceCell& currentCell = m_octree.cell(specification.m_level, i);
                    const double localMetric = currentCell.metric(specification.m_metricType);
                    double normalizedMetric = (localMetric - metricFirstColor) / (metricLastColor - metricFirstColor); // Normalized metric value (0...1)
                    if (normalizedMetric < 0.0)
                        normalizedMetric = 0.0;
                    if (normalizedMetric > 1.0)
                        normalizedMetric = 1.0;
                    const double colorGradientStepSize = 1.0 / ((double)specification.m_colorGradient.size() - 1.0);
                    const size_t colorGradientSegmentIndex = normalizedMetric / colorGradientStepSize;
                    if (colorGradientSegmentIndex + 1 >= specification.m_colorGradient.size())
                        currentColor = specification.m_colorGradient.back();
                    else {
                        const auto& firstColor = specification.m_colorGradient[colorGradientSegmentIndex];
                        const auto& secondColor = specification.m_colorGradient[colorGradientSegmentIndex + 1];
                        const double x = (normalizedMetric - colorGradientSegmentIndex * colorGradientStepSize) / colorGradientStepSize;
                        for (Eigen::Index c = 0; c < currentColor.size(); c++) {
                            const double firstValue = firstColor(c);
                            const double secondValue = secondColor(c);
                            const int64_t interpolatedValue = std::round(firstValue + x * (secondValue - firstValue));
                            if (interpolatedValue < 0)
                                currentColor(c) = 0;
                            else if (interpolatedValue > 255)
                                currentColor(c) = 255;
                            else
                                currentColor(c) = interpolatedValue;
                        }
                    }
                }
            }
        }

        //! Computes a grid storing the **interpolated** color information of the volume
        /*!
         * \warning The data structure has to be locked for read when calling this method!
         *
         * \param [in] specification Specification of the mesh used to colorize the volume
         * \param [in] colorGrid The "original" color grid
         * \param [out] colorGridInterpolated The interpolated color grid
         */
        void interpolateColorGrid(const TaskSpaceMeshSpecification& specification, const memory::MultiVector<geometry::CGMesh::ColorType, 3>& colorGrid, memory::MultiVector<geometry::CGMesh::ColorType, 3>& colorGridInterpolated) const
        {
            // Allocate memory
            const std::array<size_t, 3>& colorGridSize = colorGrid.size();
            const std::array<size_t, 3> colorGridInterpolatedSize{ { colorGridSize[0] + 1, colorGridSize[1] + 1, colorGridSize[2] + 1 } }; // Target size of interpolated color grid
            colorGridInterpolated.resize(colorGridInterpolatedSize);

            // Iterate through voxels
            for (int64_t x = 0; x < (int64_t)colorGridInterpolatedSize[0]; x++) {
                for (int64_t y = 0; y < (int64_t)colorGridInterpolatedSize[1]; y++) {
                    for (int64_t z = 0; z < (int64_t)colorGridInterpolatedSize[2]; z++) {
                        // Get reference to this color
                        geometry::CGMesh::ColorType& currentColor = colorGridInterpolated(x, y, z);

                        // Compute locations of grid points in neighborhood
                        std::array<Eigen::Vector3i, 8> gridPointLocation{ { Eigen::Vector3i(x, y - 1, z - 1),
                            Eigen::Vector3i(x, y, z - 1),
                            Eigen::Vector3i(x - 1, y, z - 1),
                            Eigen::Vector3i(x - 1, y - 1, z - 1),
                            Eigen::Vector3i(x, y - 1, z),
                            Eigen::Vector3i(x, y, z),
                            Eigen::Vector3i(x - 1, y, z),
                            Eigen::Vector3i(x - 1, y - 1, z) } };

                        // Iterate through neighboring grid points
                        Eigen::Vector4i interpolatedColor = Eigen::Vector4i::Zero();
                        size_t validGridPoints = 0;
                        for (size_t n = 0; n < 8; n++) {
                            // Check, if neighbor is within grid
                            if (gridPointLocation[n](0) >= 0 && gridPointLocation[n](0) < (int64_t)colorGridSize[0] // x-dimension
                                && gridPointLocation[n](1) >= 0 && gridPointLocation[n](1) < (int64_t)colorGridSize[1] // y-dimension
                                && gridPointLocation[n](2) >= 0 && gridPointLocation[n](2) < (int64_t)colorGridSize[2]) // z-dimension
                            {
                                const std::array<size_t, 3> currentGridPointLocation{ { (size_t)gridPointLocation[n](0), (size_t)gridPointLocation[n](1), (size_t)gridPointLocation[n](2) } };
                                const TaskSpaceCell& gridPointCell = m_octree(specification.m_level, currentGridPointLocation);
                                if (gridPointCell.m_totalSamples > 0) {
                                    // Add contribution of this grid point
                                    validGridPoints++;
                                    const geometry::CGMesh::ColorType& gridPointColor = colorGrid(currentGridPointLocation);
                                    for (int c = 0; c < 4; c++)
                                        interpolatedColor(c) += gridPointColor(c);
                                }
                            }
                        }

                        // Store interpolated color in grid
                        for (int c = 0; c < 4; c++) {
                            const double interpolatedValue = ((double)interpolatedColor(c)) / ((double)validGridPoints);
                            if (interpolatedValue < 0)
                                currentColor(c) = 0;
                            else if (interpolatedValue > 255)
                                currentColor(c) = 255;
                            else
                                currentColor(c) = std::round(interpolatedValue);
                        }
                    }
                }
            }
        }

        //! Creates a color buffer for the given voxel
        /*!
         * \warning The data structure has to be locked for read when calling this method!
         *
         * \param [in,out] voxelMesh The voxel mesh to colorize (compute the color buffer for)
         * \param [in] x The x-index of the voxel in the grid
         * \param [in] y The y-index of the voxel in the grid
         * \param [in] z The z-index of the voxel in the grid
         * \param [in] colorGrid The color grid
         * \param [in] colorGridInterpolated The interpolated color grid
         * \param [in] meshType The type of the mesh
         */
        void colorizeVoxelMesh(geometry::CGMesh& voxelMesh, const size_t& x, const size_t& y, const size_t& z, const memory::MultiVector<geometry::CGMesh::ColorType, 3>& colorGrid, const memory::MultiVector<geometry::CGMesh::ColorType, 3>& colorGridInterpolated, const TaskSpaceMesh::Type& meshType) const
        {
            // Initialize helpers
            static const std::array<size_t, 24> vertexCornerMapping{ { 0, 1, 5, 4, 2, 3, 7, 6, 1, 2, 6, 5, 3, 0, 4, 7, 4, 5, 6, 7, 3, 2, 1, 0 } };

            // Add color buffer
            voxelMesh.m_colorBuffer.resize(Eigen::NoChange, voxelMesh.m_vertexBuffer.cols());

            // Check shading type
            if (meshType == TaskSpaceMesh::Type::VOXEL_BOX_SMOOTH) {
                // ...interpolated shading

                // Get corner locations (in grid)
                std::array<Eigen::Vector3i, 8> cornerLocation{ { Eigen::Vector3i(x + 1, y, z),
                    Eigen::Vector3i(x + 1, y + 1, z),
                    Eigen::Vector3i(x, y + 1, z),
                    Eigen::Vector3i(x, y, z),
                    Eigen::Vector3i(x + 1, y, z + 1),
                    Eigen::Vector3i(x + 1, y + 1, z + 1),
                    Eigen::Vector3i(x, y + 1, z + 1),
                    Eigen::Vector3i(x, y, z + 1) } };

                // Iterate through all vertices (for mapping see geometry::CGMeshFactory::createBox)
                for (Eigen::Index v = 0; v < 24; v++) {
                    const auto& vertexLocation = cornerLocation[vertexCornerMapping[v]];
                    voxelMesh.m_colorBuffer.col(v) = colorGridInterpolated((size_t)vertexLocation(0), (size_t)vertexLocation(1), (size_t)vertexLocation(2));
                }
            } else {
                // ...simple flat shading
                const geometry::CGMesh::ColorType& currentColor = colorGrid(x, y, z);
                for (Eigen::Index v = 0; v < voxelMesh.m_vertexBuffer.cols(); v++)
                    voxelMesh.m_colorBuffer.col(v) = currentColor;
            }
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
