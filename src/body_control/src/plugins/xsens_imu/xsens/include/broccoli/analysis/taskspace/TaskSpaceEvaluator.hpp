/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/filesystem.hpp"
#include "../../io/ply/PLYFileConverter.hpp"
#include "TaskSpaceEvaluatorInput.hpp"
#include "TaskSpaceEvaluatorOutput.hpp"
#include "TaskSpaceMainSampler.hpp"
#include "TaskSpacePreSampler.hpp"
#include "TaskSpaceSamplerGroup.hpp"

namespace broccoli {
namespace analysis {
    //! Manager class for taskspace evaluation
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * The typical usage is as follows:
     *   * Create instance of this class (and specify desired count of octree levels through the template parameter)
     *   * Set the input data (kinematic chain, options, etc.) through \ref input()
     *   * Trigger \ref preProcess() (setup of the problem + pre-evaluation)
     *   * Trigger \ref process() (main task - sampling of task space, filling octree)
     *   * Trigger \ref postProcess() (create output files, meshes, etc.)
     *   * Extract additional data from the output data structure - see \ref output()
     *
     * The runtimes of \ref preProcess(), \ref process() and \ref postProcess() can be arbitrarily large (depending on the count of samples to evaluate). Thus,
     * the user might want to run \ref preProcess() and \ref process() in one run and execute \ref postProcess() with different output file specifications in different
     * program runs. One one would like to trigger another postprocessing step without re-running \ref preProcess() and \ref process(). For this the class implements
     * \ref saveToFile() and \ref loadFromFile() to save the **current status** of the evaluator. The complete class is serialized and written in a binary file such
     * that one can re-run substeps without loss of any information. Note that per-default the evaluator saves its status at the beginning and end of all three processing
     * stages.
     *
     * \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L = 4>
    class TaskSpaceEvaluator : public io::serialization::SerializableData {
    public:
        //! Default constructor
        TaskSpaceEvaluator()
        {
        }

        //! Specialized constructor
        TaskSpaceEvaluator(const TaskSpaceEvaluatorInput<L>& input)
            : m_input(input)
        {
        }

        // Members
        // -------
    protected:
        // Input
        TaskSpaceEvaluatorInput<L> m_input; //!< \copybrief input()

        // Output
        TaskSpaceEvaluatorOutput<L> m_output; //!< \copybrief output()

        // Runtime measurements
        core::Time m_runTimePreProcess; //!< \copybrief runTimePreProcess()
        core::Time m_runTimeProcess; //!< \copybrief runTimeProcess()
        core::Time m_runTimePostProcess; //!< \copybrief runTimePostProcess()

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const TaskSpaceEvaluator& reference) const
        {
            // Compare members
            if (m_input != reference.m_input || //
                m_output != reference.m_output || //
                m_runTimePreProcess != reference.m_runTimePreProcess || //
                m_runTimeProcess != reference.m_runTimeProcess || //
                m_runTimePostProcess != reference.m_runTimePostProcess)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const TaskSpaceEvaluator& reference) const { return !(*this == reference); }

        // Processing steps
        // ----------------
    public:
        //! Perform pre-processing steps
        /*! \return `true` on success, `false` otherwise */
        bool preProcess()
        {
            // Ouput kinematic chain
            if (input().m_consoleOutput) {
                io::Console::info().print("---------------\n");
                io::Console::info().print("Kinematic chain\n");
                io::Console::info().print("---------------\n");
                io::Console::info().print(input().m_kinematicChain.toString());
                io::Console::flush();
            }

            // Start preprocessing
            if (input().m_consoleOutput) {
                io::Console::info().print("-------------\n");
                io::Console::info().print("Preprocessing\n");
                io::Console::info().print("-------------\n");
                io::Console::flush();
            }

            // File system
            // -----------
            if (input().m_writeOutputFiles == true) {
                // Create output folder
                if (input().m_outputFolder != "" && io::filesystem::directoryExists(input().m_outputFolder) == false) {
                    if (io::filesystem::createDirectory(input().m_outputFolder) == false) {
                        if (input().m_consoleOutput) {
                            io::Console::error().print("Preprocessing failed! Could not create output folder!\n");
                            io::Console::flush();
                        }
                        return false;
                    }
                }
            }

            // Save status of evaluator
            if (saveCurrentStatus("PreProcess_Begin") == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Preprocessing failed! Saving status failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // Start runtime measurement
            const core::Time startTime = core::Time::currentTime();

            // Pre-sampling
            // ------------
            // Setup input for pre-sampling
            TaskSpaceEvaluatorInput<L> preSamplingInput = input();
            for (size_t i = 0; i < preSamplingInput.m_kinematicChain.segments().size(); i++) {
                KinematicChainSegment& currentFrame = preSamplingInput.m_kinematicChain.segment(i);
                if (currentFrame.dofType() != KinematicChainSegment::DOFType::FIXED) {
                    currentFrame.samples() = currentFrame.samples() / 2; // Only evaluate every 2nd sample
                    if (currentFrame.samples() < 2)
                        currentFrame.samples() = 2; // Evaluate 2 samples per segment as minimum
                }
            }

            // Trigger pre sampler group
            TaskSpaceSamplerGroup<L, TaskSpacePreSampler<L>> preSamplerGroup(preSamplingInput, "PreSampler_");
            preSamplerGroup.trigger(m_output);

            // Extract bounding box paramenters
            const Eigen::Vector3d boundingBoxMinimum = m_output.boundingBoxPreSampling().minimum();
            const Eigen::Vector3d boundingBoxMaximum = m_output.boundingBoxPreSampling().maximum();

            // Setup of task space
            // -------------------
            if (m_output.taskspace().setup(boundingBoxMinimum, boundingBoxMaximum, input().m_minimumCellDimensions, input().m_extraCellCount) == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Preprocessing failed! Setup of task space failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // End runtime measurement
            m_runTimePreProcess = core::Time::currentTime() - startTime;
            if (input().m_consoleOutput) {
                io::Console::info().print("Preprocessing finished (duration: " + m_runTimePreProcess.encodeToDurationString() + "s)\n");
                io::Console::flush();
            }

            // Output statistics
            if (input().m_consoleOutput) {
                std::string consoleText;
                consoleText.reserve(4096);

                // Bounding box
                consoleText += "Computed bounding box (pre-sampling): ";
                consoleText += "[xmin=" + io::encoding::encodeToString(boundingBoxMinimum(0));
                consoleText += ", ymin=" + io::encoding::encodeToString(boundingBoxMinimum(1));
                consoleText += ", zmin=" + io::encoding::encodeToString(boundingBoxMinimum(2));
                consoleText += ", xmax=" + io::encoding::encodeToString(boundingBoxMaximum(0));
                consoleText += ", ymax=" + io::encoding::encodeToString(boundingBoxMaximum(1));
                consoleText += ", zmax=" + io::encoding::encodeToString(boundingBoxMaximum(2));
                consoleText += "]\n";

                // Task space
                const auto octreeCenter = m_output.taskspace().center();
                consoleText += "Octree center: [" + io::encoding::encodeToString(octreeCenter(0)) + ", " + io::encoding::encodeToString(octreeCenter(1)) + ", " + io::encoding::encodeToString(octreeCenter(2)) + "]\n";
                const auto octreeDimensions = m_output.taskspace().dimensions();
                consoleText += "Octree dimensions: [" + io::encoding::encodeToString(octreeDimensions(0)) + ", " + io::encoding::encodeToString(octreeDimensions(1)) + ", " + io::encoding::encodeToString(octreeDimensions(2)) + "]\n";
                for (unsigned int l = 0; l < L; l++) {
                    const auto levelSize = m_output.taskspace().size(l);
                    consoleText += "Size of octree-level " + io::encoding::encodeToString(l) + ": [" + io::encoding::encodeToString(levelSize[0]) + ", " + io::encoding::encodeToString(levelSize[1]) + ", " + io::encoding::encodeToString(levelSize[2]) + "] (voxel-count: " + io::encoding::encodeToString(m_output.taskspace().cellCount(l)) + ")\n";
                }

                io::Console::info().print(consoleText);
                io::Console::flush();
            }

            // Save status of evaluator
            if (saveCurrentStatus("PreProcess_End") == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Preprocessing failed! Saving status failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // Success
            return true;
        }

        //! Perform main processing steps
        /*! \return `true` on success, `false` otherwise */
        bool process()
        {
            if (input().m_consoleOutput) {
                io::Console::info().print("----------\n");
                io::Console::info().print("Processing\n");
                io::Console::info().print("----------\n");
                io::Console::flush();
            }

            // Save status of evaluator
            if (saveCurrentStatus("Process_Begin") == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Processing failed! Saving status failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // Start runtime measurement
            const core::Time startTime = core::Time::currentTime();

            // Trigger main sampler group
            TaskSpaceSamplerGroup<L, TaskSpaceMainSampler<L>> mainSamplerGroup(input(), "MainSampler_");
            mainSamplerGroup.trigger(m_output);

            // Extract bounding box paramenters
            const Eigen::Vector3d boundingBoxMinimum = m_output.boundingBoxMainSampling().minimum();
            const Eigen::Vector3d boundingBoxMaximum = m_output.boundingBoxMainSampling().maximum();

            // Trigger update of the octree (bottom up)
            m_output.taskspace().update();

            // End runtime measurement
            m_runTimeProcess = core::Time::currentTime() - startTime;
            if (input().m_consoleOutput) {
                io::Console::info().print("Processing finished (duration: " + m_runTimeProcess.encodeToDurationString() + "s)\n");
                io::Console::flush();
            }

            // Output statistics
            if (input().m_consoleOutput) {
                std::string consoleText;
                consoleText.reserve(4096);

                // Bounding box
                consoleText += "Computed bounding box (main-sampling): ";
                consoleText += "[xmin=" + io::encoding::encodeToString(boundingBoxMinimum(0));
                consoleText += ", ymin=" + io::encoding::encodeToString(boundingBoxMinimum(1));
                consoleText += ", zmin=" + io::encoding::encodeToString(boundingBoxMinimum(2));
                consoleText += ", xmax=" + io::encoding::encodeToString(boundingBoxMaximum(0));
                consoleText += ", ymax=" + io::encoding::encodeToString(boundingBoxMaximum(1));
                consoleText += ", zmax=" + io::encoding::encodeToString(boundingBoxMaximum(2));
                consoleText += "]\n";

                // Meta-cell
                const TaskSpaceCell metaCell = m_output.taskspace().metaCell();
                consoleText += "Meta-cell: total samples = " + io::encoding::encodeToString(metaCell.m_totalSamples) + "\n";
                consoleText += "Meta-cell: condition index = [min: " + io::encoding::encodeToString(metaCell.m_conditionIndexMinimum) + ", mean: " + io::encoding::encodeToString(metaCell.m_conditionIndexMean) + ", max: " + io::encoding::encodeToString(metaCell.m_conditionIndexMaximum) + "]\n";
                consoleText += "Meta-cell: manipulability measure = [min: " + io::encoding::encodeToString(metaCell.m_manipulabilityMeasureMinimum) + ", mean: " + io::encoding::encodeToString(metaCell.m_manipulabilityMeasureMean) + ", max: " + io::encoding::encodeToString(metaCell.m_manipulabilityMeasureMaximum) + "]\n";
                consoleText += "Meta-cell: joint range availability = [min: " + io::encoding::encodeToString(metaCell.m_jointRangeAvailabilityMinimum) + ", mean: " + io::encoding::encodeToString(metaCell.m_jointRangeAvailabilityMean) + ", max: " + io::encoding::encodeToString(metaCell.m_jointRangeAvailabilityMaximum) + "]\n";

                // Generic
                consoleText += "Dropped samples: " + io::encoding::encodeToString(m_output.taskspace().droppedSamples()) + "\n";

                io::Console::info().print(consoleText);
                io::Console::flush();
            }

            // Save status of evaluator
            if (saveCurrentStatus("Process_End") == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Processing failed! Saving status failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // Success
            return true;
        }

        //! Perform post-processing steps
        /*! \return `true` on success, `false` otherwise */
        bool postProcess()
        {
            // Initialize helpers
            std::string fileName = "";

            if (input().m_consoleOutput) {
                io::Console::info().print("--------------\n");
                io::Console::info().print("Postprocessing\n");
                io::Console::info().print("--------------\n");
                io::Console::flush();
            }

            // Save status of evaluator
            if (saveCurrentStatus("PostProcess_Begin") == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Postprocessing failed! Saving status failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // Start runtime measurement
            const core::Time startTime = core::Time::currentTime();

            // Write output files
            if (input().m_writeOutputFiles == true) {
                // Write bounding box files
                // ------------------------
                // Pre-sampling bounding box
                if (input().m_writeOutputFilesBoundingBoxPreSampling)
                    if (writeBoundingBoxMeshWithInfo(m_output.boundingBoxPreSampling(), input().m_outputFilesBoundingBoxPreSamplingWithColors, input().m_outputFilesBoundingBoxPreSamplingName) == false)
                        return false;

                // Main sampling bounding box
                if (input().m_writeOutputFilesBoundingBoxMainSampling)
                    if (writeBoundingBoxMeshWithInfo(m_output.boundingBoxMainSampling(), input().m_outputFilesBoundingBoxMainSamplingWithColors, input().m_outputFilesBoundingBoxMainSamplingName) == false)
                        return false;

                // Write octree files
                // ------------------
                if (input().m_writeOutputFilesOctree) {
                    // Raw-data: meta-cell
                    if (writeOctreeRawDataWithInfo(-1, input().m_outputFilesOctreePrefix + "_M") == false)
                        return false;

                    // Raw-data: levels
                    for (unsigned int l = 0; l < L; l++)
                        if (writeOctreeRawDataWithInfo(l, input().m_outputFilesOctreePrefix + "_L" + std::to_string(l)) == false)
                            return false;

                    // Meshes: according to list
                    for (size_t i = 0; i < input().m_outputFilesOctreeMeshList.size(); i++) {
                        const TaskSpaceMeshSpecification& specification = input().m_outputFilesOctreeMeshList[i];
                        fileName = input().m_outputFilesOctreePrefix + "_" + specification.m_name;
                        if (input().m_consoleOutput) {
                            io::Console::info().print("Writing output file (mesh '" + fileName + ".ply')...");
                            io::Console::flush();
                        }
                        if (writePlyMesh(m_output.taskspace().createMesh(specification), fileName) == false) {
                            if (input().m_consoleOutput) {
                                io::Console::info().print("failed!\n");
                                io::Console::error().print("Postprocessing failed!\n");
                                io::Console::flush();
                            }
                            return false;
                        }
                        if (input().m_consoleOutput) {
                            io::Console::info().print("done!\n");
                            io::Console::flush();
                        }
                    }
                }
            }

            // End runtime measurement
            m_runTimePostProcess = core::Time::currentTime() - startTime;
            if (input().m_consoleOutput) {
                io::Console::info().print("Postprocessing finished (duration: " + m_runTimePostProcess.encodeToDurationString() + "s)\n");
                io::Console::flush();
            }

            // Save status of evaluator
            if (saveCurrentStatus("PostProcess_End") == false) {
                if (input().m_consoleOutput) {
                    io::Console::error().print("Postprocessing failed! Saving status failed!\n");
                    io::Console::flush();
                }
                return false;
            }

            // Success
            return true;
        }

        // Saving and loading
        // ------------------
    public:
        //! Saves the **current status** of the evaluator class to the specified file (binary)
        /*!
         * \param [in] filePath The path to the file
         * \param [out] result Pointer to the result of the algorithm
         * \return `true` on success, `false` otherwise
         */
        bool saveToFile(const std::string& filePath, TaskSpaceEvaluatorResult::Type* const result = nullptr) const
        {
            // Initialize stream
            io::serialization::BinaryStream stream;

            // Serialize
            if (serialize(stream, io::serialization::Endianness::LITTLE) != stream.size()) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_SERIALIZATION;
                assert(false);
                return false;
            }

            // Open binary file
            std::ofstream outputStream(filePath, std::ofstream::binary);
            if (outputStream.is_open() == false) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_FILE_OPEN;
                assert(false);
                return false;
            }

            // Write binary file
            outputStream.write(reinterpret_cast<char*>(stream.data()), stream.size());
            outputStream.close();

            // Success
            if (result != nullptr)
                *result = TaskSpaceEvaluatorResult::Type::SUCCESS;
            return true;
        }

        //! Loads the **current status** of the evaluator class from the specified file (binary)
        /*!
         * \param [in] filePath The path to the file
         * \param [out] result Pointer to the result of the algorithm
         * \return `true` on success, `false` otherwise
         */
        bool loadFromFile(const std::string& filePath, TaskSpaceEvaluatorResult::Type* const result = nullptr)
        {
            // Abort, if file does not exist
            if (io::filesystem::fileExists(filePath) == false) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_FILE_NOEXIST;
                assert(false);
                return false;
            }

            // Open binary file
            std::ifstream inputStream(filePath, std::ofstream::binary);
            if (inputStream.is_open() == false) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_FILE_OPEN;
                assert(false);
                return false;
            }

            // Determine file size in bytes
            inputStream.seekg(0, inputStream.end); // Move to end of file
            const size_t fileSize = inputStream.tellg();
            inputStream.seekg(0);

            // Read data to stream
            io::serialization::BinaryStream stream(fileSize);
            inputStream.read(reinterpret_cast<char*>(stream.data()), stream.size());

            // Close file
            inputStream.close();

            // Deserialize
            if (deSerialize(stream, 0, io::serialization::Endianness::LITTLE) != stream.size()) {
                if (result != nullptr)
                    *result = TaskSpaceEvaluatorResult::Type::ERROR_DESERIALIZATION;
                assert(false);
                return false;
            }

            // Success
            if (result != nullptr)
                *result = TaskSpaceEvaluatorResult::Type::SUCCESS;
            return true;
        }

        // Getters
        // -------
    public:
        //! Input data container of the evaluator
        const TaskSpaceEvaluatorInput<L>& input() const { return m_input; }

        //! \copydoc input() const
        TaskSpaceEvaluatorInput<L>& input() { return m_input; }

        //! Output data container of the evaluator
        const TaskSpaceEvaluatorOutput<L>& output() const { return m_output; }

        //! Runtime of \ref preProcess()
        const core::Time& runTimePreProcess() const { return m_runTimePreProcess; }

        //! Runtime of \ref process()
        const core::Time& runTimeProcess() const { return m_runTimeProcess; }

        //! Runtime of \ref postProcess()
        const core::Time& runTimePostProcess() const { return m_runTimePostProcess; }

        // Serialization
        // -------------
    public:
        //! Compute serialized size of this object (in bytes)
        io::serialization::BinaryStreamSize computeBinaryStreamSize() const
        {
            io::serialization::BinaryStreamSize totalSize = sizeof(io::serialization::BinaryStreamSize); // Own header

            // Contribution of members
            totalSize += m_input.computeBinaryStreamSize(); // Contribution of m_input
            totalSize += m_output.computeBinaryStreamSize(); // Contribution of m_output
            totalSize += sizeof(m_runTimePreProcess); // Contribution of m_runTimePreProcess
            totalSize += sizeof(m_runTimeProcess); // Contribution of m_runTimeProcess
            totalSize += sizeof(m_runTimePostProcess); // Contribution of m_runTimePostProcess

            // Pass back total size of the stream
            return totalSize;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            stream.reserve(stream.size() + computeBinaryStreamSize());
            return io::serialization::serialize(stream, endianness, //
                m_input, m_output, m_runTimePreProcess, m_runTimeProcess, m_runTimePostProcess);
        }

        // Deserialization of payload (see base class for details)
        virtual io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            return io::serialization::deSerialize(stream, index, endianness, //
                m_input, m_output, m_runTimePreProcess, m_runTimeProcess, m_runTimePostProcess);
        }

        // Helpers
        // -------
    public:
        //! Writes the given mesh as PLY file to the given file path
        /*!
         * \param [in] mesh The mesh to be written
         * \param [in] fileName The file name of the mesh (**without** extension)
         * \return `true` on success, `false` otherwise
         */
        bool writePlyMesh(const geometry::CGMesh& mesh, const std::string& fileName) const
        {
            // Assemble file path
            std::string filePath = fileName + ".ply";
            if (input().m_outputFolder != "")
                filePath = input().m_outputFolder + "/" + filePath;

            // Convert to ply file
            io::PLYFile plyFile;
            if (io::PLYFileConverter::convert(mesh, plyFile) == false)
                return false;

            // Try to write ply file
            if (plyFile.writeFile(filePath, input().m_plyFileFormat) == false)
                return false;

            // Success
            return true;
        }

        //! Writes the given bounding box as PLY file to the given file path
        /*!
         * \param [in] boundingBox The bounding box to visualize
         * \param [in] withColors If `true` the bounding box mesh is written with colors (from face normal)
         * \param [in] fileName The file name of the mesh (**without** extension)
         * \return `true` on success, `false` otherwise
         */
        bool writeBoundingBoxMesh(const TaskSpaceBoundingBox& boundingBox, const bool& withColors, const std::string& fileName) const { return writePlyMesh(boundingBox.createMesh(withColors), fileName); }

        //! Writes the raw data of the given octree level to the given file path
        /*!
         * \param [in] level The level of the octree to output (use <0 for meta-cell)
         * \param [in] fileName The file name (**without** extension)
         * \return `true` on success, `false` otherwise
         */
        bool writeOctreeRawData(const int64_t& level, const std::string& fileName) const { return m_output.taskspace().writeOctreeLevelToFile(level, input().m_outputFolder, fileName, true); }

    protected:
        //! \copybrief writeBoundingBoxMesh() (with console output if enabled)
        /*! \copydetails writeBoundingBoxMesh() */
        bool writeBoundingBoxMeshWithInfo(const TaskSpaceBoundingBox& boundingBox, const bool& withColors, const std::string& fileName) const
        {
            if (input().m_consoleOutput) {
                io::Console::info().print("Writing output file (bounding box '" + fileName + ".ply')...");
                io::Console::flush();
            }
            if (writeBoundingBoxMesh(boundingBox, withColors, fileName) == false) {
                if (input().m_consoleOutput) {
                    io::Console::info().print("failed!\n");
                    io::Console::error().print("Postprocessing failed!\n");
                    io::Console::flush();
                }
                return false;
            } else {
                if (input().m_consoleOutput) {
                    io::Console::info().print("done!\n");
                    io::Console::flush();
                }
                return true;
            }
        }

        //! \copybrief writeOctreeRawData() (with console output if enabled)
        /*! \copydetails writeOctreeRawData() */
        bool writeOctreeRawDataWithInfo(const int64_t& level, const std::string& fileName) const
        {
            if (input().m_consoleOutput) {
                if (level < 0)
                    io::Console::info().print("Writing output file (raw meta-cell data '" + fileName + ".log.gz')...");
                else
                    io::Console::info().print("Writing output file (raw cell data of level " + io::encoding::encodeToString(level) + " '" + fileName + "')...");
                io::Console::flush();
            }
            if (writeOctreeRawData(level, fileName) == false) {
                if (input().m_consoleOutput) {
                    io::Console::info().print("failed!\n");
                    io::Console::error().print("Postprocessing failed!\n");
                    io::Console::flush();
                }
                return false;
            } else {
                if (input().m_consoleOutput) {
                    io::Console::info().print("done!\n");
                    io::Console::flush();
                }
                return true;
            }
        }

        //! Saves the current status of the evaluator instance to the output folder (only if enabled)
        /*!
         * \param [in] fileNameSuffix The filename suffix (after prefix)
         * \return `true` on success, `false` otherwise
         */
        bool saveCurrentStatus(const std::string& fileNameSuffix) const
        {
            // Check, if writing status files is enabled
            if (input().m_writeOutputFiles == true && input().m_writeOutputFilesEvaluator == true) {
                // Setup file path
                std::string filePath = input().m_outputFilesEvaluatorPrefix + "_" + fileNameSuffix + ".bin";
                if (input().m_outputFolder != "")
                    filePath = input().m_outputFolder + "/" + filePath;

                // Trigger saving
                if (input().m_consoleOutput) {
                    io::Console::info().print("Saving status of evaluator to binary file '" + filePath + "'...");
                    io::Console::flush();
                }
                TaskSpaceEvaluatorResult::Type result = TaskSpaceEvaluatorResult::Type::UNKNOWN;
                if (saveToFile(filePath, &result) == false) {
                    if (input().m_consoleOutput) {
                        io::Console::info().print("failed! (" + TaskSpaceEvaluatorResult::toString(result) + ")\n");
                        io::Console::flush();
                    }
                    return false;
                } else {
                    if (input().m_consoleOutput) {
                        io::Console::info().print("done!\n");
                        io::Console::flush();
                    }
                    return true;
                }
            } else {
                // Nothing to be done
                return true;
            }
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
