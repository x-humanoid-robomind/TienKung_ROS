/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/Time.hpp"
#include "../../io/console/Console.hpp"
#include "../../io/encoding.hpp"
#include "TaskSpaceEvaluatorInput.hpp"
#include "TaskSpaceEvaluatorOutput.hpp"
#include <Eigen/StdVector>
#include <math.h>

namespace broccoli {
namespace analysis {
    //! Representation of a group of parallel samplers
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * \tparam L Count of levels of the taskspace octree
     */
    template <unsigned int L, typename SamplerType>
    class TaskSpaceSamplerGroup {
    public:
        //! Default constructor
        /*!
         * \param [in] input Initializes \ref m_input - \copybrief m_input
         * \param [in] namePrefix Initializes \ref m_namePrefix - \copybrief m_namePrefix
         */
        TaskSpaceSamplerGroup(const TaskSpaceEvaluatorInput<L>& input, const std::string& namePrefix)
            : m_input(input)
            , m_namePrefix(namePrefix)
        {
        }

    protected:
        // Members
        // -------
        TaskSpaceEvaluatorInput<L> m_input; //!< \copybrief input()
        std::string m_namePrefix; //!< \copybrief namePrefix()

        // General
        // -------
    public:
        //! Triggers execution of sampler group
        /*! \param [in] output Reference to output data structure */
        void trigger(TaskSpaceEvaluatorOutput<L>& output)
        {
            // Compute total count of samples
            const uint64_t sampleCount = input().m_kinematicChain.totalSamples();

            // Limit count of samplers in group
            size_t samplerCount = input().m_maximumThreadCount;
            if (samplerCount > sampleCount)
                samplerCount = sampleCount; // There can't be more samplers than samples to compute

            // Create group of samplers
            std::vector<SamplerType, Eigen::aligned_allocator<SamplerType>> samplerGroup;
            samplerGroup.reserve(samplerCount);
            const uint64_t sampleBatchSize = floor((double)sampleCount / samplerCount); // Count of sample to cover by each sampler
            for (size_t i = 0; i < samplerCount; i++) {
                // Compute first and last sample of this batch
                const uint64_t firstSampleIndex = i * sampleBatchSize;
                uint64_t lastSampleIndex = firstSampleIndex + sampleBatchSize - 1;
                if (i == samplerCount - 1)
                    lastSampleIndex = sampleCount - 1;

                // Create sampler for this batch
                samplerGroup.push_back(SamplerType(namePrefix() + std::to_string(i), input(), firstSampleIndex, lastSampleIndex, output));
            }

            // Trigger samplers
            const core::Time samplingStartTime = core::Time::currentTime();
            for (size_t i = 0; i < samplerCount; i++) {
                if (input().m_consoleOutput == true) {
                    io::Console::info().print("Triggering sampler '" + samplerGroup[i].name() + "' (first sample: " + io::encoding::encodeToString(samplerGroup[i].firstSampleIndex()) + ", last sample: " + io::encoding::encodeToString(samplerGroup[i].lastSampleIndex()) + ")\n");
                    io::Console::flush();
                }
                samplerGroup[i].start();
            }

            // Wait for all samplers to finish their work
            size_t maximumLineCharacters = 0; // Remembers how many characters have been printed to the progress line in the console
            while (true) {
                // Evaluate status of samplers
                bool allSamplersFinished = true;
                uint64_t totalEvaluatedSamples = 0;
                for (size_t i = 0; i < samplerGroup.size(); i++) {
                    // Check, if this sampler finished his work
                    if (samplerGroup[i].finished() == false)
                        allSamplersFinished = false;

                    // Check progress of this sampler
                    totalEvaluatedSamples += samplerGroup[i].evaluatedSamples();
                }

                // Print progress to console
                if (input().m_consoleOutput == true) {
                    const double progress = ((double)totalEvaluatedSamples / (double)sampleCount) * 100.0; // Progress in percent
                    const core::Time currentRunTime = core::Time::currentTime() - samplingStartTime;
                    const double samplesPerSecond = (double)totalEvaluatedSamples / currentRunTime.toDouble();
                    std::string estimatedTimeOfArrival = "N/A";
                    if (samplesPerSecond > 0)
                        estimatedTimeOfArrival = (samplingStartTime + core::Time((double)sampleCount / samplesPerSecond)).encodeToDateTimeString(true);
                    std::string consoleText;
                    consoleText.reserve(4096);
                    consoleText += "Progress: " + io::encoding::encodeToString(progress, "%.3f") + "%";
                    consoleText += " (ETA: " + estimatedTimeOfArrival + ", samples per second: " + io::encoding::encodeToString(samplesPerSecond, "%g") + ")";
                    while (consoleText.size() < maximumLineCharacters)
                        consoleText += " ";
                    maximumLineCharacters = consoleText.size();
                    consoleText += "\r";
                    io::Console::plain().print(consoleText);
                    io::Console::flush();
                }

                // Check, if all samplers finished their work
                if (allSamplersFinished == true) {
                    // Join samplers and proceed
                    for (size_t i = 0; i < samplerGroup.size(); i++)
                        samplerGroup[i].join();
                    break;
                } else
                    core::Time::sleep(0.1);
            }

            // Stop runtime measurement
            const core::Time samplingStopTime = core::Time::currentTime();
            const core::Time samplingRunTime = samplingStopTime - samplingStartTime;

            // Clear console
            if (input().m_consoleOutput == true) {
                std::string consoleText;
                consoleText.reserve(maximumLineCharacters + 1);
                while (consoleText.size() < maximumLineCharacters)
                    consoleText += " ";
                consoleText += "\r";
                io::Console::plain().print(consoleText);
                io::Console::flush();
            }

            // Compute total runtime and output statistics
            if (input().m_consoleOutput == true) {
                const double samplesPerSecond = (double)sampleCount / samplingRunTime.toDouble();
                io::Console::info().print("All samplers finished! (" + io::encoding::encodeToString(samplesPerSecond) + " samples per second)\n");
                io::Console::flush();
            }
        }

        // Getters
        // -------
    public:
        //! Input data container
        const TaskSpaceEvaluatorInput<L>& input() const { return m_input; }

        //! Name prefix for sampler instances
        const std::string& namePrefix() const { return m_namePrefix; }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
