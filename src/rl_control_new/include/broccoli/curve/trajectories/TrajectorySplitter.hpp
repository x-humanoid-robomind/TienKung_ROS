/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../splines/SplineSplitter.hpp"
#include "Trajectory.hpp"

namespace broccoli {
namespace curve {
    //! Helper class which allows to split and join trajectories
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * \tparam TrajectoryType Type of the trajectory to split/join.
     * \tparam SplineType Type of the underlying splines of the trajectory.
     * \tparam CurveType Type of the spline segments.
     *
     * \attention The \p SplineType has to implement a method to compute a target segment which is equivalent to given source segment(s).
     * See \ref PolynomialSpline::computeTargetSegmentForSplitJoin() as a reference implementation.
     */
    template <class TrajectoryType, class SplineType, class CurveType>
    class TrajectorySplitter {
    public:
        //! Constructor
        TrajectorySplitter()
        {
        }

        //! Destructor
        virtual ~TrajectorySplitter()
        {
        }

        //! Splits the given "source" trajectory into multiple "target" trajectories (of the same type). The interconnection ("chaining") of the target trajectories preserves the shape of the source trajectory.
        /*!
         * \param [in] sourceTrajectory Reference to source trajectory which should be splitted
         * \param [in,out] targetTrajectories List of pointers to target trajectories (list is constant, (values of) elements are modified)
         * \param [in] adaptTargetTrajectoryDurations If `true`, the durations of the target trajectories are scaled equally to fit the duration of the source trajectory.
         * \param [out] trajectoryResult Pointer to flag in which the result concerning the trajectory should be stored (optional, use `nullptr` to skip output)
         * \param [out] splineResult Pointer to flag in which the result concerning the underlying splines should be stored (optional, use `nullptr` to skip output)
         * \return `true` on success, `false` otherwise
         *
         * \attention The durations of the target trajectories specify the segmentation of the source trajectory.
         *
         * \par Notes on Implementation
         * \code
         * |------------------S-------------------| <-- source trajectory (known)
         * |---s1---|---s2---|--------s3----------| <-- source spline segments (known)
         * |------------T1---------|------T2------| <-- target trajectories (partitioning known through durations)
         * |---t11--|---t12--|-t13-|------t21-----| <-- target spline segments (unknown)
         * \endcode
         */
        static bool split(const TrajectoryType& sourceTrajectory, const std::vector<TrajectoryType*>& targetTrajectories, const bool& adaptTargetTrajectoryDurations, TrajectoryResult* const trajectoryResult, SplineResult* const splineResult)
        {
            // Assume success
            if (trajectoryResult != nullptr)
                *trajectoryResult = TrajectoryResult::SUCCESS;
            if (splineResult != nullptr)
                *splineResult = SplineResult::SUCCESS;

            // Check input for validity
            // ------------------------
            // Check, if source trajectory duration is valid (spline is checked in split-function of spline)
            if (sourceTrajectory.m_duration < 0) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE;
                assert(false);
                return false;
            }

            // Check if list of target trajectories is empty
            if (targetTrajectories.size() == 0) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORIES;
                assert(false);
                return false;
            }

            // Check if any pointer in list of target trajectories is invalid, or if duration of target trajectories is invalid
            double targetTrajectoryDurationsSum = 0;
            for (size_t i = 0; i < targetTrajectories.size(); i++) {
                // Check pointer
                if (targetTrajectories[i] == nullptr) {
                    if (trajectoryResult != nullptr)
                        *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORIES;
                    assert(false);
                    return false;
                } else {
                    // ...pointer is valid -> check duration
                    if (targetTrajectories[i]->m_duration < 0) {
                        if (trajectoryResult != nullptr)
                            *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE;
                        assert(false);
                        return false;
                    } else
                        targetTrajectoryDurationsSum += targetTrajectories[i]->m_duration;
                }
            }

            // Check, if trajectory durations match
            bool trajectoryDurationsDiffer = false;
            if (sourceTrajectory.m_duration == 0) {
                if (targetTrajectoryDurationsSum > minimumSplineSegmentProportion /* <-- allow some tolerance to avoid numeric errors */)
                    trajectoryDurationsDiffer = true;
            } else {
                if (fabs(targetTrajectoryDurationsSum - sourceTrajectory.m_duration) / sourceTrajectory.m_duration > minimumSplineSegmentProportion /* <-- allow some tolerance to avoid numeric errors */)
                    trajectoryDurationsDiffer = true;
            }
            if (trajectoryDurationsDiffer == true && trajectoryResult != nullptr)
                *trajectoryResult = TrajectoryResult::WARNING_DURATION_MISMATCH;

            // Compute target trajectory durations
            // -----------------------------------
            // Scale target trajectory durations (if required)
            if (adaptTargetTrajectoryDurations == true && targetTrajectoryDurationsSum > 0) {
                double scalingFactor = sourceTrajectory.m_duration / targetTrajectoryDurationsSum;
                for (size_t i = 0; i < targetTrajectories.size(); i++)
                    targetTrajectories[i]->m_duration *= scalingFactor;
                targetTrajectoryDurationsSum = sourceTrajectory.m_duration;
            }

            // Split source spline into target splines
            // ---------------------------------------
            // Compute target spline proportions
            std::vector<double> targetSplineProportions;
            if (targetTrajectoryDurationsSum > 0) {
                targetSplineProportions.reserve(targetTrajectories.size());
                for (size_t i = 0; i < targetTrajectories.size(); i++)
                    targetSplineProportions.push_back(targetTrajectories[i]->m_duration / targetTrajectoryDurationsSum);
            } else {
                // ...all target trajectories have zero duration -> use uniform partitioning
                targetSplineProportions.resize(targetTrajectories.size(), (double)1.0 / targetTrajectories.size());
            }

            // Split splines for every dimension
            for (size_t i = 0; i < sourceTrajectory.dimension(); i++) {
                // Setup list of target splines
                std::vector<SplineType*> targetSplines;
                targetSplines.reserve(targetTrajectories.size());
                for (size_t j = 0; j < targetTrajectories.size(); j++)
                    targetSplines.push_back(&(targetTrajectories[j]->m_splines[i]));

                // Split source spline
                if (SplineSplitter<SplineType, CurveType>::split(sourceTrajectory.m_splines[i], targetSplines, &targetSplineProportions, splineResult) == false) {
                    if (trajectoryResult != nullptr)
                        *trajectoryResult = TrajectoryResult::ERROR_SPLINEERROR;
                    assert(false);
                    return false;
                }
            }

            // Success
            return true;
        }

        //! Constructs a "target" trajectory by joining multiple "source" trajectories (of the same type). The interconnection ("chaining") of the source trajectories defines the shape of the target trajectory.
        /*!
         * \param [out] targetTrajectory Reference to target trajectory to be joined
         * \param [in] sourceTrajectories List of pointers to source trajectories (list is constant, elements are constant too)
         * \param [in] adaptTargetTrajectoryDuration If `true`, the duration of the target trajectory is manually set to the sum of durations of the source trajectories.
         * \param [out] trajectoryResult Pointer to flag in which the result concerning the trajectory should be stored (optional, use `nullptr` to skip output)
         * \param [out] splineResult Pointer to flag in which the result concerning the underlying splines should be stored (optional, use `nullptr` to skip output)
         * \return `true` on success, `false` otherwise
         *
         * \par Notes on Implementation
         * \code
         * |-----------S1----------------|--------S2-------| <-- source trajectories (known)
         * |----------s11----------|-s12-|--s21--|---s22---| <-- source spline segments (known)
         * |-----------------------T-----------------------| <-- target trajectory (unknown)
         * |-----------t1----------|--t2-|---t3--|----t4---| <-- target spline segments (unknown)
         * \endcode
         */
        static bool join(TrajectoryType& targetTrajectory, const std::vector<TrajectoryType const*>& sourceTrajectories, const bool& adaptTargetTrajectoryDuration, TrajectoryResult* const trajectoryResult, SplineResult* const splineResult)
        {
            // Reset outputs
            if (trajectoryResult != nullptr)
                *trajectoryResult = TrajectoryResult::SUCCESS;
            if (splineResult != nullptr)
                *splineResult = SplineResult::SUCCESS;

            // Check input for validity
            // ------------------------
            // Check, if target trajectory duration is valid
            if (targetTrajectory.m_duration < 0) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE;
                assert(false);
                return false;
            }

            // Check if list of source trajectories is empty
            if (sourceTrajectories.size() == 0) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORIES;
                assert(false);
                return false;
            }

            // Check if any pointer in list of source trajectories is invalid, or if duration of source trajectories is invalid
            double sourceTrajectoryDurationsSum = 0;
            for (size_t i = 0; i < sourceTrajectories.size(); i++) {
                // Check pointer
                if (sourceTrajectories[i] == nullptr) {
                    if (trajectoryResult != nullptr)
                        *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORIES;
                    assert(false);
                    return false;
                } else {
                    // ...pointer is valid -> check duration
                    if (sourceTrajectories[i]->m_duration < 0) {
                        if (trajectoryResult != nullptr)
                            *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE;
                        assert(false);
                        return false;
                    } else
                        sourceTrajectoryDurationsSum += sourceTrajectories[i]->m_duration;
                }
            }

            // Check, if trajectory durations match
            bool trajectoryDurationsDiffer = false;
            if (sourceTrajectoryDurationsSum == 0) {
                if (targetTrajectory.m_duration > minimumSplineSegmentProportion /* <-- allow some tolerance to avoid numeric errors */)
                    trajectoryDurationsDiffer = true;
            } else {
                if (fabs(targetTrajectory.m_duration - sourceTrajectoryDurationsSum) / sourceTrajectoryDurationsSum > minimumSplineSegmentProportion /* <-- allow some tolerance to avoid numeric errors */)
                    trajectoryDurationsDiffer = true;
            }
            if (trajectoryDurationsDiffer == true && trajectoryResult != nullptr)
                *trajectoryResult = TrajectoryResult::WARNING_DURATION_MISMATCH;

            // Compute target trajectory duration
            // ----------------------------------
            // Set target trajectory duration (if required)
            if (adaptTargetTrajectoryDuration == true)
                targetTrajectory.m_duration = sourceTrajectoryDurationsSum;

            // Join source splines into target spline
            // --------------------------------------
            // Compute source spline proportions
            std::vector<double> sourceSplineProportions;
            if (sourceTrajectoryDurationsSum > 0) {
                sourceSplineProportions.reserve(sourceTrajectories.size());
                for (size_t i = 0; i < sourceTrajectories.size(); i++)
                    sourceSplineProportions.push_back(sourceTrajectories[i]->m_duration / sourceTrajectoryDurationsSum);
            } else {
                // ...all source trajectories have zero duration -> use uniform partitioning
                sourceSplineProportions.resize(sourceTrajectories.size(), (double)1.0 / sourceTrajectories.size());
            }

            // Join splines for every dimension
            for (size_t i = 0; i < targetTrajectory.dimension(); i++) {
                // Setup list of source splines
                std::vector<SplineType const*> sourceSplines;
                sourceSplines.reserve(sourceTrajectories.size());
                for (size_t j = 0; j < sourceTrajectories.size(); j++)
                    sourceSplines.push_back(&(sourceTrajectories[j]->m_splines[i]));

                // Join source splines
                if (SplineSplitter<SplineType, CurveType>::join(targetTrajectory.m_splines[i], sourceSplines, &sourceSplineProportions, splineResult) == false) {
                    if (trajectoryResult != nullptr)
                        *trajectoryResult = TrajectoryResult::ERROR_SPLINEERROR;
                    assert(false);
                    return false;
                }
            }

            // Success
            return true;
        }
    };
} // namespace curve
} // namespace broccoli
