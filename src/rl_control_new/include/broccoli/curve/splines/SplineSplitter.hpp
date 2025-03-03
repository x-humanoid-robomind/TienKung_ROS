/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "Spline.hpp"

namespace broccoli {
namespace curve {
    //! Helper class which allows to split and join splines
    /*!
     * \ingroup broccoli_curve_splines
     *
     * \tparam SplineType Type of the spline to split/join.
     * \tparam CurveType Type of the spline segments.
     *
     * \attention The \p SplineType has to implement a method to compute a target segment which is equivalent to given source segment(s).
     * See \ref PolynomialSpline::computeTargetSegmentForSplitJoin() as a reference implementation.
     */
    template <class SplineType, class CurveType>
    class SplineSplitter {
    public:
        //! Constructor
        SplineSplitter()
        {
        }

        //! Destructor
        virtual ~SplineSplitter()
        {
        }

        //! Splits the given "source" spline into multiple "target" splines (of the same type). The interconnection ("chaining") of the target splines preserves the shape of the source spline.
        /*!
         * \param [in] sourceSpline Reference to source spline which should be splitted
         * \param [out] targetSplines List of pointers to target splines (list is constant, (values of) elements are modified)
         * \param [in] targetProportions Pointer to proportions of target splines. If `nullptr`, all target splines have the same proportion (uniform distribution, "split into equal parts"), otherwise uses given proportions.
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         *
         * \par Notes on Implementation
         * In order to avoid loss of information one has to be careful when computing the segmentation of the target splines. It is **not** sufficient to evaluate the source spline at the
         * boundaries of the target splines (and use one segment per target spline), since there might be control points of the source spline within the boundaries of the target spline. Thus
         * we have to use control points at the boundaries of the target splines **and** the boundaries of the source spline segments (which may split a target spline into further segments.
         * This way we do not "miss" any control points and the original shape of the source spline is preserved. Moreover, we evaluate the source spline at the "left" and "right" boundaries
         * of the control points, since the source spline might not be continuous (probably jumps at control points).
         *
         * \code
         * |------------------S-------------------| <-- source spline (known)
         * |-----s1-----|-----s2-----|-----s3-----| <-- source spline segments (known)
         * C1           C2           C4          C5 <-- control points originating from source spline segment boundaries
         *
         * C1                 C3                 C5 <-- control points originating from target spline boundaries
         * |--------T1--------|---------T2--------| <-- target splines (partitioning known through user defined target spline proportions (optional))
         * |-----t11----|-t12-|--t21-|----t22-----| <-- target spline segments (unknown - connect control points)
         * C1           C2    C3     C4          C5 <-- control points (define segmentation of target splines and target spline segments)
         * \endcode
         */
        static bool split(const SplineType& sourceSpline, const std::vector<SplineType*>& targetSplines, const std::vector<double>* const targetProportions, SplineResult* const result = nullptr)
        {
            // Definition of control points (only used internally)
            /*
             * Helper class to store as much information as possible during setup of segmentation -> evaluation of source spline is then faster
             *
             * ...left source segment] [right source segment...
             * -----------------------|------------------------
             *                        Ci
             *
             * Note: left and right source segment index are the same at control points which do not coincide with source spline segment boundaries ("inside" of a source spline segment)
             */
            class ControlPoint {
            public:
                // Default constructor
                ControlPoint()
                    : m_sourceSplinePosition(0)
                    , m_leftSourceSegmentIndex(0)
                    , m_rightSourceSegmentIndex(0)
                    , m_leftSourceSegmentPosition(0)
                    , m_rightSourceSegmentPosition(0)
                    , m_isSourceSegmentBoundary(false)
                {
                }

                // Specialized constructor
                ControlPoint(const double& sourceSplinePosition, const int& leftSourceSegmentIndex, const int& rightSourceSegmentIndex, const double& leftSourceSegmentPosition, const double& rightSourceSegmentPosition, const bool& isSourceSegmentBoundary)
                    : m_sourceSplinePosition(sourceSplinePosition)
                    , m_leftSourceSegmentIndex(leftSourceSegmentIndex)
                    , m_rightSourceSegmentIndex(rightSourceSegmentIndex)
                    , m_leftSourceSegmentPosition(leftSourceSegmentPosition)
                    , m_rightSourceSegmentPosition(rightSourceSegmentPosition)
                    , m_isSourceSegmentBoundary(isSourceSegmentBoundary)
                {
                }

                // Members
                double m_sourceSplinePosition; // Position of control point in source spline (described in source spline coordinates)
                int m_leftSourceSegmentIndex; // Index of source spline segment on "left" side of control point
                int m_rightSourceSegmentIndex; // Index of source spline segment on "right" side of control point
                double m_leftSourceSegmentPosition; // Position of control point in "left" source spline segment (described in left source spline segment coordinates)
                double m_rightSourceSegmentPosition; // Position of control point in "right" source spline segment (described in left source spline segment coordinates)
                bool m_isSourceSegmentBoundary; // Flag indicating, if control point originates from source spline segment boundary
            };

            // Initialize helpers
            std::vector<double> targetSplineBoundaryPositions; // Positions of target spline boundaries in source spline coordinates (start and end of each target spline when "cut" out of the source spline)
            std::vector<ControlPoint> controlPoints; // Total list of control points
            std::vector<size_t> targetSplineBeginControlPointIndices; // Index of control point at beginning of target spline "i"
            std::vector<size_t> targetSplineEndControlPointIndices; // Index of control point at end of target spline "i"

            // Pre-allocate memory for speedup
            targetSplineBoundaryPositions.reserve(targetSplines.size() + 1);
            targetSplineBeginControlPointIndices.reserve(targetSplines.size()); // One begin control point for each target spline
            targetSplineEndControlPointIndices.reserve(targetSplines.size()); // One end control point for each target spline

            // Assume success
            if (result != nullptr)
                *result = SplineResult::SUCCESS;

            // Check input for validity
            // ------------------------
            // Check, if source spline is valid
            if (sourceSpline.isValid(result) == false) {
                assert(false);
                return false;
            }

            // Check if list of target splines is empty
            if (targetSplines.size() == 0) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_SPLINES;
                assert(false);
                return false;
            }

            // Check if any pointer in list of target splines is invalid
            for (size_t i = 0; i < targetSplines.size(); i++) {
                // Check pointer
                if (targetSplines[i] == nullptr) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_INVALID_SPLINES;
                    assert(false);
                    return false;
                }
            }

            // Check, if target spline proportions are specified, otherwise use uniform proportions
            std::vector<double> targetSplineProportions;
            if (targetProportions != nullptr) {
                // ...yes, proportions are specified -> check, if dimension of lists match
                if (targetProportions->size() == targetSplines.size()) {
                    // Dimensions do match -> use specified proportions
                    targetSplineProportions = *targetProportions;

                    // Check, proportion entries (each entry has to be >=0 and the sum of all entries has to be 1.0)
                    double sumOfProportions = 0;
                    for (size_t i = 0; i < targetSplineProportions.size(); i++) {
                        if (targetSplineProportions[i] < 0) {
                            if (result != nullptr)
                                *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_NEGATIVE;
                            assert(false);
                            return false;
                        }
                        sumOfProportions += targetSplineProportions[i];
                    }
                    if (fabs(sumOfProportions - 1.0) > minimumSplineSegmentProportion) {
                        // Sum of proportions is not 1 -> error
                        if (result != nullptr)
                            *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_SUM;
                        assert(false);
                        return false;
                    }
                } else {
                    // Dimensions do not match -> error!
                    if (result != nullptr)
                        *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_COUNT;
                    assert(false);
                    return false;
                }
            } else {
                // ...no, proportions are not specified -> use uniform partitioning instead
                targetSplineProportions.resize(targetSplines.size(), (double)1.0 / targetSplines.size());
            }

            // Preprocessing
            // -------------
            // Create copy of source spline WITHOUT zero-proportion segments (these have no contribution to the target splines anyway and could lead to errors in computing derivatives)
            SplineType preprocessedSourceSpline;
            preprocessedSourceSpline.m_segments.reserve(sourceSpline.m_segments.size());
            preprocessedSourceSpline.m_segmentProportions.reserve(sourceSpline.m_segmentProportions.size());
            for (size_t i = 0; i < sourceSpline.m_segments.size(); i++) {
                if (sourceSpline.m_segmentProportions[i] > 0) {
                    preprocessedSourceSpline.m_segments.push_back(sourceSpline.m_segments[i]);
                    preprocessedSourceSpline.m_segmentProportions.push_back(sourceSpline.m_segmentProportions[i]);
                }
            }

            // Compute control points
            // ----------------------
            // Get positions of source segment boundaries in source spline coordinates
            std::vector<double> sourceSegmentBoundaryPositions = preprocessedSourceSpline.getSegmentBoundaries();

            // Get positions of target spline boundaries in source spline coordinates
            targetSplineBoundaryPositions.push_back(0); // Add "left" boundary of first target spline
            for (size_t i = 0; i < targetSplineProportions.size(); i++)
                targetSplineBoundaryPositions.push_back(targetSplineBoundaryPositions.back() + targetSplineProportions[i]);
            targetSplineBoundaryPositions.back() = 1; // Force last control point to match exactly 1 (to avoid numeric errors)

            // Compute a SORTED list of control points WITHOUT DUPLICATES from combining list of source segment boundaries and list of target spline boundaries
            controlPoints.reserve(sourceSegmentBoundaryPositions.size() + targetSplineBoundaryPositions.size()); // Allocate memory for best performance (over-estimate, since there might be duplicates)
            controlPoints.push_back(ControlPoint(0, 0, 0, 0, 0, true)); // Add initial control point ("left" boundary)
            size_t currentSourceSegmentBoundaryIndex = 1; // Currently investigated element in list of source segment boundaries (skip first element which is always zero and already in list)
            size_t currentTargetSplineBoundaryIndex = 1; // Currently investigated element in list of target spline boundaries (skip first element which is always zero and already in list)
            while (currentSourceSegmentBoundaryIndex < sourceSegmentBoundaryPositions.size() || currentTargetSplineBoundaryIndex < targetSplineBoundaryPositions.size()) {
                // Initialize helpers
                bool isSourceSegmentBoundary = false; // Flag indicating, if the control point originates from a source spline segment boundary (otherwise it originates from a target spline boundary)

                // Check, from which list we should pick the new control point
                if (currentSourceSegmentBoundaryIndex < sourceSegmentBoundaryPositions.size()) {
                    // List of source segment boundaries is non-empty -> check, if list of target spline boundaries is also non-empty
                    if (currentTargetSplineBoundaryIndex < targetSplineBoundaryPositions.size()) {
                        // Both lists are non-empty -> check, which list element is closer to zero (leftmost)
                        if (sourceSegmentBoundaryPositions[currentSourceSegmentBoundaryIndex] <= targetSplineBoundaryPositions[currentTargetSplineBoundaryIndex])
                            isSourceSegmentBoundary = true;
                    } else {
                        // List of target spline boundaries is empty -> use next source segment boundary as this is our only option
                        isSourceSegmentBoundary = true;
                    }
                }
                // Otherwise: list of source segment boundaries is empty -> use next target spline boundary as this is our only option

                // Fill data for control point
                if (isSourceSegmentBoundary == true) {
                    // Candidate originates from source segment boundary
                    ControlPoint newControlPoint = ControlPoint(sourceSegmentBoundaryPositions[currentSourceSegmentBoundaryIndex], currentSourceSegmentBoundaryIndex - 1, currentSourceSegmentBoundaryIndex, 1, 0, true);

                    // Project source segment information to boundaries of source spline
                    if (newControlPoint.m_leftSourceSegmentIndex < 0) {
                        // Project to beginning of first source segment
                        newControlPoint.m_leftSourceSegmentIndex = 0;
                        newControlPoint.m_leftSourceSegmentPosition = 0;
                    }
                    if (newControlPoint.m_rightSourceSegmentIndex >= (int)preprocessedSourceSpline.m_segments.size()) {
                        // Project to end of last source segment
                        newControlPoint.m_rightSourceSegmentIndex = preprocessedSourceSpline.m_segments.size() - 1;
                        newControlPoint.m_rightSourceSegmentPosition = 1;
                    }

                    // Add new control point to list (only if it differs from previous control point)
                    if (newControlPoint.m_sourceSplinePosition - controlPoints.back().m_sourceSplinePosition > minimumSplineSegmentProportion)
                        controlPoints.push_back(newControlPoint);
                    else {
                        // Control points coincide -> replace old one if old one originates from target spline boundary (to use better information on left/right segment)
                        if (controlPoints.back().m_isSourceSegmentBoundary == false)
                            controlPoints.back() = newControlPoint;
                    }

                    // "Remove" control point from list
                    currentSourceSegmentBoundaryIndex++;
                } else {
                    // Candidate originates from target spline boundary -> assume that this control point lies WITHIN a source spline segment and does NOT coincide with source segment boundary
                    // (this assumption might be wrong, but control point will be overwritten by source segment boundary control point in case we were wrong)
                    double sourceSegmentPosition = 0;
                    int sourceSegmentIndex = preprocessedSourceSpline.getSegmentIndex(targetSplineBoundaryPositions[currentTargetSplineBoundaryIndex], &sourceSegmentPosition);
                    if (sourceSegmentIndex < 0) {
                        if (result != nullptr)
                            *result = SplineResult::ERROR_INVALID_PARAMETERS;
                        assert(false);
                        return false;
                    }
                    ControlPoint newControlPoint = ControlPoint(targetSplineBoundaryPositions[currentTargetSplineBoundaryIndex], sourceSegmentIndex, sourceSegmentIndex, sourceSegmentPosition, sourceSegmentPosition, false);

                    // Add new control point to list (only if it differs from previous control point)
                    if (newControlPoint.m_sourceSplinePosition - controlPoints.back().m_sourceSplinePosition > minimumSplineSegmentProportion)
                        controlPoints.push_back(newControlPoint);
                    // Otherwise: control points coincide -> do NOT replace old one (to avoid overwriting control points originating from source segment boundaries)

                    // "Remove" control point from list
                    currentTargetSplineBoundaryIndex++;
                }

                // Check, if this control point is related to a target spline boundary
                if (isSourceSegmentBoundary == false) {
                    // Remember, that the current target spline ended at this control point
                    targetSplineEndControlPointIndices.push_back(controlPoints.size() - 1);
                }
            }

            // Force last control point to match end of spline (to avoid numeric errors)
            controlPoints.back() = ControlPoint(1, preprocessedSourceSpline.m_segments.size() - 1, preprocessedSourceSpline.m_segments.size() - 1, 1, 1, true);

            // Compute begin control point indices for target splines
            targetSplineBeginControlPointIndices.push_back(0); // Always start with zero
            for (size_t i = 0; i < targetSplineEndControlPointIndices.size() - 1; i++)
                targetSplineBeginControlPointIndices.push_back(targetSplineEndControlPointIndices[i]); // Begin control point is end control point of previous target spline

            // Check list of control points
            // ----------------------------
            // There have to be at least two control points...
            if (controlPoints.size() < 2) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_CONTROL_POINTS;
                assert(false);
                return false;
            }

            // Interpolate target splines
            // --------------------------
            // Iterate over target splines
            for (size_t i = 0; i < targetSplines.size(); i++) {
                // Compute segment count for current target spline
                size_t segmentCount = targetSplineEndControlPointIndices[i] - targetSplineBeginControlPointIndices[i];
                if (segmentCount == 0)
                    segmentCount = 1; // Force always at least one segment

                // Compute segmentation of target spline
                targetSplines[i]->m_segmentProportions.clear();
                targetSplines[i]->m_segmentProportions.reserve(segmentCount);
                if (segmentCount == 1)
                    targetSplines[i]->m_segmentProportions.push_back(1.0);
                else
                    for (size_t j = targetSplineBeginControlPointIndices[i]; j < targetSplineEndControlPointIndices[i]; j++)
                        targetSplines[i]->m_segmentProportions.push_back((controlPoints[j + 1].m_sourceSplinePosition - controlPoints[j].m_sourceSplinePosition) / targetSplineProportions[i]);

                // Reallocate segments
                targetSplines[i]->m_segments.clear();
                targetSplines[i]->m_segments.resize(segmentCount);

                // Iterate through segments of target spline
                size_t targetSegmentIndex = 0; // Index of currently investigated segment in target spline
                for (size_t j = targetSplineBeginControlPointIndices[i]; j < targetSplineEndControlPointIndices[i]; j++) {
                    // Get reference to target spline and target segment
                    SplineType& targetSpline = *targetSplines[i];
                    CurveType& targetSegment = targetSpline.m_segments[targetSegmentIndex];

                    // Get references to corresponding begin and end control point
                    size_t& beginControlPointIndex = j;
                    size_t endControlPointIndex = j + 1;
                    if (endControlPointIndex > targetSplineEndControlPointIndices[i])
                        endControlPointIndex = targetSplineEndControlPointIndices[i];
                    const ControlPoint& beginControlPoint = controlPoints[beginControlPointIndex];
                    const ControlPoint& endControlPoint = controlPoints[endControlPointIndex];

                    // Get references to corresponding begin and end source segment to evaluate at
                    const CurveType& beginSourceSegment = preprocessedSourceSpline.m_segments[beginControlPoint.m_rightSourceSegmentIndex];
                    const CurveType& endSourceSegment = preprocessedSourceSpline.m_segments[endControlPoint.m_leftSourceSegmentIndex];

                    // Get reference to corresponding begin and end source segment proportion
                    const double& beginSourceSegmentProportion = preprocessedSourceSpline.m_segmentProportions[beginControlPoint.m_rightSourceSegmentIndex];
                    const double& endSourceSegmentProportion = preprocessedSourceSpline.m_segmentProportions[endControlPoint.m_leftSourceSegmentIndex];

                    // Get reference to corresponding begin and end source segment position to evaluate at
                    const double& beginSourceSegmentPosition = beginControlPoint.m_rightSourceSegmentPosition;
                    const double& endSourceSegmentPosition = endControlPoint.m_leftSourceSegmentPosition;

                    // Get reference to duration of current source segments
                    // WARNING: "duration" relative to SOURCE spline coordinates (complete time domain)
                    const double& beginSourceSegmentDuration = beginSourceSegmentProportion;
                    const double& endSourceSegmentDuration = endSourceSegmentProportion;

                    // Get reference to duration of current target segment
                    // WARNING: "duration" relative to SOURCE spline coordinates (complete time domain)
                    const double& targetSegmentDuration = targetSpline.m_segmentProportions[targetSegmentIndex] * targetSplineProportions[i];

                    // Compute target segment (depends on curve type)
                    SplineResult targetSegmentResult = SplineResult::UNKNOWN;
                    bool targetSegmentSuccess = targetSpline.computeTargetSegmentForSplitJoin(beginSourceSegment, beginSourceSegmentDuration, beginSourceSegmentPosition, endSourceSegment, endSourceSegmentDuration, endSourceSegmentPosition, targetSegment, targetSegmentDuration, &targetSegmentResult);
                    if (targetSegmentSuccess == false || targetSegmentResult != SplineResult::SUCCESS) {
                        // Target segment could not be computed
                        if (result != nullptr)
                            *result = targetSegmentResult;
                        assert(false);
                        return false;
                    }

                    // Update target segment index
                    targetSegmentIndex++;
                }
            }

            // Success!
            return true;
        }

        //! Constructs a "target" spline by joining multiple "source" splines (of the same type). The interconnection ("chaining") of the source splines defines the shape of the target spline.
        /*!
         * \param [out] targetSpline Reference to target spline to be joined
         * \param [in] sourceSplines List of pointers to source splines (list is constant, elements are constant too)
         * \param [in] sourceProportions Pointer to proportions of source splines. If `nullptr`, all source splines have the same proportion (uniform distribution, "join as equal parts"), otherwise uses given proportions.
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         *
         * \par Notes on Implementation
         * \code
         * |-----------S1----------|-----------S2----------| <-- source splines (known)
         * |----s11----|----s12----|----s21----|----s22----| <-- source spline segments (known)
         * C1          C2          C3          C4         C5 <-- control points originating from source spline segment boundaries
         *
         * |-----------------------T-----------------------| <-- target spline (unknown)
         * |-----t1----|-----t2----|-----t3----|-----t4----| <-- target spline segments (unknown)
         * C1          C2          C3          C4         C5 <-- control points (define segmentation of target spline)
         * \endcode
         */
        static bool join(SplineType& targetSpline, const std::vector<SplineType const*>& sourceSplines, const std::vector<double>* const sourceProportions, SplineResult* const result = nullptr)
        {
            // Definition of control points (only used internally)
            /*
             * Helper class to store as much information as possible during setup of segmentation -> evaluation of source splines is then faster
             *
             * ... left source spline} {right source spline ...
             * ...left source segment] [right source segment...
             *  ----------------------|------------------------
             *                        Ci
             */
            class ControlPoint {
            public:
                // Default constructor
                ControlPoint()
                    : m_targetSplinePosition(0)
                    , m_leftSourceSplineIndex(0)
                    , m_rightSourceSplineIndex(0)
                    , m_leftSourceSegmentIndex(0)
                    , m_rightSourceSegmentIndex(0)
                    , m_leftSourceSegmentPosition(0)
                    , m_rightSourceSegmentPosition(0)
                {
                }

                // Specialized constructor
                ControlPoint(const double& targetSplinePosition, const int& leftSourceSplineIndex, const int& rightSourceSplineIndex, const int& leftSourceSegmentIndex, const int& rightSourceSegmentIndex, const double& leftSourceSegmentPosition, const double& rightSourceSegmentPosition)
                    : m_targetSplinePosition(targetSplinePosition)
                    , m_leftSourceSplineIndex(leftSourceSplineIndex)
                    , m_rightSourceSplineIndex(rightSourceSplineIndex)
                    , m_leftSourceSegmentIndex(leftSourceSegmentIndex)
                    , m_rightSourceSegmentIndex(rightSourceSegmentIndex)
                    , m_leftSourceSegmentPosition(leftSourceSegmentPosition)
                    , m_rightSourceSegmentPosition(rightSourceSegmentPosition)
                {
                }

                // Members
                double m_targetSplinePosition; // Position of control point in target spline (described in target spline coordinates)
                int m_leftSourceSplineIndex; // Index of source spline on "left" side of control point
                int m_rightSourceSplineIndex; // Index of source spline on "right" side of control point
                int m_leftSourceSegmentIndex; // Index of source spline segment on "left" side of control point
                int m_rightSourceSegmentIndex; // Index of source spline segment on "right" side of control point
                double m_leftSourceSegmentPosition; // Position of control point in "left" source spline segment (described in left source spline segment coordinates)
                double m_rightSourceSegmentPosition; // Position of control point in "right" source spline segment (described in left source spline segment coordinates)
            };

            // Initialize helpers
            std::vector<ControlPoint> controlPoints; // Total list of control points

            // Assume success
            if (result != nullptr)
                *result = SplineResult::SUCCESS;

            // Check input for validity
            // ------------------------
            // Check if list of source splines is empty
            if (sourceSplines.size() == 0) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_SPLINES;
                assert(false);
                return false;
            }

            // Check if any pointer in list of source splines is invalid, or if source splines are invalid themselves
            for (size_t i = 0; i < sourceSplines.size(); i++) {
                // Check pointer
                if (sourceSplines[i] == nullptr) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_INVALID_SPLINES;
                    assert(false);
                    return false;
                } else {
                    // Check spline validity
                    if (sourceSplines[i]->isValid() == false) {
                        if (result != nullptr)
                            *result = SplineResult::ERROR_INVALID_SPLINES;
                        assert(false);
                        return false;
                    }
                }
            }

            // Check, if source spline proportions are specified, otherwise use uniform proportions
            std::vector<double> sourceSplineProportions;
            if (sourceProportions != nullptr) {
                // ...yes, proportions are specified -> check, if dimension of lists match
                if (sourceProportions->size() == sourceSplines.size()) {
                    // Dimensions do match -> use specified proportions
                    sourceSplineProportions = *sourceProportions;

                    // Check, proportion entries (each entry has to be >=0 and the sum of all entries has to be 1.0)
                    double sumOfProportions = 0;
                    for (size_t i = 0; i < sourceSplineProportions.size(); i++) {
                        if (sourceSplineProportions[i] < 0) {
                            if (result != nullptr)
                                *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_NEGATIVE;
                            assert(false);
                            return false;
                        }
                        sumOfProportions += sourceSplineProportions[i];
                    }
                    if (fabs(sumOfProportions - 1.0) > minimumSplineSegmentProportion) {
                        // Sum of proportions is not 1 -> error
                        if (result != nullptr)
                            *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_SUM;
                        assert(false);
                        return false;
                    }
                } else {
                    // Dimensions do not match -> error!
                    if (result != nullptr)
                        *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_COUNT;
                    assert(false);
                    return false;
                }
            } else {
                // ...no, proportions are not specified -> use uniform partitioning instead
                sourceSplineProportions.resize(sourceSplines.size(), (double)1.0 / sourceSplines.size());
            }

            // Preprocessing
            // -------------
            // Create copy of source spline list WITHOUT zero-proportion splines and zero-proportion segments (these have no contribution to the target spline anyway and could lead to errors in computing derivatives)
            std::vector<SplineType> preprocessedSourceSplines;
            std::vector<double> preprocessedSourceSplineProportions;
            preprocessedSourceSplines.reserve(sourceSplines.size());
            preprocessedSourceSplineProportions.reserve(sourceSplineProportions.size());
            for (size_t i = 0; i < sourceSplines.size(); i++) {
                // Check if spline proportion is non-zero
                if (sourceSplineProportions[i] > 0) {
                    // ...yes -> add this spline to list
                    preprocessedSourceSplines.push_back(SplineType());
                    preprocessedSourceSplineProportions.push_back(sourceSplineProportions[i]);
                    preprocessedSourceSplines.back().m_segments.reserve(sourceSplines[i]->m_segments.size());
                    preprocessedSourceSplines.back().m_segmentProportions.reserve(sourceSplines[i]->m_segmentProportions.size());

                    // Iterate through segments
                    for (size_t j = 0; j < sourceSplines[i]->m_segments.size(); j++) {
                        // Check if segment proportion is non-zero
                        if (sourceSplines[i]->m_segmentProportions[j] > 0) {
                            // ...yes -> add this segment to list
                            preprocessedSourceSplines.back().m_segments.push_back(sourceSplines[i]->m_segments[j]);
                            preprocessedSourceSplines.back().m_segmentProportions.push_back(sourceSplines[i]->m_segmentProportions[j]);
                        }
                    }
                }
            }

            // Compute control points
            // ----------------------
            // Get estimate for count of control points (over-estimate)
            size_t estimatedControlPointCount = 1;
            for (size_t i = 0; i < preprocessedSourceSplines.size(); i++)
                estimatedControlPointCount += preprocessedSourceSplines[i].m_segments.size();

            // Pre-allocate memory for best performance
            controlPoints.reserve(estimatedControlPointCount);

            // Add initial control point
            controlPoints.push_back(ControlPoint(0, 0, 0, 0, 0, 0, 0));

            // Create SORTED list of control points WITHOUT DUPLICATES
            double targetSplinePosition = 0;
            for (size_t i = 0; i < preprocessedSourceSplines.size(); i++) {
                // Add control point for every source segment boundary
                for (size_t j = 0; j < preprocessedSourceSplines[i].m_segmentProportions.size(); j++) {
                    // Update target spline position
                    targetSplinePosition += preprocessedSourceSplines[i].m_segmentProportions[j] * preprocessedSourceSplineProportions[i];

                    // Add new control point only, if there does not exist an control point at this position already
                    if (targetSplinePosition - controlPoints.back().m_targetSplinePosition > minimumSplineSegmentProportion) {
                        // Check, if this is the last segment in this spline
                        if (j < preprocessedSourceSplines[i].m_segmentProportions.size() - 1) {
                            // ...no, not last segment in current spline
                            controlPoints.push_back(ControlPoint(targetSplinePosition, i, i, j, j + 1, 1, 0));
                        } else {
                            // ...yes, last segment in current spline
                            controlPoints.push_back(ControlPoint(targetSplinePosition, i, i + 1, j, 0, 1, 0));
                        }
                    }
                }
            }

            // Force last control point to match end of spline (to avoid numeric errors)
            controlPoints.back() = ControlPoint(1, preprocessedSourceSplines.size() - 1, preprocessedSourceSplines.size() - 1, preprocessedSourceSplines.back().m_segments.size() - 1, preprocessedSourceSplines.back().m_segments.size() - 1, 1, 1);

            // Check list of control points
            // ----------------------------
            // There have to be at least two control points...
            if (controlPoints.size() < 2) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_CONTROL_POINTS;
                assert(false);
                return false;
            }

            // Interpolate target spline
            // -------------------------
            // Compute segmentation of target spline
            const size_t targetSegmentCount = controlPoints.size() - 1;
            targetSpline.m_segmentProportions.clear();
            targetSpline.m_segmentProportions.reserve(targetSegmentCount);
            for (size_t i = 0; i < targetSegmentCount; i++)
                targetSpline.m_segmentProportions.push_back(controlPoints[i + 1].m_targetSplinePosition - controlPoints[i].m_targetSplinePosition);

            // Reallocate segments
            targetSpline.m_segments.clear();
            targetSpline.m_segments.resize(targetSegmentCount);

            // Iterate through target spline segments
            for (size_t i = 0; i < targetSpline.m_segments.size(); i++) {
                // Get reference to target spline and target segment
                CurveType& targetSegment = targetSpline.m_segments[i];

                // Get references to corresponding begin and end control point
                const ControlPoint& beginControlPoint = controlPoints[i];
                const ControlPoint& endControlPoint = controlPoints[i + 1];

                // Get references to corresponding begin and end source spline to evaluate at
                const SplineType& beginSourceSpline = preprocessedSourceSplines[beginControlPoint.m_rightSourceSplineIndex];
                const SplineType& endSourceSpline = preprocessedSourceSplines[endControlPoint.m_leftSourceSplineIndex];

                // Get references to corresponding begin and end source segment to evaluate at
                const CurveType& beginSourceSegment = beginSourceSpline.m_segments[beginControlPoint.m_rightSourceSegmentIndex];
                const CurveType& endSourceSegment = endSourceSpline.m_segments[endControlPoint.m_leftSourceSegmentIndex];

                // Get reference to corresponding begin and end source segment proportion
                const double& beginSourceSegmentProportion = beginSourceSpline.m_segmentProportions[beginControlPoint.m_rightSourceSegmentIndex];
                const double& endSourceSegmentProportion = endSourceSpline.m_segmentProportions[endControlPoint.m_leftSourceSegmentIndex];

                // Get reference to corresponding begin and end source segment position to evaluate at
                const double& beginSourceSegmentPosition = beginControlPoint.m_rightSourceSegmentPosition;
                const double& endSourceSegmentPosition = endControlPoint.m_leftSourceSegmentPosition;

                // Get reference to duration of current source segments
                // WARNING: "duration" relative to TARGET spline coordinates (complete time domain)
                const double& beginSourceSegmentDuration = beginSourceSegmentProportion * preprocessedSourceSplineProportions[beginControlPoint.m_rightSourceSplineIndex];
                const double& endSourceSegmentDuration = endSourceSegmentProportion * preprocessedSourceSplineProportions[endControlPoint.m_leftSourceSplineIndex];

                // Get reference to duration of current target segment
                // WARNING: "duration" relative to TARGET spline coordinates (complete time domain)
                const double& targetSegmentDuration = targetSpline.m_segmentProportions[i];

                // Compute target segment (depends on curve type)
                SplineResult targetSegmentResult = SplineResult::UNKNOWN;
                bool targetSegmentSuccess = targetSpline.computeTargetSegmentForSplitJoin(beginSourceSegment, beginSourceSegmentDuration, beginSourceSegmentPosition, endSourceSegment, endSourceSegmentDuration, endSourceSegmentPosition, targetSegment, targetSegmentDuration, &targetSegmentResult);
                if (targetSegmentSuccess == false || targetSegmentResult != SplineResult::SUCCESS) {
                    // Target segment could not be computed
                    if (result != nullptr)
                        *result = targetSegmentResult;
                    assert(false);
                    return false;
                }
            }

            // Success!
            return true;
        }
    };
} // namespace curve
} // namespace broccoli
