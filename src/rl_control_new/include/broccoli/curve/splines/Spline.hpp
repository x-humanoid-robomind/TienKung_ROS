/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../io/encoding.hpp"
#include <array>
#include <assert.h>
#include <cmath>
#include <stdint.h>
#include <vector>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_splines
     * \{
     */

    // Configuration
    const double minimumSplineSegmentProportion = 1e-6; //!< Minimum proportion of spline segments (used to avoid numeric errors)

    //! Specification of result types of spline algorithms
    enum class SplineResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_SEGMENT, //!< An **error** occured: Spline has at least one invalid segment
        ERROR_INVALID_SPLINE_NO_SEGMENTS, //!< An **error** occured: Spline has to contain at least one segment
        ERROR_INVALID_SPLINE_PROPORTION_COUNT, //!< An **error** occured: Proportion count does **not** match segment count
        ERROR_INVALID_SPLINE_PROPORTION_NEGATIVE, //!< An **error** occured: Negative proportion found
        ERROR_INVALID_SPLINE_PROPORTION_SUM, //!< An **error** occured: Sum of proportions does **not** match 1
        ERROR_INVALID_PARAMETERS, //!< An **error** occured: invalid input parameters
        ERROR_DEGREE_TOO_LOW, //!< An **error** occured: the degree of the underlying polynomial is too low
        ERROR_INVALID_SPLINES, //!< An **error** occured: the list of splines or at least one element of it is invalid
        ERROR_INVALID_CONTROL_POINTS, //!< An **error** occured: the list of control points or at least one element of it is invalid
        ERROR_CURVE_INTERPOLATION_FAILED, //!< An **error** occured: the interpolation of a curve segment failed
        ERROR_NOTIMPLEMENTED, //!< An **error** occured: the desired algorithm is **not** yet implemented
        WARNING_HIGH_NUMERICAL_ERROR, //!< A **warning** occured: a high numerical error is expected
        SPLINERESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type for spline algorithms
    static inline std::string splineResultString(const SplineResult& result)
    {
        // Check result
        switch (result) {
        case SplineResult::UNKNOWN:
            return "UNKNOWN";
        case SplineResult::SUCCESS:
            return "SUCCESS";
        case SplineResult::ERROR_INVALID_SEGMENT:
            return "ERROR_INVALID_SEGMENT";
        case SplineResult::ERROR_INVALID_SPLINE_NO_SEGMENTS:
            return "ERROR_INVALID_SPLINE_NO_SEGMENTS";
        case SplineResult::ERROR_INVALID_SPLINE_PROPORTION_COUNT:
            return "ERROR_INVALID_SPLINE_PROPORTION_COUNT";
        case SplineResult::ERROR_INVALID_SPLINE_PROPORTION_NEGATIVE:
            return "ERROR_INVALID_SPLINE_PROPORTION_NEGATIVE";
        case SplineResult::ERROR_INVALID_SPLINE_PROPORTION_SUM:
            return "ERROR_INVALID_SPLINE_PROPORTION_SUM";
        case SplineResult::ERROR_INVALID_PARAMETERS:
            return "ERROR_INVALID_PARAMETERS";
        case SplineResult::ERROR_DEGREE_TOO_LOW:
            return "ERROR_DEGREE_TOO_LOW";
        case SplineResult::ERROR_INVALID_SPLINES:
            return "ERROR_INVALID_SPLINE_LIST";
        case SplineResult::ERROR_INVALID_CONTROL_POINTS:
            return "ERROR_INVALID_CONTROL_POINTS";
        case SplineResult::ERROR_CURVE_INTERPOLATION_FAILED:
            return "ERROR_CURVE_INTERPOLATION_FAILED";
        case SplineResult::ERROR_NOTIMPLEMENTED:
            return "ERROR_NOTIMPLEMENTED";
        case SplineResult::WARNING_HIGH_NUMERICAL_ERROR:
            return "WARNING_HIGH_NUMERICAL_ERROR";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Template class for splines as concatenation of segments (curves)
    /*!
     * Contains an ordered list of spline segments (\ref m_segments). The segment/curve type has to be specified at compile time (\p CurveType). Each segment has an underlying curve
     * function, which is bounded by the *segment-position* \f$ \in [0,\,1] \f$. The complete spline is also bounded by *spline-position* \f$ \in [0,\,1] \f$ where in general
     * *spline-position* is **not** equal to the *segment-position*. Thus there has to be a mapping between *spline-position* and *segment-position* such that we can find out
     * which segment corresponds to which *spline-position*. This is done on a percentage basis (\ref m_segmentProportions). Considering the derivative of the underlying
     * function, the gradient relates to the *spline-coordinates*, **not** to the *segment-coordinates* (mapping is performed automatically).
     *
     * \tparam CurveType Type of curve elements in \ref m_segments
     *
     * \par Segmentation
     * \code
     * |0--------1|0--------1|0-------1|  <-- segment-coordinates
     *   Segment0 | Segment1 | Segment2
     *   Propor.0 | Propor.1 | Propor.2
     *
     * |0-----------------------------1|  <-- spline-coordinates
     *              Spline
     * \endcode
     */
    template <class CurveType>
    class Spline {
    public:
        //! Constructor
        Spline()
        {
        }

        //! Destructor
        virtual ~Spline()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const Spline<CurveType>& reference) const
        {
            // Compare segments
            if (m_segments != reference.m_segments)
                return false;

            // Compare segment proportions
            if (m_segmentProportions != reference.m_segmentProportions)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const Spline<CurveType>& reference) const { return !(*this == reference); }

        // Members
        std::vector<CurveType> m_segments; //!< Ordered list of spline segments (each segment has its own boundaries: *segment-position* \f$ \in [0,\,1] \f$)
        std::vector<double> m_segmentProportions; //!< Proportion of single segments relative to complete spline "duration" (each element has to be \f$ >=0 \f$ and the sum of all elements has to be exactly 1)

        //! Checks, if the spline is properly defined (proper definition of segments and proportions)
        /*!
         * \param [out] result Pointer to flag in which the result should be stored
         * \return `true`, if the spline is valid, `false` otherwise.
         */
        virtual bool isValid(SplineResult* const result = nullptr) const
        {
            // Check, if there is at least one segment
            if (m_segments.size() == 0) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_SPLINE_NO_SEGMENTS;
                return false;
            }

            // Check, if segments are valid
            for (size_t i = 0; i < m_segments.size(); i++) {
                if (m_segments[i].isValid() == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_INVALID_SEGMENT;
                    return false;
                }
            }

            // Check, if segment and proportion count are the same
            if (m_segments.size() != m_segmentProportions.size()) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_COUNT;
                return false;
            }

            // Check, proportion entries
            // (each entry has to be >=0 and the sum of all entries has to be 1.0)
            double sumOfProportions = 0;
            for (size_t i = 0; i < m_segmentProportions.size(); i++) {
                if (m_segmentProportions[i] < 0) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_NEGATIVE;
                    return false;
                }
                sumOfProportions += m_segmentProportions[i];
            }
            if (fabs(sumOfProportions - 1.0) > minimumSplineSegmentProportion) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_SPLINE_PROPORTION_SUM;
                return false;
            }

            // Otherwise -> valid
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

        //! Computes the segment boundaries (control point positions) represented in **spline coordinates** ([0, 1])
        /*!
         * \warning You may call \ref isValid() before to check, if the spline is valid! If the spline is invalid the behaviour of this method is not defined!
         *
         * \return Ordered list of segment boundaries, or empty list in case of an error.
         *
         * \code
         * |--------S--------| <-- spline
         * |---s1---|---s2---| <-- spline segments
         * B1       B2      B3 <-- segment boundaries
         * 0       0.x       1 <-- returned values
         * \endcode
         */
        virtual std::vector<double> getSegmentBoundaries() const
        {
            // Check validity
            assert(isValid());

            // Initialize helpers
            std::vector<double> boundaries;

            // Abort, if there are no segments
            if (m_segmentProportions.size() == 0)
                return boundaries;

            // Pre-allocate memory for speedup
            boundaries.reserve(m_segmentProportions.size() + 1);

            // Add "left" boundary of fist segment
            boundaries.push_back(0);

            // Get "right" boundary of all segments
            for (size_t i = 0; i < m_segmentProportions.size(); i++)
                boundaries.push_back(boundaries.back() + m_segmentProportions[i]);

            // Force last boundary position to match exactly 1 (to avoid numeric errors)
            boundaries.back() = 1;

            // Pass back list
            return boundaries;
        }

        //! Removes segments with zero proportion (i.e. no contribution to the spline)
        /*! \return Count of removed segments */
        virtual size_t removeEmptySegments()
        {
            // Abort, if there are no segments (or invalid dimensions)
            if (m_segments.size() == 0 || m_segments.size() != m_segmentProportions.size())
                return 0;

            // Iterate through segments
            size_t removedSegments = 0;
            size_t nextFreeSlot = 0; // Next "free" in "new" segment list
            for (size_t i = 0; i < m_segments.size(); i++) {
                // Check proportion
                if (m_segmentProportions[i] > 0) {
                    // Necessary segment -> copy to next free slot (only if not already there)
                    if (i > nextFreeSlot) {
                        m_segments[nextFreeSlot] = m_segments[i];
                        m_segmentProportions[nextFreeSlot] = m_segmentProportions[i];
                    }
                    nextFreeSlot++;
                } else {
                    // Unnecessary segment -> do not copy!
                    removedSegments++;
                }
            }

            // Remove garbage data at end of vectors
            if (removedSegments > 0) {
                m_segments.resize(m_segments.size() - removedSegments);
                m_segmentProportions.resize(m_segmentProportions.size() - removedSegments);
            }

            // Pass back count of removed elements
            return removedSegments;
        }

        //! Returns the index of the segment related to the given spline position
        /*!
         * \warning You may call \ref isValid() before to check, if the spline is valid! If the spline is invalid the behaviour of this method is not defined!
         *
         * \param [in] splinePosition Position in `spline-coordinates` (must be within \f$ [0,\,1] \f$, values are projected otherwise)
         * \param [out] segmentPosition Position within segment corresponding to the given spline position (optional additional output)
         * \return Index of the segment or -1 in case of an error
         */
        virtual int getSegmentIndex(const double& splinePosition, double* const segmentPosition = nullptr) const
        {
            // Check validity
            assert(isValid());

            // Project position to boundaries
            double projectedSplinePosition = splinePosition;
            if (projectedSplinePosition < 0)
                projectedSplinePosition = 0;
            if (projectedSplinePosition > 1)
                projectedSplinePosition = 1;

            // Search for segment index and segment position (optional)
            double startPosition = 0; // Start position of current segment in spline-coordinates
            double endPosition = 0; // End position of current segment in spline-coordinates
            for (size_t i = 0; i < m_segments.size(); i++) {
                // Compute end position
                endPosition = startPosition + m_segmentProportions[i];
                if (i == m_segments.size() - 1)
                    endPosition = 1.0; // Hard-setting to 1.0 in case of the last segment (to avoid numeric errors)

                // Check, if target position is between start and end
                if (projectedSplinePosition >= startPosition && projectedSplinePosition <= endPosition) {
                    // We found the corresponding segment!

                    // Optional: get segment position from coordinate transformation
                    if (segmentPosition != nullptr) {
                        // Compute coordinate transformation (spline-coordinates <-> segment-coordinates)
                        // Linear relationship: segment-position = alpha * spline-position + beta
                        // Derivation: d(segment-position) / d(spline-position) = alpha
                        double alpha = 0;
                        if (endPosition != startPosition)
                            alpha = 1.0 / (endPosition - startPosition);
                        double beta = -alpha * startPosition;

                        // Compute segment position from spline position
                        *segmentPosition = alpha * projectedSplinePosition + beta;
                    }

                    // Stop search and pass back result
                    return (int)i;
                } else {
                    // Otherwise: go to next segment
                    startPosition = endPosition; // Start position of next segment is end position of current segment
                }
            }

            // We could not find the corresponding segment -> error
            assert(false);
            return -1;
        }

        //! Evaluation of (derivative of) underlying function
        /*!
         * \warning You may call \ref isValid() before to check, if the spline is valid! If the spline is invalid the behaviour of this method is not defined!
         *
         * \warning The derivative is returned relative to the *spline-coordinates* (**not** relative to *segment-coordinates*)!
         *
         * \param [in] splinePosition Position to evaluate (derivative) at in `spline-coordinates` (must be within \f$ [0,\,1] \f$, values are projected otherwise)
         * \param [in] derivationOrder Order of the derivation (use 0 for base function, i.e. no derivation)
         * \return Value (of derivative) at specified position - gradient is expressed in *spline-coordinates*
         */
        virtual double evaluate(const double& splinePosition, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Get segment index and position within this segment
            double segmentPosition = 0;
            const int segmentIndex = getSegmentIndex(splinePosition, &segmentPosition);

            // Abort, if we did **not** find the corresponding segment
            if (segmentIndex < 0) {
                assert(false);
                return 0;
            }

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                // ... -> just evaluate underlying function of segment and return this value
                return m_segments[segmentIndex].evaluate(segmentPosition, derivationOrder);
            } else {
                // Compute coordinate transformation (spline-coordinates <-> segment-coordinates)
                // Linear relationship: segment-position = alpha * spline-position + beta
                // Derivation: d(segment-position) / d(spline-position) = alpha
                double alpha = 0;
                if (m_segmentProportions[segmentIndex] > 0)
                    alpha = 1.0 / m_segmentProportions[segmentIndex];

                // Compute gradient relative to segment coordinates
                const double segmentGradient = m_segments[segmentIndex].evaluate(segmentPosition, derivationOrder);

                // Compute gradient relative to spline coordinates
                double splineGradient = segmentGradient * alpha;
                for (unsigned int d = 2; d <= derivationOrder; d++)
                    splineGradient *= alpha;

                // Pass back transformed gradient
                return splineGradient;
            }
        }

        //! Evaluation of value (D0) up to N-th derivative (DN) of underlying function (improved performance over calling \ref evaluate() N+1 times)
        /*!
         * \warning You may call \ref isValid() before to check, if the spline is valid! If the spline is invalid the behaviour of this method is not defined!
         *
         * \warning The derivatives are returned relative to the *spline-coordinates* (**not** relative to *segment-coordinates*)!
         *
         * \tparam N Order of highest derivative
         * \param [in] splinePosition Position to evaluate (derivatives) at in `spline-coordinates` (must be within \f$ [0,\,1] \f$, values are projected otherwise)
         * \return List of value (D0) up to N-th derivative [D0, D1, ... , Di, ... , DN] at specified position - gradients are expressed in *spline-coordinates*
         */
        template <unsigned int N>
        std::array<double, N + 1> evaluateD0ToDN(const double& splinePosition) const
        {
            // Check validity
            assert(isValid());

            // Prepare return value
            std::array<double, N + 1> Di;
            Di.fill(0);

            // Get segment index and position within this segment
            double segmentPosition = 0;
            const int segmentIndex = getSegmentIndex(splinePosition, &segmentPosition);

            // Abort, if we did **not** find the corresponding segment
            if (segmentIndex < 0) {
                assert(false);
                return Di;
            }

            // Compute coordinate transformation (spline-coordinates <-> segment-coordinates)
            // Linear relationship: segment-position = alpha * spline-position + beta
            // Derivation: d(segment-position) / d(spline-position) = alpha
            double alpha = 0;
            if (m_segmentProportions[segmentIndex] > 0)
                alpha = 1.0 / m_segmentProportions[segmentIndex];

            // Compute value and derivatives
            Di = m_segments[segmentIndex].template evaluateD0ToDN<N>(segmentPosition);
            double alphaPowered = alpha;
            for (unsigned int d = 1; d <= N; d++) {
                Di[d] *= alphaPowered;
                alphaPowered *= alpha;
            }

            // Pass back result
            return Di;
        }

        // Encoding
        // --------
        //! Encode member data as XML element and add it to the specified stream
        /*!
         * \param [in,out] stream The stream to which the member data should be appended to.
         * \param [in] XMLIndentationLevel Level of indentation in XML format (used to add spaces)
         * \param [in] XMLTabWhiteSpaces Count of whitespaces corresponding to a tab.
         * \param [in] numericFormat C-format specifier for numeric values.
         * \return Count of appended characters (elements in stream).
         */
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const size_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<Spline");

            // Write attributes
            addedElements += io::encoding::encode(stream, " SegmentProportions=\"");
            addedElements += io::encoding::encode(stream, m_segmentProportions, numericFormat);
            addedElements += io::encoding::encode(stream, "\">\n");

            // Write segments
            for (size_t i = 0; i < m_segments.size(); i++)
                addedElements += m_segments[i].encodeToXML(stream, XMLIndentationLevel + 1, XMLTabWhiteSpaces, numericFormat);

            // End XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "</Spline>\n");

            // Pass back added elements in stream
            return addedElements;
        }
    };

    //! \}
} // namespace curve
} // namespace broccoli
