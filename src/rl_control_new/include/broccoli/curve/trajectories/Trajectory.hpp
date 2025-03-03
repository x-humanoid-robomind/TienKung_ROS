/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../splines/Spline.hpp"
#include <array>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_trajectories
     * \{
     */

    //! Specification of result types of trajectory algorithms
    enum class TrajectoryResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        WARNING_DURATION_MISMATCH, //!< A **warning** occured: The durations of trajectories do not match
        ERROR_INVALID_TRAJECTORY_SPLINES, //!< An **error** occured: Underlying spline(s) is/are invalid
        ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE, //!< An **error** occured: Duration is negative
        ERROR_INVALID_TRAJECTORIES, //!< An **error** occured: the list of trajectories or at least one element of it is invalid
        ERROR_SPLINEERROR, //!< An **error** occured: an algorithm related to the underlying spline(s) failed
        ERROR_NOTIMPLEMENTED, //!< An **error** occured: the desired algorithm is **not** yet implemented
        TRAJECTORYRESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type for trajectory algorithms
    static inline std::string trajectoryResultString(const TrajectoryResult& result)
    {
        // Check result
        switch (result) {
        case TrajectoryResult::UNKNOWN:
            return "UNKNOWN";
        case TrajectoryResult::SUCCESS:
            return "SUCCESS";
        case TrajectoryResult::WARNING_DURATION_MISMATCH:
            return "WARNING_DURATION_MISMATCH";
        case TrajectoryResult::ERROR_INVALID_TRAJECTORY_SPLINES:
            return "ERROR_INVALID_TRAJECTORY_SPLINES";
        case TrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE:
            return "ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE";
        case TrajectoryResult::ERROR_INVALID_TRAJECTORIES:
            return "ERROR_INVALID_TRAJECTORIES";
        case TrajectoryResult::ERROR_SPLINEERROR:
            return "ERROR_SPLINEERROR";
        case TrajectoryResult::ERROR_NOTIMPLEMENTED:
            return "ERROR_NOTIMPLEMENTED";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Template class for trajectories (=splines with related timing information)
    /*!
     * Combines an array of splines (\ref m_splines, multidimensional spatial information) with a duration (\ref m_duration, temporal information).
     * The complete trajectory is bounded by \f$ t \in [0,\f$ \ref m_duration\f$]\f$. The trajectory can be evaluated at any time point within these boundaries
     * (values outside will be projected to the boundaries). The derivative is given with respect to time (\f$ \dot{f}(t) = \frac{\partial f(t)}{\partial t}\f$).
     * The mapping between *spline-coordinates* and *time-coordinates* is handled internally.
     *
     * \tparam SplineType The type of the underlying spline-array as derivation of \ref Spline.
     * \tparam Dimension The count of splines related to this trajectory (size of spline-array).
     */
    template <class SplineType, unsigned int Dimension>
    class Trajectory {
    public:
        //! Default constructor (initializes with zero duration)
        Trajectory()
            : m_duration(0)
        {
        }

        //! Destructor
        virtual ~Trajectory()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const Trajectory<SplineType, Dimension>& reference) const
        {
            // Compare duration
            if (m_duration != reference.m_duration)
                return false;

            // Compare splines
            if (m_splines != reference.m_splines)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const Trajectory<SplineType, Dimension>& reference) const { return !(*this == reference); }

        // Members
        std::array<SplineType, Dimension> m_splines; //!< Underlying splines containing spatial information of the trajectory
        double m_duration; //!< Duration [s] for travelling through the whole spline (from 0 to 1 in *spline-coordinates*)

        //! Returns dimension of the trajectory (count of splines)
        static unsigned int dimension() { return Dimension; }

        //! Checks, if the trajectory is properly defined (proper definition of duration and splines)
        /*!
         * \param [out] trajectoryResult Pointer to flag in which the result concerning the trajectory should be stored (optional, use `nullptr` to skip output)
         * \param [out] splineResult Pointer to flag in which the result concerning the underlying splines should be stored (optional, use `nullptr` to skip output)
         * \return `true`, if the trajectory is valid, `false` otherwise.
         */
        virtual bool isValid(TrajectoryResult* const trajectoryResult, SplineResult* const splineResult) const
        {
            // Reset outputs
            if (trajectoryResult != nullptr)
                *trajectoryResult = TrajectoryResult::UNKNOWN;
            if (splineResult != nullptr)
                *splineResult = SplineResult::UNKNOWN;

            // Check, if duration is non-negative
            if (m_duration < 0) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE;
                return false;
            }

            // Check, if splines are valid
            for (size_t i = 0; i < Dimension; i++) {
                if (m_splines[i].isValid(splineResult) == false) {
                    if (trajectoryResult != nullptr)
                        *trajectoryResult = TrajectoryResult::ERROR_INVALID_TRAJECTORY_SPLINES;
                    return false;
                }
            }

            // Otherwise -> valid
            if (trajectoryResult != nullptr)
                *trajectoryResult = TrajectoryResult::SUCCESS;
            return true;
        }

        //! Computes the segment boundaries ("control points") represented in **time domain** ([0, m_duration])
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * \return Ordered list of segment boundaries, or empty list in case of an error.
         *
         * \code
         * |--------T--------|     <-- trajectory
         * |---s1---|---s2---|     <-- spline segments
         * B1       B2      B3     <-- segment boundaries
         * 0       ...  m_duration <-- returned values (for each dimension)
         * \endcode
         */
        virtual std::array<std::vector<double>, Dimension> getSegmentBoundaries() const
        {
            // Check validity
            assert(isValid(nullptr, nullptr));

            // Initialize helpers
            std::array<std::vector<double>, Dimension> boundaries;

            // Iterate over all dimensions
            for (size_t i = 0; i < Dimension; i++) {
                // Get boundaries in spline coordinates
                boundaries[i] = m_splines[i].getSegmentBoundaries();

                // Transform from spline-coordinates to time domain
                for (size_t j = 0; j < boundaries[i].size(); j++)
                    boundaries[i][j] *= m_duration;
            }

            // Pass back list
            return boundaries;
        }

        //! Evaluation of (derivative of) underlying splines
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * \warning The derivative is returned **relative to time** (not relative to *spline-coordinates*)!
         *
         * \param [in] time Time point to evaluate (derivative) at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \param [in] derivationOrder Order of the derivation (use 0 for base function, i.e. no derivation)
         * \return Value (of derivative) at specified time - gradient is expressed relative to time
         */
        virtual std::array<double, Dimension> evaluate(const double& time, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr));

            // Project time to boundaries
            double projectedTime = time;
            if (projectedTime < 0)
                projectedTime = 0;
            if (projectedTime > m_duration)
                projectedTime = m_duration;

            // Compute spline position (in [0, 1]) (linear mapping from projected time)
            double splinePosition = 0;
            if (m_duration > 0) // If no valid duration is given -> use starting point of spline
                splinePosition = projectedTime / m_duration;

            // Prepare return value
            std::array<double, Dimension> returnValue({ { 0 } });

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                // ... -> just evaluate underlying spline value and return this value
                for (size_t i = 0; i < Dimension; i++)
                    returnValue[i] = m_splines[i].evaluate(splinePosition, 0);
            } else {
                // Check if duration is zero
                if (m_duration <= 0) {
                    // ...-> derivative would be infinite so pass back zero instead to avoid division by zero
                    return returnValue;
                }

                // Apply coordinate transformation (chain rule) for derivatives
                double chainRuleMultiplier = 1.0 / m_duration;
                for (unsigned int i = 2; i <= derivationOrder; i++)
                    chainRuleMultiplier /= m_duration;

                // Compute gradient relative to spline coordinates
                for (size_t i = 0; i < Dimension; i++)
                    returnValue[i] = chainRuleMultiplier * m_splines[i].evaluate(splinePosition, derivationOrder);
            }

            // Pass back result
            return returnValue;
        }

        //! Evaluation of value (D0) up to N-th derivative (DN) of underlying splines (improved performance over calling \ref evaluate() N+1 times)
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * \warning The derivatives are returned **relative to time** (not relative to *spline-coordinates*)!
         *
         * \param [in] time Time point to evaluate (derivatives) at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \return List of value (D0) up to N-th derivative [D0, D1, ... , Di, ... , DN] at specified time - gradients are expressed relative to time
         */
        template <unsigned int N>
        std::array<std::array<double, N + 1>, Dimension> evaluateD0ToDN(const double& time) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr));

            // Project time to boundaries
            double projectedTime = time;
            if (projectedTime < 0)
                projectedTime = 0;
            if (projectedTime > m_duration)
                projectedTime = m_duration;

            // Compute spline position (in [0, 1]) (linear mapping from projected time)
            double splinePosition = 0;
            if (m_duration > 0) // If no valid duration is given -> use starting point of spline
                splinePosition = projectedTime / m_duration;

            // Prepare return value
            std::array<std::array<double, N + 1>, Dimension> returnValue;

            // Compute coordinate transformation (chain rule) for derivatives
            double alpha = 0;
            if (m_duration > 0)
                alpha = 1.0 / m_duration;

            // Evaluate splines
            for (unsigned int i = 0; i < Dimension; i++) {
                returnValue[i] = m_splines[i].template evaluateD0ToDN<N>(splinePosition);
                double alphaPowered = alpha;
                for (unsigned int d = 1; d <= N; d++) {
                    returnValue[i][d] *= alphaPowered; // Map derivative
                    alphaPowered *= alpha;
                }
            }

            // Pass back result
            return returnValue;
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
            addedElements += io::encoding::encode(stream, "<Trajectory");

            // Write attributes
            addedElements += io::encoding::encode(stream, " Duration=\"");
            addedElements += io::encoding::encode(stream, (double)m_duration, numericFormat);
            addedElements += io::encoding::encode(stream, "\" Dimension=\"");
            addedElements += io::encoding::encode(stream, (uint64_t)Dimension);
            addedElements += io::encoding::encode(stream, "\">\n");

            // Write spline
            for (size_t i = 0; i < Dimension; i++)
                addedElements += m_splines[i].encodeToXML(stream, XMLIndentationLevel + 1, XMLTabWhiteSpaces, numericFormat);

            // End XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "</Trajectory>\n");

            // Pass back added elements in stream
            return addedElements;
        }
    };

    //! \}
} // namespace curve
} // namespace broccoli
