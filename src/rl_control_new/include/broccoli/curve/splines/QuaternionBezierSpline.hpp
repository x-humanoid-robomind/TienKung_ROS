/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../curves/QuaternionBezierCurve.hpp"
#include "InterpolatableQuaternionSpline.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    //! Class representing a quaternion spline as concatenation of quaternion Bezier curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref InterpolatableQuaternionSpline and \ref QuaternionBezierCurve for details.
     */
    class QuaternionBezierSpline : public QuaternionSpline<QuaternionBezierCurve>, public InterpolatableQuaternionSpline {
    public:
        //! Constructor
        QuaternionBezierSpline()
        {
        }

        //! Destructor
        virtual ~QuaternionBezierSpline()
        {
        }

        // Interpolation
        // -------------
        // Interpolate parameters of underlying curve to fit specific constraints (see base class for details)
        //! \copydoc QuaternionLERPSpline::interpolate()
        virtual bool interpolate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, const QuaternionSplineInterpolationMethod& method, QuaternionSplineResult* const result = nullptr)
        {
            // Recompute keyframe sequence to fix issue with antipodal keyframe quaternions
            std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> keyFrames = QuaternionCurve::recalculateKeyFrameSequence(parameters);

            // Check interpolation method
            switch (method) {
            case QuaternionSplineInterpolationMethod::BEZIER_PIECEWISE: {
                /*! Interpolation with BEZIER_PIECEWISE
                 *  -----------------------------------
                 * \copydetails interpolatePiecewise()
                 */
                return interpolatePiecewise(keyFrames, proportions, result);
            }
            case QuaternionSplineInterpolationMethod::BEZIER_SMOOTH: {
                /*! Interpolation with BEZIER_SMOOTH
                 *  --------------------------------
                 * \copydetails interpolateSmooth()
                 */
                return interpolateSmooth(keyFrames, proportions, result);
            }
            default: {
                // No interpolation methods implemented for now
                if (result != nullptr)
                    *result = QuaternionSplineResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }
            }

            // Success
            return true;
        }

    protected:
        //! **Piecewise** interpolation with Bezier segments with given control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Interpolate each segment of the spline with cubic **Bezier** quaternion curves (see \ref QuaternionBezierCurve) with **user-defined** control points.
         * Resulting quaternion spline is \f$ C^0\f$-continuous.
         *
         * \par Parameter-layout
         * Element          | Description
         * -------          | -----------
         * parameters[0]    | keyframe quaternion at beginning of first segment
         * parameters[1]    | first control point quaternion of first segment
         * parameters[2]    | second control point quaternion of first segment
         * parameters[3]    | keyframe quaternion at end of first segment (=beginning of second segment)
         * ...              | ...
         * parameters[3n-3] | keyframe quaternion at beginning of last segment (=end of second last segment)
         * parameters[3n-2] | first control point quaternion of last segment
         * parameters[3n-1] | second control point quaternion of last segment
         * parameters[3n]   | keyframe quaternion at end of last segment
         */
        virtual bool interpolatePiecewise(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, QuaternionSplineResult* const result = nullptr)
        {
            // Check, if proportions are specified
            bool proportionsSpecified = false;
            if (proportions != nullptr)
                if (proportions->size() > 0)
                    proportionsSpecified = true;

            // Clear current data
            m_segments.clear();
            if (proportionsSpecified == false)
                m_segmentProportions.clear();
            else
                m_segmentProportions = *proportions;

            // Checking parameter set
            if (parameters.size() < 4 || parameters.size() % 3 != 1 || (proportionsSpecified == true && m_segmentProportions.size() != (parameters.size() - 1) / 3)) {
                if (result != nullptr)
                    *result = QuaternionSplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            const size_t segmentCount = (parameters.size() - 1) / 3;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set control-point quaternions
            for (size_t i = 0; i < segmentCount; i++) {
                m_segments[i].m_controlPoints[0] = parameters[3 * i];
                m_segments[i].m_controlPoints[1] = parameters[3 * i + 1];
                m_segments[i].m_controlPoints[2] = parameters[3 * i + 2];
                m_segments[i].m_controlPoints[3] = parameters[3 * i + 3];
            }

            // Success!
            if (result != nullptr)
                *result = QuaternionSplineResult::SUCCESS;
            return true;
        }

        //! **Smooth** interpolation with Bezier segments with given control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Interpolate each segment of the spline with cubic **Bezier** quaternion curves (see \ref QuaternionBezierCurve) with **automatically chosen** control points.
         * Resulting quaternion spline is \f$ C^1\f$-continuous.
         *
         * \par Parameter-layout
         * Element         | Description
         * -------         | -----------
         * parameters[0]   | keyframe quaternion at beginning of first segment
         * ...             | ...
         * parameters[n-1] | keyframe quaternion at beginning of last segment
         * parameters[n]   | keyframe quaternion at end of last segment
         *
         * \par Choice of "inner" control points
         *
         * The "inner" control points of each segment are chosen automatically according to the method suggested in
         *
         * * Ken Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH Computer Graphics, ACM, New York, NY, USA, volume 19, number 3, 1985, DOI:[10.1145/325165.325242](https://www.doi.org/10.1145/325165.325242), p.245--254
         *
         * For each segment with keyframes \f$ \left(q_{i},\,q_{i+1}\right)\f$ and control points \f$ \left(q_{i},\,q_{i,1},\,q_{i,2},\,q_{i+1}\right)\f$ compute in the first run
         *
         * \f[
         * \begin{array}{ll}
         * q_{i,x}(q_{i-1},\,q_i) &= 2\left( q_{i-1} \cdot q_i\right)q_i-q_{i-1}\\
         * q_{i,1}(q_{i-1},\,q_i,\,q_{i+1}) &= \frac{q_{i,x} + q_{i+1}}{||q_{i,x} + q_{i+1}||}
         * \end{array}
         * \f]
         *
         * and in the second run
         *
         * \f[
         * \begin{array}{ll}
         * q_{i,2}(q_i,\,q_{i+1},\,q_{i+2}) &= 2\left( q_{i+1,1} \cdot q_{i+1}\right)q_{i+1}-q_{i+1,1}
         * \end{array}
         * \f]
         *
         * Additionally the very first/last inner control point is chosen to coincide with the very first/last keyframe (zero velocity at start and end). Thus \f$ q_{0,1} = q_0 \f$ and \f$ q_{n-1,2} = q_n \f$.
         *
         * \remark Note that \f$ \cdot \f$ here denotes the quaternion "dot"-product!
         */
        virtual bool interpolateSmooth(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, QuaternionSplineResult* const result = nullptr)
        {
            // Check, if proportions are specified
            bool proportionsSpecified = false;
            if (proportions != nullptr)
                if (proportions->size() > 0)
                    proportionsSpecified = true;

            // Clear current data
            m_segments.clear();
            if (proportionsSpecified == false)
                m_segmentProportions.clear();
            else
                m_segmentProportions = *proportions;

            // Checking parameter set
            if (parameters.size() <= 1 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() - 1)) {
                if (result != nullptr)
                    *result = QuaternionSplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            const size_t segmentCount = parameters.size() - 1;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Set "outer" control points (through-points)
            for (size_t i = 0; i < segmentCount; i++) {
                m_segments[i].m_controlPoints[0] = parameters[i];
                m_segments[i].m_controlPoints[3] = parameters[i + 1];
            }

            // "First run": compute first "inner" control point (alias q_{i,1})
            for (size_t i = 0; i < segmentCount; i++) {
                // Check, if this is the first segment
                if (i == 0) {
                    // ...first segment -> coincide with very first keyframe quaternion (q_{0,1} = q_0)
                    m_segments[i].m_controlPoints[1] = parameters.front();
                } else {
                    // ...not first segment -> compute according to Shoemake 1985, p.249
                    m_segments[i].m_controlPoints[1].coeffs() = 2.0 * (parameters[i - 1].dot(parameters[i])) * parameters[i].coeffs() - parameters[i - 1].coeffs() + parameters[i + 1].coeffs();
                    m_segments[i].m_controlPoints[1].normalize();
                }
            }

            // "Second run": compute second "inner" control point (alias q_{i,2})
            for (size_t i = 0; i < segmentCount; i++) {
                // Check, if this is the last segment
                if (i == segmentCount - 1) {
                    // ...last segment -> coincide with very last keyframe quaternion (q_{n-1,2} = q_n)
                    m_segments[i].m_controlPoints[2] = parameters.back();
                } else {
                    // ...not last segment -> compute according to Shoemake 1985, p.249
                    m_segments[i].m_controlPoints[2].coeffs() = 2.0 * (m_segments[i + 1].m_controlPoints[1].dot(parameters[i + 1])) * parameters[i + 1].coeffs() - m_segments[i + 1].m_controlPoints[1].coeffs();
                    m_segments[i].m_controlPoints[2].normalize();
                }
            }

            // Success!
            if (result != nullptr)
                *result = QuaternionSplineResult::SUCCESS;
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
