/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../curves/QuaternionSQUADCurve.hpp"
#include "InterpolatableQuaternionSpline.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    //! Class representing a quaternion spline as concatenation of quaternion SQUAD curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref InterpolatableQuaternionSpline and \ref QuaternionSQUADCurve for details.
     */
    class QuaternionSQUADSpline : public QuaternionSpline<QuaternionSQUADCurve>, public InterpolatableQuaternionSpline {
    public:
        //! Constructor
        QuaternionSQUADSpline()
        {
        }

        //! Destructor
        virtual ~QuaternionSQUADSpline()
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
            case QuaternionSplineInterpolationMethod::SQUAD_PIECEWISE: {
                /*! Interpolation with SQUAD_PIECEWISE
                 *  ----------------------------------
                 * \copydetails interpolatePiecewise()
                 */
                return interpolatePiecewise(keyFrames, proportions, result);
            }
            case QuaternionSplineInterpolationMethod::SQUAD_SMOOTH: {
                /*! Interpolation with SQUAD_SMOOTH
                 *  -------------------------------
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
        //! **Piecewise** interpolation with SQUAD segments with given control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Interpolate each segment of the spline with <b>S</b>pherical <b>QUAD</b>rangle curves (see \ref QuaternionSQUADCurve) with **user-defined** control points.
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

        //! **Smooth** interpolation with SQUAD segments with given control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Interpolate each segment of the spline with <b>S</b>pherical <b>QUAD</b>rangle curves (see \ref QuaternionSQUADCurve) with **automatically chosen** control points.
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
         * * Erik B. Dam et al., "Quaternions, Interpolation and Animation", Technical Report DIKU-TR-98/5, 1998, Department of Computer Science, University of Copenhagen, URL:[http://web.mit.edu/2.998/www/QuaternionReport1.pdf](http://web.mit.edu/2.998/www/QuaternionReport1.pdf)
         *
         * For each segment with keyframes \f$ \left(q_{i},\,q_{i+1}\right)\f$ and control points \f$ \left(q_{i},\,q_{i,1},\,q_{i,2},\,q_{i+1}\right)\f$ compute
         *
         * \f[
         * \begin{array}{ll}
         * q_{i,1}(q_{i-1},\,q_i,\,q_{i+1}) &= q_i\,exp\left(-\frac{1}{4}\left(log\left(q_i^{-1}\,q_{i+1}\right)+log\left(q_i^{-1}\,q_{i-1}\right)\right)\right)\\
         * q_{i,2}(q_i,\,q_{i+1},\,q_{i+2}) &= q_{i+1}\,exp\left(-\frac{1}{4}\left(log\left(q_{i+1}^{-1}\,q_{i+2}\right)+log\left(q_{i+1}^{-1}\,q_{i}\right)\right)\right)
         * \end{array}
         * \f]
         *
         * Additionally the very first/last inner control point is chosen to coincide with the very first/last keyframe (as suggested in Dam 1998) (warning: velocity is **not** zero at start/end). Thus \f$ q_{0,1} = q_0 \f$ and \f$ q_{n-1,2} = q_n \f$.
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

            // Set control points for each segment
            for (size_t i = 0; i < segmentCount; i++) {
                // Set "outer" control points (through-points)
                m_segments[i].m_controlPoints[0] = parameters[i];
                m_segments[i].m_controlPoints[3] = parameters[i + 1];

                // Set first "inner" control point (alias q_{i,1})
                if (i == 0) {
                    // ...first segment -> coincide with very first keyframe quaternion (q_{0,1} = q_0)
                    m_segments[i].m_controlPoints[1] = parameters.front();
                } else {
                    // ...not first segment -> compute according to Dam 1998, p.51
                    Eigen::Quaterniond qiinv = parameters[i].conjugate(); // q_i^{-1}
                    Eigen::Quaterniond logqiinvqip1 = geometry::quaternionNaturalLogarithm(qiinv * parameters[i + 1]); // log( q_i^{-1} * q_{i+1} )
                    Eigen::Quaterniond logqiinvqim1 = geometry::quaternionNaturalLogarithm(qiinv * parameters[i - 1]); // log( q_i^{-1} * q_{i-1} )
                    Eigen::Quaterniond frac; // -\frac{log( q_i^{-1} * q_{i+1} ) + log( q_i^{-1} * q_{i-1} )}{4}
                    frac.coeffs() = -0.25 * (logqiinvqip1.coeffs() + logqiinvqim1.coeffs());
                    m_segments[i].m_controlPoints[1] = parameters[i] * geometry::quaternionExponential(frac);
                    m_segments[i].m_controlPoints[1].normalize();
                }

                // Set second "inner" control point (alias q_{i,2})
                if (i == segmentCount - 1) {
                    // ...last segment -> coincide with very last keyframe quaternion (q_{n-1,2} = q_n)
                    m_segments[i].m_controlPoints[2] = parameters.back();
                } else {
                    // ...not last segment -> compute according to Dam 1998, p.51
                    Eigen::Quaterniond qip1inv = parameters[i + 1].conjugate(); // q_{i+1}^{-1}
                    Eigen::Quaterniond logqip1invqip2 = geometry::quaternionNaturalLogarithm(qip1inv * parameters[i + 2]); // log( q_{i+1}^{-1} * q_{i+2} )
                    Eigen::Quaterniond logqip1invqi = geometry::quaternionNaturalLogarithm(qip1inv * parameters[i]); // log( q_{i+1}^{-1} * q_{i} )
                    Eigen::Quaterniond frac; // -\frac{log( q_{i+1}^{-1} * q_{i+2} ) + log( q_{i+1}^{-1} * q_{i} )}{4}
                    frac.coeffs() = -0.25 * (logqip1invqip2.coeffs() + logqip1invqi.coeffs());
                    m_segments[i].m_controlPoints[2] = parameters[i + 1] * geometry::quaternionExponential(frac);
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
