/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../curves/QuaternionBSplineCurve.hpp"
#include "InterpolatableQuaternionSpline.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    //! Class representing a quaternion spline as concatenation of quaternion BSpline curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref InterpolatableQuaternionSpline and \ref QuaternionBSplineCurve for details.
     *
     * \tparam Degree Passed through to \ref QuaternionBSplineCurve.
     * \tparam MaximumBSplineCount Passed through to \ref QuaternionBSplineCurve.
     */
    template <unsigned int Degree, unsigned int MaximumBSplineCount = Degree + 1>
    class QuaternionBSplineSpline : public QuaternionSpline<QuaternionBSplineCurve<Degree, MaximumBSplineCount>>, public InterpolatableQuaternionSpline {
    public:
        // Name lookup (template class derived from other template class)
        // (helps the compiler to recognize the "dependent" base class members)
        using QuaternionSpline<QuaternionBSplineCurve<Degree, MaximumBSplineCount>>::m_segments;
        using QuaternionSpline<QuaternionBSplineCurve<Degree, MaximumBSplineCount>>::m_segmentProportions;

        //! Constructor
        QuaternionBSplineSpline()
        {
        }

        //! Destructor
        virtual ~QuaternionBSplineSpline()
        {
        }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of underlying B-spline basis
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of underlying B-spline basis
        static unsigned int order() { return Degree + 1; }

        // Interpolation
        // -------------
        // Interpolate parameters of underlying curve to fit specific constraints (see base class for details)
        /*!
         * \copydoc QuaternionLERPSpline::interpolate()
         *
         * Creates a quaternion spline with exactly **one segment** which represents a **quaternion B-Spline curve** of order \f$k\f$ (see \ref QuaternionBSplineCurve).
         * This curve contains
         *  * a sequence of \f$m\geq k\f$ control point quaternions (obtained from the passed \p parameters) and
         *  * an underlying B-spline basis (see \ref BSplineBasis) with a knot-sequence \f$\tau = [t_0,\,\dots\,,\,t_{n-1}]\f$ of \f$n = m+k\f$ non-decreasing knots.
         *
         * For details on the interpolation method and how to set the \p parameters check out the corresponding section in \ref QuaternionBSplineCurve::interpolate().
         *
         * \remark Since the spline only contains one single segment, passing segment \p proportions does not make sense anymore. However, in this case they are used to allow
         * a non-uniform (but still clamped) knot-sequence of the underlying B-spline basis. The \p proportions then define the positioning of the \f$m-k\f$ "interior" knots
         * \f$t_{p+1},\,\dots\,,\,t_{m-1}\f$. If no \p proportions are passed a *uniform* partitioning will be chosen.
         */
        virtual bool interpolate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, const QuaternionSplineInterpolationMethod& method, QuaternionSplineResult* const result = nullptr)
        {
            // Recompute keyframe sequence to fix issue with antipodal keyframe quaternions
            std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> keyFrames = QuaternionCurve::recalculateKeyFrameSequence(parameters);

            // Initialize helpers
            QuaternionCurveResult curveResult = QuaternionCurveResult::UNKNOWN;
            bool successCurveInterpolation = false;

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

            // Allocate memory for ONE single segment containing our curve
            m_segments.resize(1);
            m_segmentProportions.resize(1);
            m_segmentProportions[0] = 1.0;

            // Pass-through to interpolation method of underlying curve
            if (method == QuaternionSplineInterpolationMethod::BSPLINE_SIMPLE)
                successCurveInterpolation = m_segments[0].interpolate(keyFrames, proportions, QuaternionCurveInterpolationMethod::BSPLINE_SIMPLE, &curveResult);
            else if (method == QuaternionSplineInterpolationMethod::BSPLINE_THROUGHPOINTS)
                successCurveInterpolation = m_segments[0].interpolate(keyFrames, proportions, QuaternionCurveInterpolationMethod::BSPLINE_THROUGHPOINTS, &curveResult);
            else {
                // Unknown interpolation method
                if (result != nullptr)
                    *result = QuaternionSplineResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }

            // Evaluate result of curve interpolation
            if (successCurveInterpolation == false || curveResult != QuaternionCurveResult::SUCCESS) {
                if (result != nullptr) {
                    if (curveResult == QuaternionCurveResult::ERROR_INVALID_PARAMETERS)
                        *result = QuaternionSplineResult::ERROR_INVALID_PARAMETERS;
                    else if (curveResult == QuaternionCurveResult::ERROR_NOTIMPLEMENTED)
                        *result = QuaternionSplineResult::ERROR_NOTIMPLEMENTED;
                    else
                        *result = QuaternionSplineResult::UNKNOWN;
                }
                assert(false);
                return false;
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
