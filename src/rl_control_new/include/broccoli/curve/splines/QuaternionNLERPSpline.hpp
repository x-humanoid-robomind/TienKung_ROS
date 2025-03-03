/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../curves/QuaternionNLERPCurve.hpp"
#include "InterpolatableQuaternionSpline.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    //! Class representing a quaternion spline as concatenation of quaternion NLERP curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref InterpolatableQuaternionSpline and \ref QuaternionNLERPCurve for details.
     */
    class QuaternionNLERPSpline : public QuaternionSpline<QuaternionNLERPCurve>, public InterpolatableQuaternionSpline {
    public:
        //! Constructor
        QuaternionNLERPSpline()
        {
        }

        //! Destructor
        virtual ~QuaternionNLERPSpline()
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
            case QuaternionSplineInterpolationMethod::NLERP_PIECEWISE: {
                /*! Interpolation with NLERP_PIECEWISE
                 *  ----------------------------------
                 * \copydetails interpolatePiecewise()
                 */
                return interpolatePiecewise(keyFrames, proportions, result);
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
        //! **Piecewise** interpolation with NLERP segments with given control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Interpolate with <b>N</b>ormalized <b>L</b>inear int<b>ERP</b>olation (NLERP) (see \ref QuaternionNLERPCurve) between start- and end-quaternions
         * of each segment. Resulting quaternion spline is \f$ C^0\f$-continuous.
         *
         * \par Parameter-layout
         * Element         | Description
         * -------         | -----------
         * parameters[0]   | keyframe quaternion at beginning of first segment
         * ...             | ...
         * parameters[n-1] | keyframe quaternion at beginning of last segment
         * parameters[n]   | keyframe quaternion at end of last segment
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

            // Create segments and set control-point quaternions
            for (size_t i = 0; i < segmentCount; i++) {
                m_segments[i].m_controlPoints[0] = parameters[i];
                m_segments[i].m_controlPoints[1] = parameters[i + 1];
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
