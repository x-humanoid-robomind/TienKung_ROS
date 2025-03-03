/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "QuaternionCurve.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_curves
     * \{
     */

    //! Specification of quaternion curve interpolation methods
    enum class QuaternionCurveInterpolationMethod : uint8_t {
        BSPLINE_SIMPLE = 0, //!< Directly sets passed quaternions as control points of the B-spline. Curve does **not** necessarily pass through intermediate control points!
        BSPLINE_THROUGHPOINTS, //!< Automatically computes appropriate control points of the B-spline, such that the resulting curve passes through the given quaternion sequence ("through-points").
        QUATERNIONCURVEINTERPOLATIONMETHOD_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given quaternion curve interpolation method
    static inline std::string quaternionCurveInterpolationMethodString(const QuaternionCurveInterpolationMethod& method)
    {
        // Check method
        switch (method) {
        case QuaternionCurveInterpolationMethod::BSPLINE_SIMPLE:
            return "BSPLINE_SIMPLE";
        case QuaternionCurveInterpolationMethod::BSPLINE_THROUGHPOINTS:
            return "BSPLINE_THROUGHPOINTS";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Abstract representation of an **interpolatable** quaternion curve (underlying function is not specified yet)
    /*! Class acts as a prototype for quaternion curves which provide an \ref interpolate() method. */
    class InterpolatableQuaternionCurve {
    public:
        //! Constructor
        InterpolatableQuaternionCurve()
        {
        }

        //! Destructor
        virtual ~InterpolatableQuaternionCurve()
        {
        }

        //! Interpolate quaternion curve to achieve specific properties (through-points, derivatives, ...)
        /*!
         * \param [in] parameters Interpolation parameters (meaning varies depending on chosen method)
         * \param [in] proportions Interpolation proportions (meaning varies depending on chosen method)
         * \param [in] method Interpolation method to be used (has to be compatible with quaternion curve type)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         *
         * Notes on antipodal keyframes
         * ----------------------------
         * Note that \f$q\f$ and \f$-q\f$ describe the same rotation/orientation. However, using one or the other may change the result of interpolation which often tries to "walk the shortest path". In order to
         * avoid unexpected motion during interpolation, the method \ref QuaternionCurve::recalculateKeyFrameSequence() allows to automatically flip quaternions in a sequence of keyframes such that two neighboring
         * keyframes always have the shortest distance. In order to avoid undesired effects, be sure to call \ref QuaternionCurve::recalculateKeyFrameSequence() on \p parameters before passing them to this function.
         * \attention Alternatively you can use a derived class of \ref InterpolatableQuaternionSpline which may automatically call \ref QuaternionCurve::recalculateKeyFrameSequence() on the passed \p parameters,
         * such that this issue is automatically handled.
         */
        virtual bool interpolate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, const QuaternionCurveInterpolationMethod& method, QuaternionCurveResult* const result = nullptr) = 0;
    };

    //! \}
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
