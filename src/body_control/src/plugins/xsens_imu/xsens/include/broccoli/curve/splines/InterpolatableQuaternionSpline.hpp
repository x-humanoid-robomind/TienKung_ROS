/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "QuaternionSpline.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_splines
     * \{
     */

    //! Specification of quaternion spline interpolation methods
    enum class QuaternionSplineInterpolationMethod : uint8_t {
        LERP_PIECEWISE = 0, //!< Interpolate each segment of the spline with <b>L</b>inear int<b>ERP</b>olation (LERP) (resulting quaternion spline is \f$ C^0\f$-continuous)
        NLERP_PIECEWISE, //!< Interpolate each segment of the spline with <b>N</b>ormalized <b>L</b>inear int<b>ERP</b>olation (NLERP) (resulting quaternion spline is \f$ C^0\f$-continuous)
        SLERP_PIECEWISE, //!< <b>S</b>pherical <b>L</b>inear int<b>ERP</b>olation (resulting quaternion spline is \f$ C^0\f$-continuous)
        BEZIER_PIECEWISE, //!< Interpolate each segment of the spline with cubic **Bezier** quaternion curves with user-defined control points (resulting quaternion spline is \f$ C^0\f$-continuous)
        BEZIER_SMOOTH, //!< Interpolate each segment of the spline with cubic **Bezier** quaternion curves with automatically chosen control points (resulting quaternion spline is \f$ C^1\f$-continuous)
        SQUAD_PIECEWISE, //!< Interpolate each segment of the spline with <b>S</b>pherical <b>QUAD</b>rangle curves with user-defined control points (resulting quaternion spline is \f$ C^0\f$-continuous)
        SQUAD_SMOOTH, //!< Interpolate each segment of the spline with <b>S</b>pherical <b>QUAD</b>rangle curves with automatically chosen control points (resulting quaternion spline is \f$ C^1\f$-continuous)
        BSPLINE_SIMPLE, //!< Creates exactly **one** segment in the spline. This segment is interpolated with the passed quaternions which are directly used as control points of the B-spline curve. The curve does **not** necessarily pass through intermediate control points! (resulting quaternion spline is \f$ C^2\f$-continuous)
        BSPLINE_THROUGHPOINTS, //!< Creates exactly **one** segment in the spline. This segment is interpolated with the passed quaternions which are considered as "through-points", i.e. the control points of the B-spline curve are computed accordingly. The curve passes through **all** intermediate control points! (resulting quaternion spline is \f$ C^2\f$-continuous)
        QUATERNIONSPLINEINTERPOLATIONMETHOD_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given quaternion spline interpolation method
    static inline std::string quaternionSplineInterpolationMethodString(const QuaternionSplineInterpolationMethod& method)
    {
        // Check method
        switch (method) {
        case QuaternionSplineInterpolationMethod::LERP_PIECEWISE:
            return "LERP_PIECEWISE";
        case QuaternionSplineInterpolationMethod::NLERP_PIECEWISE:
            return "NLERP_PIECEWISE";
        case QuaternionSplineInterpolationMethod::SLERP_PIECEWISE:
            return "SLERP_PIECEWISE";
        case QuaternionSplineInterpolationMethod::BEZIER_PIECEWISE:
            return "BEZIER_PIECEWISE";
        case QuaternionSplineInterpolationMethod::BEZIER_SMOOTH:
            return "BEZIER_SMOOTH";
        case QuaternionSplineInterpolationMethod::SQUAD_PIECEWISE:
            return "SQUAD_PIECEWISE";
        case QuaternionSplineInterpolationMethod::SQUAD_SMOOTH:
            return "SQUAD_SMOOTH";
        case QuaternionSplineInterpolationMethod::BSPLINE_SIMPLE:
            return "BSPLINE_SIMPLE";
        case QuaternionSplineInterpolationMethod::BSPLINE_THROUGHPOINTS:
            return "BSPLINE_THROUGHPOINTS";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Abstract representation of an **interpolatable** quaternion spline
    /*! Class acts as a prototype for quaternion splines which provide an \ref interpolate() method. */
    class InterpolatableQuaternionSpline {
    public:
        //! Constructor
        InterpolatableQuaternionSpline()
        {
        }

        //! Destructor
        virtual ~InterpolatableQuaternionSpline()
        {
        }

        //! Interpolate parameters of underlying quaternion curve to fit specific constraints (start-end-quaternion, control-points, ...)
        /*!
         * \param [in] parameters Interpolation parameters (meaning varies depending on chosen method )
         * \param [in] proportions Pointer to segment proportions. If `nullptr`, all segments have the same proportion (uniform distribution), otherwise uses given proportions.
         * \param [in] method Interpolation method to be used (has to be compatible with specified curve type)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         *
         * Notation:
         * ---------
         * Symbol        | Code   | Description
         * ------        | -----  | -----------
         * \f$ n \f$     | `n`    | segment count
         * \f$ s_i \f$   | `si`   | spline segment \f$ i \f$
         * \f$ x_i \f$   | `xi`   | *segment-coordinate* of segment \f$ i \f$
         * \f$ h_i \f$   | `hi`   | segment "duration" of segment \f$ i \f$ (= segment proportion)
         * \f$ g_i \f$   | `gi`   | reciprocal of \f$ h_i \f$ (i.e. \f$ g_i = \frac{1}{h_i} \f$)
         * \f$ z \f$     | `z`    | *spline-coordinate* (\f$ z \in [0,\,1] \f$, \f$ z \f$ is **not** the time in general!)
         * \f$ y_i \f$   | `yi`   | value of spline segment \f$ i \f$
         *
         * \par Segmentation
         * \code
         * | ---- s0 ---- | ---- s1 ---- | ---- ... ---- | ---- si ---- | ---- ... ---- | ---- sn-1 ---- |
         * 0 ---- x0 --- 1|0 --- x1 --- 1|0 --- ... --- 1|0 --- xi --- 1|0 --- ... --- 1|0 --- xn-1 --- 1|
         * | ---- h0 ---- | ---- h1 ---- | ---- ... ---- | ---- hi ---- | ---- ... ---- | ---- hn-1 ---- |
         * 0 ------------------------------------------- z --------------------------------------------- 1
         * z0 ----------- z1 ----------- z2 --- ... ---- zi ---------- zi+1 -- ... --- zn-1 ------------ zn
         * y0 ----------- y1 ----------- y2 --- ... ---- yi ---------- yi+1 -- ... --- yn-1 ------------ yn
         * \endcode
         *
         * **Note:** \f$ x_{i+1} \f$ will be written as `xip1` ("p" for "plus") and \f$ x_{i-1} \f$ will be written as `xim1` ("m" for "minus")
         *
         * Notes on Coordinate Systems
         * ---------------------------
         * \par Coordinate transformation
         * The transformation from *spline-coordinates* \f$ z \f$ to *segment-coordinates* \f$x_i \f$ is given by \f$ x_i(z) = \alpha_i\,z + \beta_i \f$.
         *
         * With the boundary values:
         * - Start: \f$ x_i(z_i) = \alpha_i \, z_i + \beta_i = 0 \f$
         * - End: \f$ x_i(z_{i+1}) = \alpha_i\,z_{i+1} + \beta_i = 1 \f$
         *
         * we obtain
         * \f[ \alpha_i = \frac{1}{z_{i+1} - z_i} = \frac{1}{h_i} = g_i \f]
         * and
         * \f[ \beta_i = - \frac{z_i}{h_i} \f]
         */
        virtual bool interpolate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, const QuaternionSplineInterpolationMethod& method, QuaternionSplineResult* const result = nullptr) = 0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

    //! \}
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
