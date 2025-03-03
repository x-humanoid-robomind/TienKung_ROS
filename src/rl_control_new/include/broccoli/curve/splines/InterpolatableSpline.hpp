/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "Spline.hpp"

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_splines
     * \{
     */

    //! Specification of spline interpolation methods
    enum class SplineInterpolationMethod : uint8_t {
        POLYNOMIAL_PIECEWISE_CONSTANT = 0, //!< **Piecewise** interpolation with polynomials of **degree 0** (piecewise constant) through given points
        POLYNOMIAL_PIECEWISE_LINEAR, //!< **Piecewise** interpolation with polynomials of **degree 1** (piecewise linear) through given points
        POLYNOMIAL_PIECEWISE_CUBIC_FIRST_DERIVATIVES, //!< **Piecewise** interpolation with polynomials of **degree 3** (piecewise cubic) through given points and with given first order derivative at control points
        POLYNOMIAL_PIECEWISE_CUBIC_SECOND_DERIVATIVES, //!< **Piecewise** interpolation with polynomials of **degree 3** (piecewise cubic) through given points and with given second order derivative at control points
        POLYNOMIAL_PIECEWISE_QUINTIC_FIRST_AND_SECOND_DERIVATIVES, //!< **Piecewise** interpolation with polynomials of **degree 5** (piecewise quintic) through given points and with given first and second order derivatives at control points
#ifdef HAVE_EIGEN3
        POLYNOMIAL_SMOOTH_CUBIC_SECOND_DERIVATIVES, //!< **Smooth** interpolation (spline) with polynomials of **degree 3** (piecewise cubic) through given points, \f$ C^2 \f$ continuous and with given second order derivative at start and end of spline
        POLYNOMIAL_SMOOTH_QUINTIC_FIRST_AND_SECOND_DERIVATIVES, //!< **Smooth** interpolation (spline) with polynomials of **degree 5** (piecewise quintic) through given points, \f$ C^4 \f$ continuous and with given first and second order derivative at start and end of spline
#endif // HAVE_EIGEN3
        SPLINEINTERPOLATIONMETHOD_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given spline interpolation method
    static inline std::string splineInterpolationMethodString(const SplineInterpolationMethod& method)
    {
        // Check method
        switch (method) {
        case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CONSTANT:
            return "POLYNOMIAL_PIECEWISE_CONSTANT";
        case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_LINEAR:
            return "POLYNOMIAL_PIECEWISE_LINEAR";
        case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CUBIC_FIRST_DERIVATIVES:
            return "POLYNOMIAL_PIECEWISE_CUBIC_FIRST_DERIVATIVES";
        case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CUBIC_SECOND_DERIVATIVES:
            return "POLYNOMIAL_PIECEWISE_CUBIC_SECOND_DERIVATIVES";
        case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_QUINTIC_FIRST_AND_SECOND_DERIVATIVES:
            return "POLYNOMIAL_PIECEWISE_QUINTIC_FIRST_AND_SECOND_DERIVATIVES";
#ifdef HAVE_EIGEN3
        case SplineInterpolationMethod::POLYNOMIAL_SMOOTH_CUBIC_SECOND_DERIVATIVES:
            return "POLYNOMIAL_SMOOTH_CUBIC_SECOND_DERIVATIVES";
        case SplineInterpolationMethod::POLYNOMIAL_SMOOTH_QUINTIC_FIRST_AND_SECOND_DERIVATIVES:
            return "POLYNOMIAL_SMOOTH_QUINTIC_FIRST_AND_SECOND_DERIVATIVES";
#endif // HAVE_EIGEN3
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Abstract representation of an **interpolatable** spline
    /*! Class acts as a prototype for splines which provide an \ref interpolate() method. */
    class InterpolatableSpline {
    public:
        //! Constructor
        InterpolatableSpline()
        {
        }

        //! Destructor
        virtual ~InterpolatableSpline()
        {
        }

        //! Interpolate coefficients of underlying function to fit specific parameters (through-points, derivatives, ...)
        /*!
         * \param [in] parameters Interpolation parameters (meaning varies depending on chosen method)
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
         * \f$ y'_i \f$  | `dyi`  | first order derivative (with respect to *spline-coordinate* \f$ z \f$, **not** the time in general!) of spline segment \f$ i \f$
         * \f$ y''_i \f$ | `ddyi` | second order derivative (with respect to *spline-coordinate* \f$ z \f$, **not** the time in general!) of spline segment \f$ i \f$
         *
         * \par Segmentation
         * \code
         * | ---- s0 ---- | ---- s1 ---- | ---- ... ---- | ---- si ---- | ---- ... ---- | ---- sn-1 ---- |
         * 0 ---- x0 --- 1|0 --- x1 --- 1|0 --- ... --- 1|0 --- xi --- 1|0 --- ... --- 1|0 --- xn-1 --- 1|
         * | ---- h0 ---- | ---- h1 ---- | ---- ... ---- | ---- hi ---- | ---- ... ---- | ---- hn-1 ---- |
         * 0 ------------------------------------------- z --------------------------------------------- 1
         * z0 ----------- z1 ----------- z2 --- ... ---- zi ---------- zi+1 -- ... --- zn-1 ------------ zn
         * y0 ----------- y1 ----------- y2 --- ... ---- yi ---------- yi+1 -- ... --- yn-1 ------------ yn
         * dy0 --------- dy1 ---------- dy2 --- ... --- dyi --------- dyi+1 -- ... -- dyn-1 ----------- dyn
         * ddy0 ------- ddy1 --------- ddy2 --- ... -- ddyi -------- ddyi+1 -- ... - ddyn-1 ---------- ddyn
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
         *
         * \par Derivatives
         * When switching from *spline-coordinates* \f$ z \f$ to *segment-coordinates* \f$x_i \f$ and back one has to consider the transformation of derivatives (i.e. the chain rule).
         *
         * \f[\def\arraystretch{2}
         * \begin{array}{ll}
         * \displaystyle\frac{\partial x_i(z)}{\partial z} &= \displaystyle\alpha_i = \frac{1}{hi} \\
         * \displaystyle\frac{\partial^2 x_i(z)}{\partial z^2} &= \displaystyle0 \\
         * \displaystyle\frac{\partial s_i(z)}{\partial z} &= \displaystyle\frac{\partial s_i(x_i)}{\partial x_i} \cdot \frac{\partial x_i(z)}{\partial z} = \frac{\partial s_i(x_i)}{\partial x_i} \cdot \frac{1}{h_i} \\
         * \displaystyle\frac{\partial^2 s_i(z)}{\partial z^2} &= \displaystyle\frac{\partial^2 s_i(x_i)}{\partial x_i^2} \cdot \left(\frac{\partial x_i(z)}{\partial z}\right)^2 + \frac{\partial s_i(x_i)}{\partial x_i} \cdot \underbrace{\frac{\partial^2 x_i(z)}{\partial z^2}}_{=0} = \frac{\partial^2 s_i(x_i)}{\partial x_i^2} \cdot \left(\frac{1}{h_i}\right)^2 \\
         * \displaystyle\frac{\partial^3 s_i(z)}{\partial z^3} &= \displaystyle\frac{\partial^3 s_i(x_i)}{\partial x_i^3} \cdot \left(\frac{1}{h_i}\right)^3 \\
         * \displaystyle\frac{\partial^4 s_i(z)}{\partial z^4} &= \displaystyle\frac{\partial^4 s_i(x_i)}{\partial x_i^4} \cdot \left(\frac{1}{h_i}\right)^4 \\
         * \displaystyle & \displaystyle\vdots
         * \end{array}
         * \f]
         */
        virtual bool interpolate(const std::vector<double>& parameters, const std::vector<double>* const proportions, const SplineInterpolationMethod& method, SplineResult* const result = nullptr) = 0;
    };

    //! \}
} // namespace curve
} // namespace broccoli
