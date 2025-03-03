/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../curves/PolynomialCurve.hpp"
#include "../splines/InterpolatableSpline.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#include <Eigen/StdVector>
#endif

namespace broccoli {
namespace curve {
    //! Class representing a spline as concatenation of polynomial curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref InterpolatableSpline and \ref PolynomialCurve for details.
     *
     * \tparam Degree Passed through to \ref PolynomialCurve.
     */
    template <unsigned int Degree>
    class PolynomialSpline : public Spline<PolynomialCurve<Degree>>, public InterpolatableSpline {
    public:
        // Name lookup (template class derived from other template class)
        // (helps the compiler to recognize the "dependent" base class members)
        using Spline<PolynomialCurve<Degree>>::m_segments;
        using Spline<PolynomialCurve<Degree>>::m_segmentProportions;
        using Spline<PolynomialCurve<Degree>>::isValid;

        //! Constructor
        PolynomialSpline()
        {
        }

        //! Destructor
        virtual ~PolynomialSpline()
        {
        }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of underlying polynomial segments
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of underlying polynomial segments
        static unsigned int order() { return Degree + 1; }

        // Interpolation
        // -------------
        // Interpolate coefficients of underlying function to fit specific parameters (see base class for details)
        //! \copydoc InterpolatableSpline::interpolate()
        /*!
         * \remark Automatically checks, if degree of polynomials is high enough. If the degree of the polynomials is higher
         * than necessary, the remaining coefficients are set to zero automatically.
         */
        virtual bool interpolate(const std::vector<double>& parameters, const std::vector<double>* const proportions, const SplineInterpolationMethod& method, SplineResult* const result = nullptr)
        {
            // Check interpolation method
            switch (method) {
            case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CONSTANT: {
                /*! Interpolation with POLYNOMIAL_PIECEWISE_CONSTANT
                 *  ------------------------------------------------
                 * \copydetails interpolatePiecewiseConstant()
                 */
                return interpolatePiecewiseConstant(parameters, proportions, result);
            }
            case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_LINEAR: {
                /*! Interpolation with POLYNOMIAL_PIECEWISE_LINEAR
                 *  ----------------------------------------------
                 * \copydetails interpolatePiecewiseLinear()
                 */
                return interpolatePiecewiseLinear(parameters, proportions, result);
            }
            case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CUBIC_FIRST_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_PIECEWISE_CUBIC_FIRST_DERIVATIVES
                 *  ---------------------------------------------------------------
                 * \copydetails interpolatePiecewiseCubicFirstDerivatives()
                 */
                return interpolatePiecewiseCubicFirstDerivatives(parameters, proportions, result);
            }
            case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CUBIC_SECOND_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_PIECEWISE_CUBIC_SECOND_DERIVATIVES
                 *  ----------------------------------------------------------------
                 * \copydetails interpolatePiecewiseCubicSecondDerivatives()
                 */
                return interpolatePiecewiseCubicSecondDerivatives(parameters, proportions, result);
            }
            case SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_QUINTIC_FIRST_AND_SECOND_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_PIECEWISE_QUINTIC_FIRST_AND_SECOND_DERIVATIVES
                 *  ----------------------------------------------------------------------------
                 * \copydetails interpolatePiecewiseQuinticFirstAndSecondDerivatives()
                 */
                return interpolatePiecewiseQuinticFirstAndSecondDerivatives(parameters, proportions, result);
            }
#ifdef HAVE_EIGEN3
            case SplineInterpolationMethod::POLYNOMIAL_SMOOTH_CUBIC_SECOND_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_SMOOTH_CUBIC_SECOND_DERIVATIVES
                 *  -------------------------------------------------------------
                 * \copydetails interpolateSmoothCubicSecondDerivatives()
                 */
                return interpolateSmoothCubicSecondDerivatives(parameters, proportions, result);
            }
            case SplineInterpolationMethod::POLYNOMIAL_SMOOTH_QUINTIC_FIRST_AND_SECOND_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_SMOOTH_QUINTIC_FIRST_AND_SECOND_DERIVATIVES
                 *  -------------------------------------------------------------------------
                 * \copydetails interpolateSmoothQuinticFirstAndSecondDerivatives()
                 */
                return interpolateSmoothQuinticFirstAndSecondDerivatives(parameters, proportions, result);
            }
#endif // HAVE_EIGEN3
            default: {
                // No interpolation methods implemented for now
                if (result != nullptr)
                    *result = SplineResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }
            }

            // Success
            return true;
        }

    protected:
        //! **Piecewise** interpolation with polynomials of **degree 0** (piecewise constant) through given points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Piecewise** interpolation with polynomials of degree 0.
         *  * The segments are piecewise constant.
         *  * The spline is **not** \f$ C^0 \f$ continuous.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | function value of first segment
         * ...                | ...
         * \p parameters[n-1] | function value of last segment
         */
        virtual bool interpolatePiecewiseConstant(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            // no test necessary...

            // Checking parameter set
            if (parameters.size() == 0 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size())) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size();

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set their coefficients
            for (size_t i = 0; i < segmentCount; i++) {
                // Prepare indices (mapping in parameter array)
                const double& yi = parameters[i];

                // Interpolate curve segment
                if (m_segments[i].interpolate(std::vector<double>{ yi }, CurveInterpolationMethod::POLYNOMIAL_CONSTANT, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

        //! **Piecewise** interpolation with polynomials of **degree 1** (piecewise linear) through given points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Piecewise** interpolation with polynomials of degree 1.
         *  * The segments are piecewise linear.
         *  * Each segment is linked to its neighbor.
         *  * The spline is \f$ C^0 \f$ continuous.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | function value at beginning of first segment
         * ...                | ...
         * \p parameters[n-1] | function value at beginning of last segment
         * \p parameters[n]   | function value at end of last segment
         */
        virtual bool interpolatePiecewiseLinear(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            if (degree() < 1) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Checking parameter set
            if (parameters.size() <= 1 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() - 1)) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size() - 1;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set their coefficients
            for (size_t i = 0; i < segmentCount; i++) {
                // Prepare indices (mapping in parameter array)
                const double& yi = parameters[i];
                const double& yip1 = parameters[i + 1];

                // Interpolate curve segment
                if (m_segments[i].interpolate(std::vector<double>{ yi, yip1 }, CurveInterpolationMethod::POLYNOMIAL_LINEAR, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

        //! **Piecewise** interpolation with polynomials of **degree 3** (piecewise cubic) through given points and with given first order derivative at control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Piecewise** interpolation with polynomials of degree 3 (given first order derivatives).
         *  * The segments are piecewise cubic.
         *  * The segments are linked to their neighbors.
         *  * The spline is \f$ C^1 \f$ continuous.
         *  * The user can specify the first order derivatives (with respect to spline coordinates, **not** time in general!) at each control point.
         *
         * \par Parameter-layout
         * Element             | Description
         * -------             | -----------
         * \p parameters[0]    | function value at beginning of first segment
         * ...                 | ...
         * \p parameters[n-1]  | function value at beginning of last segment
         * \p parameters[n]    | function value at end of last segment
         * \p parameters[n+1]  | first order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * ...                 | ...
         * \p parameters[2n]   | first order derivative at beginning of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[2n+1] | first order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         */
        virtual bool interpolatePiecewiseCubicFirstDerivatives(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            if (degree() < 3) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Checking parameter set
            if (parameters.size() < 4 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() / 2 - 1) || parameters.size() % 2 != 0) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size() / 2 - 1;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set their coefficients
            for (size_t i = 0; i < segmentCount; i++) {
                // Note on derivatives:
                /* --------------------
                 * The chain rule has to be respected using the proportion of the current segment (hi)!
                 * si(xi=0)           = yi
                 * si(xi=1)           = yip1
                 * dsi(z)/dz|xi=0     = dyi    --> dsi(xi)/dxi|xi=0     = dyi * hi
                 * dsi(z)/dz|xi=1     = dyip1  --> dsi(xi)/dxi|xi=1     = dyip1 * hi
                 */

                // Prepare indices (mapping in parameter array)
                const double& yi = parameters[i];
                const double& yip1 = parameters[i + 1];
                const double& hi = m_segmentProportions[i];
                double ddyiXhi = parameters[segmentCount + 1 + i] * hi; // ddyi * hi
                double ddyip1Xhi = parameters[segmentCount + 1 + i + 1] * hi; // ddyip1 * hi

                // Interpolate curve segment
                if (m_segments[i].interpolate(std::vector<double>{ yi, yip1, ddyiXhi, ddyip1Xhi }, CurveInterpolationMethod::POLYNOMIAL_CUBIC_FIRST_DERIVATIVES, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

        //! **Piecewise** interpolation with polynomials of **degree 3** (piecewise cubic) through given points and with given second order derivative at control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Piecewise** interpolation with polynomials of degree 3 (given second order derivatives).
         *  * The segments are piecewise cubic.
         *  * The segments are linked to their neighbors.
         *  * The spline is \f$ C^0 \f$ continuous.
         *  * The user can specify the second order derivatives (with respect to spline coordinates, **not** time in general!) at each control point.
         *
         * \par Parameter-layout
         * Element             | Description
         * -------             | -----------
         * \p parameters[0]    | function value at beginning of first segment
         * ...                 | ...
         * \p parameters[n-1]  | function value at beginning of last segment
         * \p parameters[n]    | function value at end of last segment
         * \p parameters[n+1]  | second order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * ...                 | ...
         * \p parameters[2n]   | second order derivative at beginning of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[2n+1] | second order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         */
        virtual bool interpolatePiecewiseCubicSecondDerivatives(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            if (degree() < 3) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Checking parameter set
            if (parameters.size() < 4 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() / 2 - 1) || parameters.size() % 2 != 0) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size() / 2 - 1;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set their coefficients
            for (size_t i = 0; i < segmentCount; i++) {
                // Note on derivatives:
                /* --------------------
                 * The chain rule has to be respected using the proportion of the current segment (hi)!
                 * si(xi=0)           = yi
                 * si(xi=1)           = yip1
                 * dsi(z)/dz|xi=0     = dyi    --> dsi(xi)/dxi|xi=0     = dyi * hi
                 * dsi(z)/dz|xi=1     = dyip1  --> dsi(xi)/dxi|xi=1     = dyip1 * hi
                 * d^2si(z)/dz^2|xi=0 = ddyi   --> d^2si(xi)/dxi^2|xi=0 = ddyi * hi^2
                 * d^2si(z)/dz^2|xi=1 = ddyip1 --> d^2si(xi)/dxi^2|xi=1 = ddyip1 * hi^2
                 */

                // Prepare indices (mapping in parameter array)
                const double& yi = parameters[i];
                const double& yip1 = parameters[i + 1];
                const double& hi = m_segmentProportions[i];
                double his = hi * hi; // hi^2 (squared)
                double ddyiXhis = parameters[segmentCount + 1 + i] * his; // ddyi * hi^2
                double ddyip1Xhis = parameters[segmentCount + 1 + i + 1] * his; // ddyip1 * hi^2

                // Interpolate curve segment
                if (m_segments[i].interpolate(std::vector<double>{ yi, yip1, ddyiXhis, ddyip1Xhis }, CurveInterpolationMethod::POLYNOMIAL_CUBIC_SECOND_DERIVATIVES, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

        //! **Piecewise** interpolation with polynomials of **degree 5** (piecewise quintic) through given points and with given first and second order derivatives at control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Piecewise** interpolation with polynomials of degree 5.
         *  * The segments are piecewise quintic.
         *  * The segments are linked to their neighbors.
         *  * The spline is \f$ C^2 \f$ continuous.
         *  * The user can specify the first and second order derivatives (with respect to spline coordinates, **not** time in general!) at each control point.
         *
         * \par Parameter-layout
         * Element             | Description
         * -------             | -----------
         * \p parameters[0]    | function value at beginning of first segment
         * ...                 | ...
         * \p parameters[n-1]  | function value at beginning of last segment
         * \p parameters[n]    | function value at end of last segment
         * \p parameters[n+1]  | first order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * ...                 | ...
         * \p parameters[2n]   | first order derivative at beginning of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[2n+1] | first order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[2n+2] | second order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * ...                 | ...
         * \p parameters[3n+1] | second order derivative at beginning of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[3n+2] | second order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         */
        virtual bool interpolatePiecewiseQuinticFirstAndSecondDerivatives(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            if (degree() < 5) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Checking parameter set
            if (parameters.size() < 6 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() / 3 - 1) || parameters.size() % 3 != 0) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size() / 3 - 1;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set their coefficients
            for (size_t i = 0; i < segmentCount; i++) {
                // Note on derivatives:
                /* --------------------
                 * The chain rule has to be respected using the proportion of the current segment (hi)!
                 * si(xi=0)           = yi
                 * si(xi=1)           = yip1
                 * dsi(z)/dz|xi=0     = dyi    --> dsi(xi)/dxi|xi=0     = dyi * hi
                 * dsi(z)/dz|xi=1     = dyip1  --> dsi(xi)/dxi|xi=1     = dyip1 * hi
                 * d^2si(z)/dz^2|xi=0 = ddyi   --> d^2si(xi)/dxi^2|xi=0 = ddyi * hi^2
                 * d^2si(z)/dz^2|xi=1 = ddyip1 --> d^2si(xi)/dxi^2|xi=1 = ddyip1 * hi^2
                 */

                // Prepare indices (mapping in parameter array)
                const double& yi = parameters[i];
                const double& yip1 = parameters[i + 1];
                const double& hi = m_segmentProportions[i];
                double his = hi * hi; // hi^2 (squared)
                double dyiXhi = parameters[segmentCount + 1 + i] * hi; // dyi * hi
                double dyip1Xhi = parameters[segmentCount + 1 + i + 1] * hi; // dyip1 * hi
                double ddyiXhis = parameters[2 * segmentCount + 2 + i] * his; // ddyi * hi^2
                double ddyip1Xhis = parameters[2 * segmentCount + 2 + i + 1] * his; // ddyip1 * hi^2

                // Interpolate curve segment
                if (m_segments[i].interpolate(std::vector<double>{ yi, yip1, dyiXhi, dyip1Xhi, ddyiXhis, ddyip1Xhis }, CurveInterpolationMethod::POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

#ifdef HAVE_EIGEN3
        //! **Smooth** interpolation (spline) with polynomials of **degree 3** (piecewise cubic) through given points, \f$ C^2 \f$ continuous and with given second order derivative at start and end of spline (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Smooth** interpolation with polynomials of degree 3 (given second order derivatives at start and end of spline).
         *  * The segments are piecewise cubic.
         *  * The segments are linked to their neighbors.
         *  * The spline is \f$ C^2 \f$ continuous.
         *  * The second order derivatives (with respect to *spline-coordinates*, **not** time in general!) at start and end are fixed to a given value.
         *
         * \par Parameter-layout
         * Element             | Description
         * -------             | -----------
         * \p parameters[0]    | function value at beginning of first segment
         * ...                 | ...
         * \p parameters[n-1]  | function value at beginning of last segment
         * \p parameters[n]    | function value at end of last segment
         * \p parameters[n+1]  | second order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[n+2]  | second order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         */
        virtual bool interpolateSmoothCubicSecondDerivatives(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            if (degree() < 3) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Checking parameter set
            if (parameters.size() < 4 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() - 3)) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size() - 3;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Check, if there is more than one segment
            if (segmentCount > 1) {
                // Problem:
                /* --------
                 * si(xi)          =   ai*xi^3 +   bi*xi^2 + ci*xi + di
                 * dsi(xi)/dxi     = 3*ai*xi^2 + 2*bi*xi   + ci
                 * d^2si(xi)/dxi^2 = 6*ai*xi   + 2*bi
                 *
                 * C0-continuity: (2*n Eqs.)
                 * (I)    si(xi=0)            = yi    (n Eqs.)
                 * (II)   si(xi=1)            = yip1  (n Eqs.)
                 *
                 * C1-continuity: (n-1 Eqs.)
                 * (III)  dsi(z)/dz|xi=1      = dsip1(z)/dz|xip1=0     --> 1/hi * dsi(xi)/dxi|xi=1 = 1/hip1 * dsip1(xip1)/dxip1|xip1=0
                 *
                 * C2-continuity: (n-1 Eqs.)
                 * (IV)   d^2si(z)/dz^2|xi=1  = d^2sip1(z)/dz^2|xip1=0 --> (1/hi)^2 * d^2si(xi)/dxi^2|xi=1 = (1/hip1)^2 * d^2sip1(xip1)/dxip1^2|xip1=0
                 *
                 * Boundary values: (2 Eqs.)
                 * (V)    d^2s0(z)/dz^2|z=0   = ddy0                   --> d^2s0(x0)/dx0^2|x0=0 = ddy0 * h0^2
                 * (VI)   d^2snm1(z)/dz^2|z=1 = ddyn                   --> d^2snm1(xnm1)/dxnm1^2|xnm1=1 = ddyn * hnm1^2
                 */

                // Solution:
                /* ---------
                 * ai = hi^2/6*(ddyip1 - ddyi)
                 * bi = hi^2/2*ddyi
                 * ci = yip1 - yi - hi^2/6*(ddyip1 + 2*ddyi)
                 * di = yi
                 *
                 * From C1-continuity: (and inserting ai, bi, ci, di)
                 * -------------------
                 * him1*ddyim1 + 2*(him1 + hi)*ddyi + hi*ddyip1 = 6/hi*(yip1 - yi) - 6/him1*(yi - yim1)  for all i=1...n-1
                 *
                 * (n-1) unknowns: [ddy1, ddy2, ... , ddynm1]
                 * (ddy0 and ddyn are the known boundary values)
                 *
                 * --> can be written into matrix form with tridiagonal matrix A:
                 *
                 * --------------------------------------------   --------   --------
                 * |  d1    u1                            0   |   |  x1  |   |  b1  |
                 * |  l2    d2    u2                          |   |  x2  |   |  b2  |
                 * |        l3    d3    u3                    |   |  x3  |   |  b3  |
                 * |              ..    ..    ..              |   |  ..  |   |  ..  |
                 * |              li    di    ui              | * |  xi  | = |  bi  |
                 * |                    ..    ..    ..        |   |  ..  |   |  ..  |
                 * |                         ln-2  dn-2  un-2 |   | xn-2 |   | bn-2 |
                 * |  0                            ln-1  dn-1 |   | xn-1 |   | bn-1 |
                 * --------------------------------------------   --------   --------
                 *                      |                            |          |
                 *                      A                       *    x     =    b
                 *                [(n-1)x(n-1)]                  [(n-1)x1]  [(n-1)x1]
                 *
                 * with x = [ ddy1, ddy2, ... , ddyi, ... , ddyn-1 ]^T as the unknown solution of the linear system of equations
                 */

                // Step 1: Setup linear system of equations
                // ----------------------------------------
                Eigen::VectorXd lowerDiagonalOfA(segmentCount - 1); // First element is "outside" the matrix A (to be ignored)
                Eigen::VectorXd diagonalOfA(segmentCount - 1);
                Eigen::VectorXd upperDiagonalOfA(segmentCount - 1); // Last element is "outside" the matrix A (to be ignored)
                Eigen::VectorXd b(segmentCount - 1);
                const double& ddy0 = parameters[segmentCount + 1]; // Second order derivative with respect to *spline-coordinates* at start of spline
                const double& ddyn = parameters[segmentCount + 2]; // Second order derivative with respect to *spline-coordinates* at end of spline
                const double& h0 = m_segmentProportions[0]; // Proportion of first segment
                const double& hnm1 = m_segmentProportions[segmentCount - 1]; // Proportion of last segment
                for (size_t k = 0; k < segmentCount - 1; k++) {
                    size_t i = k + 1; // Index shift is necessary, since we want to skip i=0 and comply with the previous **not**ation/meaning of index "i"

                    // Create references to current values
                    const double& yim1 = parameters[i - 1];
                    const double& yi = parameters[i];
                    const double& yip1 = parameters[i + 1];
                    const double& hi = m_segmentProportions[i];
                    const double& him1 = m_segmentProportions[i - 1];

                    // Compute lower diagonal values
                    if (i == 1)
                        lowerDiagonalOfA(k) = 0; // First element is "outside" the matrix A (to be ignored)
                    else
                        lowerDiagonalOfA(k) = him1;

                    // Compute diagonal values
                    diagonalOfA(k) = 2.0 * (him1 + hi);

                    // Compute upper diagonal values
                    if (i == segmentCount - 1)
                        upperDiagonalOfA(k) = 0; // Last element is "outside" the matrix A (to be ignored)
                    else
                        upperDiagonalOfA(k) = hi;

                    // Compute values in b
                    b(k) = 6.0 / hi * (yip1 - yi) - 6.0 / him1 * (yi - yim1);
                    if (i == 1)
                        b(k) -= h0 * ddy0;
                    if (i == segmentCount - 1)
                        b(k) -= hnm1 * ddyn;
                }

                // Step 2: Solve linear system of equations
                // ----------------------------------------
                Eigen::MatrixXd X(segmentCount - 1, 1); // Solution vector x = [ ddy1, ddy2, ... , ddyi, ... , ddyn-1 ]^T
                core::math::solveTriDiagonalSystemOfEquations(lowerDiagonalOfA, diagonalOfA, upperDiagonalOfA, b, X);

                // Step 3: Compute coefficients
                // ----------------------------
                for (size_t i = 0; i < segmentCount; i++) {
                    // Create references to current values
                    const double& yi = parameters[i];
                    const double& yip1 = parameters[i + 1];
                    const double& hi = m_segmentProportions[i];
                    const double his = hi * hi; // hi^2
                    double ddyi = ddy0;
                    if (i > 0)
                        ddyi = X(i - 1, 0);
                    double ddyip1 = ddyn;
                    if (i < segmentCount - 1)
                        ddyip1 = X(i, 0);

                    // Set coefficients ai, bi, ci, di
                    m_segments[i].m_coefficients[3] = his / 6.0 * (ddyip1 - ddyi);
                    m_segments[i].m_coefficients[2] = his / 2.0 * ddyi;
                    m_segments[i].m_coefficients[1] = yip1 - yi - his / 6.0 * (ddyip1 + 2.0 * ddyi);
                    m_segments[i].m_coefficients[0] = yi;
                }

                // Force higher terms to be zero (in case underlying polynomial is of higher degree)
                if (degree() > 3)
                    for (size_t i = 0; i < segmentCount; i++)
                        for (size_t j = 4; j < m_segments[i].m_coefficients.size(); j++)
                            m_segments[i].m_coefficients[j] = 0;
            } else {
                // ...only one segment -> same solution as for piecewise cubic spline (see above for details)

                // Prepare indices (mapping in parameter array)
                const double& y0 = parameters[0];
                const double& y0p1 = parameters[1];
                const double& h0 = m_segmentProportions[0];
                double h0s = h0 * h0; // h0^2 (squared)
                double ddy0Xh0s = parameters[2] * h0s; // ddy0 * h0^2
                double ddy0p1Xh0s = parameters[3] * h0s; // ddy0p1 * h0^2

                // Interpolate curve segment
                if (m_segments[0].interpolate(std::vector<double>{ y0, y0p1, ddy0Xh0s, ddy0p1Xh0s }, CurveInterpolationMethod::POLYNOMIAL_CUBIC_SECOND_DERIVATIVES, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }

        //! **Smooth** interpolation (spline) with polynomials of **degree 5** (piecewise quintic) through given points, \f$ C^4 \f$ continuous and with given first and second order derivative at start and end of spline (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * **Smooth** interpolation with polynomials of degree 5 (given first and second order derivatives at start and end of spline).
         *  * The segments are piecewise quintic.
         *  * The segments are linked to their neighbors.
         *  * The spline is \f$ C^4 \f$ continuous.
         *  * The first and second order derivatives (with respect to *spline-coordinates*, **not** time in general!) at start and end are fixed to a given value.
         *
         * \par Parameter-layout
         * Element             | Description
         * -------             | -----------
         * \p parameters[0]    | function value at beginning of first segment
         * ...                 | ...
         * \p parameters[n-1]  | function value at beginning of last segment
         * \p parameters[n]    | function value at end of last segment
         * \p parameters[n+1]  | first order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[n+2]  | first order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[n+3]  | second order derivative at beginning of first segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         * \p parameters[n+4]  | second order derivative at end of last segment (derivative with respect to *spline-coordinates*, **not** time in general!)
         */
        virtual bool interpolateSmoothQuinticFirstAndSecondDerivatives(const std::vector<double>& parameters, const std::vector<double>* const proportions, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            CurveResult curveResult = CurveResult::UNKNOWN;

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

            // Check curve type
            if (degree() < 5) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Checking parameter set
            if (parameters.size() < 6 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() - 5)) {
                if (result != nullptr)
                    *result = SplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            size_t segmentCount = parameters.size() - 5;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Check, if there is more than one segment
            if (segmentCount > 1) {
                // Problem:
                /* --------
                 * si(xi)          =     ai*xi^5 +    bi*xi^4 +   ci*xi^3 +   di*xi^2 + ei*xi^1 + fi
                 * dsi(xi)/dxi     =   5*ai*xi^4 +  4*bi*xi^3 + 3*ci*xi^2 + 2*di*xi^1 + ei
                 * d^2si(xi)/dxi^2 =  20*ai*xi^3 + 12*bi*xi^2 + 6*ci*xi^1 + 2*di
                 * d^3si(xi)/dxi^3 =  60*ai*xi^2 + 24*bi*xi^1 + 6*ci
                 * d^4si(xi)/dxi^4 = 120*ai*xi^1 + 24*bi
                 *
                 * C0-continuity: (2*n Eqs.)
                 * (I)    si(xi=0)            = yi    (n Eqs.)
                 * (II)   si(xi=1)            = yip1  (n Eqs.)
                 *
                 * C1-continuity: (n-1 Eqs.)
                 * (III)  dsi(z)/dz|xi=1      = dsip1(z)/dz|xip1=0     --> 1/hi * dsi(xi)/dxi|xi=1 = 1/hip1 * dsip1(xip1)/dxip1|xip1=0
                 *
                 * C2-continuity: (n-1 Eqs.)
                 * (IV)   d^2si(z)/dz^2|xi=1  = d^2sip1(z)/dz^2|xip1=0 --> (1/hi)^2 * d^2si(xi)/dxi^2|xi=1 = (1/hip1)^2 * d^2sip1(xip1)/dxip1^2|xip1=0
                 *
                 * C3-continuity: (n-1 Eqs.)
                 * (V)    d^3si(z)/dz^3|xi=1  = d^3sip1(z)/dz^3|xip1=0 --> (1/hi)^3 * d^3si(xi)/dxi^3|xi=1 = (1/hip1)^3 * d^3sip1(xip1)/dxip1^3|xip1=0
                 *
                 * C4-continuity: (n-1 Eqs.)
                 * (VI)   d^4si(z)/dz^4|xi=1  = d^4sip1(z)/dz^4|xip1=0 --> (1/hi)^4 * d^4si(xi)/dxi^4|xi=1 = (1/hip1)^4 * d^4sip1(xip1)/dxip1^4|xip1=0
                 *
                 * Boundary values: (4 Eqs.)
                 * (VII)  ds0(z)/dz|z=0   = dy0                        --> ds0(x0)/dx0|x0=0 = dy0 * h0
                 * (VIII) dsnm1(z)/dz|z=1 = dyn                        --> dsnm1(xnm1)/dxnm1|xnm1=1 = dyn * hnm1
                 * (IX)   d^2s0(z)/dz^2|z=0   = ddy0                   --> d^2s0(x0)/dx0^2|x0=0 = ddy0 * h0^2
                 * (X)    d^2snm1(z)/dz^2|z=1 = ddyn                   --> d^2snm1(xnm1)/dxnm1^2|xnm1=1 = ddyn * hnm1^2
                 */

                // Solution:
                /* ---------
                 * ai = - 6*yi +  6*yip1 - 3*dyi*hi - 3*dyip1*hi - 0.5*ddyi*hi^2 + 0.5*ddyip1*hi^2
                 * bi =  15*yi - 15*yip1 + 8*dyi*hi + 7*dyip1*hi + 1.5*ddyi*hi^2 -     ddyip1*hi^2
                 * ci = -10*yi + 10*yip1 - 6*dyi*hi - 4*dyip1*hi - 1.5*ddyi*hi^2 + 0.5*ddyip1*hi^2
                 * di =                                            0.5*ddyi*hi^2
                 * ei =                      dyi*hi
                 * fi =     yi
                 *
                 * From C4-continuity: (and inserting ai, bi, ci, di, ei, fi)
                 * -------------------
                 * -56*dyim1*gim1^3 - 8*ddyim1*gim1^2 - 64*dyi*(gim1^3 + gi^3) + 12*ddyi*(gim1^2 - gi^2) -56*dyip1*gi^3 + 8*ddyip1*gi^2 = -120*gi^4*(yip1 - yi) - 120*gim1^4*(yi - yim1)   for all i=1...n-1
                 * (1st line in blocks of linear system of equations below)
                 *
                 * From C3-continuity: (and inserting ai, bi, ci, di, ei, fi)
                 * -------------------
                 * -8*dyim1*gim1^2 - ddyim1*gim1 - 12*dyi*(gim1^2 - gi^2) + 3*ddyi*(gim1 + gi) + 8*dyip1*gi^2 - ddyip1*gi = 20*gi^3*(yip1 - yi) - 20*gim1^3*(yi - yim1)   for all i=1...n-1
                 * (2nd line in blocks of linear system of equations below)
                 *
                 * 2*(n-1) unknowns: [dy1, ddy1, dy2, ddy2, ... , dynm1, ddynm1]
                 * (dy0, ddy0, dyn and ddyn are the known boundary values)
                 *
                 * --> can be written into matrix form with block-tridiagonal matrix A:
                 *
                 * --------------------------------------------   --------   --------
                 * |  D1    U1                            0   |   |  x1  |   |  b1  |
                 * |  L2    D2    U2                          |   |  x2  |   |  b2  |
                 * |        L3    D3    U3                    |   |  x3  |   |  b3  |
                 * |              ..    ..    ..              |   |  ..  |   |  ..  |
                 * |              Li    Di    Ui              | * |  xi  | = |  bi  |
                 * |                    ..    ..    ..        |   |  ..  |   |  ..  |
                 * |                         Ln-2  Dn-2  Un-2 |   | xn-2 |   | bn-2 |
                 * |  0                            Ln-1  Dn-1 |   | xn-1 |   | bn-1 |
                 * --------------------------------------------   --------   --------
                 *                      |                            |          |
                 *                      A                       *    x     =    b
                 *               [2(n-1)x2(n-1)]                [2(n-1)x1] [2(n-1)x1]
                 *
                 * with x = [ -dy1, 1/(alpha*g1)*ddy1, -dy2, 1/(alpha*g2)*ddy2, ... , -dyi, 1/(alpha*gi)*ddyi, ... , -dyn-1, 1/(alpha*gn-1)*ddyn-1 ]^T as the unknown solution of the linear system of equations
                 */

                // Step 1: Setup linear system of equations
                // ----------------------------------------
                const double alpha = sqrt(64.0 / 3.0); // Parameter to improve numerical stability (see Mund1975, p. 280)
                const double alphasquared = alpha * alpha; // alpha^2
                std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> lowerDiagonalOfA;
                lowerDiagonalOfA.resize(segmentCount - 1); // First element is "outside" the matrix A (to be ignored)
                std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> diagonalOfA;
                diagonalOfA.resize(segmentCount - 1);
                std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> upperDiagonalOfA;
                upperDiagonalOfA.resize(segmentCount - 1); // Last element is "outside" the matrix A (to be ignored)
                std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> b;
                b.resize(segmentCount - 1);
                const double& dy0 = parameters[segmentCount + 1]; // First order derivative with respect to *spline-coordinates* at start of spline
                const double& dyn = parameters[segmentCount + 2]; // First order derivative with respect to *spline-coordinates* at end of spline
                const double& ddy0 = parameters[segmentCount + 3]; // Second order derivative with respect to *spline-coordinates* at start of spline
                const double& ddyn = parameters[segmentCount + 4]; // Second order derivative with respect to *spline-coordinates* at end of spline
                const double& g0 = 1.0 / m_segmentProportions[0]; // Proportion of first segment
                const double g0s = g0 * g0; // g0^2 (squared)
                const double g0c = g0s * g0; // g0^3 (cubic)
                const double& gnm1 = 1.0 / m_segmentProportions[segmentCount - 1]; // Proportion of last segment
                const double gnm1s = gnm1 * gnm1; // gnm1^2 (squared)
                const double gnm1c = gnm1s * gnm1; // gnm1^3 (cubic)
                double gi = g0;
                double gis = g0s;
                double gic = g0c;
                double giq = gic * gi; // gi^4 (quartic)
                for (size_t k = 0; k < segmentCount - 1; k++) {
                    size_t i = k + 1; // Index shift is necessary, since we want to skip i=0 and comply with the previous **not**ation/meaning of index "i"

                    // Create references to current values
                    const double& yim1 = parameters[i - 1];
                    const double& yi = parameters[i];
                    const double& yip1 = parameters[i + 1];
                    double gim1 = gi;
                    double gim1s = gis; // gim1^2 (squared)
                    double gim1c = gic; // gim1^3 (cubic)
                    double gim1q = giq; // gim1^4 (quartic)
                    gi = 1.0 / m_segmentProportions[i];
                    gis = gi * gi; // gi^2 (squared)
                    gic = gis * gi; // gi^3 (cubic)
                    giq = gic * gi; // gi^4 (quartic)

                    // Compute upper diagonal values
                    if (i == segmentCount - 1)
                        upperDiagonalOfA[k] = Eigen::Matrix2d::Zero(); // Last element is "outside" the matrix A (to be ignored)
                    else {
                        const double gip1 = 1.0 / m_segmentProportions[i + 1];

                        upperDiagonalOfA[k](0, 0) = 56.0 * gic;
                        upperDiagonalOfA[k](0, 1) = 8.0 * alpha * gis * gip1;
                        upperDiagonalOfA[k](1, 0) = -8.0 * alpha * gic;
                        upperDiagonalOfA[k](1, 1) = -alphasquared * gis * gip1;
                    }

                    // Compute diagonal values
                    diagonalOfA[k](0, 0) = 64.0 * (gim1c + gic);
                    diagonalOfA[k](0, 1) = 12.0 * alpha * (gim1s - gis) * gi;
                    diagonalOfA[k](1, 0) = diagonalOfA[k](0, 1);
                    diagonalOfA[k](1, 1) = 3.0 * alphasquared * (gim1 + gi) * gis;

                    // Compute lower diagonal values
                    if (i == 1)
                        lowerDiagonalOfA[k] = Eigen::Matrix2d::Zero(); // First element is "outside" the matrix A (to be ignored)
                    else
                        lowerDiagonalOfA[k] = upperDiagonalOfA[k - 1].transpose();

                    // Compute values in b
                    b[k](0) = -120.0 * (giq * (yip1 - yi) + gim1q * (yi - yim1));
                    b[k](1) = 20.0 * alpha * (giq * (yip1 - yi) - gim1c * gi * (yi - yim1));
                    if (i == 1) {
                        b[k](0) += 56.0 * g0c * dy0 + 8.0 * g0s * ddy0;
                        b[k](1) += 8.0 * alpha * g0s * gi * dy0 + alpha * g0 * gi * ddy0;
                    }
                    if (i == segmentCount - 1) {
                        b[k](0) += 56.0 * gnm1c * dyn - 8.0 * gnm1s * ddyn;
                        b[k](1) += -8.0 * alpha * gnm1c * dyn + alpha * gnm1s * ddyn;
                    }
                }

                // Step 2: Solve linear system of equations
                // ----------------------------------------
                std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> x; // Solution vector x = [ -dy1, 1/(alpha*g1)*ddy1, -dy2, 1/(alpha*g2)*ddy2, ... , -dyi, 1/(alpha*gi)*ddyi, ... , -dyn-1, 1/(alpha*gn-1)*ddyn-1 ]^T
                x.resize(segmentCount - 1);
                core::math::solveBlockTriDiagonalSystemOfEquations<Eigen::Matrix2d, Eigen::Vector2d>(lowerDiagonalOfA, diagonalOfA, upperDiagonalOfA, b, x);

#ifndef NDEBUG
                // Intermediate step: compute numerical error
                // ------------------------------------------
                // Compute total error made during solving linear system of equations
                double totalError = 0;
                for (size_t k = 0; k < segmentCount - 1; k++) {
                    size_t i = k + 1; // Index shift is necessary, since we want to skip i=0 and comply with the previous **not**ation/meaning of index "i"

                    // Compute bi by matrix vector multiplication
                    Eigen::Vector2d bi(0, 0);
                    if (i > 1)
                        bi += lowerDiagonalOfA[k] * x[k - 1];
                    bi += diagonalOfA[k] * x[k];
                    if (i < segmentCount - 1)
                        bi += upperDiagonalOfA[k] * x[k + 1];

                    // Compute difference between bi of left hand side and bi of right hand side
                    Eigen::Vector2d delta = bi - b[k];
                    totalError += delta.norm();
                }

                // Output warning
                if (totalError > 1e-6)
                    if (result != nullptr)
                        *result = SplineResult::WARNING_HIGH_NUMERICAL_ERROR;
#endif // NDEBUG

                // Step 3: Transform solution
                // --------------------------
                // Transform from x -> eta
                // with   x = [ -dy1, 1/(alpha*g1)*ddy1, -dy2, 1/(alpha*g2)*ddy2, ... , -dyi, 1/(alpha*gi)*ddyi, ... , -dyn-1, 1/(alpha*gn-1)*ddyn-1 ]^T
                //          = [ x1, x2, ... , xi, ... , xn-1]^T
                //      eta = [ dy1, ddy1, dy2, ddy2, ... , dyi, ddyi, ... , dyn-1, ddyn-1 ]^T
                std::vector<double> eta;
                eta.resize(2.0 * x.size());
                for (size_t i = 0; i < segmentCount - 1; i++) {
                    eta[2 * i] = -x[i](0);
                    eta[2 * i + 1] = alpha * x[i](1) / m_segmentProportions[i + 1];
                }

                // Step 4: Compute coefficients
                // ----------------------------
                for (size_t i = 0; i < segmentCount; i++) {
                    // Create references to current values
                    const double& yi = parameters[i];
                    const double& yip1 = parameters[i + 1];
                    const double& hi = m_segmentProportions[i];
                    const double his = hi * hi; // hi^2
                    double dyiXhi = dy0 * hi; // dyi * hi
                    double ddyiXhis = ddy0 * his; // ddyi * hi^2
                    if (i > 0) {
                        dyiXhi = eta[2 * (i - 1)] * hi;
                        ddyiXhis = eta[2 * (i - 1) + 1] * his;
                    }
                    double dyip1Xhi = dyn * hi; // dyip1 * hi
                    double ddyip1Xhis = ddyn * his; // ddyip1 * hi^2
                    if (i < segmentCount - 1) {
                        dyip1Xhi = eta[2 * i] * hi;
                        ddyip1Xhis = eta[2 * i + 1] * his;
                    }

                    // Set coefficients ai, bi, ci, di, ei, fi
                    m_segments[i].m_coefficients[5] = -6.0 * yi + 6.0 * yip1 - 3.0 * dyiXhi - 3.0 * dyip1Xhi - 0.5 * ddyiXhis + 0.5 * ddyip1Xhis;
                    m_segments[i].m_coefficients[4] = 15.0 * yi - 15.0 * yip1 + 8.0 * dyiXhi + 7.0 * dyip1Xhi + 1.5 * ddyiXhis - ddyip1Xhis;
                    m_segments[i].m_coefficients[3] = -10.0 * yi + 10.0 * yip1 - 6.0 * dyiXhi - 4.0 * dyip1Xhi - 1.5 * ddyiXhis + 0.5 * ddyip1Xhis;
                    m_segments[i].m_coefficients[2] = 0.5 * ddyiXhis;
                    m_segments[i].m_coefficients[1] = dyiXhi;
                    m_segments[i].m_coefficients[0] = yi;
                }

                // Force higher terms to be zero (in case underlying polynomial is of higher degree)
                if (degree() > 5)
                    for (size_t i = 0; i < segmentCount; i++)
                        for (size_t j = 6; j < m_segments[i].m_coefficients.size(); j++)
                            m_segments[i].m_coefficients[j] = 0;
            } else {
                // ...only one segment -> same solution as for piecewise quintic spline (see above for details)

                // Prepare indices (mapping in parameter array)
                const double& y0 = parameters[0];
                const double& y1 = parameters[1];
                const double& h0 = m_segmentProportions[0];
                double h0s = h0 * h0; // h0^2 (squared)
                double dy0Xh0 = parameters[2] * h0; // dy0 * h0
                double dy1Xh0 = parameters[3] * h0; // dy1 * h0
                double ddy0Xh0s = parameters[4] * h0s; // ddy0 * h0^2
                double ddy1Xh0s = parameters[5] * h0s; // ddy1 * h0^2

                // Interpolate curve segment
                if (m_segments[0].interpolate(std::vector<double>{ y0, y1, dy0Xh0, dy1Xh0, ddy0Xh0s, ddy1Xh0s }, CurveInterpolationMethod::POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES, &curveResult) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }
            }

            // Success!
            if (result != nullptr)
                *result = SplineResult::SUCCESS;
            return true;
        }
#endif // HAVE_EIGEN3

        // Split/Join
        // ----------
    public:
        //! Computes a (single) target segment (in target spline) which is equivalent to the the given source segment(s) (in source spline(s))
        /*!
         * The target segment is computed by evaluating the given source segment(s) at the "beginning" and "end" of the target segment.
         *
         * \attention If the \p beginSourceSegment differs from \p endSourceSegment (may occur during joining only), it is assumed, that both segments
         * are neighbors and connected smoothly (highest possible smoothness). Otherwise it can not be guaranteed, that the resulting \p targetSegment
         * is equivalent to the input.
         *
         * \param [in] beginSourceSegment Reference to source spline segment used to compute the "beginning" of the target segment
         * \param [in] beginSourceSegmentDuration Duration of \p beginSourceSegment
         * \param [in] beginSourceSegmentPosition Position of "beginning" of target segment in \p beginSourceSegment
         * \param [in] endSourceSegment Reference to source spline segment used to compute the "end" of the target segment
         * \param [in] endSourceSegmentDuration Duration of \p endSourceSegment
         * \param [in] endSourceSegmentPosition Position of "end" of target segment in \p endSourceSegment
         * \param [out] targetSegment Reference to target segment to be computed
         * \param [out] targetSegmentDuration Duration of \p targetSegment
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool computeTargetSegmentForSplitJoin(const PolynomialCurve<Degree>& beginSourceSegment, const double& beginSourceSegmentDuration, const double& beginSourceSegmentPosition, const PolynomialCurve<Degree>& endSourceSegment, const double& endSourceSegmentDuration, const double& endSourceSegmentPosition, PolynomialCurve<Degree>& targetSegment, const double& targetSegmentDuration, SplineResult* const result = nullptr)
        {
            // Initialize helpers
            const double targetSegmentDurationSquared = targetSegmentDuration * targetSegmentDuration;

            // Select interpolation method depending on degree of polynomial
            switch (degree()) {
            case 0: {
                // Piecewise constant interpolation

                // Compute left boundary
                const double y0 = beginSourceSegment.evaluate(beginSourceSegmentPosition, 0);

                // Interpolate
                if (targetSegment.interpolate(std::vector<double>{ y0 }, CurveInterpolationMethod::POLYNOMIAL_CONSTANT) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }

                // Success!
                if (result != nullptr)
                    *result = SplineResult::SUCCESS;
                return true;
            }
            case 1: {
                // Piecewise linear interpolation

                // Compute left boundary
                const double y0 = beginSourceSegment.evaluate(beginSourceSegmentPosition, 0);

                // Compute right boundary
                const double y1 = endSourceSegment.evaluate(endSourceSegmentPosition, 0);

                // Interpolate
                if (targetSegment.interpolate(std::vector<double>{ y0, y1 }, CurveInterpolationMethod::POLYNOMIAL_LINEAR) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }

                // Success!
                if (result != nullptr)
                    *result = SplineResult::SUCCESS;
                return true;
            }
            case 3: {
                // Piecewise cubic interpolation

                // Compute left boundary
                const std::array<double, 2> Diy0 = beginSourceSegment.template evaluateD0ToDN<1>(beginSourceSegmentPosition);
                const double& y0 = Diy0[0];
                double alpha0 = 0;
                if (beginSourceSegmentDuration > 0)
                    alpha0 = 1.0 / beginSourceSegmentDuration;
                const double dy0 = Diy0[1] * alpha0 * targetSegmentDuration; // Derivative relative to target segment coordinates

                // Compute right boundary
                const std::array<double, 2> Diy1 = endSourceSegment.template evaluateD0ToDN<1>(endSourceSegmentPosition);
                const double& y1 = Diy1[0];
                double alpha1 = 0;
                if (endSourceSegmentDuration > 0)
                    alpha1 = 1.0 / endSourceSegmentDuration;
                const double dy1 = Diy1[1] * alpha1 * targetSegmentDuration; // Derivative relative to target segment coordinates

                // Interpolate
                if (targetSegment.interpolate(std::vector<double>{ y0, y1, dy0, dy1 }, CurveInterpolationMethod::POLYNOMIAL_CUBIC_FIRST_DERIVATIVES) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }

                // Success!
                if (result != nullptr)
                    *result = SplineResult::SUCCESS;
                return true;
            }
            case 5: {
                // Piecewise quintic interpolation

                // Compute left boundary
                const std::array<double, 3> Diy0 = beginSourceSegment.template evaluateD0ToDN<2>(beginSourceSegmentPosition);
                const double& y0 = Diy0[0];
                double alpha0 = 0;
                if (beginSourceSegmentDuration > 0)
                    alpha0 = 1.0 / beginSourceSegmentDuration;
                const double alpha0Squared = alpha0 * alpha0;
                const double dy0 = Diy0[1] * alpha0 * targetSegmentDuration; // Derivative relative to target segment coordinates
                const double ddy0 = Diy0[2] * alpha0Squared * targetSegmentDurationSquared; // Derivative relative to target segment coordinates

                // Compute right boundary
                const std::array<double, 3> Diy1 = endSourceSegment.template evaluateD0ToDN<2>(endSourceSegmentPosition);
                const double& y1 = Diy1[0];
                double alpha1 = 0;
                if (endSourceSegmentDuration > 0)
                    alpha1 = 1.0 / endSourceSegmentDuration;
                const double alpha1Squared = alpha1 * alpha1;
                const double dy1 = Diy1[1] * alpha1 * targetSegmentDuration; // Derivative relative to target segment coordinates
                const double ddy1 = Diy1[2] * alpha1Squared * targetSegmentDurationSquared; // Derivative relative to target segment coordinates

                // Interpolate
                if (targetSegment.interpolate(std::vector<double>{ y0, y1, dy0, dy1, ddy0, ddy1 }, CurveInterpolationMethod::POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES) == false) {
                    if (result != nullptr)
                        *result = SplineResult::ERROR_CURVE_INTERPOLATION_FAILED;
                    assert(false);
                    return false;
                }

                // Success!
                if (result != nullptr)
                    *result = SplineResult::SUCCESS;
                return true;
            }
            default: {
                // No appropriate interpolation method for this specific degree implemented -> we can not interpolate without loosing information -> error
                if (result != nullptr)
                    *result = SplineResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }
            }
        }
    };
} // namespace curve
} // namespace broccoli
