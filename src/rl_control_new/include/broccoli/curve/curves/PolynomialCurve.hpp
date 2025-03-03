/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "InterpolatableCurve.hpp"

namespace broccoli {
namespace curve {
    //! Template class for polynomials. \f$ f(x) = c_0 + c_1\,x + c_2\,x^2 + \dots + c_{p}\,x^p\f$
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \tparam Degree (Maximum) degree \f$ p \geq 0 \f$
     *
     * \warning Note that there is a difference between the terms *degree* \f$ p \f$ and *order* \f$ k \f$ (\f$ k = p + 1 \f$).
     *
     * \remark One may use \p Degree as highest possible degree (contains all lower degree polynomials).
     */
    template <unsigned int Degree>
    class PolynomialCurve : public Curve, public InterpolatableCurve {
    public:
        //! Default constructor
        /*! Initializes to \f$ f(x) = 0 + 0\cdot x + 0\cdot x^2 + \dots + 0\cdot x^p = 0 \f$ */
        PolynomialCurve()
            : m_coefficients({ { 0 } }) // Initialize all coefficients with zero
        {
        }

        //! Destructor
        virtual ~PolynomialCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const PolynomialCurve<Degree>& reference) const
        {
            // Compare base class
            if (Curve::operator!=(reference))
                return false;

            // Compare coefficients
            if (m_coefficients != reference.m_coefficients)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const PolynomialCurve<Degree>& reference) const { return !(*this == reference); }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::POLYNOMIAL; }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of polynomial
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of polynomial
        static unsigned int order() { return Degree + 1; }

        // Member data
        std::array<double, Degree + 1> m_coefficients; //!< Coefficients of polynomial (\f$ c_i\f$ corresponds to `m_coefficients[i]`)

        // Evaluation of (derivative of) underlying function (see base class for details)
        /*!
         * \copydoc Curve::evaluate()
         *
         * \remark Internally uses Horner's method for maximum efficiency.
         *
         * References
         * ----------
         * * Higham, Nicolas J., "Accuracy and Stability of Numerical Algorithms", p.94ff, Society for Industrial and Applied Mathematics, 2002, Philadelphia, PA, USA, ISBN: 0898715210
         */
        virtual double evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Initialize return value
            double returnValue = 0;

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                // ...yes, only base function -> apply Horner's Method [Higham 2002, p.94]
                returnValue = m_coefficients[Degree];
                for (int i = Degree - 1; i >= 0; i--)
                    returnValue = position * returnValue + m_coefficients[i];
            } else {
                // ...no, derivative is requested -> check, if derivation order is higher than the degree of the polynomial
                if (derivationOrder <= Degree) {
                    // ...no -> compute modified coefficients (exponent "moves down" to become a pre-factor)
                    // (Note: as we want to compute ONLY the k-th derivative (and not all up to the k-th derivative) we simply
                    // compute the pre-factors "manually" instead of using the method given in [Higham 2002, p.96])
                    std::array<double, Degree + 1> coefficients = m_coefficients;
                    for (unsigned int i = derivationOrder; i <= Degree; i++)
                        for (unsigned int j = 0; j < derivationOrder; j++)
                            coefficients[i] = (i - j) * coefficients[i];

                    // Apply Horner's Method (as above, but only to the remaining terms)
                    returnValue = coefficients[Degree];
                    for (int i = (int)Degree - 1; i >= (int)derivationOrder; i--)
                        returnValue = position * returnValue + coefficients[i];
                }
                // else: yes -> everything vanishes -> keep zero as result
            }

            // Pass back return value
            return returnValue;
        }

        // Evaluation of value (D0) up to N-th derivative (DN) of underlying function (see base class for details)
        /*!
         * \copydoc Curve::evaluateD0ToDN()
         *
         * \remark Internally uses Horner's method for maximum efficiency.
         *
         * References
         * ----------
         * * Higham, Nicolas J., "Accuracy and Stability of Numerical Algorithms", p.94ff, Society for Industrial and Applied Mathematics, 2002, Philadelphia, PA, USA, ISBN: 0898715210
         */
        template <unsigned int N>
        std::array<double, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Validity check
            assert(isValid());

            // Initialize return value
            std::array<double, N + 1> Di;
            Di.fill(0);

            // Algorithm 5.2 from [Higham 2002, p.97]
            Di[0] = m_coefficients[Degree];
            for (int j = ((int)Degree) - 1; j >= 0; j--) {
                for (int i = std::min((int)N, ((int)Degree) - j); i >= 1; i--)
                    Di[i] = position * Di[i] + Di[i - 1];
                Di[0] = position * Di[0] + m_coefficients[j];
            }
            int m = 1;
            for (int j = 2; j <= (int)N; j++) {
                m = m * j;
                Di[j] = m * Di[j];
            }

            // Pass back list
            return Di;
        }

        // Interpolation
        // -------------
        // Interpolate coefficients of underlying function to fit specific parameters (see base class for details)
        //! \copydoc InterpolatableCurve::interpolate()
        /*!
         * \remark Automatically checks, if degree of polynomial is high enough. If the degree of the polynomial is higher
         * than necessary, the remaining coefficients are set to zero automatically.
         */
        virtual bool interpolate(const std::vector<double>& parameters, const CurveInterpolationMethod& method, CurveResult* const result = nullptr)
        {
            // Check interpolation method
            switch (method) {
            case CurveInterpolationMethod::POLYNOMIAL_CONSTANT: {
                /*! Interpolation with POLYNOMIAL_CONSTANT
                 *  --------------------------------------
                 * \copydetails interpolateConstant()
                 */
                return interpolateConstant(parameters, result);
            }
            case CurveInterpolationMethod::POLYNOMIAL_LINEAR: {
                /*! Interpolation with POLYNOMIAL_LINEAR
                 *  ------------------------------------
                 * \copydetails interpolateLinear()
                 */
                return interpolateLinear(parameters, result);
            }
            case CurveInterpolationMethod::POLYNOMIAL_CUBIC_FIRST_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_CUBIC_FIRST_DERIVATIVES
                 *  -----------------------------------------------------
                 * \copydetails interpolateCubicFirstDerivatives()
                 */
                return interpolateCubicFirstDerivatives(parameters, result);
            }
            case CurveInterpolationMethod::POLYNOMIAL_CUBIC_SECOND_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_CUBIC_SECOND_DERIVATIVES
                 *  ------------------------------------------------------
                 * \copydetails interpolateCubicSecondDerivatives()
                 */
                return interpolateCubicSecondDerivatives(parameters, result);
            }
            case CurveInterpolationMethod::POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES: {
                /*! Interpolation with POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES
                 *  ------------------------------------------------------------------
                 * \copydetails interpolateQuinticFirstAndSecondDerivatives()
                 */
                return interpolateQuinticFirstAndSecondDerivatives(parameters, result);
            }
            default: {
                // No interpolation methods implemented for now
                if (result != nullptr)
                    *result = CurveResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }
            }
        }

    protected:
        //! Constant polynomial interpolation (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters and \p result)
        /*!
         * Set coefficients to fit a polynomial of degree zero (constant) with given function value.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | `y0` Function value \f$ y_0 = y(x=0) \f$
         *
         * \par Problem
         * -# \f$ y(x) = a \f$
         * -# \f$ y(x=0) = y(x=1) = y_0 \f$
         *
         * \par Solution
         * - \f$ a = y_0 \f$
         */
        virtual bool interpolateConstant(const std::vector<double>& parameters, CurveResult* const result = nullptr)
        {
            // Check if degree is sufficient
            // ...not necessary here, since it has to be at least of order zero (this is guaranteed anyway)

            // Check, if parameters are specified correctly
            if (parameters.size() != 1) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Set coefficients
            m_coefficients[0] = parameters[0];

            // Force higher terms to be zero (in case polynomial is of higher degree)
            for (size_t i = 1; i < m_coefficients.size(); i++)
                m_coefficients[i] = 0;

            // Success!
            if (result != nullptr)
                *result = CurveResult::SUCCESS;
            return true;
        }

        //! Linear polynomial interpolation (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters and \p result)
        /*!
         * Set coefficients to fit a polynomial of degree one (linear) with given function values at \f$ x=0 \f$ and \f$ x=1 \f$.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | `y0` Function value \f$ y_0 = y(x=0) \f$
         * \p parameters[1]   | `y1` Function value \f$ y_1 = y(x=1) \f$
         *
         * \par Problem
         * -# \f$ y(x) = a\,x + b \f$
         * -# \f$ y(x=0) = y_0 \f$
         * -# \f$ y(x=1) = y_1 \f$
         *
         * \par Solution
         * - \f$ a = -y_0 + y_1 \f$
         * - \f$ b = y_0 \f$
         */
        virtual bool interpolateLinear(const std::vector<double>& parameters, CurveResult* const result = nullptr)
        {
            // Check if degree is sufficient
            if (degree() < 1) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Check, if parameters are specified correctly
            if (parameters.size() != 2) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Set coefficients
            m_coefficients[1] = -parameters[0] + parameters[1];
            m_coefficients[0] = parameters[0];

            // Force higher terms to be zero (in case polynomial is of higher degree)
            for (size_t i = 2; i < m_coefficients.size(); i++)
                m_coefficients[i] = 0;

            // Success!
            if (result != nullptr)
                *result = CurveResult::SUCCESS;
            return true;
        }

        //! Cubic polynomial interpolation with given first derivatives (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters and \p result)
        /*!
         * Set coefficients to fit a polynomial of degree three (cubic) with given function values and first order derivatives at \f$ x=0 \f$ and \f$ x=1 \f$.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | `y0` Function value \f$ y_0 = y(x=0) \f$
         * \p parameters[1]   | `y1` Function value \f$ y_1 = y(x=1) \f$
         * \p parameters[2]   | `dy0` First order derivative \f$ y'_0 = y'(x=0) \f$ with respect to \f$ x \f$
         * \p parameters[3]   | `dy1` First order derivative \f$ y'_1 = y'(x=1) \f$ with respect to \f$ x \f$
         *
         * \par Problem
         * -# \f$ y(x) = a\,x^3 + b\,x^2 + c\,x^1 + d \f$
         * -# \f$ \frac{\partial y(x)}{\partial x} = y'(x) = 3\,a\,x^2 + 2\,b\,x^1 + c \f$
         * -# \f$ y(x=0) = y_0 \f$
         * -# \f$ y(x=1) = y_1 \f$
         * -# \f$ y'(x=0) = y'_0 \f$
         * -# \f$ y'(x=1) = y'_1 \f$
         *
         * \par Solution
         * - \f$ a = 2\,(y_0 - y_1) + y'_0 + y'_1 \f$
         * - \f$ b = 3\,(y_1 - y_0) - 2\,y'_0 - y'_1 \f$
         * - \f$ c = y'_0 \f$
         * - \f$ d = y_0 \f$
         */
        virtual bool interpolateCubicFirstDerivatives(const std::vector<double>& parameters, CurveResult* const result = nullptr)
        {
            // Check if degree is sufficient
            if (degree() < 3) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Check, if parameters are specified correctly
            if (parameters.size() != 4) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Set coefficients
            m_coefficients[3] = 2.0 * (parameters[0] - parameters[1]) + parameters[2] + parameters[3];
            m_coefficients[2] = 3.0 * (parameters[1] - parameters[0]) - 2.0 * parameters[2] - parameters[3];
            m_coefficients[1] = parameters[2];
            m_coefficients[0] = parameters[0];

            // Force higher terms to be zero (in case polynomial is of higher degree)
            for (size_t i = 4; i < m_coefficients.size(); i++)
                m_coefficients[i] = 0;

            // Success!
            if (result != nullptr)
                *result = CurveResult::SUCCESS;
            return true;
        }

        //! Cubic polynomial interpolation with given second derivatives (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters and \p result)
        /*!
         * Set coefficients to fit a polynomial of degree three (cubic) with given function values and second order derivatives at \f$ x=0 \f$ and \f$ x=1 \f$.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | `y0` Function value \f$ y_0 = y(x=0) \f$
         * \p parameters[1]   | `y1` Function value \f$ y_1 = y(x=1) \f$
         * \p parameters[2]   | `ddy0` Second order derivative \f$ y''_0 = y''(x=0) \f$ with respect to \f$ x \f$
         * \p parameters[3]   | `ddy1` Second order derivative \f$ y''_1 = y''(x=1) \f$ with respect to \f$ x \f$
         *
         * \par Problem
         * -# \f$ y(x) =   a\,x^3 +   b\,x^2 + c\,x^1 + d \f$
         * -# \f$ \frac{\partial y(x)}{\partial x} = y'(x) = 3\,a\,x^2 + 2\,b\,x^1 + c \f$
         * -# \f$ \frac{\partial^2 y(x)}{\partial x^2} = y''(x) = 6\,a\,x^1 + 2\,b \f$
         * -# \f$ y(x=0) = y_0 \f$
         * -# \f$ y(x=1) = y_1 \f$
         * -# \f$ y''(x=0) = y''_0 \f$
         * -# \f$ y''(x=1) = y''_1 \f$
         *
         * \par Solution
         * - \f$ a = \frac{1}{6}\,(y''_1 - y''_0) \f$
         * - \f$ b = \frac{1}{2}\,y''_0 \f$
         * - \f$ c = y_1 - y_0 - \frac{1}{3}\,y''_0 - \frac{1}{6}\,y''_1 \f$
         * - \f$ d = y_0 \f$
         */
        virtual bool interpolateCubicSecondDerivatives(const std::vector<double>& parameters, CurveResult* const result = nullptr)
        {
            // Check if degree is sufficient
            if (degree() < 3) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Check, if parameters are specified correctly
            if (parameters.size() != 4) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Set coefficients
            m_coefficients[3] = 1.0 / 6.0 * (parameters[3] - parameters[2]);
            m_coefficients[2] = 0.5 * parameters[2];
            m_coefficients[1] = parameters[1] - parameters[0] - 1.0 / 3.0 * parameters[2] - 1.0 / 6.0 * parameters[3];
            m_coefficients[0] = parameters[0];

            // Force higher terms to be zero (in case polynomial is of higher degree)
            for (size_t i = 4; i < m_coefficients.size(); i++)
                m_coefficients[i] = 0;

            // Success!
            if (result != nullptr)
                *result = CurveResult::SUCCESS;
            return true;
        }

        //! Quintic polynomial interpolation with given first and second derivatives (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters and \p result)
        /*!
         * Set coefficients to fit a polynomial of degree five (quintic) with given function values, first and second order derivatives at \f$ x=0 \f$ and \f$ x=1 \f$.
         *
         * \par Parameter-layout
         * Element            | Description
         * -------            | -----------
         * \p parameters[0]   | `y0` Function value \f$ y_0 = y(x=0) \f$
         * \p parameters[1]   | `y1` Function value \f$ y_1 = y(x=1) \f$
         * \p parameters[2]   | `dy0` First order derivative \f$ y'_0 = y'(x=0) \f$ with respect to \f$ x \f$
         * \p parameters[3]   | `dy1` First order derivative \f$ y'_1 = y'(x=1) \f$ with respect to \f$ x \f$
         * \p parameters[4]   | `ddy0` Second order derivative \f$ y''_0 = y''(x=0) \f$ with respect to \f$ x \f$
         * \p parameters[5]   | `ddy1` Second order derivative \f$ y''_1 = y''(x=1) \f$ with respect to \f$ x \f$
         *
         * \par Problem
         * -# \f$ y(x) = a\,x^5 + b\,x^4 + c\,x^3 + d\,x^2 + e\,x^1 + f \f$
         * -# \f$ \frac{\partial y(x)}{\partial x} = y'(x) = 5\,a\,x^4 + 4\,b\,x^3 + 3\,c\,x^2 + 2\,d\,x^1 + e \f$
         * -# \f$ \frac{\partial^2 y(x)}{\partial x^2} = y''(x) = 20\,a\,x^3 + 12\,b\,x^2 + 6\,c\,x^1 + 2\,d \f$
         * -# \f$ y(x=0) = y_0 \f$
         * -# \f$ y(x=1) = y_1 \f$
         * -# \f$ y'(x=0) = y'_0 \f$
         * -# \f$ y'(x=1) = y'_1 \f$
         * -# \f$ y''(x=0) = y''_0 \f$
         * -# \f$ y''(x=1) = y''_1 \f$
         *
         * \par Solution
         * - \f$ a =  -6\,y_0 +  6\,y_1 - 3\,y'_0 - 3\,y'_1 - 0.5\,y''_0 + 0.5\,y''_1 \f$
         * - \f$ b =  15\,y_0 - 15\,y_1 + 8\,y'_0 + 7\,y'_1 + 1.5\,y''_0 -      y''_1 \f$
         * - \f$ c = -10\,y_0 + 10\,y_1 - 6\,y'_0 - 4\,y'_1 - 1.5\,y''_0 + 0.5\,y''_1 \f$
         * - \f$ d =                                          0.5\,y''_0              \f$
         * - \f$ e =                         y'_0                                     \f$
         * - \f$ f =      y_0                                                         \f$
         */
        virtual bool interpolateQuinticFirstAndSecondDerivatives(const std::vector<double>& parameters, CurveResult* const result = nullptr)
        {
            // Check if degree is sufficient
            if (degree() < 5) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_DEGREE_TOO_LOW;
                assert(false);
                return false;
            }

            // Check, if parameters are specified correctly
            if (parameters.size() != 6) {
                if (result != nullptr)
                    *result = CurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Set coefficients
            m_coefficients[5] = -6.0 * parameters[0] + 6.0 * parameters[1] - 3.0 * parameters[2] - 3.0 * parameters[3] - 0.5 * parameters[4] + 0.5 * parameters[5];
            m_coefficients[4] = 15.0 * parameters[0] - 15.0 * parameters[1] + 8.0 * parameters[2] + 7.0 * parameters[3] + 1.5 * parameters[4] - parameters[5];
            m_coefficients[3] = -10.0 * parameters[0] + 10.0 * parameters[1] - 6.0 * parameters[2] - 4.0 * parameters[3] - 1.5 * parameters[4] + 0.5 * parameters[5];
            m_coefficients[2] = 0.5 * parameters[4];
            m_coefficients[1] = parameters[2];
            m_coefficients[0] = parameters[0];

            // Force higher terms to be zero (in case polynomial is of higher degree)
            for (size_t i = 6; i < m_coefficients.size(); i++)
                m_coefficients[i] = 0;

            // Success!
            if (result != nullptr)
                *result = CurveResult::SUCCESS;
            return true;
        }

        // Encoding
        // --------
    public:
        // Encode member data as XML element and add it to the specified stream (see base class for details)
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const size_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<PolynomialCurve");

            // Write coefficient data
            addedElements += io::encoding::encode(stream, " Coefficients=\"");
            addedElements += io::encoding::encode(stream, m_coefficients, numericFormat);
            addedElements += io::encoding::encode(stream, "\">");

            // End XML element
            addedElements += io::encoding::encode(stream, "</PolynomialCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }
    };
} // namespace curve
} // namespace broccoli
