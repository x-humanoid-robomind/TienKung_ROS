/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "Curve.hpp"

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_curves
     * \{
     */

    //! Specification of curve interpolation methods
    enum class CurveInterpolationMethod : uint8_t {
        POLYNOMIAL_CONSTANT = 0, //!< Set coefficients to fit a polynomial of degree zero (constant) with given function value.
        POLYNOMIAL_LINEAR, //!< Set coefficients to fit a polynomial of degree one (linear) with given function values at \f$ x=0 \f$ and \f$ x=1 \f$.
        POLYNOMIAL_CUBIC_FIRST_DERIVATIVES, //!< Set coefficients to fit a polynomial of degree three (cubic) with given function values and first order derivatives at \f$ x=0 \f$ and \f$ x=1 \f$.
        POLYNOMIAL_CUBIC_SECOND_DERIVATIVES, //!< Set coefficients to fit a polynomial of degree three (cubic) with given function values and second order derivatives at \f$ x=0 \f$ and \f$ x=1 \f$.
        POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES, //!< Set coefficients to fit a polynomial of degree five (quintic) with given function values, first and second order derivatives at \f$ x=0 \f$ and \f$ x=1 \f$.
        CURVEINTERPOLATIONMETHOD_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given curve interpolation method
    static inline std::string curveInterpolationMethodString(const CurveInterpolationMethod& method)
    {
        // Check method
        switch (method) {
        case CurveInterpolationMethod::POLYNOMIAL_CONSTANT:
            return "POLYNOMIAL_CONSTANT";
        case CurveInterpolationMethod::POLYNOMIAL_LINEAR:
            return "POLYNOMIAL_LINEAR";
        case CurveInterpolationMethod::POLYNOMIAL_CUBIC_FIRST_DERIVATIVES:
            return "POLYNOMIAL_CUBIC_FIRST_DERIVATIVES";
        case CurveInterpolationMethod::POLYNOMIAL_CUBIC_SECOND_DERIVATIVES:
            return "POLYNOMIAL_CUBIC_SECOND_DERIVATIVES";
        case CurveInterpolationMethod::POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES:
            return "POLYNOMIAL_QUINTIC_FIRST_AND_SECOND_DERIVATIVES";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Abstract representation of an **interpolatable** curve (underlying function is not specified yet)
    /*! Class acts as a prototype for curves which provide an \ref interpolate() method. */
    class InterpolatableCurve {
    public:
        //! Constructor
        InterpolatableCurve()
        {
        }

        //! Destructor
        virtual ~InterpolatableCurve()
        {
        }

        //! Interpolate underlying function to fit specific parameters (through-points, derivatives, ...)
        /*!
         * \param [in] parameters Interpolation parameters (meaning varies depending on chosen method)
         * \param [in] method Interpolation method to be used (has to be compatible with curve type)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool interpolate(const std::vector<double>& parameters, const CurveInterpolationMethod& method, CurveResult* const result = nullptr) = 0;
    };

    //! \}
} // namespace curve
} // namespace broccoli
