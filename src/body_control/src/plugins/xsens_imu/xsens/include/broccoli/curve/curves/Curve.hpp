/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/math.hpp"
#include "../../io/encoding.hpp"
#include <array>
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_curves
     * \{
     */

    //! Specification of result types of curve algorithms
    enum class CurveResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_PARAMETERS, //!< An **error** occured: invalid input parameters
        ERROR_DEGREE_TOO_LOW, //!< An **error** occured: the degree of the underlying polynomial is too low
        ERROR_NOTIMPLEMENTED, //!< An **error** occured: the desired algorithm is **not** yet implemented
        CURVERESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type for curve algorithms
    static inline std::string curveResultString(const CurveResult& result)
    {
        // Check result
        switch (result) {
        case CurveResult::UNKNOWN:
            return "UNKNOWN";
        case CurveResult::SUCCESS:
            return "SUCCESS";
        case CurveResult::ERROR_INVALID_PARAMETERS:
            return "ERROR_INVALID_PARAMETERS";
        case CurveResult::ERROR_DEGREE_TOO_LOW:
            return "ERROR_DEGREE_TOO_LOW";
        case CurveResult::ERROR_NOTIMPLEMENTED:
            return "ERROR_NOTIMPLEMENTED";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Abstract representation of a curve (underlying function is not specified yet)
    /*!
     * Class acts as a prototype and declares all functions and members a curve should provide.
     *
     * The underlying function is given with respect to `position` which is **not** the time in general.
     * Instead one should parametrize the function in a way, such that `position` \f$\in [0, 1]\f$ relates to
     * the area of interest. This implies, that one has to be careful when using the \ref evaluate
     * function to get the time derivative (use chain rule!).
     */
    class Curve {
    public:
        //! Specification of possible underlying function types
        enum class FunctionType : uint8_t {
            POLYNOMIAL = 0, //!< Polynomial functions (\f$a\,x^2 + b\,x + c = \dots\f$)
            TRIGONOMETRIC, //!< Trigonometric functions (\f$\sin(\dots), \cos(\dots), \dots\f$)
            EXPONENTIAL, //!< Exponential functions (\f$e^{\dots}\f$)
            FUNCTIONTYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given function type
        static std::string functionTypeString(const FunctionType& type)
        {
            // Check type
            switch (type) {
            case FunctionType::POLYNOMIAL:
                return "POLYNOMIAL";
            case FunctionType::TRIGONOMETRIC:
                return "TRIGONOMETRIC";
            case FunctionType::EXPONENTIAL:
                return "EXPONENTIAL";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Constructor
        Curve()
        {
        }

        //! Destructor
        virtual ~Curve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const Curve& reference) const
        {
            (void)reference; // Unused

            // No own members -> always equal
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const Curve& reference) const { return !(*this == reference); }

        //! Checks, if the curve is properly defined
        /*! \return `true`, if the curve is valid, `false` otherwise. */
        virtual bool isValid() const
        {
            // Dummy implementation: always valid - derived classes may check members
            return true;
        }

        //! Get type of underlying function
        virtual FunctionType functionType() const = 0;

        //! Evaluation of (derivative of) underlying function
        /*!
         * \warning You may call \ref isValid() before to check, if the curve is valid! If the curve is invalid the behaviour of this method is not defined!
         *
         * \warning The derivative is given with respect to the \p position (interpolation parameter), **not** time in general!
         *
         * \param [in] position Position to evaluate (derivative) at (should - but has not to be - within \f$[0,\,1]\f$)
         * \param [in] derivationOrder Order of the derivation (use 0 for base function, i.e. no derivation)
         * \return Value (of derivative) at specified position
         */
        virtual double evaluate(const double& position, const unsigned int& derivationOrder = 0) const = 0;

        //! Evaluation of value (D0) up to N-th derivative (DN) of underlying function (improved performance over calling \ref evaluate() N+1 times)
        /*!
         * \warning You may call \ref isValid() before to check, if the curve is valid! If the curve is invalid the behaviour of this method is not defined!
         *
         * \warning The derivatives are given with respect to the \p position (interpolation parameter), **not** time in general!
         *
         * \tparam N Order of highest derivative
         * \param [in] position Position to evaluate (derivatives) at (should - but has not to be - within \f$[0,\,1]\f$)
         * \return List of value (D0) up to N-th derivative [D0, D1, ... , Di, ... , DN] at specified position
         */
        template <unsigned int N>
        std::array<double, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Validity check
            assert(isValid());

            // Default implementation: just call evaluate N+1 times - derived classes may replace this with a more efficient method
            std::array<double, N + 1> Di;
            for (unsigned int d = 0; d <= N; d++)
                Di[d] = evaluate(position, d);
            return Di;
        }

        //! Computes **numeric** \f$ n\f$-th derivative of underlying function using finite difference method
        /*!
         * \warning You may call \ref isValid() before to check, if the curve is valid! If the curve is invalid the behaviour of this method is not defined!
         *
         * \warning The derivative is given with respect to the \p position (interpolation parameter), **not** time in general!
         *
         * \param [in] position Position \f$ x\f$ to evaluate numeric derivative at (has to be within \f$[0,\,1]\f$, otherwise it will be projected to these limits)
         * \param [in] derivationOrder Order \f$ n\f$ of the derivation (use 0 for base function, i.e. no derivation)
         * \param [in] stepSize The step size alias \f$ h \f$ used for evaluating the underlying curve at certain positions
         * \return \f$ n \f$-th derivative at specified position
         */
        virtual double evaluateNumericDerivative(const double& position, const unsigned int& derivationOrder, const double& stepSize = 1e-6) const
        {
            // Check validity
            assert(isValid());

            // Project interpolation parameter to boundaries
            double projectedPosition = position;
            if (projectedPosition < 0)
                projectedPosition = 0;
            if (projectedPosition > 1)
                projectedPosition = 1;

            // Check, if base value should be returned
            if (derivationOrder == 0)
                return evaluate(projectedPosition, 0);

            // Check, if stepsize is valid
            if (stepSize <= 0) {
                assert(false);
                return 0;
            }

            // Compute "direction"
            const double nh_2 = derivationOrder * stepSize / 2.0; // Half-size of the evaluated "x-window" (n*h/2)
            int mode = 0; // "Central"
            if (projectedPosition - nh_2 < 0.0)
                mode = 1; // "Forward"
            else if (projectedPosition + nh_2 > 1.0)
                mode = -1; // "Backward"

            /*! Method
             *  ------
             * In order to avoid exceeding the limits \f$ x \in \left[0,\,1\right] \f$, we differentiate between three cases:
             */
            double returnValue = 0;
            for (uint64_t i = 0; i <= derivationOrder; i++) {
                double coefficient = core::math::binomialCoefficient(derivationOrder, i);
                if (i % 2 == 1)
                    coefficient = -coefficient;

                /*! **Forward:** for \f$ x - \frac{n\,h}{2} < 0\f$:
                 * \f[ \frac{\partial^n f(x)}{\partial x^n} \approx \frac{1}{h^n} \left[ \sum_{i=0}^n \left( -1 \right)^i \left( \begin{array}{c} n\\i\end{array} \right)\,f\left(x+\left(n-i\right)\,h\right)\right] \f]
                 */
                if (mode == 1)
                    returnValue += coefficient * evaluate(projectedPosition + (derivationOrder - i) * stepSize, 0);
                /*! **Central:** for \f$ x - \frac{n\,h}{2} \geq 0\f$ and \f$ x + \frac{n\,h}{2} \leq 1\f$:
                 * \f[ \frac{\partial^n f(x)}{\partial x^n} \approx \frac{1}{h^n} \left[ \sum_{i=0}^n \left( -1 \right)^i \left( \begin{array}{c} n\\i\end{array} \right)\,f\left(x+\left(\frac{n}{2}-i\right)\,h\right)\right] \f]
                 */
                else if (mode == 0)
                    returnValue += coefficient * evaluate(projectedPosition + (derivationOrder / 2.0 - i) * stepSize, 0);
                /*! **Backward:** for \f$ x + \frac{n\,h}{2} > 1\f$:
                 * \f[ \frac{\partial^n f(x)}{\partial x^n} \approx \frac{1}{h^n} \left[ \sum_{i=0}^n \left( -1 \right)^i \left( \begin{array}{c} n\\i\end{array} \right)\,f\left(x-i\,h\right)\right] \f]
                 */
                else
                    returnValue += coefficient * evaluate(projectedPosition - i * stepSize, 0);
            }
            for (uint64_t i = 0; i < derivationOrder; i++)
                returnValue /= stepSize;

            // Pass back result
            return returnValue;
        }

        // Encoding
        // --------
        //! Encode member data as XML element and add it to the specified stream
        /*!
         * \param [in,out] stream The stream to which the member data should be appended to.
         * \param [in] XMLIndentationLevel Level of indentation in XML format (used to add spaces).
         * \param [in] XMLTabWhiteSpaces Count of whitespaces corresponding to a tab.
         * \param [in] numericFormat C-format specifier for numeric values.
         * \return Count of appended characters (elements in stream).
         */
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const size_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const = 0;
    };

    //! \}
} // namespace curve
} // namespace broccoli
