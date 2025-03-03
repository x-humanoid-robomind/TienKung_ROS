/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "Curve.hpp"
#include <math.h>

namespace broccoli {
namespace curve {
    //! Class for trigonometric functions. \f$ f(x) = c_0 + c_1\,func(c_2\,x + c_3) \f$
    /*!
     * \ingroup broccoli_curve_curves
     *
     * Type of trigonometric function \f$ func \f$ may be changed at any time (sine, cosine, ...).
     */
    class TrigonometricCurve : public Curve {
    public:
        enum class Type : uint8_t {
            SINE = 0, //!< \f$ \sin(x) \f$
            COSINE, //!< \f$ \cos(x) \f$
            TANGENT, //!< \f$ \tan(x) \f$
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given function type
        static std::string typeString(const Type& type)
        {
            // Check type
            switch (type) {
            case Type::SINE:
                return "SINE";
            case Type::COSINE:
                return "COSINE";
            case Type::TANGENT:
                return "TANGENT";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Default constructor
        /*! Initializes to \f$ f(x) = 0 + 1\cdot \sin(1\cdot x + 0) = \sin(x) \f$ */
        TrigonometricCurve()
            : m_type(Type::SINE)
            , m_coefficients({ { 0, 1, 1, 0 } })
        {
        }

        //! Destructor
        virtual ~TrigonometricCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const TrigonometricCurve& reference) const
        {
            // Compare base class
            if (Curve::operator!=(reference))
                return false;

            // Compare type
            if (m_type != reference.m_type)
                return false;

            // Compare coefficients
            if (m_coefficients != reference.m_coefficients)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const TrigonometricCurve& reference) const { return !(*this == reference); }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::TRIGONOMETRIC; }

        // Member data
        Type m_type; //!< Type of trigonometric function
        std::array<double, 4> m_coefficients; //!< Coefficients of trigonometric function (\f$ c_i\f$ corresponds to `m_coefficients[i]`)

        // Evaluation of (derivative of) underlying function (see base class for details)
        /*!
         * \copydoc Curve::evaluate()
         *
         * \warning The \f$n\f$-th derivative of tangent (with \f$ n>1 \f$) is not implemented analytically! In this case the numeric derivative (finite difference) will be passed back. See \ref Curve::evaluateNumericDerivative()
         */
        virtual double evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Check type
            switch (m_type) {
            case Type::SINE: {
                // Check if we only want the base function (for speed up of computation)
                if (derivationOrder == 0)
                    return m_coefficients[0] + m_coefficients[1] * sin(m_coefficients[2] * position + m_coefficients[3]);
                else {
                    // Accumulate powers
                    double c2Powered = m_coefficients[2];
                    for (unsigned int i = 1; i < derivationOrder; i++)
                        c2Powered *= m_coefficients[2];

                    // Determine sign: sign = (-1)^(floor(order/2))
                    double sign = 1;
                    if ((((int)(derivationOrder / 2.0)) % 2) == 1)
                        sign = -1;

                    // Determine if we have an even or uneven order of derivation (for selecting sin/cos)
                    if ((derivationOrder % 2) == 0)
                        return sign * m_coefficients[1] * c2Powered * sin(m_coefficients[2] * position + m_coefficients[3]);
                    else
                        return sign * m_coefficients[1] * c2Powered * cos(m_coefficients[2] * position + m_coefficients[3]);
                }
            }
            case Type::COSINE: {
                // Check if we only want the base function (for speed up of computation)
                if (derivationOrder == 0)
                    return m_coefficients[0] + m_coefficients[1] * cos(m_coefficients[2] * position + m_coefficients[3]);
                else {
                    // Accumulate powers
                    double c2Powered = m_coefficients[2];
                    for (unsigned int i = 1; i < derivationOrder; i++)
                        c2Powered *= m_coefficients[2];

                    // Determine sign: sign = (-1)^(floor(order/2+1/2))
                    double sign = 1;
                    if ((((int)(derivationOrder / 2.0 + 0.5)) % 2) == 1)
                        sign = -1;

                    // Determine if we have an even or uneven order of derivation (for selecting sin/cos)
                    if ((derivationOrder % 2) == 0)
                        return sign * m_coefficients[1] * c2Powered * cos(m_coefficients[2] * position + m_coefficients[3]);
                    else
                        return sign * m_coefficients[1] * c2Powered * sin(m_coefficients[2] * position + m_coefficients[3]);
                }
            }
            case Type::TANGENT: {
                // Check if we only want the base function (for speed up of computation)
                if (derivationOrder == 0)
                    return m_coefficients[0] + m_coefficients[1] * tan(m_coefficients[2] * position + m_coefficients[3]);
                else if (derivationOrder == 1) {
                    // First derivative: d/dx tan(x) = 1 + tan(x)^2
                    double tangent = tan(m_coefficients[2] * position + m_coefficients[3]);
                    return m_coefficients[1] * m_coefficients[2] * (1.0 + tangent * tangent);
                } else {
                    // WARNING: n-th derivative of tangent not implemented! Using numeric derivative instead!
                    return evaluateNumericDerivative(position, derivationOrder);
                }
            }
            default: {
                // Unknown type...
                assert(false);
                return 0;
            }
            }
        }

        // Evaluation of value (D0) up to N-th derivative (DN) of underlying function (see base class for details)
        /*!
         * \copydoc Curve::evaluateD0ToDN()
         *
         * \remark Efficient computation is only supported for \ref Type::SINE and \ref Type::COSINE. For \ref Type::TANGENT the
         * standard implementation (N+1 times \ref evaluate()) is used, since there is no analyitc N-th derivative (see \ref Curve::evaluateD0ToDN()).
         */
        template <unsigned int N>
        std::array<double, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Check validity
            assert(isValid());

            // Check type
            switch (m_type) {
            case Type::SINE: {
                // Initialize return value
                std::array<double, N + 1> Di;
                Di.fill(0);

                // Compute value and derivatives
                const double argument = m_coefficients[2] * position + m_coefficients[3]; // c_2 * x + c_3
                const double c1_sine = m_coefficients[1] * sin(argument); // c_1 * sin(c_2 * x + c_3)
                Di[0] = m_coefficients[0] + c1_sine;
                if (N > 0) {
                    const double c1_cosine = m_coefficients[1] * cos(argument); // c_1 * cos(c_2 * x + c_3)
                    double c2Powered = m_coefficients[2];
                    for (unsigned int d = 1; d <= N; d++) {
                        // Determine if we have an even or uneven order of derivation (for selecting sin/cos)
                        if ((d % 2) == 0)
                            Di[d] = c1_sine * c2Powered;
                        else
                            Di[d] = c1_cosine * c2Powered;

                        // Determine sign: sign = (-1)^(floor(order/2))
                        if ((((int)(d / 2.0)) % 2) == 1)
                            Di[d] = -Di[d];

                        // Update c2^... for next derivative
                        c2Powered *= m_coefficients[2];
                    }
                }

                // Pass back list
                return Di;
            }
            case Type::COSINE: {
                // Initialize return value
                std::array<double, N + 1> Di;
                Di.fill(0);

                // Compute value and derivatives
                const double argument = m_coefficients[2] * position + m_coefficients[3]; // c_2 * x + c_3
                const double c1_cosine = m_coefficients[1] * cos(argument); // c_1 * cos(c_2 * x + c_3)
                Di[0] = m_coefficients[0] + c1_cosine;
                if (N > 0) {
                    const double c1_sine = m_coefficients[1] * sin(argument); // c_1 * sin(c_2 * x + c_3)
                    double c2Powered = m_coefficients[2];
                    for (unsigned int d = 1; d <= N; d++) {
                        // Determine if we have an even or uneven order of derivation (for selecting sin/cos)
                        if ((d % 2) == 0)
                            Di[d] = c1_cosine * c2Powered;
                        else
                            Di[d] = c1_sine * c2Powered;

                        // Determine sign: sign = (-1)^(floor(order/2))
                        if ((((int)(d / 2.0 + 0.5)) % 2) == 1)
                            Di[d] = -Di[d];

                        // Update c2^... for next derivative
                        c2Powered *= m_coefficients[2];
                    }
                }

                // Pass back list
                return Di;
            }
            case Type::TANGENT: {
                // No analytic N-th derivative available -> use standard implementation
                return Curve::evaluateD0ToDN<N>(position);
            }
            default: {
                // Unknown type...
                std::array<double, N + 1> Di;
                Di.fill(0);
                assert(false);
                return Di;
            }
            }
        }

        // Encoding
        // --------
        // Encode member data as XML element and add it to the specified stream (see base class for details)
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const size_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<TrigonometricCurve");

            // Write type
            addedElements += io::encoding::encode(stream, " Type=\"");
            addedElements += io::encoding::encode(stream, typeString(m_type));

            // Write coefficient data
            addedElements += io::encoding::encode(stream, "\" Coefficients=\"");
            addedElements += io::encoding::encode(stream, m_coefficients, numericFormat);
            addedElements += io::encoding::encode(stream, "\">");

            // End XML element
            addedElements += io::encoding::encode(stream, "</TrigonometricCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }
    };

} // namespace curve
} // namespace broccoli
