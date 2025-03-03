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
    /*!
     * \brief Class for exponential functions. \f$ f(x) = c_0 + c_1\,e^{c_2\,x} \f$
     * \ingroup broccoli_curve_curves
     */
    class ExponentialCurve : public Curve {
    public:
        //! Default constructor
        /*! Initializes to \f$ f(x) = 0 + 1\cdot e^{1\cdot x} = e^x \f$ */
        ExponentialCurve()
            : m_coefficients({ { 0, 1, 1 } })
        {
        }

        //! Destructor
        virtual ~ExponentialCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const ExponentialCurve& reference) const
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
        virtual bool operator!=(const ExponentialCurve& reference) const { return !(*this == reference); }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::EXPONENTIAL; }

        // Member data
        std::array<double, 3> m_coefficients; //!< Coefficients of exponential function (\f$ c_i\f$ corresponds to `m_coefficients[i]`)

        // Evaluation of (derivative of) underlying function (see base class for details)
        virtual double evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0)
                return m_coefficients[0] + m_coefficients[1] * exp(m_coefficients[2] * position);
            else {
                // Accumulate powers
                double c2Powered = m_coefficients[2];
                for (unsigned int i = 1; i < derivationOrder; i++)
                    c2Powered *= m_coefficients[2];

                // Output derivative
                return m_coefficients[1] * c2Powered * exp(m_coefficients[2] * position);
            }
        }

        // Evaluation of value (D0) up to N-th derivative (DN) of underlying function (see base class for details)
        template <unsigned int N>
        std::array<double, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Validity check
            assert(isValid());

            // Initialize return value
            std::array<double, N + 1> Di;
            Di.fill(0);

            // Compute value and derivatives
            const double c1_exp_c2_x = m_coefficients[1] * exp(m_coefficients[2] * position); // c_1 * exp(c_2 * x)
            Di[0] = m_coefficients[0] + c1_exp_c2_x;
            if (N > 0) {
                Di[1] = c1_exp_c2_x * m_coefficients[2];
                for (unsigned int d = 2; d <= N; d++)
                    Di[d] = Di[d - 1] * m_coefficients[2];
            }

            // Pass back list
            return Di;
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
            addedElements += io::encoding::encode(stream, "<ExponentialCurve");

            // Write coefficient data
            addedElements += io::encoding::encode(stream, "\" Coefficients=\"");
            addedElements += io::encoding::encode(stream, m_coefficients, numericFormat);
            addedElements += io::encoding::encode(stream, "\">");

            // End XML element
            addedElements += io::encoding::encode(stream, "</ExponentialCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }
    };
} // namespace curve
} // namespace broccoli
