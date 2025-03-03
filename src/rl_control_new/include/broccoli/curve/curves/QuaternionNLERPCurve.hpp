/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "FixedSizeQuaternionCurve.hpp"

namespace broccoli {
namespace curve {
    //! Class abstracting quaternion cuves using <b>N</b>ormalized <b>L</b>inear int<b>ERP</b>olation (NLERP)
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \warning This class is restricted to **unit-quaternions**!
     */
    class QuaternionNLERPCurve : public FixedSizeQuaternionCurve<2> {
    public:
        //! Specialized constructor
        /*!
         * \param [in] q0 The quaternion \f$ q_0 \f$ at the beginning of the curve.
         * \param [in] q1 The quaternion \f$ q_1 \f$ at the end of the curve.
         */
        QuaternionNLERPCurve(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1)
        {
            // Set control points
            m_controlPoints[0] = q0;
            m_controlPoints[1] = q1;
        }

        //! Default constructor
        /*! Initializes as \f$ q_i = [w=1,\, x=0,\, y=0,\, z=0]^T \f$ for **all** control points */
        QuaternionNLERPCurve()
            : FixedSizeQuaternionCurve(Eigen::Quaterniond(1, 0, 0, 0))
        {
        }

        //! Destructor
        virtual ~QuaternionNLERPCurve()
        {
        }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::NLERP; }

        // Evaluation of (derivative of) underlying curve (see base class for details)
        virtual Eigen::Quaterniond evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Initialize return value
            Eigen::Quaterniond returnValue(1, 0, 0, 0);

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                /*! Evaluation of base function
                 *  ---------------------------
                 * Performs simple quaternion **normalized** LERP:
                 * \f[ q(x) = NLERP(q_0,\,q_1,\,x) = \frac{\left(1-x\right) q_0 + x \cdot q_1}{\| \left(1-x\right) q_0 + x \cdot q_1 \|} \f]
                 */
                returnValue.coeffs() = (1 - position) * m_controlPoints[0].coeffs() + position * m_controlPoints[1].coeffs();
                returnValue.normalize();
            } else {
                /*! Evaluation of derivatives
                 *  -------------------------
                 * Computes **numeric** derivative with respect to the interpolation parameter \f$ x \f$ (**not** time in general).
                 * See \ref QuaternionCurve::evaluateNumericDerivative()
                 *
                 * \note Computing the analytic derivative of NLERP is possible, however, rather complex due to \f$ \partial norm(q(x))^{-1} / \partial x\f$.
                 */
                returnValue = evaluateNumericDerivative(position, derivationOrder);
            }

            // Pass back return value
            return returnValue;
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
            addedElements += io::encoding::encode(stream, "<QuaternionNLERPCurve");

            // Write attributes
            addedElements += encodeXMLAttributes(stream, numericFormat);

            // End XML element
            addedElements += io::encoding::encode(stream, "></QuaternionNLERPCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
