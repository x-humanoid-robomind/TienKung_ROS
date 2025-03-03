/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../geometry/rotations.hpp"
#include "FixedSizeQuaternionCurve.hpp"

namespace broccoli {
namespace curve {
    //! Class abstracting quaternion cuves using <b>S</b>pherical <b>L</b>inear int<b>ERP</b>olation (SLERP) according to Shoemake 1985
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \warning This class is restricted to **unit-quaternions**!
     *
     * References
     * ----------
     * * Ken Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH Computer Graphics, ACM, New York, NY, USA, volume 19, number 3, 1985, DOI:[10.1145/325165.325242](https://www.doi.org/10.1145/325165.325242), p.245--254
     */
    class QuaternionSLERPCurve : public FixedSizeQuaternionCurve<2> {
    public:
        //! Specialized constructor
        /*!
         * \param [in] q0 The quaternion \f$ q_0 \f$ at the beginning of the curve.
         * \param [in] q1 The quaternion \f$ q_1 \f$ at the end of the curve.
         */
        QuaternionSLERPCurve(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1)
        {
            // Set control points
            m_controlPoints[0] = q0;
            m_controlPoints[1] = q1;
        }

        //! Default constructor
        /*! Initializes as \f$ q_i = [w=1,\, x=0,\, y=0,\, z=0]^T \f$ for **all** control points */
        QuaternionSLERPCurve()
            : FixedSizeQuaternionCurve(Eigen::Quaterniond(1, 0, 0, 0))
        {
        }

        //! Destructor
        virtual ~QuaternionSLERPCurve()
        {
        }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::SLERP; }

        // Evaluation of (derivative of) underlying curve (see base class for details)
        //! \copydoc FixedSizeQuaternionCurve::evaluate()
        /*! \warning \p position has to within \f$[0,\,1]\f$, otherwise it will be projected! */
        virtual Eigen::Quaterniond evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Initialize return value
            Eigen::Quaterniond returnValue(1, 0, 0, 0);

            // Project interpolation parameter to boundaries
            double projectedPosition = position;
            if (projectedPosition < 0)
                projectedPosition = 0;
            if (projectedPosition > 1)
                projectedPosition = 1;

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                /*! Evaluation of base function
                 *  ---------------------------
                 * Performs simple quaternion slerp:
                 * \f[ q(x) = SLERP(q_0,\,q_1,\,x) = q_0 \cdot (q_0^{-1} \cdot q_1)^x \f]
                 * with \f$ x \in [0,\,1] \f$
                 */
                // IMPORTANT: we do NOT use Eigen's implementation of slerp here since Eigen's implementation automatically chooses the shortest path. However, here we want that the user has the option to choose the "direction" (by flipping the quaternions).
                returnValue = geometry::quaternionSLERP(m_controlPoints[0], m_controlPoints[1], projectedPosition, false);
                returnValue.normalize(); // Normalize resulting quaternion to be sure to have a unit quaternion
            } else {
                /*! Evaluation of derivatives
                 *  -------------------------
                 * Computes **analytic** derivative with respect to interpolation parameter \f$ x \f$ (**not** time in general).
                 *
                 * SLERP:
                 * \f[ q(x) = q_0 \cdot (q_0^{-1} \cdot q_1)^x = q_0 \cdot q_2^x \f]
                 * with \f$ q_2 := q_0^{-1} \cdot q_1 = const. \f$
                 * \f[ \frac{\partial q(x)}{\partial x} = \underbrace{q_0 \cdot q_2^x}_{q(x)} \cdot \ln(q_2) = q(x) \cdot \ln(q_2) = q(x) \cdot q_3 \f]
                 * with \f$ q_3 := \ln(q_2) = \ln(q_0^{-1} \cdot q_1) = const. \f$
                 * \f[ \frac{\partial^2 q(x)}{\partial x^2} = \frac{\partial q(x)}{\partial x} \cdot q_3 = q(x) \cdot q_3 \cdot q_3 = q(x) \cdot q_3^2 \f]
                 * \f[ \vdots \f]
                 * \f[ \frac{\partial^n q(x)}{\partial x^n} = q(x) \cdot q_3^n \f]
                 */

                // Compute helper variables (note: for unit quaternions the inverse is equal to the conjugate (conjugate is easier to compute))
                const Eigen::Quaterniond q0invq1 = m_controlPoints[0].conjugate() * m_controlPoints[1];
                if (q0invq1.x() == 0 && q0invq1.y() == 0 && q0invq1.z() == 0) {
                    // ...start- and end-quaternion describe the same orientation -> derivative of q is zero (no change)!
                    return Eigen::Quaterniond(0, 0, 0, 0);
                }
                const Eigen::Quaterniond q3 = geometry::quaternionNaturalLogarithm(q0invq1);

                // Compute final derivative
                // IMPORTANT: we do NOT use Eigen's implementation of slerp here since Eigen's implementation automatically chooses the shortest path. However, here we want that the user has the option to choose the "direction" (by flipping the quaternions).
                returnValue = geometry::quaternionSLERP(m_controlPoints[0], m_controlPoints[1], projectedPosition, false);
                returnValue.normalize(); // Normalize resulting quaternion to be sure to have a unit quaternion
                // (no normalization after this point, as the quaternion is not a unit-quaternion anymore!)
                for (unsigned int i = 1; i <= derivationOrder; i++)
                    returnValue = returnValue * q3;
            }

            // Pass back return value
            return returnValue;
        }

        // Evaluation of value (D0) up to N-th derivative (DN) of underlying quaternion curve (see base class for details)
        /*!
         * \copydoc FixedSizeQuaternionCurve::evaluateD0ToDN()
         *
         * Efficient **analytic** computation of D0, ..., DN all at once (re-use of intermediate results).
         *
         * \warning \p position has to within \f$[0,\,1]\f$, otherwise it will be projected!
         */
        template <unsigned int N>
        std::array<Eigen::Quaterniond, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Validity check
            assert(isValid());

            // Project interpolation parameter to boundaries
            double projectedPosition = position;
            if (projectedPosition < 0)
                projectedPosition = 0;
            if (projectedPosition > 1)
                projectedPosition = 1;

            // Initialize return value
            std::array<Eigen::Quaterniond, N + 1> Di;
            Di.fill(Eigen::Quaterniond(0, 0, 0, 0));

            /*! Value:
             * \f[ q(x) = SLERP(q_0,\,q_1,\,x) = q_0 \cdot (q_0^{-1} \cdot q_1)^x \f]
             */
            // IMPORTANT: we do NOT use Eigen's implementation of slerp here since Eigen's implementation automatically chooses the shortest path. However, here we want that the user has the option to choose the "direction" (by flipping the quaternions).
            Di[0] = geometry::quaternionSLERP(m_controlPoints[0], m_controlPoints[1], projectedPosition, false);
            Di[0].normalize();

            // Compute helper variables for derivatives (note: for unit quaternions the inverse is equal to the conjugate (conjugate is easier to compute))
            const Eigen::Quaterniond q0invq1 = m_controlPoints[0].conjugate() * m_controlPoints[1];
            if (q0invq1.x() != 0 || q0invq1.y() != 0 || q0invq1.z() != 0) {
                // ...start- and end-quaternion differ
                const Eigen::Quaterniond q3 = geometry::quaternionNaturalLogarithm(q0invq1);

                /*! Derivatives:
                 * \f[ \frac{\partial^d q(x)}{\partial x^d} = q(x) \cdot q_3^d \f]
                 */
                for (unsigned int d = 1; d <= N; d++)
                    Di[d] = Di[d - 1] * q3;
            } // else ...start- and end-quaternion describe the same orientation -> derivatives of q are zero (no change)!

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
            addedElements += io::encoding::encode(stream, "<QuaternionSLERPCurve");

            // Write attributes
            addedElements += encodeXMLAttributes(stream, numericFormat);

            // End XML element
            addedElements += io::encoding::encode(stream, "></QuaternionSLERPCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
