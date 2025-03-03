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
    //! Class abstracting quaternion cuves using cubic **Bezier** curves according to Shoemake 1985
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \warning This class is restricted to **unit-quaternions**!
     *
     * The shape of the curve is described by a cubic Bezier curve:
     * \code
     *       q1-----------------q2
     *       /       xxxxx       \
     *      /    xxxx     xxxx    \
     *     /  xxx             xxx  \
     *    / xx                   xx \
     *   /xx                       xx\
     *  /x                           x\
     * q0                             q3
     * \endcode
     *
     * While \f$ q_0 \f$ and \f$ q_3 \f$ describe the control points (quaternions) at the beginning and end of the curve, respectively, \f$ q_1 \f$ and \f$ q_2 \f$ are "virtual" control points used to design the shape of the curve.
     * Note that \f$ q_1 \f$ and \f$ q_2 \f$ may be used to interconnect segments in a smooth (\f$C^1\f$-continuous) way.
     *
     * References
     * ----------
     * * Ken Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH Computer Graphics, ACM, New York, NY, USA, volume 19, number 3, 1985, DOI:[10.1145/325165.325242](https://www.doi.org/10.1145/325165.325242), p.245--254
     * * Gerald Farin, "Curves and Surfaces for CAGD: A Practical Guide", Morgan Kaufmann Publishers, 2002, 5-th ed., ISBN: 1-55860-737-4
     */
    class QuaternionBezierCurve : public FixedSizeQuaternionCurve<4> {
    public:
        //! Specialized constructor
        /*!
         * \param [in] q0 The quaternion \f$ q_0 \f$ at the beginning of the curve (pass-through).
         * \param [in] q1 The first *virtual* control point \f$ q_1 \f$ used to define the shape of the curve (**no** pass-through)
         * \param [in] q2 The second *virtual* control point \f$ q_2 \f$ used to define the shape of the curve (**no** pass-through)
         * \param [in] q3 The quaternion \f$ q_3 \f$ at the end of the curve (pass-through).
         */
        QuaternionBezierCurve(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const Eigen::Quaterniond& q3)
        {
            // Set control points
            m_controlPoints[0] = q0;
            m_controlPoints[1] = q1;
            m_controlPoints[2] = q2;
            m_controlPoints[3] = q3;
        }

        //! Default constructor
        /*! Initializes as \f$ q_i = [w=1,\, x=0,\, y=0,\, z=0]^T \f$ for **all** control points */
        QuaternionBezierCurve()
            : FixedSizeQuaternionCurve(Eigen::Quaterniond(1, 0, 0, 0))
        {
        }

        //! Destructor
        virtual ~QuaternionBezierCurve()
        {
        }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::BEZIER; }

        // Evaluation of (derivative of) underlying curve (see base class for details)
        //! \copydoc QuaternionCurve::evaluate()
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
                 *  (De Casteljau's algorithm, see Farin 2002, p.43ff)
                 */

                /*! First De Casteljau iteration:
                 * \f[
                 * \begin{array}{lll}
                 *     q_{A,0}(x) &= SLERP(q_0,\,q_1,\,x) &= q_0 \cdot (q_0^{-1} \cdot q_1)^x\\
                 *     q_{A,1}(x) &= SLERP(q_1,\,q_2,\,x) &= q_1 \cdot (q_1^{-1} \cdot q_2)^x\\
                 *     q_{A,2}(x) &= SLERP(q_2,\,q_3,\,x) &= q_2 \cdot (q_2^{-1} \cdot q_3)^x
                 * \end{array}FixedSize
                 * \f]
                 */
                // IMPORTANT: we do NOT use Eigen's implementation of slerp here since Eigen's implementation automatically chooses the shortest path. This is done through an if-else branch which can lead to discontinuities in special cases!
                const Eigen::Quaterniond q_A0 = geometry::quaternionSLERP(m_controlPoints[0], m_controlPoints[1], projectedPosition, false);
                const Eigen::Quaterniond q_A1 = geometry::quaternionSLERP(m_controlPoints[1], m_controlPoints[2], projectedPosition, false);
                const Eigen::Quaterniond q_A2 = geometry::quaternionSLERP(m_controlPoints[2], m_controlPoints[3], projectedPosition, false);

                /*! Second De Casteljau iteration:
                 * \f[
                 * \begin{array}{lll}
                 *     q_{B,0}(x) &= SLERP(q_{A,0}(x),\,q_{A,1}(x),\,x) &= q_{A,0}(x) \cdot (q_{A,0}^{-1}(x) \cdot q_{A,1}(x))^x\\
                 *     q_{B,1}(x) &= SLERP(q_{A,1}(x),\,q_{A,2}(x),\,x) &= q_{A,1}(x) \cdot (q_{A,1}^{-1}(x) \cdot q_{A,2}(x))^x\\
                 * \end{array}
                 * \f]
                 */
                // IMPORTANT: we do NOT use Eigen's implementation of slerp here since Eigen's implementation automatically chooses the shortest path. This is done through an if-else branch which can lead to discontinuities in special cases!
                const Eigen::Quaterniond q_B0 = geometry::quaternionSLERP(q_A0, q_A1, projectedPosition, false);
                const Eigen::Quaterniond q_B1 = geometry::quaternionSLERP(q_A1, q_A2, projectedPosition, false);

                /*! Third (final) De Casteljau iteration:
                 * \f[
                 * \begin{array}{lll}
                 *     q(x) &= BEZIER(q_0,\,q_1,\,q_2,\,q_3,\,x) = SLERP(q_{B,0}(x),\,q_{B,1}(x),\,x) &= q_{B,0}(x) \cdot (q_{B,0}^{-1}(x) \cdot q_{B,1}(x))^x
                 * \end{array}
                 * \f]
                 *
                 * with \f$ x \in [0,\,1] \f$.
                 */
                // IMPORTANT: we do NOT use Eigen's implementation of slerp here since Eigen's implementation automatically chooses the shortest path. This is done through an if-else branch which can lead to discontinuities in special cases!
                returnValue = geometry::quaternionSLERP(q_B0, q_B1, projectedPosition, false);
                returnValue.normalize(); // Normalize resulting quaternion to be sure to have a unit quaternion
            } else {
                /*! Evaluation of derivatives
                 *  -------------------------
                 * Computes **numeric** derivative with respect to the interpolation parameter \f$ x \f$ (**not** time in general).
                 * See \ref QuaternionCurve::evaluateNumericDerivative()
                 *
                 * \note There is no known closed form (analytic) solution for the \f$ n\f$-th derivative of this curve.
                 */
                returnValue = evaluateNumericDerivative(position, derivationOrder);
            }

            // Pass back return value
            return returnValue;
        }

        // Evaluation of value (D0) up to N-th derivative (DN) of underlying quaternion curve (see base class for details)
        /*!
         * \copydoc FixedSizeQuaternionCurve::evaluateD0ToDN()
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

            // Trigger default implementation
            return FixedSizeQuaternionCurve::template evaluateD0ToDN<N>(projectedPosition);
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
            addedElements += io::encoding::encode(stream, "<QuaternionBezierCurve");

            // Write attributes
            addedElements += encodeXMLAttributes(stream, numericFormat);

            // End XML element
            addedElements += io::encoding::encode(stream, "></QuaternionBezierCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
