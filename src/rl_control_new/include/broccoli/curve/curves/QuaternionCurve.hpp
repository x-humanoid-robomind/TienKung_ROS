/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/math.hpp"
#include "../../io/encoding.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <assert.h>
#include <stdint.h>
#include <string>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_curves
     * \{
     */

    //! Specification of result types of quaternion curve algorithms
    enum class QuaternionCurveResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_PARAMETERS, //!< An **error** occured: invalid input parameters
        ERROR_NOTIMPLEMENTED, //!< An **error** occured: the desired algorithm is **not** yet implemented
        QUATERNIONCURVERESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type for quaternion curve algorithms
    static inline std::string quaternionCurveResultString(const QuaternionCurveResult& result)
    {
        // Check result
        switch (result) {
        case QuaternionCurveResult::UNKNOWN:
            return "UNKNOWN";
        case QuaternionCurveResult::SUCCESS:
            return "SUCCESS";
        case QuaternionCurveResult::ERROR_INVALID_PARAMETERS:
            return "ERROR_INVALID_PARAMETERS";
        case QuaternionCurveResult::ERROR_NOTIMPLEMENTED:
            return "ERROR_NOTIMPLEMENTED";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Abstract representation of a quaternion curve (underyling function is not specified yet)
    /*!
     * Class acts as a prototype and declares all functions and members a quaternion curve should provide.
     *
     * The underlying curve is given with respect to `position` which is **not** the time in general. Instead
     * `position` describes the interpolation parameter within the interval \f$ [0, 1] \f$ where 0 corresponds to the
     * start-quaternion and 1 to the end-quaternion. This implies, that one has to be careful when using the
     * \ref evaluate function to get derivatives (use chain rule!).
     */
    class QuaternionCurve {
    public:
        //! Specification of possible function types
        enum class FunctionType : uint8_t {
            LERP = 0, //!< <b>L</b>inear int<b>ERP</b>olation between start-quaternion and end-quaternion
            NLERP, //!< <b>N</b>ormalized <b>L</b>inear int<b>ERP</b>olation between start-quaternion and end-quaternion
            SLERP, //!< <b>S</b>pherical <b>L</b>inear int<b>ERP</b>olation between start-quaternion and end-quaternion
            BEZIER, //!< Interpolation using cubic **Bezier** curves (two extra quaternions as control points)
            SQUAD, //!< <b>S</b>pherical <b>QUAD</b>rangle curve interpolation (two extra quaternions as control points)
            BSPLINE, //!< Interpolation using **B-spline** quaternion curve
            FUNCTIONTYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given function type
        static std::string functionTypeString(const FunctionType& type)
        {
            // Check type
            switch (type) {
            case FunctionType::LERP:
                return "LERP";
            case FunctionType::NLERP:
                return "NLERP";
            case FunctionType::SLERP:
                return "SLERP";
            case FunctionType::BEZIER:
                return "BEZIER";
            case FunctionType::SQUAD:
                return "SQUAD";
            case FunctionType::BSPLINE:
                return "BSPLINE";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Constructor
        QuaternionCurve()
        {
        }

        //! Destructor
        virtual ~QuaternionCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const QuaternionCurve& reference) const
        {
            (void)reference; // Unused

            // No own members -> always equal
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const QuaternionCurve& reference) const { return !(*this == reference); }

        //! Checks, if the curve is properly defined
        /*! \return `true`, if the curve is valid, `false` otherwise. */
        virtual bool isValid() const
        {
            // Dummy implementation: always valid - derived classes may check members
            return true;
        }

        //! Get type of underlying function
        virtual FunctionType functionType() const = 0;

        //! Evaluation of (derivative of) curve
        /*!
         * \warning You may call \ref isValid() before to check, if the quaternion curve is valid! If the quaternion curve is invalid the behaviour of this method is not defined!
         *
         * \warning For \p derivationOrder > 0 the evaluate function does **not** return the angular velocity/acceleration/...
         * but rather the "pure" derivative of the quaternion with respect to the interpolation parameter (not time)!
         *
         * \param [in] position Position to evaluate (derivative) at (should - but has not to be - within \f$[0,\,1]\f$)
         * \param [in] derivationOrder Order of the derivation (use 0 for base function, i.e. no derivation)
         * \return Value (of derivative) at specified position
         */
        virtual Eigen::Quaterniond evaluate(const double& position, const unsigned int& derivationOrder = 0) const = 0;

        //! Evaluation of value (D0) up to N-th derivative (DN) of underlying quaternion curve (improved performance over calling \ref evaluate() N+1 times)
        /*!
         * \warning You may call \ref isValid() before to check, if the quaternion curve is valid! If the quaternion curve is invalid the behaviour of this method is not defined!
         *
         * \warning The derivatives do **not** represent the angular velocity/acceleration/... but rather the "pure" derivatives
         * of the quaternion with respect to the interpolation parameter (not time)!
         *
         * \tparam N Order of highest derivative
         * \param [in] position Position to evaluate (derivatives) at (should - but has not to be - within \f$[0,\,1]\f$)
         * \return List of value (D0) up to N-th derivative [D0, D1, ... , Di, ... , DN] at specified position
         */
        template <unsigned int N>
        std::array<Eigen::Quaterniond, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Validity check
            assert(isValid());

            // Default implementation: just call evaluate N+1 times - derived classes may replace this with a more efficient method
            std::array<Eigen::Quaterniond, N + 1> Di;
            for (unsigned int d = 0; d <= N; d++)
                Di[d] = evaluate(position, d);
            return Di;
        }

        //! Computes **numeric** \f$ n\f$-th derivative of underlying curve using finite difference method
        /*!
         * \warning You may call \ref isValid() before to check, if the quaternion curve is valid! If the quaternion curve is invalid the behaviour of this method is not defined!
         *
         * \warning Derivatives do **not** represent the angular velocity/acceleration/... but rather the "pure" derivative of the quaternion with respect to the interpolation parameter.
         *
         * \param [in] position Position \f$ x\f$ to evaluate numeric derivative at (has to be within \f$[0,\,1]\f$, otherwise it will be projected to these limits)
         * \param [in] derivationOrder Order \f$ n\f$ of the derivation (use 0 for base function, i.e. no derivation)
         * \param [in] stepSize The step size alias \f$ h \f$ used for evaluating the underlying curve at certain positions
         * \return \f$ n \f$-th derivative at specified position
         */
        virtual Eigen::Quaterniond evaluateNumericDerivative(const double& position, const unsigned int& derivationOrder, const double& stepSize = 1e-6) const
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
                return Eigen::Quaterniond(0, 0, 0, 0);
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
            Eigen::Quaterniond returnValue = Eigen::Quaterniond(0, 0, 0, 0);
            for (uint64_t i = 0; i <= derivationOrder; i++) {
                double coefficient = core::math::binomialCoefficient(derivationOrder, i);
                if (i % 2 == 1)
                    coefficient = -coefficient;

                /*! **Forward:** for \f$ x - \frac{n\,h}{2} < 0\f$:
                 * \f[ \frac{\partial^n q(x)}{\partial x^n} \approx \frac{1}{h^n} \left[ \sum_{i=0}^n \left( -1 \right)^i \left( \begin{array}{c} n\\i\end{array} \right)\,q\left(x+\left(n-i\right)\,h\right)\right] \f]
                 */
                if (mode == 1)
                    returnValue.coeffs() += coefficient * evaluate(projectedPosition + (derivationOrder - i) * stepSize, 0).coeffs();
                /*! **Central:** for \f$ x - \frac{n\,h}{2} \geq 0\f$ and \f$ x + \frac{n\,h}{2} \leq 1\f$:
                 * \f[ \frac{\partial^n q(x)}{\partial x^n} \approx \frac{1}{h^n} \left[ \sum_{i=0}^n \left( -1 \right)^i \left( \begin{array}{c} n\\i\end{array} \right)\,q\left(x+\left(\frac{n}{2}-i\right)\,h\right)\right] \f]
                 */
                else if (mode == 0)
                    returnValue.coeffs() += coefficient * evaluate(projectedPosition + (derivationOrder / 2.0 - i) * stepSize, 0).coeffs();
                /*! **Backward:** for \f$ x + \frac{n\,h}{2} > 1\f$:
                 * \f[ \frac{\partial^n q(x)}{\partial x^n} \approx \frac{1}{h^n} \left[ \sum_{i=0}^n \left( -1 \right)^i \left( \begin{array}{c} n\\i\end{array} \right)\,q\left(x-i\,h\right)\right] \f]
                 */
                else
                    returnValue.coeffs() += coefficient * evaluate(projectedPosition - i * stepSize, 0).coeffs();
            }
            for (uint64_t i = 0; i < derivationOrder; i++)
                returnValue.coeffs() /= stepSize;

            // Pass back result
            return returnValue;
        }

        // Helpers
        // -------
        //! Recomputes sequence of keyframe quaternions by flipping single keyframes such that two neighboring keyframes have the shortest distance.
        /*!
         * Starting from the first keyframe each pair of neighboring keyframes is checked. This method is used to prevent unexpected motion during interpolation between
         * **antipodal** keyframes (\f$q\f$ and \f$-q\f$ describe the same rotation!).
         *
         * \param [in] keyFrames List of **input** keyframe quaternions
         * \return Recalculated list of keyframe quaternions with shortest difference between each other
         */
        static inline std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> recalculateKeyFrameSequence(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& keyFrames)
        {
            // Initialize helpers
            std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> recalculatedList = keyFrames; // Working copy of keyframe list

            // Pass through each consecutive "pair" of keyframes
            for (size_t i = 0; i < recalculatedList.size() - 1; i++) {
                // Check dot product of quaternions
                if (recalculatedList[i].dot(recalculatedList[i + 1]) < 0) {
                    // "Flipped" version has shorter distance -> use flipped version
                    recalculatedList[i + 1].coeffs() = -recalculatedList[i + 1].coeffs();
                }
            }

            // Pass back recalculated list
            return recalculatedList;
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

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

    //! \}
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
