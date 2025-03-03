/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/math.hpp"
#include "../splines/QuaternionSpline.hpp"
#include "../splines/Spline.hpp"
#include <Eigen/StdVector>
#include <array>

namespace broccoli {
namespace curve {
    /*!
     * \addtogroup broccoli_curve_trajectories
     * \{
     */

    //! Specification of result types of quaternion-trajectory algorithms
    enum class QuaternionTrajectoryResult : uint8_t {
        UNKNOWN = 0, //!< Unknown result (this should **never** happen)
        SUCCESS, //!< Algorithm was successful
        ERROR_INVALID_TRAJECTORY_PARAMETERSPLINE, //!< An **error** occured: Underlying parameter-spline is invalid
        ERROR_INVALID_TRAJECTORY_QUATERNIONSPLINE, //!< An **error** occured: Underlying quaternion-spline is invalid
        ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE, //!< An **error** occured: Duration is negative
        ERROR_INVALID_TRAJECTORIES, //!< An **error** occured: the list of quaternion-trajectories or at least one element of it is invalid
        ERROR_PARAMETERSPLINEERROR, //!< An **error** occured: an algorithm related to the underlying parameter-spline failed
        ERROR_QUATERNIONSPLINEERROR, //!< An **error** occured: an algorithm related to the underlying quaternion-spline failed
        ERROR_NOTIMPLEMENTED, //!< An **error** occured: the desired algorithm is **not** yet implemented
        QUATERNIONTRAJECTORYRESULT_COUNT //!< Total count of elements
    };

    //! Returns the string representation of the given result type for quaternion-trajectory algorithms
    static inline std::string quaternionTrajectoryResultString(const QuaternionTrajectoryResult& result)
    {
        // Check result
        switch (result) {
        case QuaternionTrajectoryResult::UNKNOWN:
            return "UNKNOWN";
        case QuaternionTrajectoryResult::SUCCESS:
            return "SUCCESS";
        case QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORY_PARAMETERSPLINE:
            return "ERROR_INVALID_TRAJECTORY_PARAMETERSPLINE";
        case QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORY_QUATERNIONSPLINE:
            return "ERROR_INVALID_TRAJECTORY_QUATERNIONSPLINE";
        case QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE:
            return "ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE";
        case QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORIES:
            return "ERROR_INVALID_TRAJECTORIES";
        case QuaternionTrajectoryResult::ERROR_PARAMETERSPLINEERROR:
            return "ERROR_PARAMETERSPLINEERROR";
        case QuaternionTrajectoryResult::ERROR_QUATERNIONSPLINEERROR:
            return "ERROR_QUATERNIONSPLINEERROR";
        case QuaternionTrajectoryResult::ERROR_NOTIMPLEMENTED:
            return "ERROR_NOTIMPLEMENTED";
        default: {
            assert(false);
            return "UNKNOWN";
        }
        }
    }

    //! Template class for quaternion trajectories
    /*!
     * \warning This class is restricted to **unit quaternions**!
     *
     * \tparam QuaternionSplineType Specifies the type of the underlying *quaternion-spline*.
     * \tparam ParameterSplineType Specifies the type of the underlying *parameter-spline*.
     *
     * This template class combines a *quaternion-spline* (\ref m_quaternionSpline) with an additional
     * one-dimensional *parameter-spline* (\ref m_parameterSpline) describing the current interpolation
     * position (i.e. the time mapping). The additional *parameter-spline* gives more freedom during
     * interpolation. The "function value" of the *parameter-spline* defines the interpolation parameter
     * used to interpolate the *quaternion-spline* and has to lie within \f$ [0,\,1]\f$ (range of
     * *quaternion-spline*). Values outside will be projected to these boundaries. Note that the segmentation
     * (segment-count and proportions) of the *parameter-spline* and the *quaternion-spline* do **not** have
     * to be the same. However, the *parameter-spline* needs to be **monotonically increasing** (otherwise
     * there may be more than one quaternion curve segments linked to a specific *parameter-spline* value
     * which would result in indefinite behaviour!).
     *
     * \par Example A: same segmentation
     * \code
     * Quaternion-spline Keyframes: qk0               qk1               qk2               qk3               qk4
     * Quaternion-spline Segments:   | ----- qs0 ----- | ----- qs1 ----- | ----- qs2 ----- | ----- qs3 ----- |
     * Parameter-spline Values:     0.00 [0.00, 0.25] 0.25 [0.25, 0.50] 0.50 [0.50, 0.75] 0.75 [0.75, 1.00] 1.00
     * Parameter-spline Segments:    | ----- ps0 ----- | ----- ps1 ----- | ----- ps2 ----- | ----- ps3 ----- |
     * Trajectory-Duration:          | ---------------------------- (m_duration) --------------------------- |
     * \endcode
     *
     * \par Example B: different segmentation: (linked interconnection)
     * \code
     * Quaternion-spline Keyframes: qk0               qk1               qk2               qk3               qk4
     * Quaternion-spline Segments:   | ----- qs0 ----- | ----- qs1 ----- | ----- qs2 ----- | ----- qs3 ----- |
     * Parameter-spline Values:     0.00         [0.00, 0.50]           0.50          [0.50, 1.00]          1.00
     * Parameter-spline Segments:    | -------------- ps0 -------------- | -------------- ps1 -------------- |
     * Trajectory-Duration:          | ---------------------------- (m_duration) --------------------------- |
     * \endcode
     *
     * \par Example C: different segmentation: (completely custom)
     * \code
     * Quaternion-spline Keyframes: qk0               qk1               qk2               qk3               qk4
     * Quaternion-spline Segments:   | ----- qs0 ----- | ----- qs1 ----- | ----- qs2 ----- | ----- qs3 ----- |
     * Parameter-spline Values:     0.00   [0.00, 0.30]     0.30   [0.30, 0.70]     0.70   [0.70, 1.00]     1.00
     * Parameter-spline Segments:    | -------- ps0 -------- | -------- ps1 -------- | -------- ps2 -------- |
     * Trajectory-Duration:          | ---------------------------- (m_duration) --------------------------- |
     * \endcode
     */
    template <class QuaternionSplineType, class ParameterSplineType>
    class QuaternionTrajectory {
    public:
        //! Default constructor (initializes with zero duration)
        QuaternionTrajectory()
            : m_duration(0)
        {
        }

        //! Destructor
        virtual ~QuaternionTrajectory()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const QuaternionTrajectory<QuaternionSplineType, ParameterSplineType>& reference) const
        {
            // Compare duration
            if (m_duration != reference.m_duration)
                return false;

            // Compare splines
            if (m_parameterSpline != reference.m_parameterSpline)
                return false;
            if (m_quaternionSpline != reference.m_quaternionSpline)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const QuaternionTrajectory<QuaternionSplineType, ParameterSplineType>& reference) const { return !(*this == reference); }

        // Members
        QuaternionSplineType m_quaternionSpline; //!< Actual *quaternion-spline* describing the quaternion interpolation curve
        ParameterSplineType m_parameterSpline; //!< One-dimensional auxillary *parameter-spline* for time mapping
        double m_duration; //!< Duration [s] for travelling through the whole spline (from 0 to 1 in *spline-coordinates*)

        //! Checks, if the quaternion-trajectory is properly defined (proper definition of duration and splines)
        /*!
         * \param [out] trajectoryResult Pointer to flag in which the result concerning the quaternion-trajectory should be stored (optional, use `nullptr` to skip output)
         * \param [out] parameterSplineResult Pointer to flag in which the result concerning the underlying parameter spline should be stored (optional, use `nullptr` to skip output)
         * \param [out] quaternionSplineResult Pointer to flag in which the result concerning the underlying quaternion spline should be stored (optional, use `nullptr` to skip output)
         * \return `true`, if the quaternion-trajectory is valid, `false` otherwise.
         */
        virtual bool isValid(QuaternionTrajectoryResult* const trajectoryResult, SplineResult* const parameterSplineResult, QuaternionSplineResult* const quaternionSplineResult) const
        {
            // Reset outputs
            if (trajectoryResult != nullptr)
                *trajectoryResult = QuaternionTrajectoryResult::UNKNOWN;
            if (parameterSplineResult != nullptr)
                *parameterSplineResult = SplineResult::UNKNOWN;
            if (quaternionSplineResult != nullptr)
                *quaternionSplineResult = QuaternionSplineResult::UNKNOWN;

            // Check, if duration is non-negative
            if (m_duration < 0) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORY_DURATION_NEGATIVE;
                return false;
            }

            // Check, if underlying parameter spline is valid
            if (m_parameterSpline.isValid(parameterSplineResult) == false) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORY_PARAMETERSPLINE;
                return false;
            }

            // Check, if underlying quaternion spline is valid
            if (m_quaternionSpline.isValid(quaternionSplineResult) == false) {
                if (trajectoryResult != nullptr)
                    *trajectoryResult = QuaternionTrajectoryResult::ERROR_INVALID_TRAJECTORY_QUATERNIONSPLINE;
                return false;
            }

            // Otherwise -> valid
            if (trajectoryResult != nullptr)
                *trajectoryResult = QuaternionTrajectoryResult::SUCCESS;
            return true;
        }

        //! Computes the **parameter spline** segment boundaries ("control points") represented in **time domain** ([0, m_duration])
        /*!
         * \warning You may call \ref isValid() before to check, if the quaternion-trajectory is valid! If the quaternion-trajectory is invalid the behaviour of this method is not defined!
         *
         * \warning In general the segment boundaries of the parameter spline do **not** coincide with the segment boundaries of the quaternion spline! Note that computing the segment boundaries
         * of the quaternion spline in *time domain* is not trivial, since one has to compute the inverse of the parameter-spline mapping (which heavily depends on the curve-type of the
         * parameter spline and may have no, one, multiple, or infinitely many solutions). However, if one chooses a parametrization as in **Example A** in \ref QuaternionTrajectory one may
         * use this method to obtain the quaternion spline segment boundaries in time domain.
         *
         * \return Ordered list of *parameter spline* segment boundaries, or empty list in case of an error.
         *
         * \code
         * |--------T--------|     <-- quaternion-trajectory
         * |---s1---|---s2---|     <-- parameter spline segments
         * B1       B2      B3     <-- parameter spline segment boundaries
         * 0       ...  m_duration <-- returned values (in time domain)
         * \endcode
         */
        virtual std::vector<double> getParameterSplineSegmentBoundaries() const
        {
            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Get boundaries in spline coordinates
            std::vector<double> boundaries = m_parameterSpline.getSegmentBoundaries();

            // Transform from spline-coordinates to time domain
            for (size_t i = 0; i < boundaries.size(); i++)
                boundaries[i] *= m_duration;

            // Pass back list
            return boundaries;
        }

        //! Evaluation of (derivative of) quaternion trajectory
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * \warning For \p derivationOrder > 0 the evaluate function does **not** return the angular velocity/acceleration/...
         * but rather the "pure" derivative of the quaternion with respect to time (**not** relative to *segment-coordinates* or
         * *spline-coordinates*)! You may want to use the functions \ref evaluateAngularVelocity() or
         * \ref evaluateAngularAcceleration() (or the much more efficient version \ref evaluateD0V() or \ref evaluateD0VA()) instead.
         *
         * \warning Due to the chain rule and a possibly non-linear *parameter-spline*, the computation of higher order derivatives
         * is computationally very expensive (uses Faà di Bruno's formula)! However for the first three derivatives the formulas are
         * implemented "manually" for speed-up.
         *
         * \param [in] time Time point to evaluate *parameter-spline* at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \param [in] derivationOrder Order of the derivation (use 0 for base function, i.e. no derivation)
         * \return Value or derviative at specified time - gradient is expressed relative to time
         */
        virtual Eigen::Quaterniond evaluate(const double& time, const unsigned int& derivationOrder = 0) const
        {
            /*! Notation
             *  --------
             *  * \f$q(p)\f$ ...quaternion-spline (depends on parameter-spline value \f$p\f$)
             *  * \f$p(t)\f$ ...parameter-spline (depends on given time \f$t\f$)
             *
             * \f$ \rightarrow q(t) = q(p(t))\f$
             */

            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Project time to boundaries
            double projectedTime = time;
            if (projectedTime < 0)
                projectedTime = 0;
            if (projectedTime > m_duration)
                projectedTime = m_duration;

            // Compute parameter-spline position (in [0, 1]) (linear mapping from projected time)
            double parameterSplinePosition = 0;
            if (m_duration > 0) // If no valid duration is given -> use starting point of parameter-spline
                parameterSplinePosition = projectedTime / m_duration;

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                // Evaluate quaternion-spline q = q(p(t))
                return m_quaternionSpline.evaluate(m_parameterSpline.evaluate(parameterSplinePosition, 0), 0);
            } else {
                /*!
                 * Derivatives
                 * -----------
                 * Chain rule: (note that \f$p(t)\f$ is scalar!)
                 *  * 1-st derivative: \f$ \frac{dq}{dt} = \frac{\partial q}{\partial p} \cdot \frac{dp}{dt} \f$
                 *  * 2-nd derivative: \f$ \frac{d^2q}{dt^2} = \frac{\partial^2q}{\partial p^2} \cdot \left(\frac{dp}{dt}\right)^2 + \frac{\partial q}{\partial p} \cdot \frac{d^2p}{dt^2} \f$
                 *  * 3-rd derivative: \f$ \frac{d^3q}{dt^3} = \frac{\partial^3q}{\partial p^3} \cdot \left(\frac{dp}{dt}\right)^3 + 3 \cdot \frac{\partial^2q}{\partial p^2} \cdot \frac{dp}{dt} \cdot \frac{d^2p}{dt^2} + \frac{\partial q}{\partial p} \cdot \frac{d^3p}{dt^3} \f$
                 *  * ...
                 *  * n-th derivative: (Faà di Bruno's formula)
                 * \f[
                 * \frac{d^nq}{dt^n} = D^{(n)}q(t) = \sum \left[ \frac{n!}{(m_1!\cdot 1!^{m_1} \cdot m_2!\cdot 2!^{m_2} \cdot \cdots \cdot m_n!\cdot n!^{m_n})} \cdot D^{(m_1+...+m_n)}q(p) \cdot \prod_{j=1}^{n} (D^{(j)}p(t))^{m_j} \right]
                 * \f]
                 * with the sum over all tuples \f$ (m_1,\,m_2,\,\dots\,,\,m_n)\f$ of nonnegative integers satisfying \f$ 1\cdot m_1 + 2\cdot m_2 + \cdots + n\cdot m_n = n \f$
                 */

                // Prepare return value
                Eigen::Quaterniond returnValue(0, 0, 0, 0);

                // Check if duration is zero
                // -------------------------
                if (m_duration <= 0) {
                    // ...-> derivative would be infinite so pass back zero instead to avoid division by zero
                    return Eigen::Quaterniond(0, 0, 0, 0);
                }

                // Manual implementation of first derivatives (for speedup)
                // ------------------------------------------
                if (derivationOrder == 1) {
                    // Manual implementation for 1-st derivative:
                    // ------------------------------------------
                    // dq/dt = dq/dp * dp/dt

                    returnValue.coeffs() = m_quaternionSpline.evaluate(m_parameterSpline.evaluate(parameterSplinePosition, 0), 1).coeffs() * m_parameterSpline.evaluate(parameterSplinePosition, 1);
                } else if (derivationOrder == 2) {
                    // Manual implementation for 2-nd derivative: (for speedup)
                    // ------------------------------------------
                    // d^2q/dt^2 = d^2q/dp^2 * (dp/dt)^2 + dq/dp * d^2/dt^2

                    // Evaluate parameter-spline value and its derivatives
                    const std::array<double, 3> Dpi = m_parameterSpline.template evaluateD0ToDN<2>(parameterSplinePosition);

                    // Evaluate quaternion-spline value and its derivatives
                    const std::array<Eigen::Quaterniond, 3> Dqi = m_quaternionSpline.template evaluateD0ToDN<2>(Dpi[0]);

                    // First part: d^2q/dp^2 * (dp/dt)^2
                    returnValue.coeffs() = Dqi[2].coeffs() * (Dpi[1] * Dpi[1]);

                    // Second part: dq/dp * d^2p/dt^2
                    returnValue.coeffs() += Dqi[1].coeffs() * Dpi[2];
                } else if (derivationOrder == 3) {
                    // Manual implementation for 3-rd derivative: (for speedup)
                    // ------------------------------------------
                    // d^3q/dt^3 = d^3q/dp^3 * (dp/dt)^3 + 3 * d^2q/dp^2 * dp/dt * d^2/dt^2 + dq/dp * d^3p/dt^3

                    // Evaluate parameter-spline value and its derivatives
                    const std::array<double, 4> Dpi = m_parameterSpline.template evaluateD0ToDN<3>(parameterSplinePosition);

                    // Evaluate quaternion-spline value and its derivatives
                    const std::array<Eigen::Quaterniond, 4> Dqi = m_quaternionSpline.template evaluateD0ToDN<3>(Dpi[0]);

                    // First part: d^3/dp^3 * (dp/dt)^3
                    returnValue.coeffs() = Dqi[3].coeffs() * (Dpi[1] * Dpi[1] * Dpi[1]);

                    // Second part: 3 * d^2q/dp^2 * dp/dt * d^2p/dt^2
                    returnValue.coeffs() += 3.0 * Dqi[2].coeffs() * (Dpi[1] * Dpi[2]);

                    // Third part: dq/dp * d^3p/dt^3
                    returnValue.coeffs() += Dqi[1].coeffs() * Dpi[3];
                } else {
                    // Algorithmic impementation for i-th derivative (Faà di Bruno's formula)
                    // ---------------------------------------------
                    // d^nq/dt^n = D^(n)q(t) = sum [ n! / (m1!*1!^m1 * m2!*2!^m2 * ... * mn!*n!^mn) * D^(m1+...+mn)q(p) * product_{j=1}^{n} (D^(j)p(t))^mj ]

                    // Step 1: Compute tuples (m1, m2, ..., mn)
                    // ----------------------------------------
                    const core::math::FaaDiBrunoTuples tuples = core::math::faaDiBrunoTuples(derivationOrder);

                    // Step 2: Compute sum
                    // -------------------
                    // Evaluate parameter-spline value and its derivatives
                    std::vector<double> Dpi(1 + derivationOrder);
                    for (unsigned int d = 0; d <= derivationOrder; d++)
                        Dpi[d] = m_parameterSpline.evaluate(parameterSplinePosition, d);

                    // Evaluate quaternion-spline value and its derivatives
                    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> Dqi(1 + derivationOrder);
                    for (unsigned int d = 1 /* <-- Skip base value for d = 0 (not needed) */; d <= derivationOrder; d++)
                        Dqi[d] = m_quaternionSpline.evaluate(Dpi[0], d);

                    // Loop over all contributors
                    for (size_t i = 0; i < tuples.size(); i++) {
                        // Step 2.1: Compute denominator of leading fraction
                        // -------------------------------------------------
                        // (m1!*1!^m1 * m2!*2!^m2 * ... * mn!*n!^mn)
                        double denominator = 1;
                        double factorialOfIndex = 1; // Factorial of m's index
                        for (size_t j = 1; j <= derivationOrder; j++) {
                            denominator *= core::math::factorial(tuples[i][j - 1]); // mj!
                            factorialOfIndex *= j;
                            for (uint64_t k = 1; k <= tuples[i][j - 1]; k++)
                                denominator *= factorialOfIndex;
                        }

                        // Step 2.2: Compute leading fraction
                        // ----------------------------------
                        // n! / (m1!*1!^m1 * m2!*2!^m2 * ... * mn!*n!^mn)
                        const double leadingFraction = factorialOfIndex / denominator;

                        // Step 2.3: Compute (m1+...+mn)-th derivative of q with respect to p
                        // ------------------------------------------------------------------
                        // D^(m1+...+mn)q(p)
                        double order = 0;
                        for (size_t j = 0; j < derivationOrder; j++)
                            order += tuples[i][j];
                        const Eigen::Quaterniond& derivationOfq = Dqi[order];

                        // Step 2.4: Compute trailing product
                        // ----------------------------------
                        // product_{j=1}^{n} (D^(j)p(t))^mj
                        double trailingProduct = 1;
                        for (size_t j = 1; j <= derivationOrder; j++) {
                            // Check, if mj > 0, otherwise the 0-th power of anything is 1 -> has no effect
                            if (tuples[i][j - 1] > 0) {
                                // Step 2.4.1: Compute j-th derivative of p with respect to t
                                // ----------------------------------------------------------
                                // D^(j)p(t)
                                const double& derivativeOfp = Dpi[j];

                                // Step 2.4.2: Compute mj-th power of this
                                // ---------------------------------------
                                // (D^(j)p(t))^mj
                                double powerOfDerivativeOfp = derivativeOfp;
                                for (size_t k = 1; k < tuples[i][j - 1]; k++)
                                    powerOfDerivativeOfp *= derivativeOfp;

                                // Step 2.4.3: Add contribution to product
                                // ---------------------------------------
                                trailingProduct *= powerOfDerivativeOfp;
                            }
                        }

                        // Step 2.5: Add contribution to sum (component-wise addition)
                        // ---------------------------------
                        if (leadingFraction != 0 && trailingProduct != 0)
                            returnValue.coeffs() += (leadingFraction * trailingProduct) * derivationOfq.coeffs();
                    }
                }

                // Step 3: Chain rule for mapping *parameter-spline* [0, 1] to actual time [0, m_duration]
                // ---------------------------------------------------------------------------------------
                // Note: since the relationship is linear, the chain rule is easy again (no need for Faà di Bruno's formula here)

                // Apply coordinate transformation (chain rule) for derivatives
                double chainRuleMultiplier = 1.0 / m_duration;
                for (unsigned int i = 2; i <= derivationOrder; i++)
                    chainRuleMultiplier /= m_duration;
                returnValue.coeffs() *= chainRuleMultiplier;

                // Pass back result
                return returnValue;
            }
        }

        //! Evaluation of value (D0) up to N-th derivative (DN) of quaternion trajectory (improved performance over calling \ref evaluate() N+1 times)
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * \warning For \p derivationOrder > 0 the evaluate function does **not** return the angular velocity/acceleration/...
         * but rather the "pure" derivative of the quaternion with respect to time (**not** relative to *segment-coordinates* or
         * *spline-coordinates*)! You may want to use the functions \ref evaluateAngularVelocity() or
         * \ref evaluateAngularAcceleration() (or the much more efficient version \ref evaluateD0V() or \ref evaluateD0VA()) instead.
         *
         * \warning Due to the chain rule and a possibly non-linear *parameter-spline*, the computation of higher order derivatives
         * is computationally very expensive (uses Faà di Bruno's formula)! However for the first three derivatives the formulas are
         * implemented "manually" for speed-up.
         *
         * \tparam N Order of highest derivative
         * \param [in] time Time point to evaluate *parameter-spline* at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \return List of value (D0) up to N-th derivative [D0, D1, ... , Di, ... , DN] at specified position - gradients are expressed relative to time
         */
        template <unsigned int N>
        std::array<Eigen::Quaterniond, N + 1> evaluateD0ToDN(const double& time) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Prepare return value
            std::array<Eigen::Quaterniond, N + 1> Di;
            Di.fill(Eigen::Quaterniond(0, 0, 0, 0));

            // Project time to boundaries
            double projectedTime = time;
            if (projectedTime < 0)
                projectedTime = 0;
            if (projectedTime > m_duration)
                projectedTime = m_duration;

            // Compute parameter-spline position (in [0, 1]) (linear mapping from projected time)
            double parameterSplinePosition = 0;
            if (m_duration > 0) // If no valid duration is given -> use starting point of parameter-spline
                parameterSplinePosition = projectedTime / m_duration;

            // Evaluate parameter-spline value and its derivatives
            const std::array<double, N + 1> Dpi = m_parameterSpline.template evaluateD0ToDN<N>(parameterSplinePosition);

            // Evaluate quaternion-spline value and its derivatives
            const std::array<Eigen::Quaterniond, N + 1> Dqi = m_quaternionSpline.template evaluateD0ToDN<N>(Dpi[0]);

            // Fill value
            // ----------
            Di[0] = Dqi[0];

            // Check if duration is zero
            // -------------------------
            if (m_duration <= 0) // ...-> derivatives would be infinite so pass back list at this point already (use zero derivatives instead)
                return Di;

            // Manual implementation for 1-st derivative: (for speedup)
            // ------------------------------------------
            // dq/dt = dq/dp * dp/dt
            if (N >= 1)
                Di[1].coeffs() = Dqi[1].coeffs() * Dpi[1];

            // Manual implementation for 2-nd derivative: (for speedup)
            // ------------------------------------------
            // d^2q/dt^2 = d^2q/dp^2 * (dp/dt)^2 + dq/dp * d^2/dt^2
            if (N >= 2) {
                // First part: d^2q/dp^2 * (dp/dt)^2
                Di[2].coeffs() = Dqi[2].coeffs() * (Dpi[1] * Dpi[1]);

                // Second part: dq/dp * d^2p/dt^2
                Di[2].coeffs() += Dqi[1].coeffs() * Dpi[2];
            }

            // Manual implementation for 3-rd derivative: (for speedup)
            // ------------------------------------------
            // d^3q/dt^3 = d^3q/dp^3 * (dp/dt)^3 + 3 * d^2q/dp^2 * dp/dt * d^2/dt^2 + dq/dp * d^3p/dt^3
            if (N >= 3) {
                // First part: d^3/dp^3 * (dp/dt)^3
                Di[3].coeffs() = Dqi[3].coeffs() * (Dpi[1] * Dpi[1] * Dpi[1]);

                // Second part: 3 * d^2q/dp^2 * dp/dt * d^2p/dt^2
                Di[3].coeffs() += 3.0 * Dqi[2].coeffs() * (Dpi[1] * Dpi[2]);

                // Third part: dq/dp * d^3p/dt^3
                Di[3].coeffs() += Dqi[1].coeffs() * Dpi[3];
            }

            // Algorithmic impementation for i-th derivative (Faà di Bruno's formula)
            // ---------------------------------------------
            // d^nq/dt^n = D^(n)q(t) = sum [ n! / (m1!*1!^m1 * m2!*2!^m2 * ... * mn!*n!^mn) * D^(m1+...+mn)q(p) * product_{j=1}^{n} (D^(j)p(t))^mj ]
            for (unsigned int d = 4; d <= N; d++) {
                // Step 1: Compute tuples (m1, m2, ..., mn)
                // ----------------------------------------
                const core::math::FaaDiBrunoTuples tuples = core::math::faaDiBrunoTuples(d);

                // Step 2: Compute sum
                // -------------------
                for (size_t i = 0; i < tuples.size(); i++) {
                    // Step 2.1: Compute denominator of leading fraction
                    // -------------------------------------------------
                    // (m1!*1!^m1 * m2!*2!^m2 * ... * mn!*n!^mn)
                    double denominator = 1;
                    double factorialOfIndex = 1; // Factorial of m's index
                    for (size_t j = 1; j <= d; j++) {
                        denominator *= core::math::factorial(tuples[i][j - 1]); // mj!
                        factorialOfIndex *= j;
                        for (uint64_t k = 1; k <= tuples[i][j - 1]; k++)
                            denominator *= factorialOfIndex;
                    }

                    // Step 2.2: Compute leading fraction
                    // ----------------------------------
                    // n! / (m1!*1!^m1 * m2!*2!^m2 * ... * mn!*n!^mn)
                    const double leadingFraction = factorialOfIndex / denominator;

                    // Step 2.3: Compute (m1+...+mn)-th derivative of q with respect to p
                    // ------------------------------------------------------------------
                    // D^(m1+...+mn)q(p)
                    double order = 0;
                    for (size_t j = 0; j < d; j++)
                        order += tuples[i][j];
                    const Eigen::Quaterniond& derivationOfq = Dqi[order];

                    // Step 2.4: Compute trailing product
                    // ----------------------------------
                    // product_{j=1}^{n} (D^(j)p(t))^mj
                    double trailingProduct = 1;
                    for (size_t j = 1; j <= d; j++) {
                        // Check, if mj > 0, otherwise the 0-th power of anything is 1 -> has no effect
                        if (tuples[i][j - 1] > 0) {
                            // Step 2.4.1: Compute j-th derivative of p with respect to t
                            // ----------------------------------------------------------
                            // D^(j)p(t)
                            const double& derivativeOfp = Dpi[j];

                            // Step 2.4.2: Compute mj-th power of this
                            // ---------------------------------------
                            // (D^(j)p(t))^mj
                            double powerOfDerivativeOfp = derivativeOfp;
                            for (size_t k = 1; k < tuples[i][j - 1]; k++)
                                powerOfDerivativeOfp *= derivativeOfp;

                            // Step 2.4.3: Add contribution to product
                            // ---------------------------------------
                            trailingProduct *= powerOfDerivativeOfp;
                        }
                    }

                    // Step 2.5: Add contribution to sum (component-wise addition)
                    // ---------------------------------
                    if (leadingFraction != 0 && trailingProduct != 0)
                        Di[d].coeffs() += (leadingFraction * trailingProduct) * derivationOfq.coeffs();
                }
            }

            // Step 3: Chain rule for mapping *parameter-spline* [0, 1] to actual time [0, m_duration]
            // ---------------------------------------------------------------------------------------
            // Note: since the relationship is linear, the chain rule is easy again (no need for Faà di Bruno's formula here)

            // Apply coordinate transformation (chain rule) for derivatives
            double chainRuleMultiplier = 1.0 / m_duration;
            for (unsigned int d = 1; d <= N; d++) {
                Di[d].coeffs() *= chainRuleMultiplier;
                chainRuleMultiplier /= m_duration;
            }

            // Pass back list
            return Di;
        }

        //! Evaluation of angular velocity
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * Computes the angular velocity at the given time according to
         * \f[ \omega(t) := 2 \cdot \dot{q}(t) \cdot q^{-1}(t) \f]
         *
         * \param [in] time Time point to evaluate at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \return Angular velocity at specified time
         */
        virtual Eigen::Vector3d evaluateAngularVelocity(const double time) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Compute value and derivatives of quaternion trajectory
            std::array<Eigen::Quaterniond, 2> Di = evaluateD0ToDN<1>(time);

            // Compute (half) angular velocity represented as quaternion (with zero scalar value)
            Eigen::Quaterniond halfAngularVelocity = Di[1] * Di[0].conjugate(); // For unit-quaternions the inverse is equal to the conjugate

            // Convert to 3-dimensional vector and pass back
            return (2.0 * halfAngularVelocity.vec());
        }

        //! Evaluation of angular acceleration
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * Computes the angular acceleration at the given time according to
         * \f[ \dot{\omega}(t) = \frac{\partial w(t)}{\partial t} = 2 \cdot \left[\ddot{q}(t) \cdot q^{-1}(t) - \left(\dot{q}(t) \cdot q^{-1}(t)\right)^2\right] \f]
         *
         * \par Proof:
         * We start with the definition of the angular velocity which is
         * \f[ \omega(t) := 2 \cdot \dot{q}(t) \cdot q^{-1}(t)\f]
         * or equivalently
         * \f[ \omega(t) \cdot q(t) = 2 \cdot \dot{q}(t) \f]
         * Now we compute the time derivative of the left and right hand side and get
         * \f[ \dot{\omega}(t) \cdot q(t) + \omega(t) \cdot \dot{q}(t) = 2 \cdot \ddot{q}(t) \f]
         * or
         * \f[ \dot{\omega}(t) = 2 \cdot \ddot{q}(t) \cdot q^{-1}(t) - \omega(t) \cdot \dot{q}(t) \cdot q^{-1}(t)\f]
         * which can be rewritten to
         * \f[ \dot{\omega}(t) = 2 \cdot \ddot{q}(t) \cdot q^{-1}(t) - 2 \cdot \left(\dot{q}(t) \cdot q^{-1}(t)\right)\left(\dot{q}(t) \cdot q^{-1}(t)\right)\f]
         * This leads finally to
         * \f[ \dot{\omega}(t) = \frac{\partial w(t)}{\partial t} = 2 \cdot \left[\ddot{q}(t) \cdot q^{-1}(t) - \left(\dot{q}(t) \cdot q^{-1}(t)\right)^2\right] \f]
         * q.e.d.
         *
         * \param [in] time Time point to evaluate at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \return Angular acceleration at specified time
         */
        virtual Eigen::Vector3d evaluateAngularAcceleration(const double time) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Compute value and derivatives of quaternion trajectory
            std::array<Eigen::Quaterniond, 3> Di = evaluateD0ToDN<2>(time);

            // Compute intermediate results
            Eigen::Quaterniond qinv = Di[0].conjugate(); // q^-1(t) (for unit-quaternions the inverse is equal to the conjugate)
            Eigen::Quaterniond dotqtimesqinv = Di[1] * qinv; // dq(t)/dt * q^-1(t)
            Eigen::Quaterniond firstPart = Di[2] * qinv; // d^2q(t)/dt^2 * q^-1(t)
            Eigen::Quaterniond secondPart = dotqtimesqinv * dotqtimesqinv; // (dq(t)/dt * q^-1(t))^2

            // Compute (half) angular acceleration represented as quaternion (with zero scalar value)
            Eigen::Quaterniond halfAngularAcceleration;
            halfAngularAcceleration.coeffs() = firstPart.coeffs() - secondPart.coeffs();

            // Convert to 3-dimensional vector and pass back
            return (2.0 * halfAngularAcceleration.vec());
        }

        //! Evaluation of value (D0) and angular velocity (V) of underlying spline (improved performance over calling \ref evaluate() and \ref evaluateAngularVelocity())
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * Computes value and angular velocity at the given time according to
         * \f[ \omega(t) := 2 \cdot \dot{q}(t) \cdot q^{-1}(t) \f]
         * (see \ref evaluateAngularVelocity() for details)
         *
         * \param [in] time Time point to evaluate at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \param [out] value Value at specified time
         * \param [out] angularVelocity Angular velocity at specified time
         */
        virtual void evaluateD0V(const double& time, Eigen::Quaterniond& value, Eigen::Vector3d& angularVelocity) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Compute value and derivatives of quaternion trajectory
            std::array<Eigen::Quaterniond, 2> Di = evaluateD0ToDN<1>(time);

            // Set value
            value = Di[0];

            // Compute (half) angular velocity represented as quaternion (with zero scalar value)
            Eigen::Quaterniond halfAngularVelocity = Di[1] * Di[0].conjugate(); // For unit-quaternions the inverse is equal to the conjugate

            // Convert to 3-dimensional vector
            angularVelocity = 2.0 * halfAngularVelocity.vec();
        }

        //! Evaluation of value (D0), angular velocity (V), and angular acceleration (A) of underlying spline (improved performance over calling \ref evaluate(), \ref evaluateAngularVelocity(), and \ref evaluateAngularAcceleration())
        /*!
         * \warning You may call \ref isValid() before to check, if the trajectory is valid! If the trajectory is invalid the behaviour of this method is not defined!
         *
         * Computes value, angular velocity, and angular acceleration at the given time according to
         * \f[ \omega(t) := 2 \cdot \dot{q}(t) \cdot q^{-1}(t) \f]
         * \f[ \dot{\omega}(t) = \frac{\partial w(t)}{\partial t} = 2 \cdot \left[\ddot{q}(t) \cdot q^{-1}(t) - \left(\dot{q}(t) \cdot q^{-1}(t)\right)^2\right] \f]
         * (see \ref evaluateAngularVelocity() and \ref evaluateAngularAcceleration() for details)
         *
         * \param [in] time Time point to evaluate at (must be within \f$ [0,\f$ \ref m_duration\f$]\f$, values are projected otherwise)
         * \param [out] value Value at specified time
         * \param [out] angularVelocity Angular velocity at specified time
         * \param [out] angularAcceleration Angular acceleration at specified time
         */
        virtual void evaluateD0VA(const double& time, Eigen::Quaterniond& value, Eigen::Vector3d& angularVelocity, Eigen::Vector3d& angularAcceleration) const
        {
            // Check validity
            assert(isValid(nullptr, nullptr, nullptr));

            // Compute value and derivatives of quaternion trajectory
            std::array<Eigen::Quaterniond, 3> Di = evaluateD0ToDN<2>(time);

            // Set value
            value = Di[0];

            // Compute intermediate results
            Eigen::Quaterniond inverseValue = Di[0].conjugate(); // q^-1(t) (for unit-quaternions the inverse is equal to the conjugate)

            // Compute angular velocity
            // ------------------------
            // Compute (half) angular velocity represented as quaternion (with zero scalar value)
            Eigen::Quaterniond halfAngularVelocity = Di[1] * inverseValue; // dq(t)/dt * q^-1(t)

            // Convert to 3-dimensional vector
            angularVelocity = 2.0 * halfAngularVelocity.vec();

            // Compute angular acceleration
            // ----------------------------
            // Compute (half) angular acceleration represented as quaternion (with zero scalar value)
            Eigen::Quaterniond firstPart = Di[2] * inverseValue; // d^2q(t)/dt^2 * q^-1(t)
            Eigen::Quaterniond secondPart = halfAngularVelocity * halfAngularVelocity; // (dq(t)/dt * q^-1(t))^2
            Eigen::Quaterniond halfAngularAcceleration;
            halfAngularAcceleration.coeffs() = firstPart.coeffs() - secondPart.coeffs();

            // Convert to 3-dimensional vector and pass back
            angularAcceleration = 2.0 * halfAngularAcceleration.vec();
        }

        // Encoding
        // --------
        //! Encode member data as XML element and add it to the specified stream
        /*!
         * \param [in,out] stream The stream to which the member data should be appended to.
         * \param [in] XMLIndentationLevel Level of indentation in XML format (used to add spaces)
         * \param [in] XMLTabWhiteSpaces Count of whitespaces corresponding to a tab.
         * \param [in] numericFormat C-format specifier for numeric values.
         * \return Count of appended characters (elements in stream).
         */
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const uint8_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<QuaternionTrajectory");

            // Write attributes
            addedElements += io::encoding::encode(stream, " Duration=\"");
            addedElements += io::encoding::encode(stream, (double)m_duration, numericFormat);
            addedElements += io::encoding::encode(stream, "\">\n");

            // Write *quaternion-spline*
            addedElements += m_quaternionSpline.encodeToXML(stream, XMLIndentationLevel + 1, XMLTabWhiteSpaces, numericFormat);

            // Write *parameter-spline*
            addedElements += m_parameterSpline.encodeToXML(stream, XMLIndentationLevel + 1, XMLTabWhiteSpaces, numericFormat);

            // End XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "</QuaternionTrajectory>\n");

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

    //! \}
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
