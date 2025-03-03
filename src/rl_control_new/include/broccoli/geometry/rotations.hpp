/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#ifdef HAVE_EIGEN3 // needs Eigen
#include "../core/floats.hpp"
#include <Eigen/Geometry>
#include <math.h>

/*!
 * \file rotations.hpp
 * \brief Defines extensions to Eigen's Geometry module regarding rotations in 3D
 */

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    /*!
     * \brief Compute exponential \f$exp(q)=e^q\f$ of an arbitrary quaternion
     *
     * \param [in] quaternion The quaternion \f$q\f$ to compute the exponential for
     * \return The exponential of the given quaternion as quaternion by itself.
     */
    template <typename Scalar>
    Eigen::Quaternion<Scalar> quaternionExponential(const Eigen::Quaternion<Scalar>& quaternion)
    {
        /*! Explanation:
          * ------------
          * Given a quaternion \f$q = \left[w,\ x,\ y,\ z\right]^T = \left[w,\ v\right]^T\f$ with \f$v = \left[x,\ y,\ z\right]^T\f$, the
          * exponential of \f$q\f$ is given as
          * \f[ exp(q) = e^q = \left[e^w\,\cos(||v||),\,e^w\,\sin(||v||)\frac{v}{||v||}\right]^T\f]
         */

        // Initialize return value
        Eigen::Quaternion<Scalar> exponential(0, 0, 0, 0);

        // Compute exp(w)=e^w
        const Scalar expw = exp(quaternion.w());

        // Compute norm of vector part
        const Scalar normv = quaternion.vec().norm();

        // Avoid division by zero
        if (!core::isZero(normv)) {
            exponential.w() = expw * cos(normv);
            Scalar expwsinvnormv = expw * sin(normv) / normv; // e^w * sin(||v||) / ||v||
            exponential.x() = expwsinvnormv * quaternion.x();
            exponential.y() = expwsinvnormv * quaternion.y();
            exponential.z() = expwsinvnormv * quaternion.z();
        } else {
            // ...use [e^w, 0, 0, 0]^T
            exponential.w() = expw;
        }

        // Pass back result
        return exponential;
    }

    /*!
     * \brief Compute natural logarithm \f$ln(q)\f$ of an arbitrary quaternion
     *
     * \note For quaternions with small imaginary part (unit quaternions describing small angle rotations),
     * the natural logarithm is approximated by a taylor series.
     *
     * \attention Passing a quaternion with a norm almost zero or zero returns the original passed
     * quaternion. User code must handle this condition.
     *
     * \param [in] quaternion The quaternion \f$q\f$ to compute the natural logarithm for
     * \return The natural logarithm of the given quaternion (another quaternion)
     */
    template <typename Scalar>
    Eigen::Quaternion<Scalar> quaternionNaturalLogarithm(const Eigen::Quaternion<Scalar>& quaternion)
    {
        /*! ### Explanation:
          * Given a quaternion \f$q = \left[w,\ x,\ y,\ z\right]^T = \left[w,\ v\right]^T\f$ with \f$v = \left[x,\ y,\ z\right]^T\f$, and a machine precision barrier \f$\epsilon > 0\f$, the natural
          * logarithm of \f$q\f$ is given by
          * \f[ \ln(q) = \begin{cases} \left[\ln(||q||),\ \frac{v}{||v||} \mathrm{atan2} \left( ||v||, w\right)\right]^T & \text{for} & ||v|| \ge \epsilon \\
          * \left[\ln(||q||),\ \frac{v}{w} \left ( 1 - \frac{||v||^2}{3 w^2} \right ) \right]^T & \text{for} & ||v|| < \epsilon \wedge w > \epsilon \\
          * q & \text{for} & ||v|| < \epsilon \wedge w < \epsilon \\
          * \end{cases}
          * \f]
         */

        Eigen::Quaternion<Scalar> result(quaternion);

        const Scalar qNorm = quaternion.norm();
        const Scalar vNorm = quaternion.vec().norm();

        if (!core::isZero(vNorm)) {

            result.w() = log(qNorm);
            result.vec() = quaternion.vec() / vNorm * atan2(vNorm, quaternion.w());

        } else if (!core::isZero(qNorm)) {

            // Taylor series approximation
            result.w() = log(qNorm);
            result.vec() = quaternion.vec() / quaternion.w() * (1 - vNorm * vNorm / (3 * quaternion.w() * quaternion.w()));
        }

        return result;
    }

    /*!
     * \brief Compute power \f$q_1^{q_2}\f$ where \f$q_1\f$ and \f$q_2\f$ are both arbitrary quaternions
     *
     * \param [in] base The base \f$q_1\f$ (quaternion)
     * \param [in] exponent The exponent \f$q_2\f$ (quaternion)
     * \return the power \f$q_1^{q_2}\f$ as quaternion by itself
     */
    template <typename Scalar>
    Eigen::Quaternion<Scalar> quaternionPower(const Eigen::Quaternion<Scalar>& base, const Eigen::Quaternion<Scalar>& exponent)
    {
        /*! Explanation:
          * ------------
          * Given two quaternions \f$q_1\f$ and \f$q_2\f$, the power \f$q_1^{q_2}\f$ is given as
          * \f[ q_1^{q_2} = exp\left(ln(q_1) \cdot q_2\right) \f]
          * with \f$\cdot\f$ as the quaternion product.
         */
        return quaternionExponential(quaternionNaturalLogarithm(base) * exponent);
    }

    /*!
     * \brief Compute power \f$q^x\f$ where \f$q\f$ is an arbitrary quaternion and \f$x\f$ is an arbitrary real valued scalar
     *
     * \param [in] base The base \f$q\f$ (quaternion)
     * \param [in] exponent The exponent \f$x\f$ (real valued scalar)
     * \return the power \f$q^x\f$ as quaternion by itself
     */
    template <typename Scalar>
    Eigen::Quaternion<Scalar> quaternionPower(const Eigen::Quaternion<Scalar>& base, const Scalar& exponent)
    {
        /*! Explanation:
          * ------------
          * Given a quaternion \f$q\f$ and a real-valued scalar \f$x\f$, the power \f$q^x\f$ is given as
          * \f[ q^x = exp\left(ln(q) \cdot x\right) \f]
         */
        Eigen::Quaternion<Scalar> lnqx = quaternionNaturalLogarithm(base); // ln(q) * x
        lnqx.coeffs() *= exponent;
        return quaternionExponential(lnqx);
    }

    //! Perform quaternion <b>S</b>pherical <b>L</b>inear Int<b>erp</b>olation
    /*!
     * \f[
     * q(x) = SLERP(q_0,\,q_1,\,x) = q_0 \cdot (q_0^{-1} \cdot q_1)^x = \frac{\sin((1-x)\,\theta)}{\sin(\theta)}\,q_0 + \frac{\sin(x\,\theta)}{\sin(\theta)}\,q_1
     * \f]
     *
     * \remark If \f$ |q_0\cdot q_1|\approx 1\f$, linear interpolation is used instead in order to avoid division by zero (caused by \f$ \sin(\theta) \f$)
     *
     * \param [in] firstQuaternion First **normalized** quaternion \f$ q_0\f$
     * \param [in] secondQuaternion Second **normalized** quaternion \f$ q_1 \f$
     * \param [in] interpolationParameter Interpolation parameter \f$x \in \left[0,\,1\right]\f$
     * \param [in] shortestPath If `true`, the second quaternion \f$q_1\f$ may be flipped (antipodal quaternion) in order to obtain the shortest great arc on the unit sphere
     * \return \f$ q(x) = SLERP(q_0,\,q_1,\,x) \f$
     *
     * References
     * ----------
     * * Ken Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH Computer Graphics, ACM, New York, NY, USA, volume 19, number 3, 1985, DOI:[10.1145/325165.325242](https://www.doi.org/10.1145/325165.325242), p.245--254
     */
    template <typename Scalar>
    Eigen::Quaternion<Scalar> quaternionSLERP(const Eigen::Quaternion<Scalar>& firstQuaternion, const Eigen::Quaternion<Scalar>& secondQuaternion, const Scalar& interpolationParameter, const bool& shortestPath = true)
    {
        // Initialize helpers
        bool secondQuaternionNeedsFlipping = false;
        Scalar firstCoefficient, secondCoefficient;
        Scalar dotProduct = firstQuaternion.dot(secondQuaternion);
        const Scalar absDotProduct = fabs(dotProduct);

        // Check, if second quaternion has to be flipped to obtain the shortest path
        if (shortestPath && dotProduct < 0) {
            // Flip second quaternion
            secondQuaternionNeedsFlipping = true;
            dotProduct = absDotProduct;
        }

        // Avoid division by zero with sin(theta) ~= 0
        if (absDotProduct > 1.0 - 1e-6) {
            // ...yes -> use linear interpolation instead as approximation
            // "Walking" on the unit-sphere on an very short arc segment is the same as just walking on a straight line segment ("tangent").
            firstCoefficient = 1.0 - interpolationParameter;
            secondCoefficient = interpolationParameter;
        } else {
            // ...no -> compute SLERP according to Shoemake 1985
            Scalar theta = acos(dotProduct);
            Scalar sinTheta = sin(theta);
            firstCoefficient = sin((1.0 - interpolationParameter) * theta) / sinTheta;
            secondCoefficient = sin(interpolationParameter * theta) / sinTheta;
        }
        if (!secondQuaternionNeedsFlipping)
            return Eigen::Quaternion<Scalar>(firstCoefficient * firstQuaternion.coeffs() + secondCoefficient * secondQuaternion.coeffs());
        else
            return Eigen::Quaternion<Scalar>(firstCoefficient * firstQuaternion.coeffs() - secondCoefficient * secondQuaternion.coeffs());
    }

    /*!
     * \brief Applies the tilde operator on the given 3d vector
     *
     * For a 3d vector \f$v = \left [v_x, \ v_y, \ v_z \right ]\f$, this returns the skew-symmetric matrix
     * \f[
     * \tilde v = \left ( \begin{array}{ccc} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{array} \right )
     * \f]
     *
     * \param vector The input vector \f$v\f$
     */
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> tildeOperator(const Eigen::MatrixBase<Derived>& vector)
    {
        static_assert(Derived::SizeAtCompileTime < 0 || Derived::SizeAtCompileTime == 3, "tildeOperator accepts only 3d vectors");
        assert(vector.size() == 3 && "tildeOperator accepts only 3d vectors");

        if (vector.size() != 3)
            return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();

        Eigen::Matrix<typename Derived::Scalar, 3, 3> result;
        result << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
        return result;
    }

    /*!
    * \brief Returns the rotation vector \f$ {}_L v_{L,R} \f$ for a rotation quaternion \f$ {}_L q_R \f$.
    *
    * Given a quaternion \f$ {}_L q_R \f$, which describes the frame rotation from frame \f$ R \f$ to frame \f$ L \f$,
    * this method returns the corresponding rotation vector \f$ {}_L v_{L,R} = \theta_{L,R} \, {}_L u \f$
    * (angle \f$ \theta \f$, axis \f$ u \f$) written in the left frame of reference.
    *
    * \param q The quaternion \f$ {}_L q_R \f$ describing the frame transformation from \f$ R \f$ FoR to \f$ L \f$ FoR
    * \return The rotation vector \f$ {}_L v_{L,R} \f$ from \f$ L \f$ to \f$ R \f$, written in \f$ L \f$.
    */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 1> rotationVectorFrom(const Eigen::Quaternion<Scalar>& q)
    {
        return 2.0 * quaternionNaturalLogarithm(q).vec();
    }

    /*!
     * \copydoc broccoli::geometry::rotationVectorFrom
     * \note This overloaded method is provided for convenience and takes the coefficients of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> rotationVectorFrom(const Eigen::MatrixBase<Derived>& q)
    {
        return rotationVectorFrom(Eigen::Quaternion<typename Derived::Scalar>(q));
    }

    /*!
     * \brief Returns a rotation quaternion \f${}_L q_R \f$ for given rotation vector \f${}_L v_{L,R}\f$.
     *
     * Given a rotation vector \f$ {}_L v_{L,R} = \theta_{L,R} \, {}_L u \f$
     * (angle \f$ \theta \f$, axis \f$ u \f$) written in the left frame of reference
     * this method returns the corresponding quaternion \f${}_L q_R \f$,
     * which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$.
     *
     * \param rotationVector The rotation vector \f${}_L v_{L,R}\f$ from \f$L\f$ to \f$R\f$, written in \f$L\f$.
     * \return The quaternion \f${}_L q_R \f$ describing the frame transformation from \f$R\f$ FoR to \f$L\f$ FoR.
     */
    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> quaternionFrom(const Eigen::MatrixBase<Derived>& rotationVector)
    {
        Eigen::Quaternion<typename Derived::Scalar> halfAngleVector;
        halfAngleVector.w() = 0.0;
        halfAngleVector.vec() = 0.5 * rotationVector;
        return quaternionExponential(halfAngleVector);
    }

    /*!
     * \brief Calculates the quaternion time derivative \f${}_L \dot q_R\f$ from the angular velocity \f${}_L \omega_{L,R}\f$ in the \f$L\f$ frame
     *
     * Given a quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$,
     * this method calculates its time derivative from the angular velocity between the frames \f${}_L \omega_{L,R}\f$ written in the \f$L\f$ frame.
     *
     * \param q A quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$
     * \param omega Angular velocity between the frames \f${}_L \omega_{L,R}\f$ (\f$R\f$ relative to \f$L\f$, written in \f$L\f$).
     * \return The quaternion time derivative \f${}_L \dot q_R\f$
     */
    template <typename Scalar, typename DerivedOmega>
    Eigen::Quaternion<Scalar> quaternionDerivativeLeftFoR(const Eigen::Quaternion<Scalar>& q, const Eigen::MatrixBase<DerivedOmega>& omega)
    {
        Eigen::Quaternion<Scalar> halfAngleVelocity;
        halfAngleVelocity.w() = 0.0;
        halfAngleVelocity.vec() = 0.5 * omega;
        return halfAngleVelocity * q;
    }

    /*!
     * \copydoc broccoli::geometry::quaternionDerivativeLeftFoR
     * \note This overloaded method is provided for convenience and takes the coefficients of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename DerivedQ, typename DerivedOmega>
    Eigen::Quaternion<typename DerivedQ::Scalar> quaternionDerivativeLeftFoR(const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedOmega>& omega)
    {
        return quaternionDerivativeLeftFoR(Eigen::Quaternion<typename DerivedQ::Scalar>(q), omega);
    }

    /*!
     * \brief Calculates the quaternion time derivative \f${}_L \dot q_R\f$ from the angular velocity \f${}_R \omega_{L,R}\f$ in the \f$R\f$ frame
     *
     * Given a quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$,
     * this method calculates its time derivative from the angular velocity between the frames \f${}_R \omega_{L,R}\f$ written in the \f$R\f$ frame.
     *
     * \param q A quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$
     * \param omega Angular velocity between the frames \f${}_R \omega_{L,R}\f$ (\f$R\f$ relative to \f$L\f$, written in \f$R\f$).
     * \return The quaternion time derivative \f${}_L \dot q_R\f$
     */
    template <typename Scalar, typename DerivedOmega>
    Eigen::Quaternion<Scalar> quaternionDerivativeRightFoR(const Eigen::Quaternion<Scalar>& q, const Eigen::MatrixBase<DerivedOmega>& omega)
    {
        Eigen::Quaternion<Scalar> halfAngleVelocity;
        halfAngleVelocity.w() = 0.0;
        halfAngleVelocity.vec() = 0.5 * omega;
        return q * halfAngleVelocity;
    }

    /*!
     * \copydoc broccoli::geometry::quaternionDerivativeRightFoR
     * \note This overloaded method is provided for convenience and takes the coefficients of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename DerivedQ, typename DerivedOmega>
    Eigen::Quaternion<typename DerivedQ::Scalar> quaternionDerivativeRightFoR(const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedOmega>& omega)
    {
        return quaternionDerivativeRightFoR(Eigen::Quaternion<typename DerivedQ::Scalar>(q), omega);
    }

    /*!
     * \brief Calculates angular velocity \f${}_R \omega_{L,R}\f$ in the \f$R\f$ frame for a given quaternion
     * and its derivative from \f${}_L q_R,\ {}_L \dot q_R\f$.
     *
     * Given a quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$, and its derivative
     * \f${}_L \dot q_R\f$, this method calculates the angular velocity between the frames \f${}_R \omega_{L,R}\f$ written in the \f$R\f$ frame.
     *
     * \param q A quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$
     * \param dotQ The quaternion time derivative \f${}_L \dot q_R \f$
     * \return Angular velocity between the frames \f${}_R \omega_{L,R}\f$ (\f$R\f$ relative to \f$L\f$, written in \f$R\f$).
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 1> angularVelocityRightFoR(const Eigen::Quaternion<Scalar>& q, const Eigen::Quaternion<Scalar>& dotQ)
    {
        return 2.0 * (q.conjugate() * dotQ).vec();
    }

    /*!
     * \copydoc broccoli::geometry::angularVelocityRightFoR
     * \note This overloaded method is provided for convenience and takes the coefficients / derivative of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename DerivedQ, typename DerivedDotQ>
    Eigen::Matrix<typename DerivedQ::Scalar, 3, 1> angularVelocityRightFoR(const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedDotQ>& dotQ)
    {
        return angularVelocityRightFoR(Eigen::Quaternion<typename DerivedQ::Scalar>(q), Eigen::Quaternion<typename DerivedDotQ::Scalar>(dotQ));
    }

    /*!
     * \brief Calculates angular velocity \f${}_L \omega_{L,R}\f$ in the \f$L\f$ frame for a given quaternion
     * and its derivative from \f${}_L q_R,\ {}_L \dot q_R\f$.
     *
     * Given a quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$, and its derivative
     * \f${}_L \dot q_R\f$, this method calculates the angular velocity between the frames \f${}_L \omega_{L,R}\f$ written in the \f$L\f$ frame.
     *
     * \param q A quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$
     * \param dotQ The quaternion time derivative \f${}_L \dot q_R \f$
     * \return Angular velocity between the frames \f${}_L \omega_{L,R}\f$ (\f$R\f$ relative to \f$L\f$, written in \f$L\f$).
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 1> angularVelocityLeftFoR(const Eigen::Quaternion<Scalar>& q, const Eigen::Quaternion<Scalar>& dotQ)
    {
        return 2.0 * (dotQ * q.conjugate()).vec();
    }

    /*!
     * \copydoc broccoli::geometry::angularVelocityLeftFoR
     * \note This overloaded method is provided for convenience and takes the coefficients / derivative of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename DerivedQ, typename DerivedDotQ>
    Eigen::Matrix<typename DerivedQ::Scalar, 3, 1> angularVelocityLeftFoR(const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedDotQ>& dotQ)
    {
        return angularVelocityLeftFoR(Eigen::Quaternion<typename DerivedQ::Scalar>(q), Eigen::Quaternion<typename DerivedDotQ::Scalar>(dotQ));
    }

    /*!
     * \brief Calculates the Jacobian \f${}_L J_Q = \frac{\partial {}_L \dot q_R}{\partial {}_L \omega_{L,R}}\f$ for a given quaternion \f${}_L q_R\f$.
     * \param q A quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$
     * \return The Jacobian \f$\frac{\partial {}_L \dot q_R}{\partial {}_L \omega_{L,R}}\f$
     * using angular velocities written in the left FoR \f$L\f$
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 4, 3> quaternionJacobianLeftFoR(const Eigen::Quaternion<Scalar>& q)
    {
        Eigen::Matrix<Scalar, 4, 3> result;
        result.template block<3, 3>(0, 0) = 0.5 * (Eigen::Matrix<Scalar, 3, 3>::Identity() * q.w() + tildeOperator(q.vec()).transpose());
        result.template block<1, 3>(3, 0) = -0.5 * q.vec();
        return result;
    }

    /*!
     * \copydoc broccoli::geometry::quaternionJacobianLeftFoR
     * \note This overloaded method is provided for convenience and takes the coefficients of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 4, 3> quaternionJacobianLeftFoR(const Eigen::MatrixBase<Derived>& q)
    {
        return quaternionJacobianLeftFoR(Eigen::Quaternion<typename Derived::Scalar>(q));
    }

    /*!
     * \brief Calculates the Jacobian \f${}_R J_Q = \frac{\partial {}_L \dot q_R}{\partial {}_R \omega_{L,R}}\f$ for a given quaternion \f${}_L q_R\f$.
     * \param q A quaternion \f${}_L q_R \f$, which describes the frame rotation from frame \f$R\f$ to frame \f$L\f$
     * \return The Jacobian \f$\frac{\partial {}_L \dot q_R}{\partial {}_R \omega_{L,R}}\f$
     * using angular velocities \f${}_R \omega_{L,R}\f$ written in the right FoR \f$R\f$
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 4, 3> quaternionJacobianRightFoR(const Eigen::Quaternion<Scalar>& q)
    {
        Eigen::Matrix<Scalar, 4, 3> result;
        result.template block<3, 3>(0, 0) = 0.5 * (Eigen::Matrix<Scalar, 3, 3>::Identity() * q.w() + tildeOperator(q.vec()));
        result.template block<1, 3>(3, 0) = -0.5 * q.vec();
        return result;
    }

    /*!
     * \copydoc broccoli::geometry::quaternionJacobianRightFoR
     * \note This overloaded method is provided for convenience and takes the coefficients of the quaternion as 4d vector (x,y,z,w)
     */
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 4, 3> quaternionJacobianRightFoR(const Eigen::MatrixBase<Derived>& q)
    {
        return quaternionJacobianRightFoR(Eigen::Quaternion<typename Derived::Scalar>(q));
    }
    //! \}

} // namespace geometry
} // namespace broccoli

#endif