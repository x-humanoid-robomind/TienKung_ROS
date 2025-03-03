/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/math.hpp"
#include "../../geometry/rotations.hpp"
#include "BSplineBasis.hpp"
#include "CumulativeBSplineBasisSample.hpp"
#include "DynamicSizeQuaternionCurve.hpp"
#include "InterpolatableQuaternionCurve.hpp"
#include <Eigen/StdVector>
#include <array>

namespace broccoli {
namespace curve {
    //! Class abstracting **B-spline** quaternion cuves according to Kim 1995
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \tparam Degree \f$ p \geq 0 \f$ of the underlying BSpline basis.
     * \tparam MaximumBSplineCount (Expected) maximum count of B-Splines (alias control-point quaternions) (a higher B-Spline count at runtime is also supported, but less efficient)
     *
     * \warning Note that there is a difference between the terms *degree* \f$ p \f$ and *order* \f$ k \f$ (\f$ k = p + 1 \f$).
     *
     * \warning This class is restricted to **unit-quaternions**!
     *
     * References
     * ----------
     * * Carl de Boor, "A Practical Guide to Splines", Springer, New York, ISBN: 978-0387953663, 2001
     * * Myoung-Jun Kim et al., "A General Construction Scheme for Unit Quaternion Curves with Simple High Order Derivatives", Proceedings of the 22nd Annual Conference on Computer Graphics and Interactive Techniques, 1995, SIGGRAPH '95, ACM, pages = 369--376, DOI:[10.1145/218380.218486](https://www.doi.org/10.1145/218380.218486)
     * * Myoung-Jun Kim et al., "A C2-continuous B-spline Quaternion Curve Interpolating a Given Sequence of Solid Orientations", Proceedings Computer Animation'95, 1995, pages 72--81, DOI:[10.1109/CA.1995.393545](https://www.doi.org/10.1109/CA.1995.393545)
     *
     * Definition
     * ----------
     * Given a sequence of \f$m\f$ control points \f$ [q_0,\,\dots\,,\,q_{m-1}]\f$ (\ref m_controlPoints) and a B-spline basis \f${B}_{i,k}(x)\f$ (\ref m_basis) of order \f$k=p+1\f$ with \f$n=m+k\f$ knots \f$[t_0,\,\dots\,,\,t_{n-1}]\f$, the base function is defined by (Kim 1995)
     * \f[
     * q(x) = q_0^{\widetilde{B}_{o,k}(x)} \prod_{i=1}^{m-1}\exp(\underbrace{\ln(q_{i-1}^{-1}\,q_i)}_{=:\omega_i}\,\widetilde{B}_{i,k}(x)) = \prod_{i=0}^{m-1}\exp(\omega_i\,\widetilde{B}_{i,k}(x))
     * \qquad \mbox{with} \quad x \in\;[t_p,\,t_m[
     * \f]
     * where \f$q_{-1} := -1 + 0i + 0j + 0k\f$ ("identity"), \f$\omega_i:=\ln(q_{i-1}^{-1}q_i)\f$ and the **cumulative** basis is defined as
     * \f[
     * \widetilde{B}_{i,k}(x) = \sum_{c=i}^{m-1} {B}_{c,k}(x)
     * \f]
     *
     * For efficiency we split the product into three factors:
     * \f[
     * q(x) = \underbrace{Q_1}_{const.} \cdot Q_2(x) \cdot \underbrace{Q_3}_{const.}
     * \f]
     * where
     *  * \f$Q_1\f$ relates to the (first) \f$f_1\f$ factors with \f$ \widetilde{B}_{i,k} = 1 \f$ for \f$i = 0,\,\dots\,,\,(f_1-1)\f$,
     * \f[
     * Q_1 = \prod_{i=0}^{f_1-1}\exp(\omega_i\,\underbrace{\widetilde{B}_{i,k}(x)}_{=1})
     * =\prod_{i=0}^{f_1-1}\exp(\omega_i)
     * =\prod_{i=0}^{f_1-1}\exp(\ln(q_{i-1}^{-1}\,q_i))
     * =(q_{-1}^{-1}q_0)\cdot(q_{0}^{-1}q_1)\cdot(q_{1}^{-1}q_2)\cdot\dots\cdot(q_{f_1-2}^{-1}q_{f_1-1})
     * =q_{f_1-1}
     * \f]
     *  * \f$Q_2(x)\f$ relates to the (intermediate) \f$f_2\f$ factors with \f$ \widetilde{B}_{i,k} \in\,[0,\,1]\f$ for \f$i = f_1 ,\,\dots\,,\,(f_1+f_2-1)\f$,
     * \f[
     * Q_2(x) = \prod_{i=f_1}^{f_1+f_2-1}\exp(\omega_i\,\underbrace{\widetilde{B}_{i,k}(x)}_{\in\;[0,\,1]})
     * \f]
     *  * \f$Q_3\f$ relates to the (last) \f$f_3\f$ factors with \f$ \widetilde{B}_{i,k} = 0 \f$ for \f$i = (f_1+f_2),\,\dots\,,\,(f_1+f_2+f_3-1 = m-1)\f$.
     *\f[
     * Q_3 = \prod_{i=f_1+f_2}^{m-1}\exp(\omega_i\,\underbrace{\widetilde{B}_{i,k}(x)}_{=0})
     * =\prod_{i=f_1+f_2}^{m-1}\exp(\omega_i\cdot 0)
     * =\prod_{i=f_1+f_2}^{m-1}\exp(0)
     * = 1+0i+0j+0k\quad\mbox{(identity)}
     * \f]
     *
     * Finally we obtain
     *\f[
     * q(x) = q_{f_1-1} \cdot \prod_{i=f_1}^{f_1+f_2-1}\exp(\omega_i\,\widetilde{B}_{i,k}(x))
     * \f]
     *
     * Derivatives
     * -----------
     * Note that \f$Q_1\f$ and \f$Q_3\f$ are constant. Moreover, \f$Q_3\f$ is the identity which is why it is just skipped in the following.
     * The \f$d\f$-th derivative of \f$q(x)\f$ is then given by
     * \f[
     * \frac{\partial^d q(x)}{\partial x^d}
     * =\frac{\partial^d}{\partial x^d}\left(Q_1\cdot Q_2(x)\right)
     * =Q_1 \cdot \frac{\partial^d}{\partial x^d}\left(Q_2(x)\right)
     * =Q_1 \cdot \frac{\partial^d}{\partial x^d}\left(\prod_{i=f_1}^{f_1+f_2-1}\underbrace{\exp(\omega_i\,\widetilde{B}_{i,k}(x))}_{=:\xi_i(x)}\right)
     * =Q_1 \cdot \frac{\partial^d}{\partial x^d}\left(\prod_{i=f_1}^{f_1+f_2-1} \xi_{i}(x))\right)
     * =Q_1 \cdot \frac{\partial^d}{\partial x^d}\left(\prod_{z=1}^{f_2} \xi_{z+f_1-1}(x))\right)
     * \f]
     * with \f$Q_1 = q_{f_1-1} \f$ (see above) and
     * \f[
     * \xi_i(x) = \exp(\omega_i\,\widetilde{B}_{i,k}(x))
     * \f]
     *
     * In order to compute the \f$d\f$-th derivative of \f$Q_2(x)\f$ we have to use the (generalized) **general Leibniz rule** (to compute the \f$d\f$-th derivative of a product of \f$f_2\f$ factors):
     * \f[
     * \frac{\partial^d}{\partial x^d}\left(Q_2(x)\right)
     * =\frac{\partial^d}{\partial x^d}\left( \prod_{z=1}^{f_2} \xi_{z+f_1-1}(x)) \right)
     * =\sum_{T_L} \left( \begin{array}{c}d\\k_1,\,k_2,\,\dots\,,\,k_{f_2}\end{array}\right) \prod_{z=1}^{f_2} \frac{\partial^{k_z} \xi_{z+f_1-1}(x)}{\partial x^{k_z}}
     * \f]
     * where \f$T_L\f$ is the set of all \f$f_2\f$-tuples \f$(k_1,\,\dots,\,k_{f_2})\f$ of non-negative integers which satisfy
     * \f[
     * k_1+k_2+\dots +k_{f_2} = d
     * \f]
     * and the **multinomial coefficient** is given by
     * \f[
     * \left( \begin{array}{c}d\\k_1,\,k_2,\,\dots\,,\,k_{f_2}\end{array}\right) = \frac{d!}{k_1!\,k_2!\,\dots\,k_{f_2}!}
     * \f]
     *
     * In order to compute the \f$d\f$-th derivative of \f$\xi_i(x)\f$ we have to use the **formula of Faa di Bruno**:
     * \f[
     * \frac{\partial^d \xi_i(\widetilde{B}_{i,k}(x))}{\partial x^d}
     * =\sum_{T_F} \frac{d!}{m_1!\,m_2!\,\dots\,m_d!} \cdot \xi^{(m_1+\cdots+m_d)}_i(\widetilde{B}_{i,k}(x)) \cdot \prod_{j=1}^d\left(\frac{\widetilde{B}_{i,k}^{(j)}(x)}{j!}\right)^{m_j}
     * \f]
     * where \f$T_F\f$ is the set of all \f$d\f$-tuples \f$(m_1,\,\dots,\,m_d)\f$ of non-negative integers which satisfy
     * \f[
     *  1\cdot m_1+ 2\cdot m_2+\dots + d\cdot m_d = d
     * \f]
     * and \f$\xi^{(s)}_i(\widetilde{B}_{i,k}(x))\f$ is
     * \f[
     * \xi^{(s)}_i(\widetilde{B}_{i,k}(x)) = \xi_i(x) \prod_{j=1}^s \omega_i
     * \f]
     *
     * The \f$d\f$-th derivative of \f$\widetilde{B}_{i,k}(x)\f$ is handled in \ref CumulativeBSplineBasisSample.
     */
    template <unsigned int Degree, unsigned int MaximumBSplineCount = Degree + 1>
    class QuaternionBSplineCurve : public DynamicSizeQuaternionCurve, public InterpolatableQuaternionCurve {
    public:
        //! Default constructor
        /*! Initializes with empty list of control points and empty B-spline knot sequence */
        QuaternionBSplineCurve()
        {
        }

        //! Destructor
        virtual ~QuaternionBSplineCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const QuaternionBSplineCurve& reference) const
        {
            // Compare base class
            if (DynamicSizeQuaternionCurve::operator!=(reference))
                return false;

            // Compare B-spline basis
            if (m_basis != reference.m_basis)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const QuaternionBSplineCurve& reference) const { return !(*this == reference); }

        // Get type of underlying function (see base class for details)
        virtual FunctionType functionType() const { return FunctionType::BSPLINE; }

        // Checks, if the curve is properly defined (see base class for details)
        virtual bool isValid() const
        {
            // Check base class validity
            if (DynamicSizeQuaternionCurve::isValid() == false)
                return false;

            // Check, validity of B-spline basis
            if (m_basis.isValid() == false)
                return false;

            // Check, if count of control points matches size of knot sequence in B-spline basis
            if (((int)m_controlPoints.size()) != ((int)m_basis.m_knotSequence.size() - (int)order()))
                return false;

            // No error -> valid
            return true;
        }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of underlying B-spline basis
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of underlying B-spline basis
        static unsigned int order() { return Degree + 1; }

        // Member data
        BSplineBasis<Degree, MaximumBSplineCount> m_basis; //!< Underlying B-spline basis (see \ref BSplineBasis)

        // Evaluation of (derivative of) underlying curve (see base class for details)
        /*!
         * \copydoc DynamicSizeQuaternionCurve::evaluate()
         *
         * Computes **analytic** derivatives with respect to interpolation parameter \f$ x \f$ (**not** time in general).
         *
         * \warning Since the B-spline curve is only defined for \f$ x \in\,[t_p,\,t_m[\f$ (see definition above), the \p position \f$x\f$ will be projected to these bounds.
         * For the right interval bound \f$t_m\f$ (which is open!) a projection \f$x=t_m \rightarrow x=t_m-\epsilon\f$ is performed.
         */
        virtual Eigen::Quaterniond evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Check validity
            assert(isValid());

            // Project position to bounds [t_p, t_m[
            const double projectedPosition = projectPositionToBounds(position);

            // Check if we only want the base function (for speed up of computation)
            if (derivationOrder == 0) {
                // Initialize return value (identity)
                Eigen::Quaterniond returnValue(1, 0, 0, 0);

                // Evaluate B-spline basis and cumulative basis at this position
                auto basisSample = m_basis.evaluate(projectedPosition, derivationOrder);
                CumulativeBSplineBasisSample<Degree, MaximumBSplineCount> cumulativeBasis;
                cumulativeBasis.evaluate(basisSample);

                // Check, if the cumulative basis is a zero vector
                if (cumulativeBasis.m_f[2] == (int)cumulativeBasis.m_B.size())
                    return returnValue; // ...yes -> q(t) = q_0^0 -> use identity
                // else: tildeB_0,k > 0

                // Compute contribution of Q_1
                if (cumulativeBasis.m_f[0] > 0) // Skip if f_1 = 0 (we would have to use q_{-1} which is just the identity)
                    returnValue = m_controlPoints[cumulativeBasis.m_f[0] - 1]; // Q1 = q_{f1-1}

                // Compute contribution of Q_2
                for (int i = cumulativeBasis.m_f[0]; i <= cumulativeBasis.m_f[0] + cumulativeBasis.m_f[1] - 1; i++) {
                    Eigen::Quaterniond q_im1inv_qi; // = q_{i-1}^{-1} * q_i
                    if (i == 0)
                        q_im1inv_qi = m_controlPoints[i]; // Just use q_i since q_{i-1} = q_{-1} = identity
                    else
                        q_im1inv_qi = m_controlPoints[i - 1].conjugate() * m_controlPoints[i];

                    // Introduction of "newReturnValue" to avoid "returnValue" on both sides of the equation (otherwise: could lead to inconsistent data - see Eigen documentation for details!)
                    Eigen::Quaterniond newReturnValue = returnValue * geometry::quaternionPower(q_im1inv_qi, cumulativeBasis.m_B[i]); // = ...* (q_{i-1}^{-1} * q_i)^{tildeB_i,k}
                    returnValue = newReturnValue;
                }

                // Compute contribution of Q_3
                // -> identity -> nothing to do...

                // Pass back return value
                return returnValue;
            } else {
                // Initialize return value (zero-derivative)
                Eigen::Quaterniond returnValue(0, 0, 0, 0);

                // Compute B-spline basis and cumulative basis
                // -------------------------------------------
                std::vector<BSplineBasisSample<Degree, MaximumBSplineCount>> D_B_ik; // Vector of derivatives of B-spline basis samples D^j B_{i,k}(x) with j = 0...d
                std::vector<CumulativeBSplineBasisSample<Degree, MaximumBSplineCount>> D_tildeB_ik; // Vector of derivatives of cumulative B-spline basis samples D^j tildeB_{i,k}(x) with j = 0...d
                D_B_ik.reserve(derivationOrder + 1);
                D_tildeB_ik.reserve(derivationOrder + 1);
                for (unsigned int d = 0; d <= derivationOrder; d++) {
                    // Evaluate (derivative of) B-spline basis sample
                    D_B_ik.push_back(m_basis.evaluate(projectedPosition, d));

                    // Evaluate (derivative of) cumulative B-spline basis sample
                    CumulativeBSplineBasisSample<Degree, MaximumBSplineCount> next_D_tildeB_ik;
                    next_D_tildeB_ik.evaluate(D_B_ik.back());
                    D_tildeB_ik.push_back(next_D_tildeB_ik);
                }

                // Remember structure of cumulative B-spline basis
                const int& f1 = D_tildeB_ik[0].m_f[0];
                const int& f2 = D_tildeB_ik[0].m_f[1];

                // Abort, if f2=0 (all elements in the cumulative basis are zero -> derivative is zero!)
                if (f2 == 0)
                    return returnValue;

                // Compute omega_i
                // ---------------
                std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> omega_i(f2); // Vector of omega_i for i = f1, ... , f1 + f2 - 1
                for (int i = f1; i <= f1 + f2 - 1; i++) {
                    if (i == 0)
                        omega_i[i - f1] = geometry::quaternionNaturalLogarithm(m_controlPoints[i]); // Just use q_i since q_{i-1} = q_{-1} = identity
                    else
                        omega_i[i - f1] = geometry::quaternionNaturalLogarithm(m_controlPoints[i - 1].conjugate() * m_controlPoints[i]); // = q_{i-1}^{-1} * q_i
                }

                // Compute xi_i and its (partial) derivatives with respect to tildeB_ik
                // --------------------------------------------------------------------
                std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>> dxii_dtildeBik(f2); // Chained list of (partial) derivatives of xi_i with respect to tildeB_ik (Indices: [i][s] = (d^s xi_i)/(dtildeBik^s)) for i = f1, ... , f1 + f2 - 1
                for (int i = f1; i <= f1 + f2 - 1; i++) {
                    // Pre-allocate memory
                    dxii_dtildeBik[i - f1].reserve(derivationOrder + 1); // (include also 0-th derivative, i.e. xi_i(x))

                    // Compute (derivatives of) xi (with respect to tildeB_ik)
                    Eigen::Quaterniond omegai_tildeBik = omega_i[i - f1]; // omega_i * tildeB_ik(x)
                    omegai_tildeBik.coeffs() *= D_tildeB_ik[0].m_B[i];
                    dxii_dtildeBik[i - f1].push_back(geometry::quaternionExponential(omegai_tildeBik)); // 0-th derivative of xi_i
                    for (unsigned int d = 1; d <= derivationOrder; d++)
                        dxii_dtildeBik[i - f1].push_back(dxii_dtildeBik[i - f1].back() * omega_i[i - f1]);
                }

                // Compute x_i and its (total) derivatives with respect to x (uses formula of Faa di Bruno)
                // ---------------------------------------------------------
                // Pre-compute Faa di Bruno tuples
                std::vector<core::math::FaaDiBrunoTuples> T_F(derivationOrder); // List of Faa di Bruno tuples for all "d" derivatives
                for (unsigned int d = 1; d <= derivationOrder; d++)
                    T_F[d - 1] = core::math::faaDiBrunoTuples(d);

                // Compute (derivatives of) xi (with respect to x)
                std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>> D_xi_i(f2); // Chained list of (total) derivatives of xi_i with respect to x (Indices: [i][s] = (d^s xi_i)/(dx^s)) for i = f1, ... , f1 + f2 - 1
                for (int i = f1; i <= f1 + f2 - 1; i++) {
                    // Pre-allocate memory
                    D_xi_i[i - f1].reserve(derivationOrder + 1);

                    // Compute (derivatives of) xi (with respect to x)
                    D_xi_i[i - f1].push_back(dxii_dtildeBik[i - f1].front()); // 0-th total derivative = 0-th partial derivative = xi_i(x)
                    for (unsigned int d = 1; d <= derivationOrder; d++) {
                        // Initialize sum
                        Eigen::Quaterniond Dd_xi_i(0, 0, 0, 0);

                        // Iterate over all Faa di Bruno tuples for this derivative
                        for (size_t t = 0; t < T_F[d - 1].size(); t++) {
                            // Compute sum m_1 + m_2 + ... + m_d
                            uint64_t sumOfTupleElements = 0; // = m_1 + m_2 + ... + m_d
                            for (size_t j = 0; j < T_F[d - 1][t].size(); j++)
                                sumOfTupleElements += T_F[d - 1][t][j];

                            // Compute fraction of factorials coefficient
                            double fractionFactorials = core::math::factorial(d);
                            for (size_t j = 0; j < T_F[d - 1][t].size(); j++)
                                fractionFactorials /= core::math::factorial(T_F[d - 1][t][j]);

                            // Compute product of D_tildeB_ik's
                            double prod_D_tildeB_ik = 1.0;
                            for (unsigned int j = 1; j <= d; j++) {
                                double factor = D_tildeB_ik[j].m_B[i] / (double)core::math::factorial(j); // D^j_tildeB_ik/j!
                                for (int u = 1; u <= (int)T_F[d - 1][t][j - 1]; u++)
                                    prod_D_tildeB_ik *= factor;
                            }

                            // Add term related to this tuple to the sum
                            Dd_xi_i.coeffs() += (fractionFactorials * dxii_dtildeBik[i - f1][sumOfTupleElements].coeffs() * prod_D_tildeB_ik);
                        }

                        // Add finalized sum to list
                        D_xi_i[i - f1].push_back(Dd_xi_i);
                    }
                }

                // Compute d-th derivative of Q2 with respect to x (uses general Leibniz rule)
                // -----------------------------------------------
                // Compute general Leibniz tuples
                core::math::GeneralLeibnizTuples T_L = core::math::generalLeibnizTuples(derivationOrder, f2); // List of general Leibniz tuples

                // Sum up over all tuples
                Eigen::Quaterniond Dd_Q2(0, 0, 0, 0); // D^d Q_2(x) ("d"-th derivative with respect to x)
                for (size_t t = 0; t < T_L.size(); t++) {
                    // Compute multinomial coefficient
                    const double multinomial = core::math::multinomialCoefficient(T_L[t]);

                    // Compute product
                    Eigen::Quaterniond product = D_xi_i[0][T_L[t][0]]; // Initialize as first factor of product
                    for (int z = 2; z <= f2; z++) {
                        // Introduction of "newProduct" to avoid "product" on both sides of the equation (otherwise: could lead to inconsistent data - see Eigen documentation for details!)
                        Eigen::Quaterniond newProduct = product * D_xi_i[z - 1][T_L[t][z - 1]];
                        product = newProduct;
                    }

                    // Add contribution to overall sum
                    Dd_Q2.coeffs() += (multinomial * product.coeffs());
                }

                // Compute d-th derivative of q(x) with respect to x
                // -------------------------------------------------
                // Compute Q1 * D^d Q_2(x)
                if (f1 == 0) // Skip Q1 if f_1 = 0 (we would have to use q_{-1} which is just the identity)
                    returnValue = Dd_Q2;
                else
                    returnValue = m_controlPoints[f1 - 1] * Dd_Q2; // Q1 = q_{f1-1}

                // Pass back return value
                return returnValue;
            }
        }

        // Evaluation of value (D0) up to N-th (DN) derivative of underlying curve (see base class for details)
        /*!
         * Efficient **analytic** computation of D0, ..., DN all at once (re-use of intermediate results).
         *
         * \warning Since the B-spline curve is only defined for \f$ x \in\,[t_p,\,t_m[\f$ (see definition above), the \p position \f$x\f$ will be projected to these bounds.
         * For the right interval bound \f$t_m\f$ (which is open!) a projection \f$x=t_m \rightarrow x=t_m-\epsilon\f$ is performed.
         */
        template <unsigned int N>
        std::array<Eigen::Quaterniond, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Check validity
            assert(isValid());

            // Project position to bounds [t_p, t_m[
            const double projectedPosition = projectPositionToBounds(position);

            // Initialize return value
            std::array<Eigen::Quaterniond, N + 1> Di;

            // Compute B-spline basis and cumulative basis
            // -------------------------------------------
            std::array<BSplineBasisSample<Degree, MaximumBSplineCount>, N + 1> D_B_ik = m_basis.template evaluateD0ToDN<N>(projectedPosition); // Array of derivatives of B-spline basis samples D^j B_{i,k}(x) with j = 0...N
            std::array<CumulativeBSplineBasisSample<Degree, MaximumBSplineCount>, N + 1> D_tildeB_ik; // Array of derivatives of cumulative B-spline basis samples D^j tildeB_{i,k}(x) with j = 0...N

            // Evaluate (derivative of) cumulative B-spline basis sample
            for (unsigned int d = 0; d <= N; d++) {
                D_tildeB_ik[d] = CumulativeBSplineBasisSample<Degree, MaximumBSplineCount>();
                D_tildeB_ik[d].evaluate(D_B_ik[d]);
            }

            // Remember structure of cumulative B-spline basis
            const int& f1 = D_tildeB_ik[0].m_f[0];
            const int& f2 = D_tildeB_ik[0].m_f[1];
            const int& f3 = D_tildeB_ik[0].m_f[2];

            // Compute omega_i
            // ---------------
            memory::SmartVector<Eigen::Quaterniond, Degree + 1> omega_i(f2); // Vector of omega_i for i = f1, ... , f1 + f2 - 1
            for (int i = f1; i <= f1 + f2 - 1; i++) {
                if (i == 0)
                    omega_i[i - f1] = geometry::quaternionNaturalLogarithm(m_controlPoints[i]); // Just use q_i since q_{i-1} = q_{-1} = identity
                else
                    omega_i[i - f1] = geometry::quaternionNaturalLogarithm(m_controlPoints[i - 1].conjugate() * m_controlPoints[i]); // = q_{i-1}^{-1} * q_i
            }

            // Compute xi_i and its (partial) derivatives with respect to tildeB_ik
            // --------------------------------------------------------------------
            memory::SmartVector<std::array<Eigen::Quaterniond, N + 1>, Degree + 1> dxii_dtildeBik(f2); // Chained list of (partial) derivatives of xi_i with respect to tildeB_ik (Indices: [i][s] = (d^s xi_i)/(dtildeBik^s)) for i = f1, ... , f1 + f2 - 1
            for (int i = f1; i <= f1 + f2 - 1; i++) {
                // Compute (derivatives of) xi (with respect to tildeB_ik)
                Eigen::Quaterniond omegai_tildeBik = omega_i[i - f1]; // omega_i * tildeB_ik(x)
                omegai_tildeBik.coeffs() *= D_tildeB_ik[0].m_B[i];
                dxii_dtildeBik[i - f1][0] = geometry::quaternionExponential(omegai_tildeBik); // 0-th derivative of xi_i
                for (unsigned int d = 1; d <= N; d++)
                    dxii_dtildeBik[i - f1][d] = dxii_dtildeBik[i - f1][d - 1] * omega_i[i - f1];
            }

            // Compute value D0
            // ================
            // Reset value to identity
            Di[0] = Eigen::Quaterniond(1, 0, 0, 0);

            // Check, if the cumulative basis is a zero vector
            if (f3 < (int)D_tildeB_ik[0].m_B.size()) {
                // ...no -> tildeB_0,k > 0

                // Compute contribution of Q_1
                if (f1 > 0) // Skip if f_1 = 0 (we would have to use q_{-1} which is just the identity)
                    Di[0] = m_controlPoints[f1 - 1]; // Q1 = q_{f1-1}

                // Compute contribution of Q_2
                for (int i = f1; i <= f1 + f2 - 1; i++) {
                    // Introduction of "newValue" to avoid "value" on both sides of the equation (otherwise: could lead to inconsistent data - see Eigen documentation for details!)
                    Eigen::Quaterniond newValue = (Di[0]) * dxii_dtildeBik[i - f1][0];
                    Di[0] = newValue;
                }

                // Compute contribution of Q_3
                // -> identity -> nothing to do...
            }
            // else: -> q(t) = q_0^0 -> use identity -> do not change value anymore

            // Compute derivatives DN
            // ======================
            if (N > 0) {
                // If f2=0 -> all elements in the cumulative basis are zero -> all derivatives are zero!
                if (f2 == 0) {
                    for (unsigned int d = 1; d <= N; d++)
                        Di[d] = Eigen::Quaterniond(0, 0, 0, 0);
                } else {
                    // Compute x_i and its (total) derivatives with respect to x (uses formula of Faa di Bruno)
                    // ---------------------------------------------------------
                    // Pre-compute Faa di Bruno tuples
                    std::array<core::math::FaaDiBrunoTuples, N> T_F; // List of Faa di Bruno tuples for all "N" derivatives
                    for (unsigned int d = 1; d <= N; d++)
                        T_F[d - 1] = core::math::faaDiBrunoTuples(d);

                    // Compute (derivatives of) xi (with respect to x)
                    memory::SmartVector<std::array<Eigen::Quaterniond, N + 1>, Degree + 1> D_xi_i(f2); // Chained list of (total) derivatives of xi_i with respect to x (Indices: [i][s] = (d^s xi_i)/(dx^s)) for i = f1, ... , f1 + f2 - 1
                    for (int i = f1; i <= f1 + f2 - 1; i++) {
                        // Compute (derivatives of) xi (with respect to x)
                        D_xi_i[i - f1][0] = dxii_dtildeBik[i - f1][0]; // 0-th total derivative = 0-th partial derivative = xi_i(x)
                        for (unsigned int d = 1; d <= N; d++) {
                            // Initialize sum
                            D_xi_i[i - f1][d] = Eigen::Quaterniond(0, 0, 0, 0);

                            // Iterate over all Faa di Bruno tuples for this derivative
                            for (size_t t = 0; t < T_F[d - 1].size(); t++) {
                                // Compute sum m_1 + m_2 + ... + m_d
                                uint64_t sumOfTupleElements = 0; // = m_1 + m_2 + ... + m_d
                                for (size_t j = 0; j < T_F[d - 1][t].size(); j++)
                                    sumOfTupleElements += T_F[d - 1][t][j];

                                // Compute fraction of factorials coefficient
                                double fractionFactorials = core::math::factorial(d);
                                for (size_t j = 0; j < T_F[d - 1][t].size(); j++)
                                    fractionFactorials /= core::math::factorial(T_F[d - 1][t][j]);

                                // Compute product of D_tildeB_ik's
                                double prod_D_tildeB_ik = 1.0;
                                for (unsigned int j = 1; j <= d; j++) {
                                    double factor = D_tildeB_ik[j].m_B[i] / (double)core::math::factorial(j); // D^j_tildeB_ik/j!
                                    for (int u = 1; u <= (int)T_F[d - 1][t][j - 1]; u++)
                                        prod_D_tildeB_ik *= factor;
                                }

                                // Add term related to this tuple to the sum
                                D_xi_i[i - f1][d].coeffs() += (fractionFactorials * dxii_dtildeBik[i - f1][sumOfTupleElements].coeffs() * prod_D_tildeB_ik);
                            }
                        }
                    }

                    // Compute i-th derivative of q(x) with respect to x
                    // -------------------------------------------------
                    for (unsigned int d = 1; d <= N; d++) {
                        // Compute i-th derivative of Q2 with respect to x (uses general Leibniz rule)
                        // -----------------------------------------------
                        // Compute general Leibniz tuples
                        core::math::GeneralLeibnizTuples T_L_Di = core::math::generalLeibnizTuples(d, f2); // List of general Leibniz tuples

                        // Sum up over all tuples
                        Eigen::Quaterniond Di_Q2(0, 0, 0, 0); // D^i Q_2(x) (i-th derivative with respect to x)
                        for (size_t t = 0; t < T_L_Di.size(); t++) {
                            // Compute multinomial coefficient
                            const double multinomial = core::math::multinomialCoefficient(T_L_Di[t]);

                            // Compute product
                            Eigen::Quaterniond product = D_xi_i[0][T_L_Di[t][0]]; // Initialize as first factor of product
                            for (int z = 2; z <= f2; z++) {
                                // Introduction of "newProduct" to avoid "product" on both sides of the equation (otherwise: could lead to inconsistent data - see Eigen documentation for details!)
                                Eigen::Quaterniond newProduct = product * D_xi_i[z - 1][T_L_Di[t][z - 1]];
                                product = newProduct;
                            }

                            // Add contribution to overall sum
                            Di_Q2.coeffs() += (multinomial * product.coeffs());
                        }

                        // Compute i-th derivative of q(x) with respect to x
                        // -------------------------------------------------
                        // Compute Q1 * D^i Q_2(x)
                        if (f1 == 0) // Skip Q1 if f_1 = 0 (we would have to use q_{-1} which is just the identity)
                            Di[d] = Di_Q2;
                        else
                            Di[d] = m_controlPoints[f1 - 1] * Di_Q2; // Q1 = q_{f1-1}
                    }
                }
            }

            // Pass back list
            return Di;
        }

        // Computes **numeric** \f$ n\f$-th derivative of underlying curve using finite difference method (see base class for details
        /*!
         * \copydoc DynamicSizeQuaternionCurve::evaluateNumericDerivative()
         *
         * \warning Since the B-spline curve is only defined for \f$ x \in\,[t_p,\,t_m[\f$ (see definition above), the \p position \f$x\f$ will be projected to these bounds.
         * For the right interval bound \f$t_m\f$ (which is open!) a projection \f$x=t_m \rightarrow x=t_m-\epsilon\f$ is performed.
         */
        virtual Eigen::Quaterniond evaluateNumericDerivative(const double& position, const unsigned int& derivationOrder, const double& stepSize = 1e-6) const
        {
            // Initialize helpers
            const double nh_2 = derivationOrder * stepSize / 2.0; // Half-size of the evaluated "x-window" (n*h/2)

            // Project position to bounds [t_p, t_m[ (ATTENTION: we have to consider the evaluated "x-window"!
            double projectedPosition = position;
            const double t_p = m_basis.m_knotSequence[Degree];
            const double t_m = m_basis.m_knotSequence[m_basis.m_knotSequence.size() - order()];
            static const double epsilon = 1e-9;
            const double t_m_epsilon = t_m - epsilon; // "New" upper bound
            if (projectedPosition < t_p)
                projectedPosition = t_p;
            if (projectedPosition + nh_2 > t_m_epsilon)
                projectedPosition = t_m_epsilon - nh_2;

            // Call base class method
            return DynamicSizeQuaternionCurve::evaluateNumericDerivative(projectedPosition, derivationOrder, stepSize);
        }

        // Interpolation
        // -------------
        // Interpolate quaternion curve to achieve specific properties (through-points, derivatives, ...) (see base class for details)
        /*! \copydoc InterpolatableQuaternionCurve::interpolate() */
        virtual bool interpolate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, const QuaternionCurveInterpolationMethod& method, QuaternionCurveResult* const result = nullptr)
        {
            // Check interpolation method
            switch (method) {
            case QuaternionCurveInterpolationMethod::BSPLINE_SIMPLE: {
                /*! Interpolation with BSPLINE_SIMPLE
                 *  ---------------------------------
                 * \copydetails interpolateSimple()
                 */
                return interpolateSimple(parameters, proportions, result);
            }
            case QuaternionCurveInterpolationMethod::BSPLINE_THROUGHPOINTS: {
                /*! Interpolation with BSPLINE_THROUGHPOINTS
                 *  ----------------------------------------
                 * \copydetails interpolateThroughPoints()
                 */
                return interpolateThroughPoints(parameters, proportions, result);
            }
            default: {
                // No interpolation methods implemented for now
                if (result != nullptr)
                    *result = QuaternionCurveResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }
            }
        }

    protected:
        //! Simple interpolation with given *virtual* quaternion control points (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Directly sets passed quaternions as control points of the B-spline. Curve does **not** necessarily pass through intermediate control points!
         *
         * \par Parameter-layout
         * Element         | Description
         * -------         | -----------
         * \p parameters[0]   | first keyframe quaternion (will be passed through for \f$x=0\f$)
         * ...             | ...
         * \p parameters[m-1] | last keyframe quaternion (will be passed through for \f$x=1\f$)
         *
         *
         * (Re-)initializes a **quaternion B-Spline curve** of order \f$k\f$ consisting of
         *  * a sequence of \f$m\geq k\f$ control-point quaternions (obtained from \p parameters) and
         *  * an underlying B-spline basis (see \ref m_basis) with a knot-sequence \f$\tau = [t_0,\,\dots\,,\,t_{n-1}]\f$ (\ref BSplineBasis::m_knotSequence) containing \f$n = m+k\f$ non-decreasing knots.
         *
         * The knot-sequence of the B-spline basis is automatically chosen (uniform, clamped) depending on the count of passed keyframe quaternions (see \ref BSplineBasis::setClampedUniformKnots() for details).
         *
         * The resulting spline is \f$\mathcal{C}^{k-2}\f$-continuous, where \f$k = p + 1\f$ denotes the *order* and \f$p\f$ the *degree* of the underlying B-spline basis.
         *
         * \par Proportions
         * If \p proportions are passed to this function, a custom (non-uniform) clamped knot-sequence can be chosen. The \p proportions define the positioning of the \f$m-k\f$ "interior" knots
         * \f$t_{p+1},\,\dots\,,\,t_{m-1}\f$. If no \p proportions are passed, a *uniform* partitioning will be chosen.
         *
         * The control-point quaternions are chosen to be the given keyframe quaternion (\p parameters). The curve passes through the very first and very last keyframe quaternion (clamped knot sequence), but
         * it **does not** pass through intermediate keyframe quaternions!
         *
         * For details on the underlying method see
         * * Myoung-Jun Kim et al., "A C2-continuous B-spline Quaternion Curve Interpolating a Given Sequence of Solid Orientations", Proceedings Computer Animation'95, 1995, pages 72--81, DOI:[10.1109/CA.1995.393545](https://www.doi.org/10.1109/CA.1995.393545)
         *
         * \attention You have to pass **at least** \f$k\f$ quaternions in \p parameters. Accordingly \p proportions must have \f$m-k\f$ elements (if specified, otherwise use `nullptr` for a uniform knot sequence).
         */
        virtual bool interpolateSimple(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, QuaternionCurveResult* const result = nullptr)
        {
            // Check, if proportions are specified
            bool proportionsSpecified = false;
            if (proportions != nullptr)
                if (proportions->size() > 1) // There have to be at least two proportions specified
                    proportionsSpecified = true;

            // Checking parameter set
            if (parameters.size() /* m */ < order() /* k */ || (proportionsSpecified == true && ((int)proportions->size() - 1) != ((int)parameters.size() - (int)order()) /* m - k */)) {
                if (result != nullptr)
                    *result = QuaternionCurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Compute knot-sequence of basis
            if (proportionsSpecified == true) {
                // Resize knot sequence n = m + k
                m_basis.m_knotSequence.resize(parameters.size() + order()); // n = m + k

                // Clamped: set first k knots to 0 and last k knots to 1
                for (unsigned int i = 0; i < order(); i++) {
                    m_basis.m_knotSequence[i] = 0;
                    m_basis.m_knotSequence[m_basis.m_knotSequence.size() - 1 - i] = 1;
                }

                // Compute non-uniform distribution of "interior" knots
                const int interiorKnotCount = m_basis.m_knotSequence.size() - 2 * order(); // Count of "interior" knots
                if (interiorKnotCount > 0)
                    for (int i = 0; i < interiorKnotCount; i++)
                        m_basis.m_knotSequence[order() + i] = m_basis.m_knotSequence[order() + i - 1] + (*proportions)[i];
            } else
                m_basis.setClampedUniformKnots(parameters.size() /* m */);

            // Set control-point quaternions directly
            m_controlPoints = parameters;

            // Success!
            if (result != nullptr)
                *result = QuaternionCurveResult::SUCCESS;
            return true;
        }

        //! Interpolation with given *through-point* quaternion control points (returns `true` on success, `false` otherwise) (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Automatically computes appropriate control points of the B-spline, such that the resulting curve passes through the given quaternion sequence ("through-points").
         *
         * \par Parameter-layout
         * Element         | Description
         * -------         | -----------
         * \p parameters[0]   | first keyframe quaternion (will be passed through for \f$x=0\f$)
         * ...             | ...
         * \p parameters[m-1] | last keyframe quaternion (will be passed through for \f$x=1\f$)
         *
         *
         * (Re-)initializes a **quaternion B-Spline curve** of order \f$k\f$ consisting of
         *  * a sequence of \f$m\geq k\f$ control-point quaternions (obtained from \p parameters) and
         *  * an underlying B-spline basis (see \ref m_basis) with a knot-sequence \f$\tau = [t_0,\,\dots\,,\,t_{n-1}]\f$ (\ref BSplineBasis::m_knotSequence) containing \f$n = m+k\f$ non-decreasing knots.
         *
         * The knot-sequence of the B-spline basis is automatically chosen (uniform, clamped) depending on the count of passed keyframe quaternions (see \ref BSplineBasis::setClampedUniformKnots() for details).
         *
         * The resulting spline is \f$\mathcal{C}^{k-2}\f$-continuous, where \f$k = p + 1\f$ denotes the *order* and \f$p\f$ the *degree* of the underlying B-spline basis.
         *
         * \par Proportions
         * If \p proportions are passed to this function, a custom (non-uniform) clamped knot-sequence can be chosen. The \p proportions define the positioning of the \f$m-k\f$ "interior" knots
         * \f$t_{p+1},\,\dots\,,\,t_{m-1}\f$. If no \p proportions are passed, a *uniform* partitioning will be chosen.
         *
         * The control-point quaternions are chosen, such that the resulting curve interpolates the given keyframe quaternions (\p parameters). Thus the curve passes through **all** given keyframe quaternions.
         *
         * \warning This interpolation method has not been implemented yet!
         *
         * For details on the underlying method see
         * * Myoung-Jun Kim et al., "A C2-continuous B-spline Quaternion Curve Interpolating a Given Sequence of Solid Orientations", Proceedings Computer Animation'95, 1995, pages 72--81, DOI:[10.1109/CA.1995.393545](https://www.doi.org/10.1109/CA.1995.393545)
         *
         * \attention You have to pass **at least** \f$k\f$ quaternions in \p parameters. Accordingly \p proportions must have \f$m-k\f$ elements (if specified, otherwise use `nullptr` for a uniform knot sequence).
         */
        virtual bool interpolateThroughPoints(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, QuaternionCurveResult* const result = nullptr)
        {
            // Check, if proportions are specified
            bool proportionsSpecified = false;
            if (proportions != nullptr)
                if (proportions->size() > 1) // There have to be at least two proportions specified
                    proportionsSpecified = true;

            // Checking parameter set
            if (parameters.size() /* m */ < order() /* k */ || (proportionsSpecified == true && ((int)proportions->size() - 1) != ((int)parameters.size() - (int)order()) /* m - k */)) {
                if (result != nullptr)
                    *result = QuaternionCurveResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Compute knot-sequence of basis
            if (proportionsSpecified == true) {
                // Resize knot sequence n = m + k
                m_basis.m_knotSequence.resize(parameters.size() + order()); // n = m + k

                // Clamped: set first k knots to 0 and last k knots to 1
                for (unsigned int i = 0; i < order(); i++) {
                    m_basis.m_knotSequence[i] = 0;
                    m_basis.m_knotSequence[m_basis.m_knotSequence.size() - 1 - i] = 1;
                }

                // Compute non-uniform distribution of "interior" knots
                const int interiorKnotCount = m_basis.m_knotSequence.size() - 2 * order(); // Count of "interior" knots
                if (interiorKnotCount > 0)
                    for (int i = 0; i < interiorKnotCount; i++)
                        m_basis.m_knotSequence[order() + i] = m_basis.m_knotSequence[order() + i - 1] + (*proportions)[i];
            } else
                m_basis.setClampedUniformKnots(parameters.size() /* m */);

            // TODO: Set control-point quaternions accordingly
            if (result != nullptr)
                *result = QuaternionCurveResult::ERROR_NOTIMPLEMENTED;
            assert(false);
            return false;

            // Success!
            if (result != nullptr)
                *result = QuaternionCurveResult::SUCCESS;
            return true;
        }

        // Helpers
        // -------
    protected:
        //! Projects the given position (interpolation parameter) to its bounds \f$ x \in\,[t_p,\,t_m[\f$
        /*!
         * Since the B-spline curve is only defined for \f$ x \in\,[t_p,\,t_m[\f$ (see definition above), the \p position \f$x\f$ has to be projected to these bounds.
         * For the right interval bound \f$t_m\f$ (which is open!) a projection \f$x=t_m \rightarrow x=t_m-\epsilon\f$ is performed.
         *
         * \warning You may call \ref isValid() before to check, if the quaternion curve is valid! If the quaternion curve is invalid the behaviour of this method is not defined!
         *
         * \param [in] position *Desired* position to evaluate (derivative) at (may lie outside of \f$[0,\,1]\f$)
         * \return *Projected* position \f$ x \in\,[t_p,\,t_m[\f$
         */
        virtual double projectPositionToBounds(const double& position) const
        {
            // Compute bounds
            const double t_p = m_basis.m_knotSequence[Degree];
            const double t_m = m_basis.m_knotSequence[m_basis.m_knotSequence.size() - order()];

            // Define "tolerance" for right bound
            static const double epsilon = 1e-9;

            // Compute "new" upper bound
            const double t_m_epsilon = t_m - epsilon;

            // Project
            if (position < t_p)
                return t_p;
            if (position > t_m_epsilon)
                return t_m_epsilon;
            return position;
        }

        // Encoding
        // --------
    public:
        // Encode member data as XML element and add it to the specified stream (see base class for details)
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const size_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<QuaternionBSplineCurve");

            // Write attributes
            addedElements += encodeXMLAttributes(stream, numericFormat);
            addedElements += io::encoding::encode(stream, ">\n");

            // Write children
            addedElements += m_basis.encodeToXML(stream, XMLIndentationLevel + 1, XMLTabWhiteSpaces, numericFormat);

            // End XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "</QuaternionBSplineCurve>\n");

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
