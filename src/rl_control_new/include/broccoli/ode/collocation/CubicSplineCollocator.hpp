/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/math.hpp"
#include "../../curve/trajectories/PolynomialTrajectory.hpp"
#include "SplineCollocatorBase.hpp"
#include <assert.h>
#include <string>
#include <vector>

namespace broccoli {
namespace ode {
    //! Collocation manager for **cubic** spline collocation of second-order linear time-variant ordinary differential equations (ODEs)
    /*!
     * \ingroup broccoli_ode_collocation
     *
     * \copydetails broccoli::ode::SplineCollocatorBase
     *
     * \par Additional input parameters
     *   * \ref m_method - \copybrief m_method
     */
    class CubicSplineCollocator : public SplineCollocatorBase {
    public:
        //! Specification of collocation method
        enum class CollocationMethod : uint8_t {
            COLLOCATION_WITHOUT_FIRST_DERIVATIVES = 0, //!< Cubic spline collocation **without** first derivative boundary conditions (they have to be specified but will be ignored)
            COLLOCATION_WITH_FIRST_DERIVATIVES, //!< Cubic spline collocation **with** first derivative boundary conditions (enables virtual control points)
            COLLOCATIONMETHOD_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given collocation method
        static std::string collocationMethodString(const CollocationMethod& method)
        {
            // Check method
            switch (method) {
            case CollocationMethod::COLLOCATION_WITHOUT_FIRST_DERIVATIVES:
                return "COLLOCATION_WITHOUT_FIRST_DERIVATIVES";
            case CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES:
                return "COLLOCATION_WITH_FIRST_DERIVATIVES";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of **own** collocation method
        inline std::string collocationMethodString() const { return collocationMethodString(m_method); }

        // Constructor
        /*!
         * \copydoc broccoli::ode::SplineCollocatorBase::SplineCollocatorBase()
         * \param [in] method Intitializes \ref m_method - \copybrief m_method
         */
        CubicSplineCollocator(const CollocationMethod& method = CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES)
            : m_method(method)
        {
        }

        // Destructor
        /*! \copydoc broccoli::ode::SplineCollocatorBase::~SplineCollocatorBase() */
        virtual ~CubicSplineCollocator()
        {
        }

        // Input members
        // -------------
        CollocationMethod m_method; //!< **Input:** Collocation method to be used

        // Internal members (buffers)
        // ----------------
        Eigen::VectorXd m_lowerDiagonalOfA; //!< Buffer for lower diagonal elements of tridiagonal matrix \f$ A \f$
        Eigen::VectorXd m_diagonalOfA; //!< Buffer for diagonal elements of tridiagonal matrix \f$ A \f$
        Eigen::VectorXd m_upperDiagonalOfA; //!< Buffer for upper diagonal elements of tridiagonal matrix \f$ A \f$
        Eigen::MatrixXd m_dB_dlambda; //!< Buffer for right hand side \f$ \left( \partial B / \partial \lambda\right) \f$
        Eigen::MatrixXd m_dB_dr; //!< Buffer for right hand side \f$ \left( \partial B / \partial r\right) \f$
        Eigen::MatrixXd m_grad_lambda_dot_s0xi0_xi00; //!< Buffer for \f$ \nabla_{\lambda} \dot{s}_0(\xi_0)|_{\xi_0=0} \f$
        Eigen::MatrixXd m_grad_r_dot_s0xi0_xi00; //!< Buffer for \f$ \nabla_{r} \dot{s}_0(\xi_0)|_{\xi_0=0} \f$
        Eigen::MatrixXd m_grad_lambda_dot_snm1xinm1_xinm11; //!< Buffer for \f$ \nabla_{\lambda} \dot{s}_{n-1}(\xi_{n-1})|_{\xi_{n-1}=1} \f$
        Eigen::MatrixXd m_grad_r_dot_snm1xinm1_xinm11; //!< Buffer for \f$ \nabla_{r} \dot{s}_{n-1}(\xi_{n-1})|_{\xi_{n-1}=1} \f$

        // Output members
        // --------------
        broccoli::curve::PolynomialTrajectory<3, 1> m_trajectory; //!< **Output:** Trajectory encapsulating the spline \f$ y(t) \f$ which approximates the solution of \f$ F(t) \f$

        // Processing
        // ----------
        // Perform remapping of spline knots (adds virtual control points if necessary) (see base class for details)
        virtual bool substep_addVirtualControlPoints(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Initialize helpers
            const size_t originalKnotCount = m_knotSites.rows();
            if (originalKnotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                assert(false);
                return false;
            }

            // Check, if virtual control points are necessary
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES) {
                // ...we need to satisfy first order boundary conditions too -> we accomplish this by introducing two virtual control points which
                // give us the necessary degrees of freedom

                // Resize buffer of remapped knots
                const Eigen::Index remappedKnotCount = originalKnotCount + 2; // Add two virtual control points
                m_knotSitesRemapped.resize(remappedKnotCount); // Allocate buffer

                // Copy boundary knots
                m_knotSitesRemapped(0) = m_knotSites(0); // t_0
                m_knotSitesRemapped(remappedKnotCount - 1) = m_knotSites(originalKnotCount - 1); // t_n

                // Copy collocation sites (aka "original interior knots)
                m_knotSitesRemapped.block(2, 0, originalKnotCount - 2, 1) = m_knotSites.block(1, 0, originalKnotCount - 2, 1);

                // Add virtual control points (in center of first and last "original" segment)
                m_knotSitesRemapped(1) = (m_knotSitesRemapped(0) + m_knotSitesRemapped(2)) / 2.0;
                m_knotSitesRemapped(remappedKnotCount - 2) = (m_knotSitesRemapped(remappedKnotCount - 3) + m_knotSitesRemapped(remappedKnotCount - 1)) / 2.0;
            } else {
                // ...we do not need to satisfy first order boundary conditions -> do not remap (just use the same knots as specified by the user)
                m_knotSitesRemapped = m_knotSites;
            }

            // No error occured -> success
            return true;
        }

        // Setup of (internal) (block-)tridiagonal system from knot sites (see base class for details)
        virtual bool substep_setupBlockTridiagonalSystem(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Initialize helpers
            const size_t knotCount = m_knotSitesRemapped.rows();
            if (knotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                assert(false);
                return false;
            }
            const size_t interiorKnotCount = knotCount - 2; // Count of interior knots (may include virtual control points if present)

            // Initialize tridiagonal matrix
            m_lowerDiagonalOfA.resize(interiorKnotCount); // First element is "outside" the matrix A (to be ignored)
            m_diagonalOfA.resize(interiorKnotCount);
            m_upperDiagonalOfA.resize(interiorKnotCount); // Last element is "outside" the matrix A (to be ignored)

            // Initialize right hand side (dB/dlambda) and (dB/dr)
            m_dB_dlambda = Eigen::MatrixXd::Zero(interiorKnotCount, interiorKnotCount);
            m_dB_dr = Eigen::MatrixXd::Zero(interiorKnotCount, 4);

            // Get timing of first segment
            const double h0 = m_knotSitesRemapped(1) - m_knotSitesRemapped(0); // Duration of first segment
            if (h0 <= 0) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                assert(false);
                return false;
            }

            // Get timing of last segment
            const double hnm1 = m_knotSitesRemapped(knotCount - 1) - m_knotSitesRemapped(knotCount - 2); // Duration of last segment
            if (hnm1 <= 0) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                assert(false);
                return false;
            }

            // Iterate through rows of system
            double hi = h0; // hi
            for (size_t j = 0; j < interiorKnotCount; j++) {
                // Index shift is necessary, since we want to skip i=0 and comply with the general notation/meaning of index "i"
                const size_t i = j + 1;

                // Update current values
                const double him1 = hi; // h_{i-1}
                hi = m_knotSitesRemapped(i + 1) - m_knotSitesRemapped(i); // Duration of current segment
                if (hi <= 0) {
                    if (result != nullptr)
                        *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                    assert(false);
                    return false;
                }

                // Compute upper diagonal element of A
                if (j == interiorKnotCount - 1)
                    m_upperDiagonalOfA(j) = 0; // Last element is "outside" the matrix A (to be ignored)
                else
                    m_upperDiagonalOfA(j) = hi;

                // Compute diagonal element of A
                m_diagonalOfA(j) = 2.0 * (him1 + hi);

                // Compute lower diagonal element of A
                if (j == 0)
                    m_lowerDiagonalOfA(j) = 0; // First element is "outside" the matrix A (to be ignored)
                else
                    m_lowerDiagonalOfA(j) = him1;

                // Compute elements in (dB/dlambda)
                if (i > 1)
                    m_dB_dlambda(j, i - 2) = 6.0 / him1;
                m_dB_dlambda(j, i - 1) = -6.0 / hi - 6.0 / him1;
                if (i < interiorKnotCount)
                    m_dB_dlambda(j, i) = 6.0 / hi;

                // Compute elements in (dB/dr)
                if (i == 1) {
                    m_dB_dr(j, 0) = 6.0 / him1;
                    m_dB_dr(j, 1) = -him1;
                }
                if (i == interiorKnotCount) {
                    m_dB_dr(j, 2) = 6.0 / hi;
                    m_dB_dr(j, 3) = -hi;
                }
            }

            // No error occured -> success
            return true;
        }

        // Solve (internal) (block-)tridiagonal system (see base class for details)
        virtual bool substep_solveBlockTridiagonalSystem(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Initialize helpers
            const size_t knotCount = m_knotSitesRemapped.rows();
            if (knotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                assert(false);
                return false;
            }
            broccoli::core::math::Result LSEsolveResult;

            // Check count of knots to decide which method is the fastest
            if (knotCount <= 50 /* <-- seems to be the break-even point */) {
                // ...dimension of lambda is in the same domain as the dimension of r
                // --> solving for (deta/dr) will take a similar runtime when compared to solving for (deta/dlambda)
                //   --> stack (deta/dlambda) and (deta/dr) to compute them at the same time

                // Create stacks [(dB/dlambda) (dB/dr)] and [(deta/dlambda) (deta/dr)]
                Eigen::MatrixXd dB_dlambda_dr(m_dB_dlambda.rows(), m_dB_dlambda.cols() + m_dB_dr.cols()); // [(dB/dlambda) (dB/dr)]
                dB_dlambda_dr << m_dB_dlambda, m_dB_dr; // Horizontal stacking
                Eigen::MatrixXd deta_dlambda_dr(m_deta_dlambda.rows(), m_deta_dlambda.cols() + m_deta_dr.cols()); // [(deta/dlambda) (deta/dr)]

                // Solve for [(deta/dlambda) (deta/dr)]
                if (broccoli::core::math::solveTriDiagonalSystemOfEquations(m_lowerDiagonalOfA, m_diagonalOfA, m_upperDiagonalOfA, dB_dlambda_dr, deta_dlambda_dr, &LSEsolveResult) == false) {
                    if (result != nullptr)
                        *result = Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH;
                    assert(false);
                    return false;
                }
                if (LSEsolveResult == broccoli::core::math::Result::WARNING_CLOSE_TO_SINGULAR)
                    if (result != nullptr)
                        *result = Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR;

                // Extract (deta/dlambda) and (deta/dr) from [(deta/dlambda) (deta/dr)]
                m_deta_dlambda = deta_dlambda_dr.block(0, 0, m_dB_dlambda.rows(), m_dB_dlambda.cols());
                m_deta_dr = deta_dlambda_dr.block(0, m_dB_dlambda.cols(), m_dB_dr.rows(), m_dB_dr.cols());
            } else {
                // ...dimension of lambda is big in comparison to r
                // --> solving for (deta/dr) is much faster than solving for (deta/dlambda)
                //   --> handle (deta/dlambda) and (deta/dr) separately to avoid wasting time with stacking

                // Solve for (dX/dlambda) = (deta/dlambda)
                if (broccoli::core::math::solveTriDiagonalSystemOfEquations(m_lowerDiagonalOfA, m_diagonalOfA, m_upperDiagonalOfA, m_dB_dlambda, m_deta_dlambda, &LSEsolveResult) == false) {
                    if (result != nullptr)
                        *result = Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH;
                    assert(false);
                    return false;
                }
                if (LSEsolveResult == broccoli::core::math::Result::WARNING_CLOSE_TO_SINGULAR)
                    if (result != nullptr)
                        *result = Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR;

                // Solve for (dX/dr) = (deta/dr)
                if (broccoli::core::math::solveTriDiagonalSystemOfEquations(m_lowerDiagonalOfA, m_diagonalOfA, m_upperDiagonalOfA, m_dB_dr, m_deta_dr, &LSEsolveResult) == false) {
                    if (result != nullptr)
                        *result = Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH;
                    assert(false);
                    return false;
                }
                if (LSEsolveResult == broccoli::core::math::Result::WARNING_CLOSE_TO_SINGULAR)
                    if (result != nullptr)
                        *result = Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR;
            }

            // (no need to map between X and eta at this point, since they are the SAME for cubic splines (S = indentity))

            // No error occured -> success
            return true;
        }

        // Compute spline gradients (see base class for details)
        /*!
         * \copydoc SplineCollocatorBase::substep_computeSplineGradients()
         *
         * \remark Additionally computes \f$ \nabla_{\lambda}\dot{s}_0(0) \f$, \f$ \nabla_{r}\dot{s}_0(0) \f$, \f$ \nabla_{\lambda}\dot{s}_{n-1}(1) \f$, and \f$ \nabla_{r}\dot{s}_{n-1}(1) \f$ if virtual control points are used
         */
        virtual bool substep_computeSplineGradients(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Initialize helpers
            const size_t knotCount = m_knotSitesRemapped.rows();
            if (knotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                assert(false);
                return false;
            }
            const size_t interiorKnotCount = knotCount - 2; // Count of interior knots (may include virtual control points if present)
            size_t collocationSiteCount = knotCount - 2; // Do not collocate at boundaries
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES)
                collocationSiteCount -= 2; // Do not collocate at virtual control points

            /* Spline gradients:
             * -----------------
             *    si(xi)      =        ai*xi^3 +        bi*xi^2 +        ci*xi^1 +        di
             *   dsi(xi)/dt   =   3/hi*ai*xi^2 +   2/hi*bi*xi^1 +   1/hi*ci
             * d^2si(xi)/dt^2 = 6/hi^2*ai*xi^1 + 2/hi^2*bi
             *
             * lambda = [y1, ..., ynm1] = [lambda1, ..., lambdanm1]
             * eta    = [ddot_y1, ..., ddot_ynm1] = [eta1, eta2, ..., etanm1]
             * r      = [y0, ddot_y0, yn, ddot_yn] = [r1, ..., r4]
             *
             * si(xi) is a linear function with respect to lambda, eta and r, thus:
             * --> si(xi) = (dsi(xi)/dlambda) * lambda + (dsi(xi)/deta) * eta + (dsi(xi)/dr) * r
             * --> dsi(xi)/dt = (d(dot_si(xi))/dlambda) * lambda + (d(dot_si(xi))/deta) * eta + (d(dot_si(xi))/dr) * r
             * --> d^2si(xi)/dt^2 = (d(ddot_si(xi))/dlambda) * lambda + (d(ddot_si(xi))/deta) * eta + (d(ddot_si(xi))/dr) * r
             *
             * Further eta is a linear function of lambda and r, thus:
             * --> si(xi) = [(dsi(xi)/dlambda) + (dsi(xi)/deta) * (deta/dlambda)] * lambda + [(dsi(xi)/dr) + (dsi(xi)/deta) * (deta/dr)] * r
             *               |_______________|   |____________|   |____________|              |__________|   |____________|   |_______|
             *                       |                 |                |                          |               |              |
             *                      (I)               (II)            (III)                       (IV)            (V)            (VI)
             *              |___________________________________________________|            |_________________________________________|
             *                                        |                                                            |
             *                                 grad_lambda_si(xi)                                             grad_r_si(xi)
             *
             * --> dsi(xi)/dt = [(d(dot_si(xi))/dlambda) + (d(dot_si(xi))/deta) * (deta/dlambda)] * lambda + [(d(dot_si(xi))/dr) + (d(dot_si(xi))/deta) * (deta/dr)] * r
             *                   |_____________________|   |__________________|   |____________|              |________________|   |__________________|   |_______|
             *                              |                       |                   |                             |                     |                 |
             *                             (I)                     (II)               (III)                          (IV)                  (V)               (VI)
             *                  |________________________________________________________________|            |______________________________________________________|
             *                                                  |                                                                            |
             *                                         grad_lambda_dot_si(xi)                                                        grad_r_dot_si(xi)
             *
             * --> d^2si(xi)/dt^2 = [(d(ddot_si(xi))/dlambda) + (d(ddot_si(xi))/deta) * (deta/dlambda)] * lambda + [(d(ddot_si(xi))/dr) + (d(ddot_si(xi))/deta) * (deta/dr)] * r
             *                       |______________________|   |___________________|   |____________|              |_________________|   |___________________|   |_______|
             *                                  |                         |                   |                              |                      |                 |
             *                                 (I)                       (II)               (III)                           (IV)                   (V)               (VI)
             *                      |_________________________________________________________________|            |_______________________________________________________|
             *                                                      |                                                                            |
             *                                           grad_lambda_ddot_si(xi)                                                        grad_r_ddot_si(xi)
             *
             * Compact notation:
             * --> si(xi) = grad_lambda_si(xi) * lambda + grad_r_si(xi) * r
             * --> dsi(xi)/dt = grad_lambda_dot_si(xi) * lambda + grad_r_dot_si(xi) * r
             * --> d^2si(xi)/dt^2 = grad_lambda_ddot_si(xi) * lambda + grad_r_ddot_si(xi) * r
             */

            // Initialize gradients for collocation sites
            m_grad_lambda_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, interiorKnotCount);
            m_grad_r_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, 4);
            m_grad_lambda_dot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, interiorKnotCount);
            m_grad_r_dot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, 4);
            m_grad_lambda_ddot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, interiorKnotCount);
            m_grad_r_ddot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, 4);

            // Initialize gradients for virtual control points
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES) {
                m_grad_lambda_dot_s0xi0_xi00 = Eigen::MatrixXd::Zero(1, interiorKnotCount);
                m_grad_r_dot_s0xi0_xi00 = Eigen::MatrixXd::Zero(1, 4);
                m_grad_lambda_dot_snm1xinm1_xinm11 = Eigen::MatrixXd::Zero(1, interiorKnotCount);
                m_grad_r_dot_snm1xinm1_xinm11 = Eigen::MatrixXd::Zero(1, 4);
            }

            // Compute gradients for collocation sites
            // ---------------------------------------
            for (size_t k = 0; k < collocationSiteCount; k++) {
                // Index shift
                size_t i = k + 1; // Default: first collocation point is second knot
                if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES)
                    i += 1; // With virtual control points -> do not collocate at virtual control points -> first collocation point is third knot

                // Get timing
                const double hi = m_knotSitesRemapped(i + 1) - m_knotSitesRemapped(i);
                const double gi = 1.0 / hi;

                // Term (I), (II)*(III), (IV) and (V)*(VI):
                // ----------------------------------------
                // Term (I):
                /* ---------
                 * (d(si(xi))/dlambda)      =        (dai/dlambda)*xi^3 +        (dbi/dlambda)*xi^2 +      (dci/dlambda)*xi^1 + (ddi/dlambda)
                 * (d(dot_si(xi))/dlambda)  =   3/hi*(dai/dlambda)*xi^2 +   2/hi*(dbi/dlambda)*xi^1 + 1/hi*(dci/dlambda)
                 * (d(ddot_si(xi))/dlambda) = 6/hi^2*(dai/dlambda)*xi^1 + 2/hi^2*(dbi/dlambda)
                 *
                 * Evaluation at xi = 0:
                 * (d(si(xi))/dlambda)|xi=0      = (ddi/dlambda)
                 * (d(dot_si(xi))/dlambda)|xi=0  = 1/hi*(dci/dlambda)
                 * (d(ddot_si(xi))/dlambda)|xi=0 = 2/hi^2*(dbi/dlambda)
                 */

                // ddi/dlambdau = 1 for u=i, 0 otherwise (WARNING: i=0...n-1 and u=1...n-1)
                // --> (d(si(xi))/dlambdau)|xi=0 = 1 for u=i, 0 otherwise
                m_grad_lambda_skxik_xik0(k, i - 1) += 1;

                // dci/dlambdau = -1 for u=i, 1 for u=i+1, 0 otherwise (WARNING: i=0...n-1 and u=1...n-1)
                // --> (d(dot_si(xi))/dlambdau)|xi=0 = 1/hi*(-1) = -1/hi = -gi for u=i
                //                                   = 1/hi*(1) = 1/hi = gi    for u=i+1
                //                                   = 0                       otherwise
                m_grad_lambda_dot_skxik_xik0(k, i - 1) -= gi;
                if (i < interiorKnotCount)
                    m_grad_lambda_dot_skxik_xik0(k, i) += gi;

                // (d(ddot_si(xi))/dlambda)|xi=0 = 0 since (dbi/dlambda) = 0

                // Term (II)*(III) and (V)*(VI):
                /* -----------------------------
                 * (d(si(xi))/deta)      =        (dai/deta)*xi^3 +        (dbi/deta)*xi^2 +      (dci/deta)*xi^1 + (ddi/deta)
                 * (d(dot_si(xi))/deta)  =   3/hi*(dai/deta)*xi^2 +   2/hi*(dbi/deta)*xi^1 + 1/hi*(dci/deta)
                 * (d(ddot_si(xi))/deta) = 6/hi^2*(dai/deta)*xi^1 + 2/hi^2*(dbi/deta)
                 *
                 * Evaluation at xi = 0:
                 * (d(si(xi))/deta)|xi=0      = (ddi/deta)
                 * (d(dot_si(xi))/deta)|xi=0  = 1/hi*(dci/deta)
                 * (d(ddot_si(xi))/deta)|xi=0 = 2/hi^2*(dbi/deta)
                 */

                // (d(si(xi))/deta)|xi=0 = 0 since (ddi/deta) = 0

                // dci/detav = -hi^2/3 for v=i, -hi^2/6 for v=i+1, 0 otherwise (WARNING: i=0...n-1 and v=1...n-1)
                // --> (d(dot_si(xi))/detav)|xi=0 = 1/hi*(-hi^2/3) = -hi/3 for v=i
                //                                = 1/hi*(-hi^2/6) = -hi/6 for v=i+1
                //                                = 0                      otherwise
                // --> pick corresponding rows from (deta/dlambda) and (deta/dr) to avoid unneccessary computations (only two elements of dci/detav are non-zero)
                m_grad_lambda_dot_skxik_xik0.block(k, 0, 1, interiorKnotCount) += -(hi / 3.0) * m_deta_dlambda.block(i - 1, 0, 1, interiorKnotCount);
                if (i < interiorKnotCount)
                    m_grad_lambda_dot_skxik_xik0.block(k, 0, 1, interiorKnotCount) += -(hi / 6.0) * m_deta_dlambda.block(i, 0, 1, interiorKnotCount);
                m_grad_r_dot_skxik_xik0.block<1, 4>(k, 0) += -(hi / 3.0) * m_deta_dr.block<1, 4>(i - 1, 0);
                if (i < interiorKnotCount)
                    m_grad_r_dot_skxik_xik0.block<1, 4>(k, 0) += -(hi / 6.0) * m_deta_dr.block<1, 4>(i, 0);

                // dbi/detav = hi^2/2 for v=i, 0 otherwise (WARNING: i=0...n-1 and v=1...n-1)
                // --> (d(ddot_si(xi))/detav)|xi=0 = 2/hi^2*(hi^2/2) = 1 for v=i, 0 otherwise
                // --> pick corresponding row from (deta/dlambda) and (deta/dr) to avoid unneccessary computations (only one element of dbi/detav is non-zero)
                m_grad_lambda_ddot_skxik_xik0.block(k, 0, 1, interiorKnotCount) += m_deta_dlambda.block(i - 1, 0, 1, interiorKnotCount);
                m_grad_r_ddot_skxik_xik0.block<1, 4>(k, 0) += m_deta_dr.block<1, 4>(i - 1, 0);

                // Term (IV):
                /* ----------
                 * (d(si(xi))/dr)      =        (dai/dr)*xi^3 +        (dbi/dr)*xi^2 +      (dci/dr)*xi^1 + (ddi/dr)
                 * (d(dot_si(xi))/dr)  =   3/hi*(dai/dr)*xi^2 +   2/hi*(dbi/dr)*xi^1 + 1/hi*(dci/dr)
                 * (d(ddot_si(xi))/dr) = 6/hi^2*(dai/dr)*xi^1 + 2/hi^2*(dbi/dr)
                 *
                 * Evaluation at xi = 0:
                 * (d(si(xi))/dr)|xi=0      = (ddi/dr)
                 * (d(dot_si(xi))/dr)|xi=0  = 1/hi*(dci/dr)
                 * (d(ddot_si(xi))/dr)|xi=0 = 2/hi^2*(dbi/dr)
                 */

                // ddi/drw = 1 for i=0 and w=1, 0 otherwise (WARNING: i=0...n-1 and w=1...4)
                // --> (d(si(xi))/drw)|xi=0 = 1 for i=0 and w=1, 0 otherwise
                if (i == 0)
                    m_grad_r_skxik_xik0(k, 0) += 1;

                // dci/drw = -1 for i=0 and w=1, -hi^2/3 for i=0 and w=2, 1 for i=n-1 and w=3, -hi^2/6 for i=n-1 and w=4, 0 otherwise (WARNING: i=0...n-1 and w=1...4)
                // --> (d(dot_si(xi))/drw)|xi=0 = 1/hi*(-1) = -1/hi = -gi for i=0 and w=1
                //                              = 1/hi*(-hi^2/3) = -hi/3  for i=0 and w=2
                //                              = 1/hi*(1) = 1/hi = gi    for i=n-1 and w=3
                //                              = 1/hi*(-hi^2/6) = -hi/6  for i=n-1 and w=4
                //                              = 0                       otherwise
                if (i == 0) {
                    m_grad_r_dot_skxik_xik0(k, 0) += -gi;
                    m_grad_r_dot_skxik_xik0(k, 1) += -hi / 3.0;
                }
                if (i == knotCount - 2 /* <- n-1 */) {
                    m_grad_r_dot_skxik_xik0(k, 2) += gi;
                    m_grad_r_dot_skxik_xik0(k, 3) += -hi / 6.0;
                }

                // dbi/drw = hi^2/2 for i=0 and w=2, 0 otherwise (WARNING: i=0...n-1 and w=1...4)
                // --> (d(ddot_si(xi))/drw)|xi=0 = 2/hi^2*(hi^2/2) = 1 for i=0 and w=2, 0 otherwise
                if (i == 0)
                    m_grad_r_ddot_skxik_xik0(k, 1) += 1;
            }

            // Compute gradients for virtual control points
            // --------------------------------------------
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES) {
                // Get timing
                const double h0 = m_knotSitesRemapped(1) - m_knotSitesRemapped(0);
                const double g0 = 1.0 / h0;
                const double hnm1 = m_knotSitesRemapped(knotCount - 1) - m_knotSitesRemapped(knotCount - 2);
                const double gnm1 = 1.0 / hnm1;

                // Term (I), (II)*(III), (IV) and (V)*(VI):
                // ----------------------------------------
                // Term (I):
                /* ---------
                 * (d(dot_si(xi))/dlambda)        = 3/hi*(dai/dlambda)*xi^2 + 2/hi*(dbi/dlambda)*xi^1 + 1/hi*(dci/dlambda)
                 *
                 * (d(dot_s0(xi))/dlambda)|xi=0   = 1/h0*(dc0/dlambda)
                 * (d(dot_snm1(xi))/dlambda)|xi=1 = 3/hnm1*(danm1/dlambda) + 2/hnm1*(dbnm1/dlambda) + 1/hnm1*(dcnm1/dlambda)
                 */

                // dc0/dlambdau = -1 for u=i (NOT POSSIBLE), 1 for u=i+1, 0 otherwise (WARNING: i=0...n-1 and u=1...n-1)
                // --> (d(dot_s0(xi))/dlambdau)|xi=0 = 1/h0*(1) = 1/h0 = g0 for u=i+1, 0 otherwise
                m_grad_lambda_dot_s0xi0_xi00(0, 0) += g0;

                // (danm1/dlambda) = 0
                // (dbnm1/dlambda) = 0
                // dcnm1/dlambdau = -1 for u=i, 1 for u=i+1 (NOT POSSIBLE), 0 otherwise (WARNING: i=0...n-1 and u=1...n-1)
                // --> (d(dot_snm1(xi))/dlambdau)|xi=1 = 3/hnm1*(0) + 2/hnm1*(0) + 1/hnm1*(-1) = -1/hnm1 = -gnm1 for u=i, 0 otherwise
                m_grad_lambda_dot_snm1xinm1_xinm11(0, interiorKnotCount - 1) += -gnm1;

                // Term (II)*(III) and (V)*(VI):
                /* -----------------------------
                 * (d(dot_si(xi))/deta)        = 3/hi*(dai/deta)*xi^2 + 2/hi*(dbi/deta)*xi^1 + 1/hi*(dci/deta)
                 *
                 * (d(dot_s0(xi))/deta)|xi=0   = 1/h0*(dc0/deta)
                 * (d(dot_snm1(xi))/deta)|xi=1 = 3/hnm1*(danm1/deta) + 2/hnm1*(dbnm1/deta) + 1/hnm1*(dcnm1/deta)
                 */

                // dc0/detav = -h0^2/3 for v=i (NOT POSSIBLE), -h0^2/6 for v=i+1, 0 otherwise (WARNING: i=0...n-1 and v=1...n-1)
                // --> (d(dot_s0(xi))/detav)|xi=0 = 1/h0*(-h0^2/6) = -h0/6 for v=i+1, 0 otherwise
                // --> pick corresponding row from (deta/dlambda) and (deta/dr) to avoid unneccessary computations (only one element of dc0/detav is non-zero)
                m_grad_lambda_dot_s0xi0_xi00 += -(h0 / 6.0) * m_deta_dlambda.block(0, 0, 1, interiorKnotCount);
                m_grad_r_dot_s0xi0_xi00 += -(h0 / 6.0) * m_deta_dr.block<1, 4>(0, 0);

                // danm1/detav = -hnm1^2/6 for v=i, hnm1^2/6 for v=i+1 (NOT POSSIBLE), 0 otherwise (WARNING: i=0...n-1 and v=1...n-1)
                // dbnm1/detav = hnm1^2/2 for v=i, 0 otherwise (WARNING: i=0...n-1 and v=1...n-1)
                // dcnm1/detav = -hnm1^2/3 for v=i, -hnm1^2/6 for v=i+1 (NOT POSSIBLE), 0 otherwise (WARNING: i=0...n-1 and v=1...n-1)
                // --> (d(dot_snm1(xi))/detav)|xi=1 = 3/hnm1*(-hnm1^2/6) + 2/hnm1*(hnm1^2/2) + 1/hnm1*(-hnm1^2/3) = hnm1/6 for v=i, 0 otherwise
                // --> pick corresponding row from (deta/dlambda) and (deta/dr) to avoid unneccessary computations
                m_grad_lambda_dot_snm1xinm1_xinm11 += (hnm1 / 6.0) * m_deta_dlambda.block(interiorKnotCount - 1, 0, 1, interiorKnotCount);
                m_grad_r_dot_snm1xinm1_xinm11 += (hnm1 / 6.0) * m_deta_dr.block<1, 4>(interiorKnotCount - 1, 0);

                // Term (IV):
                /* ----------
                 * (d(dot_si(xi))/dr)         = 3/hi*(dai/dr)*xi^2 + 2/hi*(dbi/dr)*xi^1 + 1/hi*(dci/dr)
                 *
                 * (d(dot_s0(xi))/dr)|xi=0    = 1/h0*(dc0/dr)
                 * (d(ddot_snm1(xi))/dr)|xi=1 = 3/hnm1*(danm1/dr) + 2/hnm1*(dbnm1/dr) + 1/hnm1*(dcnm1/dr)
                 */

                // dc0/drw = -1 for i=0 and w=1, -h0^2/3 for i=0 and w=2, 0 otherwise (WARNING: i=0...n-1 and w=1...4)
                // --> (d(dot_s0(xi))/drw)|xi=0 = 1/h0*(-1) = -1/h0 = -g0 for i=0 and w=1
                //                              = 1/h0*(-h0^2/3) =-h0/3   for i=0 and w=2
                m_grad_r_dot_s0xi0_xi00(0, 0) += -g0;
                m_grad_r_dot_s0xi0_xi00(0, 1) += -h0 / 3.0;

                // danm1/drw = hnm1^2/6 for i=n-1 and w = 4, 0 otherwise (WARNING: i=0...n-1 and w=1...n-1)
                // (dbnm1/dr) = 0
                // dcnm1/drw = 1 for i=n-1 and w=3, -hnm1^2/6 for i=n-1 and w=4, 0 otherwise (WARNING: i=0...n-1 and w=1...4)
                // --> (d(ddot_snm1(xi))/drw)|xi=1 = 3/hnm1*(0) + 2/hnm1*(0) + 1/hnm1*(1) = 1/hnm1 = gnm1         for w=3
                //                                 = 3/hnm1*(hnm1^2/6) + 2/hnm1*(0) + 1/hnm1*(-hnm1^2/6) = hnm1/3 for w=4
                m_grad_r_dot_snm1xinm1_xinm11(0, 2) += gnm1;
                m_grad_r_dot_snm1xinm1_xinm11(0, 3) += hnm1 / 3.0;
            }

            // No error occured -> success
            return true;
        }

        // Assemble linear system of equations for collocation (see base class for details)
        virtual bool substep_assembleCollocationLSE(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Initialize helpers
            const size_t knotCount = m_knotSitesRemapped.rows();
            if (knotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                assert(false);
                return false;
            }
            const size_t interiorKnotCount = knotCount - 2; // Count of interior knots (may include virtual control points if present)
            size_t collocationSiteCount = knotCount - 2; // Do not collocate at boundaries
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES)
                collocationSiteCount -= 2; // Do not collocate at virtual control points

            // Assemble:
            // ---------
            // Allocate memory
            m_A_coll.resize(interiorKnotCount, interiorKnotCount);
            m_B_coll.resize(interiorKnotCount);
            m_r = Eigen::VectorXd::Zero(4);
            m_r(0) = m_boundaryConditions(0); // y_0
            m_r(1) = m_boundaryConditions(2); // ddoty_0
            m_r(2) = m_boundaryConditions(3); // y_n
            m_r(3) = m_boundaryConditions(5); // ddoty_n

            // Collocate at {t_k}:
            // -> (alpha * grad_lambda_ddot_skxik_xik0 + beta * grad_lambda_dot_skxik_xik0 + gamma * grad_lambda_skxik_xik0) * lambda = tau - (alpha * grad_r_ddot_skxik_xik0 + beta * grad_r_dot_skxik_xik0 + gamma * grad_r_skxik_xik0) * r
            size_t virtualControlPointShift = 0; // Shift of collocation point indices due to virtual control points
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES)
                virtualControlPointShift = 1; // First collocation site is third knot site
            for (size_t k = 0; k < collocationSiteCount; k++) {
                m_A_coll.block(k + virtualControlPointShift, 0, 1, interiorKnotCount) = m_ODEAlpha(k) * m_grad_lambda_ddot_skxik_xik0.block(k, 0, 1, interiorKnotCount) + m_ODEBeta(k) * m_grad_lambda_dot_skxik_xik0.block(k, 0, 1, interiorKnotCount) + m_ODEGamma(k) * m_grad_lambda_skxik_xik0.block(k, 0, 1, interiorKnotCount);
                m_B_coll(k + virtualControlPointShift) = m_ODETau(k) - (m_ODEAlpha(k) * m_grad_r_ddot_skxik_xik0.block<1, 4>(k, 0) + m_ODEBeta(k) * m_grad_r_dot_skxik_xik0.block<1, 4>(k, 0) + m_ODEGamma(k) * m_grad_r_skxik_xik0.block<1, 4>(k, 0)) * m_r;
            }

            // Virtual control points (optional):
            if (m_method == CollocationMethod::COLLOCATION_WITH_FIRST_DERIVATIVES) {
                // -> grad_lambda_dot_s0xi0_xi00 * lambda = doty_0 - grad_r_dot_s0xi0_xi00 * r
                m_A_coll.block(0, 0, 1, interiorKnotCount) = m_grad_lambda_dot_s0xi0_xi00;
                m_B_coll.block<1, 1>(0, 0) = m_boundaryConditions.block<1, 1>(1, 0) - m_grad_r_dot_s0xi0_xi00 * m_r;

                // -> grad_lambda_dot_snm1xinm1_xinm11 * lambda = doty_n - grad_r_dot_snm1xinm1_xinm11 * r
                m_A_coll.block(interiorKnotCount - 1, 0, 1, interiorKnotCount) = m_grad_lambda_dot_snm1xinm1_xinm11;
                m_B_coll.block<1, 1>(interiorKnotCount - 1, 0) = m_boundaryConditions.block<1, 1>(4, 0) - m_grad_r_dot_snm1xinm1_xinm11 * m_r;
            }

            // No error occured -> success
            return true;
        }

        // Compute spline segments and pack them into a \ref broccoli::curve::Trajectory(see base class for details)
        virtual bool substep_convertToTrajectory(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Initialize helpers
            const size_t knotCount = m_knotSitesRemapped.rows();
            if (knotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                assert(false);
                return false;
            }
            const size_t segmentCount = knotCount - 1;

            // Set duration of trajectory
            const double duration = m_knotSitesRemapped(knotCount - 1) - m_knotSitesRemapped(0); // Duration is tn - t0
            const double durationSquared = duration * duration;
            m_trajectory.m_duration = duration;

            // Setup segments
            std::vector<double> m_interpolationParameters; // Buffer for interpolation parameters
            std::vector<double> m_interpolationProportions; // Buffer for interpolation proportions
            m_interpolationParameters.resize(2 * knotCount, 0);
            m_interpolationProportions.resize(segmentCount, 0);

            // Add starting knot (boundary condition)
            m_interpolationParameters[0] = m_boundaryConditions(0);
            m_interpolationParameters[knotCount] = m_boundaryConditions(2) * durationSquared;

            // Add interior knots
            for (size_t i = 1; i < knotCount - 1; i++) {
                m_interpolationParameters[i] = m_lambda(i - 1);
                m_interpolationParameters[knotCount + i] = m_eta(i - 1) * durationSquared;
            }

            // Add ending knot (boundary condition)
            m_interpolationParameters[knotCount - 1] = m_boundaryConditions(3);
            m_interpolationParameters[2 * knotCount - 1] = m_boundaryConditions(5) * durationSquared;

            // Compute segment proportions
            for (size_t i = 0; i < segmentCount; i++)
                m_interpolationProportions[i] = (m_knotSitesRemapped(i + 1) - m_knotSitesRemapped(i)) / m_trajectory.m_duration;

            if (m_trajectory.m_splines[0].interpolate(m_interpolationParameters, &m_interpolationProportions, broccoli::curve::SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_CUBIC_SECOND_DERIVATIVES) == false) {
                if (result != nullptr)
                    *result = Result::ERROR_FINAL_INTERPOLATION_FAILED;
                assert(false);
                return false;
            }

            // No error occured -> success
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace ode
} // namespace broccoli

#endif // HAVE_EIGEN3
