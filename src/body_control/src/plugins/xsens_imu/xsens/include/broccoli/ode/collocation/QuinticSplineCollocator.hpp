/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/math.hpp"
#include "../../curve/trajectories/PolynomialTrajectory.hpp"
#include "SplineCollocatorBase.hpp"
#include <Eigen/StdVector>
#include <assert.h>
#include <vector>

namespace broccoli {
namespace ode {
    //! Collocation manager for **quintic** spline collocation of second-order linear time-variant ordinary differential equations (ODEs)
    /*!
     * \ingroup broccoli_ode_collocation
     *
     * \copydetails broccoli::ode::SplineCollocatorBase
     */
    class QuinticSplineCollocator : public SplineCollocatorBase {
    public:
        // Constructor
        /*! \copydoc broccoli::ode::SplineCollocatorBase::SplineCollocatorBase() */
        QuinticSplineCollocator()
            : m_kappa(sqrt(64.0 / 3.0))
        {
        }

        // Destructor
        /*! \copydoc broccoli::ode::SplineCollocatorBase::~SplineCollocatorBase() */
        virtual ~QuinticSplineCollocator()
        {
        }

        // Input members
        // -------------
        const double m_kappa; //!< Parameter to improve numeric stability (for solving block-tridiagonal system)

        // "Internal" members (buffers)
        // ------------------
        // Block-tridiagonal system of equations
        std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> m_lowerDiagonalOfA; //!< Buffer for lower diagonal elements of block-tridiagonal matrix \f$ A \f$
        std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> m_diagonalOfA; //!< Buffer for diagonal elements of block-tridiagonal matrix \f$ A \f$
        std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> m_upperDiagonalOfA; //!< Buffer for upper diagonal elements of block-tridiagonal matrix \f$ A \f$
        std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_dB_dlambda; //!< Buffer for right hand side \f$ \left( \partial B / \partial \lambda\right) \f$
        std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_dB_dr; //!< Buffer for right hand side \f$ \left( \partial B / \partial r\right) \f$
        std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_dX_dlambda; //!< Buffer for solution \f$ \left( \partial X / \partial \lambda\right) \f$ of block-tridiagonal system
        std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_dX_dr; //!< Buffer for solution \f$ \left( \partial X / \partial r\right) \f$ of block-tridiagonal system

        // Output members
        // --------------
        broccoli::curve::PolynomialTrajectory<5, 1> m_trajectory; //!< **Output:** Trajectory encapsulating the spline \f$ y(t) \f$ which approximates the solution of \f$ F(t) \f$

        // Processing
        // ----------
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
            const size_t interiorKnotCount = knotCount - 2;
            const double kappasquared = m_kappa * m_kappa; // kappa^2

            // Initialize block-tridiagonal matrix
            m_lowerDiagonalOfA.resize(interiorKnotCount); // First element is "outside" the matrix A (to be ignored)
            m_diagonalOfA.resize(interiorKnotCount);
            m_upperDiagonalOfA.resize(interiorKnotCount); // Last element is "outside" the matrix A (to be ignored)

            // Initialize right hand side (dB/dlambda) and (dB/dr)
            m_dB_dlambda.resize(interiorKnotCount);
            m_dB_dr.resize(interiorKnotCount);

            // Get timing of first segment
            const double h0 = m_knotSitesRemapped(1) - m_knotSitesRemapped(0); // Duration of first segment
            if (h0 <= 0) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                assert(false);
                return false;
            }
            const double g0 = 1.0 / h0; // g0
            const double g0s = g0 * g0; // g0^2 (squared)
            const double g0c = g0s * g0; // g0^3 (cubic)
            const double g0q = g0c * g0; // g0^3 (quartic)

            // Get timing of last segment
            const double hnm1 = m_knotSitesRemapped(knotCount - 1) - m_knotSitesRemapped(knotCount - 2); // Duration of last segment
            if (hnm1 <= 0) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                assert(false);
                return false;
            }
            const double gnm1 = 1.0 / hnm1; // gnm1
            const double gnm1s = gnm1 * gnm1; // gnm1^2 (squared)
            const double gnm1c = gnm1s * gnm1; // gnm1^3 (cubic)

            // Iterate through rows of system
            double gi = g0; // gi
            double gis = g0s; // gi^2 (squared)
            double gic = g0c; // gi^3 (cubic)
            double giq = gic * gi; // gi^4 (quartic)
            for (size_t j = 0; j < interiorKnotCount; j++) {
                // Index shift is necessary, since we want to skip i=0 and comply with the general notation/meaning of index "i"
                const size_t i = j + 1;

                // Update current values
                const double gim1 = gi; // gim1
                const double gim1s = gis; // gim1^2 (squared)
                const double gim1c = gic; // gim1^3 (cubic)
                const double gim1q = giq; // gim1^4 (quartic)
                const double hi = m_knotSitesRemapped(i + 1) - m_knotSitesRemapped(i); // Duration of current segment
                if (hi <= 0) {
                    if (result != nullptr)
                        *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                    assert(false);
                    return false;
                }
                gi = 1.0 / hi; // gi
                gis = gi * gi; // gi^2 (squared)
                gic = gis * gi; // gi^3 (cubic)
                giq = gic * gi; // gi^4 (quartic)

                // Compute upper diagonal elements of A
                if (j == interiorKnotCount - 1)
                    m_upperDiagonalOfA[j] = Eigen::Matrix2d::Zero(); // Last element is "outside" the matrix A (to be ignored)
                else {
                    const double gip1 = 1.0 / (m_knotSitesRemapped(i + 2) - m_knotSitesRemapped(i + 1));

                    m_upperDiagonalOfA[j](0, 0) = 56.0 * gic;
                    m_upperDiagonalOfA[j](0, 1) = 8.0 * m_kappa * gis * gip1;
                    m_upperDiagonalOfA[j](1, 0) = -8.0 * m_kappa * gic;
                    m_upperDiagonalOfA[j](1, 1) = -kappasquared * gis * gip1;
                }

                // Compute diagonal elements of A
                m_diagonalOfA[j](0, 0) = 64.0 * (gim1c + gic);
                m_diagonalOfA[j](0, 1) = 12.0 * m_kappa * (gim1s - gis) * gi;
                m_diagonalOfA[j](1, 0) = m_diagonalOfA[j](0, 1); // Use symmetry
                m_diagonalOfA[j](1, 1) = 3.0 * kappasquared * (gim1 + gi) * gis;

                // Compute lower diagonal elements of A
                if (j == 0)
                    m_lowerDiagonalOfA[j] = Eigen::Matrix2d::Zero(); // First element is "outside" the matrix A (to be ignored)
                else
                    m_lowerDiagonalOfA[j] = m_upperDiagonalOfA[j - 1].transpose(); // Use symmetry

                // Compute elements in (dB/dlambda)
                m_dB_dlambda[j] = Eigen::MatrixXd::Zero(2, interiorKnotCount);
                if (i > 1) {
                    m_dB_dlambda[j](0, i - 2) = 120.0 * gim1q;
                    m_dB_dlambda[j](1, i - 2) = 20.0 * m_kappa * gim1c * gi;
                }
                m_dB_dlambda[j](0, i - 1) = 120.0 * (giq - gim1q);
                m_dB_dlambda[j](1, i - 1) = -20.0 * m_kappa * gi * (gic + gim1c);
                if (i < interiorKnotCount) {
                    m_dB_dlambda[j](0, i) = -120.0 * giq;
                    m_dB_dlambda[j](1, i) = 20.0 * m_kappa * giq;
                }

                // Compute elements in (dB/dr)
                m_dB_dr[j] = Eigen::MatrixXd::Zero(2, 6);
                if (i == 1) {
                    m_dB_dr[j](0, 0) = 120.0 * g0q;
                    m_dB_dr[j](1, 0) = 20.0 * m_kappa * g0c * gi;
                    m_dB_dr[j](0, 1) = 56.0 * g0c;
                    m_dB_dr[j](1, 1) = 8.0 * m_kappa * g0s * gi;
                    m_dB_dr[j](0, 2) = 8.0 * g0s;
                    m_dB_dr[j](1, 2) = m_kappa * g0 * gi;
                }
                if (i == interiorKnotCount) {
                    m_dB_dr[j](0, 3) = -120.0 * giq;
                    m_dB_dr[j](1, 3) = 20.0 * m_kappa * giq;
                    m_dB_dr[j](0, 4) = 56.0 * gnm1c;
                    m_dB_dr[j](1, 4) = -8.0 * m_kappa * gnm1c;
                    m_dB_dr[j](0, 5) = -8.0 * gnm1s;
                    m_dB_dr[j](1, 5) = m_kappa * gnm1s;
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
            const size_t interiorKnotCount = knotCount - 2;
            broccoli::core::math::Result LSEsolveResult;

            // Check count of knots to decide which method is the fastest
            if (knotCount <= 50 /* <-- seems to be the break-even point */) {
                // ...dimension of lambda is in the same domain as the dimension of r
                // --> solving for (dX/dr) will take a similar runtime when compared to solving for (dX/dlambda)
                //   --> stack (dX/dlambda) and (dX/dr) to compute them at the same time

                // Create stacks [(dB/dlambda) (dB/dr)] and [(dX/dlambda) (dX/dr)]
                std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> dB_dlambda_dr; // [(dB/dlambda) (dB/dr)]
                std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> dX_dlambda_dr; // [(dX/dlambda) (dX/dr)]
                dB_dlambda_dr.resize(m_dB_dlambda.size()); // Allocate memory (block count)
                dX_dlambda_dr.resize(m_dX_dlambda.size()); // Allocate memory (block count)
                for (size_t i = 0; i < m_dB_dlambda.size(); i++) {
                    dB_dlambda_dr[i].resize(m_dB_dlambda[i].rows(), m_dB_dlambda[i].cols() + m_dB_dr[i].cols());
                    dB_dlambda_dr[i] << m_dB_dlambda[i], m_dB_dr[i]; // Horizontal stacking
                }

                // Solve for [(dX/dlambda) (dX/dr)]
                if (broccoli::core::math::solveBlockTriDiagonalSystemOfEquations<Eigen::Matrix2d, Eigen::MatrixXd>(m_lowerDiagonalOfA, m_diagonalOfA, m_upperDiagonalOfA, dB_dlambda_dr, dX_dlambda_dr, &LSEsolveResult) == false) {
                    if (result != nullptr)
                        *result = Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH;
                    assert(false);
                    return false;
                }
                if (LSEsolveResult == broccoli::core::math::Result::WARNING_CLOSE_TO_SINGULAR)
                    if (result != nullptr)
                        *result = Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR;

                // Extract (dX/dlambda) and (dX/dr) from [(dX/dlambda) (dX/dr)]
                m_dX_dlambda.resize(dX_dlambda_dr.size());
                for (size_t i = 0; i < m_dX_dlambda.size(); i++)
                    m_dX_dlambda[i] = dX_dlambda_dr[i].block(0, 0, 2, m_dB_dlambda[i].cols());
                m_dX_dr.resize(dX_dlambda_dr.size());
                for (size_t i = 0; i < m_dX_dr.size(); i++)
                    m_dX_dr[i] = dX_dlambda_dr[i].block(0, m_dB_dlambda[i].cols(), 2, m_dB_dr[i].cols());
            } else {
                // ...dimension of lambda is big in comparison to r
                // --> solving for (dX/dr) is much faster than solving for (dX/dlambda)
                //   --> handle (dX/dlambda) and (dX/dr) separately to avoid wasting time with stacking

                // Solve for (dX/dlambda)
                if (broccoli::core::math::solveBlockTriDiagonalSystemOfEquations<Eigen::Matrix2d, Eigen::MatrixXd>(m_lowerDiagonalOfA, m_diagonalOfA, m_upperDiagonalOfA, m_dB_dlambda, m_dX_dlambda, &LSEsolveResult) == false) {
                    if (result != nullptr)
                        *result = Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH;
                    assert(false);
                    return false;
                }
                if (LSEsolveResult == broccoli::core::math::Result::WARNING_CLOSE_TO_SINGULAR)
                    if (result != nullptr)
                        *result = Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR;

                // Solve for (dX/dr)
                if (broccoli::core::math::solveBlockTriDiagonalSystemOfEquations<Eigen::Matrix2d, Eigen::MatrixXd>(m_lowerDiagonalOfA, m_diagonalOfA, m_upperDiagonalOfA, m_dB_dr, m_dX_dr, &LSEsolveResult) == false) {
                    if (result != nullptr)
                        *result = Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH;
                    assert(false);
                    return false;
                }
                if (LSEsolveResult == broccoli::core::math::Result::WARNING_CLOSE_TO_SINGULAR)
                    if (result != nullptr)
                        *result = Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR;
            }

            // Compute (deta/dlambda) = S^-1 * (dX/dlambda) and (deta/dr) = S^-1 * (dX/dr)
            // (since S is diagonal we can perform this operation efficiently for each entry)
            m_deta_dlambda = Eigen::MatrixXd::Zero(2 * interiorKnotCount, interiorKnotCount);
            m_deta_dr = Eigen::MatrixXd::Zero(2 * interiorKnotCount, 6);
            Eigen::Matrix2d inv_Si = Eigen::Matrix2d::Zero(); // Si^-1 = diag(-1, kappa*gi)
            inv_Si(0, 0) = -1;
            for (size_t k = 0; k < interiorKnotCount; k++) {
                inv_Si(1, 1) = m_kappa / (m_knotSitesRemapped(k + 2) - m_knotSitesRemapped(k + 1));
                m_deta_dlambda.block(2 * k, 0, 2, interiorKnotCount) = inv_Si * m_dX_dlambda[k];
                m_deta_dr.block<2, 6>(2 * k, 0) = inv_Si * m_dX_dr[k];
            }

            // No error occured -> success
            return true;
        }

        // Compute spline gradients (see base class for details)
        /*! \copydoc SplineCollocatorBase::substep_computeSplineGradients() */
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
            const size_t collocationSiteCount = knotCount - 2; // Do not collocate at boundaries

            /* Spline gradients:
             * -----------------
             *    si(xi)      =         ai*xi^5 +         bi*xi^4 +        ci*xi^3 +        di*xi^2 +      ei*xi^1 + fi
             *   dsi(xi)/dt   =    5/hi*ai*xi^4 +    4/hi*bi*xi^3 +   3/hi*ci*xi^2 +   2/hi*di*xi^1 + 1/hi*ei
             * d^2si(xi)/dt^2 = 20/hi^2*ai*xi^3 + 12/hi^2*bi*xi^2 + 6/hi^2*ci*xi^1 + 2/hi^2*di
             *
             * lambda = [y1, ..., ynm1] = [lambda1, ..., lambdanm1]
             * eta    = [dot_y1, ddot_y1, ..., dot_ynm1, ddot_ynm1] = [eta1, eta2, ..., eta2nm1]
             * r      = [y0, dot_y0, ddot_y0, yn, dot_yn, ddot_yn] = [r1, ..., r6]
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
            m_grad_lambda_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, collocationSiteCount);
            m_grad_r_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, 6);
            m_grad_lambda_dot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, collocationSiteCount);
            m_grad_r_dot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, 6);
            m_grad_lambda_ddot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, collocationSiteCount);
            m_grad_r_ddot_skxik_xik0 = Eigen::MatrixXd::Zero(collocationSiteCount, 6);

            // Compute gradients for collocation sites
            // ---------------------------------------
            for (size_t k = 0; k < collocationSiteCount; k++) {
                // Index shift (first collocation point is second control point)
                size_t i = k + 1;

                // Term (I), (II)*(III), (IV) and (V)*(VI):
                // ----------------------------------------
                // Term (I):
                /* ---------
                 * (d(si(xi))/dlambda)      =         (dai/dlambda)*xi^5 +         (dbi/dlambda)*xi^4 +        (dci/dlambda)*xi^3 +        (ddi/dlambda)*xi^2 +      (dei/dlambda)*xi^1 * (dfi/dlambda)
                 * (d(dot_si(xi))/dlambda)  =    5/hi*(dai/dlambda)*xi^4 +    4/hi*(dbi/dlambda)*xi^3 +   3/hi*(dci/dlambda)*xi^2 +   2/hi*(ddi/dlambda)*xi^1 + 1/hi*(dei/dlambda)
                 * (d(ddot_si(xi))/dlambda) = 20/hi^2*(dai/dlambda)*xi^3 + 12/hi^2*(dbi/dlambda)*xi^2 + 6/hi^2*(dci/dlambda)*xi^1 + 2/hi^2*(ddi/dlambda)
                 *
                 * Evaluation at xi = 0:
                 * (d(si(xi))/dlambda)|xi=0      = (dfi/dlambda)
                 * (d(dot_si(xi))/dlambda)|xi=0  = 1/hi*(dei/dlambda)
                 * (d(ddot_si(xi))/dlambda)|xi=0 = 2/hi^2*(ddi/dlambda)
                 */

                // dfi/dlambdau = 1 for u=i, 0 otherwise (WARNING: i=0...n-1 and u=1...n-1)
                // --> (d(si(xi))/dlambdau)|xi=0 = 1 for u=i, 0 otherwise
                m_grad_lambda_skxik_xik0(k, i - 1) += 1;

                // (d(dot_si(xi))/dlambda)|xi=0 = 0 since (dei/dlambda) = 0

                // (d(ddot_si(xi))/dlambda)|xi=0 = 0 since (ddi/dlambda) = 0

                // Term (II)*(III) and (V)*(VI):
                /* -----------------------------
                 * (d(si(xi))/deta)      =         (dai/deta)*xi^5 +         (dbi/deta)*xi^4 +        (dci/deta)*xi^3 +        (ddi/deta)*xi^2 +      (dei/deta)*xi^1 * (dfi/deta)
                 * (d(dot_si(xi))/deta)  =    5/hi*(dai/deta)*xi^4 +    4/hi*(dbi/deta)*xi^4 +   3/hi*(dci/deta)*xi^2 +   2/hi*(ddi/deta)*xi^1 + 1/hi*(dei/deta)
                 * (d(ddot_si(xi))/deta) = 20/hi^2*(dai/deta)*xi^3 + 12/hi^2*(dbi/deta)*xi^2 + 6/hi^2*(dci/deta)*xi^1 + 2/hi^2*(ddi/deta)
                 *
                 * Evaluation at xi = 0:
                 * (d(si(xi))/deta)|xi=0      = (dfi/deta)
                 * (d(dot_si(xi))/deta)|xi=0  = 1/hi*(dei/deta)
                 * (d(ddot_si(xi))/deta)|xi=0 = 2/hi^2*(ddi/deta)
                 */

                // (d(si(xi))/deta)|xi=0 = 0 since (dfi/deta) = 0

                // dei/detav = hi for v=2*i-1, 0 otherwise (WARNING: i=0...n-1 and v=1...2(n-1))
                // --> (d(dot_si(xi))/detav)|xi=0 = 1/hi*(hi) = 1 for v=2*i-1, 0 otherwise
                // --> pick corresponding row from (deta/dlambda) and (deta/dr) to avoid unneccessary computations (only one element of dei/detav is non-zero)
                m_grad_lambda_dot_skxik_xik0.block(k, 0, 1, collocationSiteCount) += m_deta_dlambda.block(2 * i - 2, 0, 1, collocationSiteCount);
                m_grad_r_dot_skxik_xik0.block<1, 6>(k, 0) += m_deta_dr.block<1, 6>(2 * i - 2, 0);

                // ddi/detav = 1/2*hi^2 for v=2*i, 0 otherwise (WARNING: i=0...n-1 and v=1...2(n-1))
                // --> (d(ddot_si(xi))/detav)|xi=0 = 2/hi^2*(1/2*hi^2) = 1 for v=2*i, 0 otherwise
                // --> pick corresponding row from (deta/dlambda) and (deta/dr) to avoid unneccessary computations (only one element of ddi/detav is non-zero)
                m_grad_lambda_ddot_skxik_xik0.block(k, 0, 1, collocationSiteCount) += m_deta_dlambda.block(2 * i - 1, 0, 1, collocationSiteCount);
                m_grad_r_ddot_skxik_xik0.block<1, 6>(k, 0) += m_deta_dr.block<1, 6>(2 * i - 1, 0);

                // Term (IV):
                /* ----------
                 * (d(si(xi))/dr)      =         (dai/dr)*xi^5 +         (dbi/dr)*xi^4 +        (dci/dr)*xi^3 +        (ddi/dr)*xi^2 +      (dei/dr)*xi^1 * (dfi/dr)
                 * (d(dot_si(xi))/dr)  =    5/hi*(dai/dr)*xi^4 +    4/hi*(dbi/dr)*xi^3 +   3/hi*(dci/dr)*xi^2 +   2/hi*(ddi/dr)*xi^1 + 1/hi*(dei/dr)
                 * (d(ddot_si(xi))/dr) = 20/hi^2*(dai/dr)*xi^3 + 12/hi^2*(dbi/dr)*xi^2 + 6/hi^2*(dci/dr)*xi^1 + 2/hi^2*(ddi/dr)
                 *
                 * Evaluation at xi = 0:
                 * (d(si(xi))/dr)|xi=0      = (dfi/dr)
                 * (d(dot_si(xi))/dr)|xi=0  = 1/hi*(dei/dr)
                 * (d(ddot_si(xi))/dr)|xi=0 = 2/hi^2*(ddi/dr)
                 */
                if (i == 0) {
                    // dfi/drw = 1 for i=0 and w=1, 0 otherwise (WARNING: i=0...n-1 and w=1...6)
                    // --> (d(si(xi))/drw)|xi=0 = 1 for i=0 and w=1, 0 otherwise
                    m_grad_r_skxik_xik0(k, 0) += 1;

                    // dei/drw = hi for i=0 and w=2, 0 otherwise (WARNING: i=0...n-1 and w=1...6)
                    // --> (d(dot_si(xi))/drw)|xi=0 = 1/hi*(hi) = 1 for i=0 and w=2, 0 otherwise
                    m_grad_r_dot_skxik_xik0(k, 1) += 1;

                    // ddi/drw = 1/2*hi^2 for i=0 and w=3, 0 otherwise (WARNING: i=0...n-1 and w=1...6)
                    // --> (d(ddot_si(xi))/drw)|xi=0 = 2/hi^2*(1/2*hi^2) = 1 for i=0 and w=3, 0 otherwise
                    m_grad_r_ddot_skxik_xik0(k, 2) += 1;
                }
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
            const size_t interiorKnotCount = knotCount - 2;
            const size_t collocationSiteCount = knotCount - 2; // Do not collocate at boundaries

            // Assemble:
            // Allocate memory
            m_A_coll.resize(interiorKnotCount, interiorKnotCount);
            m_B_coll.resize(interiorKnotCount);
            m_r = m_boundaryConditions;

            // Collocate at {t_k}:
            // -> (alpha * grad_lambda_ddot_skxik_xik0 + beta * grad_lambda_dot_skxik_xik0 + gamma * grad_lambda_skxik_xik0) * lambda = tau - (alpha * grad_r_ddot_skxik_xik0 + beta * grad_r_dot_skxik_xik0 + gamma * grad_r_skxik_xik0) * r
            for (size_t k = 0; k < collocationSiteCount; k++) {
                m_A_coll.block(k, 0, 1, interiorKnotCount) = m_ODEAlpha(k) * m_grad_lambda_ddot_skxik_xik0.block(k, 0, 1, interiorKnotCount) + m_ODEBeta(k) * m_grad_lambda_dot_skxik_xik0.block(k, 0, 1, interiorKnotCount) + m_ODEGamma(k) * m_grad_lambda_skxik_xik0.block(k, 0, 1, interiorKnotCount);
                m_B_coll(k) = m_ODETau(k) - (m_ODEAlpha(k) * m_grad_r_ddot_skxik_xik0.block<1, 6>(k, 0) + m_ODEBeta(k) * m_grad_r_dot_skxik_xik0.block<1, 6>(k, 0) + m_ODEGamma(k) * m_grad_r_skxik_xik0.block<1, 6>(k, 0)) * m_r;
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
            m_interpolationParameters.resize(3 * knotCount, 0);
            m_interpolationProportions.resize(segmentCount, 0);

            // Add starting knot (boundary condition)
            m_interpolationParameters[0] = m_boundaryConditions(0);
            m_interpolationParameters[knotCount] = m_boundaryConditions(1) * duration;
            m_interpolationParameters[2 * knotCount] = m_boundaryConditions(2) * durationSquared;

            // Add interior knots
            for (size_t i = 1; i < knotCount - 1; i++) {
                m_interpolationParameters[i] = m_lambda(i - 1);
                m_interpolationParameters[knotCount + i] = m_eta(2 * (i - 1)) * duration;
                m_interpolationParameters[2 * knotCount + i] = m_eta(2 * (i - 1) + 1) * durationSquared;
            }

            // Add ending knot (boundary condition)
            m_interpolationParameters[knotCount - 1] = m_boundaryConditions(3);
            m_interpolationParameters[2 * knotCount - 1] = m_boundaryConditions(4) * duration;
            m_interpolationParameters[3 * knotCount - 1] = m_boundaryConditions(5) * durationSquared;

            // Compute segment proportions
            for (size_t i = 0; i < segmentCount; i++)
                m_interpolationProportions[i] = (m_knotSitesRemapped(i + 1) - m_knotSitesRemapped(i)) / m_trajectory.m_duration;

            if (m_trajectory.m_splines[0].interpolate(m_interpolationParameters, &m_interpolationProportions, broccoli::curve::SplineInterpolationMethod::POLYNOMIAL_PIECEWISE_QUINTIC_FIRST_AND_SECOND_DERIVATIVES) == false) {
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
