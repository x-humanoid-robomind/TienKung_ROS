/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include <Eigen/Dense>
#include <assert.h>

namespace broccoli {
namespace ode {
    //! (Virtual) base class for spline collocation managers to approximate second-order linear time-variant ordinary differential equations (ODEs)
    /*!
     * \ingroup broccoli_ode_collocation
     *
     * Given a second-order linear time-variant ordinary differential equation (ODE)
     *
     * \f[ \alpha(t)\,\ddot{F}(t) + \beta(t)\,\dot{F}(t) + \gamma(t)\,F(t) = \tau(t) \f]
     *
     * with boundary conditions
     *
     * \f[ [F_0,\,\dot{F}_0,\,\ddot{F}_0,\,F_n,\,\dot{F}_n,\,\ddot{F}_n]^T \f]
     *
     * this class provides the necessary functionality to construct a spline \f$ y(t) \f$ approximating the solution of this boundary value problem (BVP).
     *
     * The suggested workflow for the parent/caller is
     *   * Set input members of this class to define the collocation options:
     *     * \ref m_knotSites - \copybrief m_knotSites
     *     * \ref m_ODEAlpha - \copybrief m_ODEAlpha
     *     * \ref m_ODEBeta - \copybrief m_ODEBeta
     *     * \ref m_ODEGamma - \copybrief m_ODEGamma
     *     * \ref m_ODETau - \copybrief m_ODETau
     *     * \ref m_boundaryConditions - \copybrief m_boundaryConditions
     *   * Run \ref process() (triggers sub-steps in the right order)
     *   * Use the computed spline (packed into a \ref broccoli::curve::Trajectory) or the output members
     *     * \ref m_lambda - \copybrief m_lambda
     *     * \ref m_eta - \copybrief m_eta
     *
     * Alternatively one may call the substeps manually (e.g. if runtime has to be measured, or results should be reused). For the
     * correct execution order of substeps see \ref process().
     */
    class SplineCollocatorBase {
    public:
        //! Specification of results of processing steps
        enum class Result : uint8_t {
            SUCCESS = 0, //!< Everything was OK
            ERROR_INPUT_NOT_INITIALIZED, //!< At least one input parameter has not been initialized properly
            ERROR_INPUT_TOO_FEW_KNOTS, //!< Too few knot sites are specified (minimum is 3!)
            ERROR_INPUT_DIMENSION_MISMATCH, //!< The dimensions of the input parameters do not match to each other
            ERROR_INPUT_NOT_ASCENDING_KNOTS, //!< The knot sites are either not distinct or not ascending
            ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH, //!< (Block-)tridiagonal linear system of equations has invalid dimensions
            WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR, //!< (Block-)tridiagonal linear system of equations is close to singular
            WARNING_COLLOCATION_LSE_CLOSE_TO_SINGULAR, //!< Linear system of equations for collocation is close to singular
            ERROR_FINAL_INTERPOLATION_FAILED, //!< Interpolation to obtain resulting trajectory (containing spline approximation) failed
            RESULT_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given result
        static std::string resultString(const Result& result)
        {
            // Check result
            switch (result) {
            case Result::SUCCESS:
                return "SUCCESS";
            case Result::ERROR_INPUT_NOT_INITIALIZED:
                return "ERROR_INPUT_NOT_INITIALIZED";
            case Result::ERROR_INPUT_TOO_FEW_KNOTS:
                return "ERROR_INPUT_TOO_FEW_KNOTS";
            case Result::ERROR_INPUT_DIMENSION_MISMATCH:
                return "ERROR_INPUT_DIMENSION_MISMATCH";
            case Result::ERROR_INPUT_NOT_ASCENDING_KNOTS:
                return "ERROR_INPUT_NOT_ASCENDING_KNOTS";
            case Result::ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH:
                return "ERROR_BLOCKTRIDIAGONAL_DIMENSION_MISMATCH";
            case Result::WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR:
                return "WARNING_BLOCKTRIDIAGONAL_LSE_CLOSE_TO_SINGULAR";
            case Result::WARNING_COLLOCATION_LSE_CLOSE_TO_SINGULAR:
                return "WARNING_COLLOCATION_LSE_CLOSE_TO_SINGULAR";
            case Result::ERROR_FINAL_INTERPOLATION_FAILED:
                return "ERROR_FINAL_INTERPOLATION_FAILED";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Constructor \warning Keeps members uninitialized!
        SplineCollocatorBase()
        {
        }

        //! Destructor
        virtual ~SplineCollocatorBase()
        {
        }

        //! Execute complete algorithm at once (triggers sub-steps in the correct order)
        /*!
         * \param [in] checkSingularity If `true` singularity checks are performed (triggers a warning if a (near-)singularity is detected)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         *
         * \par Triggers following substeps:
         */
        virtual bool process(const bool& checkSingularity, Result* const result = nullptr)
        {
            // Initialize helpers
            Result substepResult = Result::SUCCESS;

            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Run through steps
            // -----------------
            //! Step 1: \ref substep_checkInput() - \copybrief substep_checkInput()
            if (substep_checkInput(&substepResult) == false || substepResult != Result::SUCCESS) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }

            //! Step 2: \ref substep_addVirtualControlPoints() - \copybrief substep_addVirtualControlPoints()
            if (substep_addVirtualControlPoints(&substepResult) == false || substepResult != Result::SUCCESS) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }

            //! Step 3: \ref substep_setupBlockTridiagonalSystem() - \copybrief substep_setupBlockTridiagonalSystem()
            if (substep_setupBlockTridiagonalSystem(&substepResult) == false || substepResult != Result::SUCCESS) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }

            //! Step 4: \ref substep_solveBlockTridiagonalSystem() - \copybrief substep_solveBlockTridiagonalSystem()
            if (substep_solveBlockTridiagonalSystem(&substepResult) == false) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }
            if (substepResult != Result::SUCCESS && result != nullptr)
                *result = substepResult;

            //! Step 5: \ref substep_computeSplineGradients() - \copybrief substep_computeSplineGradients()
            if (substep_computeSplineGradients(&substepResult) == false || substepResult != Result::SUCCESS) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }

            //! Step 6: \ref substep_assembleCollocationLSE() - \copybrief substep_assembleCollocationLSE()
            if (substep_assembleCollocationLSE(&substepResult) == false || substepResult != Result::SUCCESS) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }

            //! Step 7: \ref substep_solveCollocationLSE() - \copybrief substep_solveCollocationLSE()
            if (substep_solveCollocationLSE(checkSingularity, &substepResult) == false) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }
            if (substepResult != Result::SUCCESS && result != nullptr)
                *result = substepResult;

            //! Step 8: \ref substep_convertToTrajectory() - \copybrief substep_convertToTrajectory()
            if (substep_convertToTrajectory(&substepResult) == false || substepResult != Result::SUCCESS) {
                if (result != nullptr)
                    *result = substepResult;
                return false;
            }

            // No error occured -> success
            return true;
        }

        // Input members
        // -------------
        // Partitioning
        Eigen::VectorXd m_knotSites; //!< **Input:** (Desired) **distinct** and **ascending** knot sites \f$\lbrace t_i\rbrace\f$ (alias time points) of resulting spline \details First and last element denote the boundaries \f$ t_0\f$ and \f$ t_n\f$. Interior knots represent the collocation sites \f$ t_k \f$. \f[ \left[ t_0,\,\dots,\,t_n\right]^T \in \mathbb{R}^{n+1}\f]

        // Coefficients and right hand side of underlying ODE
        Eigen::VectorXd m_ODEAlpha; //!< **Input:** \f$ \alpha(t) \f$ coefficients of underlying ODE for each collocation site \f$ t_k \f$ \details \f[ \left[ \alpha(t_1),\,\dots,\,\alpha(t_k),\,\dots,\,\alpha(t_{n-1})\right]^T \in \mathbb{R}^{n-1} \f]
        Eigen::VectorXd m_ODEBeta; //!< **Input:** \f$ \beta(t) \f$ coefficients of underlying ODE for each collocation site \f$ t_k \f$ \details \f[ \left[ \beta(t_1),\,\dots,\,\beta(t_k),\,\dots,\,\beta(t_{n-1})\right]^T \in \mathbb{R}^{n-1} \f]
        Eigen::VectorXd m_ODEGamma; //!< **Input:** \f$ \gamma(t) \f$ coefficients of underlying ODE for each collocation site \f$ t_k \f$ \details \f[ \left[ \gamma(t_1),\,\dots,\,\gamma(t_k),\,\dots,\,\gamma(t_{n-1})\right]^T \in \mathbb{R}^{n-1} \f]
        Eigen::VectorXd m_ODETau; //!< **Input:** Right hand side \f$ \tau(t) \f$ of underlying ODE for each collocation site \f$ t_k \f$ \details \f[ \left[ \tau(t_1),\,\dots,\,\tau(t_k),\,\dots,\,\tau(t_{n-1})\right]^T \in \mathbb{R}^{n-1} \f]

        // Boundary conditions
        Eigen::VectorXd m_boundaryConditions; //!< **Input:** Boundary conditions for approximating spline \f$ \left[ y_0,\,\dot{y}_0,\,\ddot{y}_0,\,y_n,\,\dot{y}_n,\,\ddot{y}_n\right]^T\in \mathbb{R}^6 \f$

        // "Internal" members (buffers)
        // ------------------
        // Partitioning
        Eigen::VectorXd m_knotSitesRemapped; //!< Buffer for remapped knot sites (same as \ref m_knotSites but with optionally added virtual control points)

        // Spline gradients
        Eigen::MatrixXd m_deta_dlambda; //!< Buffer for \f$ \left( \partial \eta / \partial \lambda\right) \f$
        Eigen::MatrixXd m_deta_dr; //!< Buffer for \f$ \left( \partial \eta / \partial r\right) \f$
        Eigen::MatrixXd m_grad_lambda_skxik_xik0; //!< Buffer for \f$ \nabla_{\lambda} s_k(\xi_k)|_{\xi_k=0} \f$ for \f$ k=1,\,\dots,\,n-1 \f$
        Eigen::MatrixXd m_grad_r_skxik_xik0; //!< Buffer for \f$ \nabla_{r} s_k(\xi_k)|_{\xi_k=0} \f$ for \f$ k=1,\,\dots,\,n-1 \f$
        Eigen::MatrixXd m_grad_lambda_dot_skxik_xik0; //!< Buffer for \f$ \nabla_{\lambda} \dot{s}_k(\xi_k)|_{\xi_k=0} \f$ for \f$ k=1,\,\dots,\,n-1 \f$
        Eigen::MatrixXd m_grad_r_dot_skxik_xik0; //!< Buffer for \f$ \nabla_{r} \dot{s}_k(\xi_k)|_{\xi_k=0} \f$ for \f$ k=1,\,\dots,\,n-1 \f$
        Eigen::MatrixXd m_grad_lambda_ddot_skxik_xik0; //!< Buffer for \f$ \nabla_{\lambda} \ddot{s}_k(\xi_k)|_{\xi_k=0} \f$ for \f$ k=1,\,\dots,\,n-1 \f$
        Eigen::MatrixXd m_grad_r_ddot_skxik_xik0; //!< Buffer for \f$ \nabla_{r} \ddot{s}_k(\xi_k)|_{\xi_k=0} \f$ for \f$ k=1,\,\dots,\,n-1 \f$

        // Linear system of equations for collocation
        Eigen::MatrixXd m_A_coll; //!< Buffer for matrix \f$ A_{coll} \f$ of LSE for collocation
        Eigen::VectorXd m_B_coll; //!< Buffer for right hand side \f$ B_{coll} \f$ of LSE for collocation

        // Output members
        // --------------
        Eigen::VectorXd m_lambda; //!< **Output:** Solution \f$ \lambda = [y_1,\,\dots,\,y_{n-1}]^T \f$ of LSE of collocation
        Eigen::VectorXd m_eta; //!< **Output:** Solution \f$ \eta = [\ddot{y}_1,\,\dots,\,\ddot{y}_{n-1}]^T \f$ (cubic) or \f$ \eta = [\dot{y}_1,\,\ddot{y}_1,\,\dots,\,\dot{y}_{n-1},\,\ddot{y}_{n-1}]^T \f$ (quintic) of LSE of collocation
        Eigen::VectorXd m_r; //!< **Output:** Boundary "vector" \f$ r = [y_0,\,\ddot{y}_0,\,y_n,\,\ddot{y}_n]^T \f$ (cubic) or \f$ r = [y_0,\,\dot{y}_0,\,\ddot{y}_0,\,y_n,\,\dot{y}_n,\,\ddot{y}_n]^T \f$ (quintic)

        // Processing
        // ----------
        //! Checks the validity of the input members (dimensions, ...)
        /*!
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_checkInput(Result* const result = nullptr) const
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Check, if input parameters have been initialized
            if (m_knotSites.rows() == 0 || m_knotSites.cols() == 0 || //
                m_ODEAlpha.rows() == 0 || m_ODEAlpha.cols() == 0 || //
                m_ODEBeta.rows() == 0 || m_ODEBeta.cols() == 0 || //
                m_ODEGamma.rows() == 0 || m_ODEGamma.cols() == 0 || //
                m_ODETau.rows() == 0 || m_ODETau.cols() == 0 || //
                m_boundaryConditions.rows() == 0 || m_boundaryConditions.cols() == 0) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_NOT_INITIALIZED;
                return false;
            }

            // Check, if enough knots have been specified
            const Eigen::Index knotCount = m_knotSites.rows(); // Extract count of knot sites
            if (knotCount < 3) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_TOO_FEW_KNOTS;
                return false;
            }
            const Eigen::Index collocationSiteCount = knotCount - 2; // Subtract start and end boundary knots

            // Check, if dimensions of input parameters match
            if (m_knotSites.rows() != knotCount || m_knotSites.cols() != 1 || //
                m_ODEAlpha.rows() != collocationSiteCount || m_ODEAlpha.cols() != 1 || //
                m_ODEBeta.rows() != collocationSiteCount || m_ODEBeta.cols() != 1 || //
                m_ODEGamma.rows() != collocationSiteCount || m_ODEGamma.cols() != 1 || //
                m_ODETau.rows() != collocationSiteCount || m_ODETau.cols() != 1 || //
                m_boundaryConditions.rows() != 6 || m_boundaryConditions.cols() != 1) {
                if (result != nullptr)
                    *result = Result::ERROR_INPUT_DIMENSION_MISMATCH;
                return false;
            }

            // Check, if knot sites are distinct and ascending
            for (Eigen::Index i = 1; i < m_knotSites.rows(); i++) {
                if (m_knotSites(i) <= m_knotSites(i - 1)) {
                    if (result != nullptr)
                        *result = Result::ERROR_INPUT_NOT_ASCENDING_KNOTS;
                    return false;
                }
            }

            // No error occured -> success
            return true;
        }

        //! Perform remapping of spline knots (adds virtual control points if necessary)
        /*!
         * \attention Depends only on \ref m_knotSites
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_addVirtualControlPoints(Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Default: no virtual control points -> keep spline knots the same
            m_knotSitesRemapped = m_knotSites;

            // No error occured -> success
            return true;
        }

        //! Setup of (internal) (block-)tridiagonal system from knot sites
        /*!
         * Computes \f$ L_i \f$, \f$ D_i \f$, \f$ U_i \f$, \f$ \left(\partial B / \partial \lambda \right) \f$, and \f$ \left(\partial B / \partial r \right) \f$
         * and stores the result to an internal buffer.
         *
         * \attention Depends only on \ref m_knotSitesRemapped
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_setupBlockTridiagonalSystem(Result* const result = nullptr) = 0;

        //! Solve (internal) (block-)tridiagonal system
        /*!
         * Computes \f$ \left(\partial X / \partial \lambda \right) \f$, \f$ \left(\partial X / \partial r \right) \f$, \f$ \left(\partial \eta / \partial \lambda \right) \f$, and \f$ \left(\partial \eta / \partial r \right) \f$
         * and stores the result to an internal buffer.
         *
         * \attention Depends only on \ref m_knotSitesRemapped
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_solveBlockTridiagonalSystem(Result* const result = nullptr) = 0;

        //! Compute spline gradients
        /*!
         * Computes \f$ \nabla_{\lambda}s_k(0) \f$, \f$ \nabla_{\lambda}\dot{s}_k(0) \f$, \f$ \nabla_{\lambda}\ddot{s}_k(0) \f$, \f$ \nabla_{r}s_k(0) \f$, \f$ \nabla_{r}\dot{s}_k(0) \f$, and \f$ \nabla_{r}\ddot{s}_k(0) \f$
         * for all collocation sites \f$\lbrace t_k \rbrace\f$ and stores the result to an internal buffer.
         *
         * \attention Depends only on \ref m_knotSitesRemapped
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_computeSplineGradients(Result* const result = nullptr) = 0;

        //! Assemble linear system of equations for collocation
        /*!
         * Computes \f$ A_{coll} \f$ and \f$ B_{coll} \f$ and stores the result to an internal buffer.
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_assembleCollocationLSE(Result* const result = nullptr) = 0;

        //! Solve linear system of equations for collocation
        /*!
         * Solves \f$ A_{coll}\,\lambda = B_{coll}\f$ for \f$ \lambda \f$, computes \f$ \eta \f$ and stores both in \ref m_lambda and \ref m_eta.
         *
         * \param [in] checkSingularity If `true` \f$ A_{coll} \f$ is checked for singularity (triggers a warning if \f$ A_{coll} \f$ is close to be singular)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_solveCollocationLSE(const bool& checkSingularity, Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Solve linear system
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(m_A_coll); // Compute QR decomposition of A
            m_lambda = QR.solve(m_B_coll);

            // Check for singularity of A_coll
            if (checkSingularity == true && result != nullptr) {
                if ((m_A_coll * m_lambda).isApprox(m_B_coll, 1e-6) == false)
                    *result = Result::WARNING_COLLOCATION_LSE_CLOSE_TO_SINGULAR;
            }

            // Compute eta out of lambda and r
            m_eta = m_deta_dlambda * m_lambda + m_deta_dr * m_r;

            // No error occured -> success
            return true;
        }

        //! Compute spline segments and pack them into a \ref broccoli::curve::Trajectory
        /*!
         * Uses \f$ \lambda \f$, \f$ \eta \f$, and \f$ r \f$ to compute all spline segments. The segments
         * are packed into a \ref broccoli::curve::Trajectory.
         *
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        virtual bool substep_convertToTrajectory(Result* const result = nullptr) = 0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace ode
} // namespace broccoli

#endif // HAVE_EIGEN3
