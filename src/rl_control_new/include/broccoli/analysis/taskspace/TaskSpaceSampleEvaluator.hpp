/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "KinematicChain.hpp"
#include "TaskSpaceSample.hpp"
#include <math.h>

namespace broccoli {
namespace analysis {
    //! Helper class for efficient evaluation of task-space samples
    /*!
     * \ingroup broccoli_analysis_taskspace
     *
     * Contains buffers for intermediate results to avoid extensive dynamic memory allocation during evaluation of many samples.
     */
    class TaskSpaceSampleEvaluator {
    public:
        //! Specialized constructor
        /*!
         * \param [in] kinematicChain Initializes \ref m_kinematicChain - \copybrief m_kinematicChain
         * \param [in] taskSpaceSelectionMatrix Initializes \ref m_taskSpaceSelectionMatrix - \copybrief m_taskSpaceSelectionMatrix
         */
        TaskSpaceSampleEvaluator(const KinematicChain& kinematicChain, const Eigen::Matrix<double, Eigen::Dynamic, 6>& taskSpaceSelectionMatrix)
            : m_kinematicChain(kinematicChain)
            , m_taskSpaceSelectionMatrix(taskSpaceSelectionMatrix)
            , m_jointSpaceDimension(kinematicChain.dofCount())
            , m_taskSpaceDimension(taskSpaceSelectionMatrix.rows())
        {
        }

        // Members
        // -------
    protected:
        // General
        // -------
        //! The kinematic chain to be evaluated
        const KinematicChain m_kinematicChain;

        //! Task space selection matrix \f$S \in \mathbb{R}^{m \times 6}\f$
        /*!
         * The task space selection matrix \f$ S \f$ maps the task space coordinates to the 6 spatial coordinates \f$(v_x,\,v_y,\,v_z,\,\omega_x,\,\omega_y,\,\omega_z)\f$.
         * It has direct influence on the task-space jacobian and thus the task-space metrics (manipulability, etc.). Note that this matrix can also be used to weight
         * single dofs (e.g. translations vs. rotations).
         *
         * Examples:
         * ---------
         * Position-only:
         * \f[ S = \left[\begin{array}{cccccc} 1, 0, 0, 0, 0, 0\\ 0, 1, 0, 0, 0, 0\\ 0, 0, 1, 0, 0, 0\end{array}\right] \f]
         *
         * Orientation-only:
         * \f[ S = \left[\begin{array}{cccccc} 0, 0, 0, 1, 0, 0\\ 0, 0, 0, 0, 1, 0\\ 0, 0, 0, 0, 0, 1\end{array}\right] \f]
         */
        const Eigen::Matrix<double, Eigen::Dynamic, 6> m_taskSpaceSelectionMatrix;

        // Dimensions
        const uint64_t m_jointSpaceDimension; //!< Joint space dimension (count of joints)
        const uint64_t m_taskSpaceDimension; //!< Task space dimension

        // Initial values
        const Eigen::Vector3d m_initialPosition = Eigen::Vector3d::Zero(); //!< Initial value for positions
        const Eigen::Matrix3d m_initialRotationMatrix = Eigen::Matrix3d::Identity(); //!< Initial value for rotation matrices
        const Eigen::Matrix<double, 3, Eigen::Dynamic> m_initialJacobianTranslationRotation = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, m_jointSpaceDimension); //!< Initial value for jacobians of translation and rotation (stacked)
        const Eigen::Matrix<double, 6, Eigen::Dynamic> m_initialJacobianStacked = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, m_jointSpaceDimension); //!< Initial value for jacobians of translation and rotation (stacked)

        // Buffers
        // -------
        // Buffers for rotation matrices
        Eigen::Matrix3d m_w_A_im1 = m_initialRotationMatrix; //!< Rotation matrix from dof frame of (i-1)-th segment to world frame
        Eigen::Matrix3d m_w_A_i = m_initialRotationMatrix; //!< Rotation matrix from dof frame of i-th segment to world frame
        Eigen::Matrix3d m_im1_A_ii = m_initialRotationMatrix; //!< Rotation matrix from initial frame of i-th segment to dof frame of (i-1)-th segment
        Eigen::Matrix3d m_ii_A_im1 = m_initialRotationMatrix; //!< Transpose of \ref m_im1_A_ii
        Eigen::Matrix3d m_ii_A_i = m_initialRotationMatrix; //!< Rotation matrix from dof frame of i-th segment to initial frame of i-th segment
        Eigen::Matrix3d m_i_A_ii = m_initialRotationMatrix; //!< Transpose of \ref m_ii_A_i
        Eigen::Matrix3d m_im1_A_i = m_initialRotationMatrix; //!< Rotation matrix from dof frame of i-th segment to dof frame of (i-1)-th segment
        Eigen::Matrix3d m_i_A_im1 = m_initialRotationMatrix; //!< Transpose of \ref m_im1_A_i
        // Buffers for positions
        Eigen::Vector3d m_im1_r_im1 = m_initialPosition; //!< Absolute position of dof frame of (i-1)-th segment in dof frame of (i-1)-th segment
        Eigen::Vector3d m_i_r_i = m_initialPosition; //!< Absolute position of dof frame of i-th segment in dof frame of i-th segment
        Eigen::Vector3d m_im1_r_im1_ii = m_initialPosition; //!< Relative position of initial frame of i-th segment to dof frame of (i-1)-th segment in dof frame of (i-1)-th segment
        Eigen::Vector3d m_ii_r_ii_i = m_initialPosition; //!< Relative position of dof frame of i-th segment to initial frame of i-th segment in initial frame of i-th segment
        Eigen::Vector3d m_im1_r_ii_i = m_initialPosition; //!< Relative position of dof frame of i-th segment to initial frame of i-th segment in dof frame of (i-1)-th segment
        Eigen::Vector3d m_im1_r_im1_i = m_initialPosition; //!< Relative position of dof frame of i-th segment to dof frame of (i-1)-th segment in dof frame of (i-1)-th segment
        Eigen::Vector3d m_im1_r_i = m_initialPosition; //!< Absolute position of dof frame of i-th segment in dof frame of (i-1)-th segment
        Eigen::Vector3d m_w_r_tcp = m_initialPosition; //!< Absolute position of tcp frame in world frame
        // Buffers for jacobians (translation)
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_im1_JT_im1 = m_initialJacobianTranslationRotation; //!< Absolute translational jacobian of dof frame of (i-1)-th segment in dof frame of (i-1)-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_i_JT_i = m_initialJacobianTranslationRotation; //!< Absolute translational jacobian of dof frame of i-th segment in dof frame of i-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_im1_JT_im1_ii = m_initialJacobianTranslationRotation; //!< Relative translational jacobian of initial frame of i-th segment to dof frame of (i-1)-th segment in dof frame of (i-1)-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_ii_JT_ii_i = m_initialJacobianTranslationRotation; //!< Relative translational jacobian of dof frame of i-th segment to initial frame of i-th segment in initial frame of i-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_w_JT_tcp = m_initialJacobianTranslationRotation; //!< Absolute translational jacobian of tcp frame in world frame
        // Buffers for jacobians (rotation)
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_im1_JR_im1 = m_initialJacobianTranslationRotation; //!< Absolute rotational jacobian of dof frame of (i-1)-th segment in dof frame of (i-1)-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_i_JR_i = m_initialJacobianTranslationRotation; //!< Absolute rotational jacobian of dof frame of i-th segment in dof frame of i-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_ii_JR_ii_i = m_initialJacobianTranslationRotation; //!< Relative rotational jacobian of dof frame of i-th segment to initial frame of i-th segment in initial frame of i-th segment
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_w_JR_tcp = m_initialJacobianTranslationRotation; //!< Absolute rotational jacobian of tcp frame in world frame
        // Buffers for jacobians (generic)
        Eigen::Matrix<double, 6, Eigen::Dynamic> m_jacobianStacked = m_initialJacobianStacked; //!< Stacked jabobian (translation over rotation)
        // Buffer for singular value decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> m_svdJ; //!< Singular value decomposition of the task-space jacobian

        // Evaluation
        // ----------
    public:
        //! Evaluates complete kinematic chain at a certain configuration
        /*!
         * \param [in] sampleIndex Unique index of this sample
         * \param [in] configuration The configuration of the kinematic chain used for evaluation ("pose" of the robot)
         * \param [out] sample The evaluated sample (reference to output data structure)
         */
        void evaluate(const uint64_t& sampleIndex, const Eigen::VectorXd& configuration, TaskSpaceSample& sample)
        {
            /* Explanation:
             * ------------
             * For each segment, there are two (consecutive) frames:
             *   * "initial" frame (fixed rotation and translation, but no motion) and
             *   * "dof" frame (either rotation around local z-axis, or translation along local z-axis)
             *
             * world ..."world" frame (= parent of root segment)
             * im1   ...dof-frame of (i-1)-th segment
             * ii    ...initial-frame of i-th segment
             * i     ...dof-frame of i-th segment
             * tcp   ...tool-center-point frame (= dof-frame of last segment)
             */

            // Initialize buffers for rotation matrices
            m_w_A_im1 = m_initialRotationMatrix;
            m_w_A_i = m_initialRotationMatrix;

            // Initialize buffers for positions
            m_im1_r_im1 = m_initialPosition;
            m_i_r_i = m_initialPosition;

            // Jacobian (translation)
            m_im1_JT_im1 = m_initialJacobianTranslationRotation;
            m_i_JT_i = m_initialJacobianTranslationRotation;
            m_im1_JT_im1_ii = m_initialJacobianTranslationRotation;
            m_ii_JT_ii_i = m_initialJacobianTranslationRotation;

            // Jacobian (rotation)
            m_im1_JR_im1 = m_initialJacobianTranslationRotation;
            m_i_JR_i = m_initialJacobianTranslationRotation;
            m_ii_JR_ii_i = m_initialJacobianTranslationRotation;

            // Copy data
            sample.m_sampleIndex = sampleIndex;
            sample.m_configuration = configuration;

            // Run through segments of the chain
            Eigen::Index currentDOFIndex = 0; // Index of currently "active" DOF
            for (size_t i = 0; i < m_kinematicChain.segments().size(); i++) {
                // Update current segment
                const KinematicChainSegment& currentSegment = m_kinematicChain.segments()[i];
                if (i > 0) {
                    m_w_A_im1 = m_w_A_i;
                    m_im1_r_im1 = m_i_r_i;
                    m_im1_JT_im1 = m_i_JT_i;
                    m_im1_JR_im1 = m_i_JR_i;
                }

                // Process "initial" frame
                // -----------------------
                // Position and orientation
                m_im1_A_ii = currentSegment.initialTransform().linear();
                m_ii_A_im1 = m_im1_A_ii.transpose();
                m_im1_r_im1_ii = currentSegment.initialTransform().translation();

                // Compute relative jacobians
                if (i > 0) {
                    Eigen::Matrix3d im1_rtilde_im1_ii_transpose; // {}_{i-1} \tilde{r}_{i-1,ii}^T
                    im1_rtilde_im1_ii_transpose << 0, m_im1_r_im1_ii(2), -m_im1_r_im1_ii(1), //
                        -m_im1_r_im1_ii(2), 0, m_im1_r_im1_ii(0), //
                        m_im1_r_im1_ii(1), -m_im1_r_im1_ii(0), 0;
                    m_im1_JT_im1_ii = im1_rtilde_im1_ii_transpose * m_im1_JR_im1;
                }

                // Process "dof" frame
                // -------------------
                // Position and orientation
                m_ii_A_i = m_initialRotationMatrix;
                m_ii_r_ii_i = m_initialPosition;
                const double& dofValue = configuration(i);
                if (currentSegment.dofType() == KinematicChainSegment::DOFType::ROTATION) {
                    m_ii_A_i(0, 0) = cos(dofValue);
                    m_ii_A_i(1, 0) = sin(dofValue);
                    m_ii_A_i(0, 1) = -m_ii_A_i(1, 0); // -sin(angle)
                    m_ii_A_i(1, 1) = m_ii_A_i(0, 0); // cos(angle)
                } else if (currentSegment.dofType() == KinematicChainSegment::DOFType::TRANSLATION)
                    m_ii_r_ii_i(2) = dofValue;
                m_i_A_ii = m_ii_A_i.transpose();
                m_im1_A_i = m_im1_A_ii * m_ii_A_i;
                m_i_A_im1 = m_im1_A_i.transpose();
                m_im1_r_ii_i = m_im1_A_ii * m_ii_r_ii_i;
                m_im1_r_im1_i = m_im1_r_im1_ii + m_im1_r_ii_i;
                m_im1_r_i = m_im1_r_im1 + m_im1_r_im1_i;
                m_i_r_i = m_i_A_im1 * m_im1_r_i;

                // Compute relative jacobians
                Eigen::Matrix3d ii_rtilde_ii_i_transpose; // {}_{ii} \tilde{r}_{ii,i}^T
                ii_rtilde_ii_i_transpose << 0, m_ii_r_ii_i(2), -m_ii_r_ii_i(1), //
                    -m_ii_r_ii_i(2), 0, m_ii_r_ii_i(0), //
                    m_ii_r_ii_i(1), -m_ii_r_ii_i(0), 0;
                m_ii_JT_ii_i = ii_rtilde_ii_i_transpose * m_ii_A_im1 * m_im1_JR_im1;
                m_ii_JR_ii_i = m_initialJacobianTranslationRotation;
                if (currentSegment.dofType() == KinematicChainSegment::DOFType::ROTATION)
                    m_ii_JR_ii_i(2, currentDOFIndex) += 1;
                else if (currentSegment.dofType() == KinematicChainSegment::DOFType::TRANSLATION)
                    m_ii_JT_ii_i(2, currentDOFIndex) += 1;

                // Update absolute jacobians
                m_i_JT_i = m_i_A_im1 * (m_im1_JT_im1 + m_im1_JT_im1_ii) + m_i_A_ii * m_ii_JT_ii_i;
                m_i_JR_i = m_i_A_im1 * m_im1_JR_im1 + m_i_A_ii * m_ii_JR_ii_i; // Note that i_A_im1 * im1_JR_im1_ii = 0

                // Update transform relative to world
                m_w_A_i = m_w_A_im1 * m_im1_A_i;

                // Update dof index
                if (currentSegment.dofType() != KinematicChainSegment::DOFType::FIXED)
                    currentDOFIndex++;
            }

            // Compute quantities of tool-center-point frame
            const Eigen::Matrix3d& w_A_tcp = m_w_A_i; // Rotation matrix from tcp frame to world frame
            m_w_r_tcp = m_w_A_i * m_i_r_i;
            m_w_JT_tcp = m_w_A_i * m_i_JT_i;
            m_w_JR_tcp = m_w_A_i * m_i_JR_i;

            // Position and orientation
            sample.m_position = m_w_r_tcp;
            sample.m_orientation = Eigen::Quaterniond(w_A_tcp);

            // Jacobians of translation and rotation
            sample.m_jacobianTranslation = m_w_JT_tcp;
            sample.m_jacobianRotation = m_w_JR_tcp;

            // Task-space jacobian
            m_jacobianStacked = m_initialJacobianStacked;
            m_jacobianStacked << m_w_JT_tcp, m_w_JR_tcp;
            sample.m_jacobianTaskSpace = m_taskSpaceSelectionMatrix * m_jacobianStacked;

            // Compute condition index
            m_svdJ.compute(sample.m_jacobianTaskSpace);
            sample.m_conditionIndex = m_svdJ.singularValues().minCoeff() / m_svdJ.singularValues().maxCoeff();

            // Compute manipulability measure
            sample.m_manipulabilityMeasure = 0.0;
            if (m_jointSpaceDimension >= m_taskSpaceDimension) {
                sample.m_manipulabilityMeasure = 1.0;
                for (int i = 0; i < m_svdJ.singularValues().size(); i++)
                    sample.m_manipulabilityMeasure *= m_svdJ.singularValues()(i);
            }

            // Compute joint range availability
            size_t jraContributors = 0;
            double jraProduct = 1.0;
            for (size_t i = 0; i < m_kinematicChain.segments().size(); i++) {
                // Only consider contribution, if this is a DOF with non-zero joint range
                const KinematicChainSegment& currentSegment = m_kinematicChain.segments()[i];
                if (currentSegment.dofType() != KinematicChainSegment::DOFType::FIXED && currentSegment.maximumPosition() != currentSegment.minimumPosition()) {
                    // Add contribution
                    jraProduct *= std::min(std::fabs(currentSegment.maximumPosition() - configuration(i)), std::fabs(configuration(i) - currentSegment.minimumPosition())) / (0.5 * std::fabs(currentSegment.maximumPosition() - currentSegment.minimumPosition()));
                    jraContributors++;
                }
            }
            sample.m_jointRangeAvailability = 0.0;
            if (jraContributors > 0)
                sample.m_jointRangeAvailability = std::pow(jraProduct, 1.0 / ((double)jraContributors));
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
