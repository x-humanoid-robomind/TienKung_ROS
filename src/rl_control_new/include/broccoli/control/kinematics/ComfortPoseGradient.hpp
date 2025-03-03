/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "NullSpaceGradient.hpp"

namespace broccoli {
namespace control {
    /*!
     * \brief Implements a comfort pose null-space gradient.
     * \ingroup broccoli_control_kinematics
     *
     * This calculates the gradient \f$\frac{\partial H}{\partial q}\f$ of the
     * comfort pose cost function
     * \f[
     *  H = w \frac{1}{2} (s^T (q - q_{\text{cmf}}))^2,
     * \f]
     * with the current configuration \f$q \in \mathcal{R}^n\f$, a weighing scalar \f$w\f$,
     * a scaling vector \f$s \in \mathcal{R}^n\f$, and the desired comfort pose of the robot \f$q_{\text{cmf}}\f$.
     * The scaling vector \f$s\f$ can be defined in three different ways:\n
     * 1) Using equal scaling, i.e. \f$s\f$ equals identity. (**default**)\n
     * 2) Using range-based scaling, i.e. \f$s_i\f$ for joint \f$i\f$ is calculated based on the joint's minimum
     * and maximum values (\f$q_{i, \text{min}}, q_{i, \text{max}}\f$).(**recommended**)\n
     * 3) Using a custom scaling vector \f$s\f$.\n
     *
     * \tparam NrOfDofs The number of joint-space degrees
     */
    template <std::size_t NrOfDofs>
    class ComfortPoseGradient : public NullSpaceGradient<NrOfDofs> {
    public:
        using JointSpaceType = typename NullSpaceGradient<NrOfDofs>::JointSpaceType;

        /*!
         * \brief Scale the gradient equally for all joints.
         *
         * This sets the scaling vector \f$s\f$ to identity.
         */
        void useEqualScaling()
        {
            m_jointScalingVector = JointSpaceType::Ones();
        }

        /*!
         * \brief Scale the gradient of each joint based on the joint's range
         *
         * The comfort pose is scaled relatively to the joint with the largest range.
         * This sets \f$s_i = \frac{max(\Delta q_{\text{range}})}{\Delta q_{\text{range},i}}\f$ for joint \f$i\f$.
         *
         * \param jointRanges \f$\Delta q_{\text{range}}\f$
         */
        void useRangeBasedScaling(const JointSpaceType& jointRanges)
        {
            useCustomScaling(jointRanges.maxCoeff() * jointRanges.cwiseInverse());
        }

        /*!
         * \brief Scale the gradient of each joint based on the joint's maximum and minimum value
         *
         * The comfort pose is scaled relatively to the joint with the largest range.
         * This sets \f$s_i = \frac{max(\Delta q_{\text{range}})}{\Delta q_{\text{range},i}}\f$ for joint \f$i\f$ with
         * \f$\Delta q_{\text{range}} = q_{\text{max}} - q_{\text{min}}\f$.
         *
         * \param minJointValues \f$q_{\text{min}}\f$
         * \param maxJointValues \f$q_{\text{max}}\f$
         */
        void useRangeBasedScaling(const JointSpaceType& minJointValues, const JointSpaceType& maxJointValues)
        {
            useRangeBasedScaling(maxJointValues - minJointValues);
        }

        /*!
         * \brief Use a custom scaling vector \f$s\f$
         * \param jointScalingVector The scaling vector \f$s\f$
         */
        void useCustomScaling(const JointSpaceType& jointScalingVector)
        {
            for (std::size_t i = 0; i < NrOfDofs; i++) {
                assert(m_jointScalingVector(i) >= 0 && "Scaling vector must not have negative entries!");
            }

            m_jointScalingVector = jointScalingVector.cwiseAbs();
        }

        /*!
         * \brief Sets the desired comfort pose
         * \param comfortPose The comfort pose \f$q_{\text{cmf}}\f$
         */
        void setPose(const JointSpaceType& comfortPose)
        {
            m_comfortPose = comfortPose;
        }

        //! Returns the current scaling vector \f$s\f$
        const JointSpaceType& jointScalingVector() const
        {
            return m_jointScalingVector;
        }

    protected:
        JointSpaceType calculateGradient(const JointSpaceType& actualJointPosition, const JointSpaceType&) override
        {
            return m_jointScalingVector.cwiseProduct(actualJointPosition - m_comfortPose);
        }

        //! The comfort pose in joint-space \f$q_{\text{cmf}}\f$
        JointSpaceType m_comfortPose = JointSpaceType::Zero();

        //! The scaling vector \f$s\f$
        JointSpaceType m_jointScalingVector = JointSpaceType::Ones();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace control
} // namespace broccoli
