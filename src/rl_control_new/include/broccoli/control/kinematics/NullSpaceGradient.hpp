/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace broccoli {
namespace control {
    /*!
     * \brief Abstract base class for null-space cost function gradients.
     * \ingroup broccoli_control_kinematics
     *
     * This interface provides \f$w \frac{\partial H}{\partial q}\f$ for a scalar weight factor \f$w\f$ and
     * a scalar cost function \f$H\f$.
     *
     * \tparam NrOfDofs The number of joint-space degrees (\f$dim(q)\f$)
     */
    template <std::size_t NrOfDofs>
    class NullSpaceGradient {
    public:
        using JointSpaceType = Eigen::Matrix<double, NrOfDofs, 1>;
        static constexpr std::size_t Size = NrOfDofs;

        virtual ~NullSpaceGradient() {}

        //! Returns the resulting gradient \f$w \frac{\partial H}{\partial q}\f$
        JointSpaceType gradient() const
        {
            return m_weightedGradient;
        }

        /*!
         * \brief Update the gradient based on \f$q, \dot q\f$.
         * \param actualJointPosition The actual joint-space position \f$q\f$
         * \param actualJointVelocity The actual joint-space velocity \f$\dot q\f$
         * \return The resulting gradient \f$w \frac{\partial H}{\partial q}\f$
         */
        virtual const JointSpaceType process(const JointSpaceType& actualJointPosition, const JointSpaceType& actualJointVelocity)
        {
            m_unweightedGradient = calculateGradient(actualJointPosition, actualJointVelocity);
            m_weightedGradient = calculateWeightedGradient();
            return m_weightedGradient;
        }

        //! Set the scalar weight factor \f$w\f$
        void setWeight(const double& weight)
        {
            m_weight = weight;
            m_weightedGradient = calculateWeightedGradient();
        }

        //! Returns the scalar weight factor \f$w\f$
        const double& weight() const
        {
            return m_weight;
        }

    protected:
        /*!
         * \brief Internal method, which does the actual computation of the null-space gradient
         * \param actualJointPosition The actual joint-space position \f$q\f$
         * \param actualJointVelocity The actual joint-space velocity \f$\dot q\f$
         * \returns The resulting null-space gradient
         */
        virtual JointSpaceType calculateGradient(const JointSpaceType& actualJointPosition, const JointSpaceType& actualJointVelocity) = 0;

        //! Returns the weighted gradient \f$w \frac{\partial H}{\partial q}\f$ based on the unweighted gradient and the current weight
        virtual JointSpaceType calculateWeightedGradient() { return m_weight * m_unweightedGradient; }

    private:
        //! The unweighted gradient
        JointSpaceType m_unweightedGradient = JointSpaceType::Zero();

        //! The weighted gradient
        JointSpaceType m_weightedGradient = JointSpaceType::Zero();

        //! The weighing factor
        double m_weight = 1.0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace control
} // namespace broccoli
