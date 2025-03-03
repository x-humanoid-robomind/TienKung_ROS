/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/math.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace control {
    /*!
     * \brief Base class for velocity-level inverse kinematics algorithms
     * \ingroup broccoli_control_kinematics
     *
     * All inverse kinematics objects of this type take a desired position \f$w_d\f$ and
     * velocity \f$\dot w_d\f$ in task-space as input argument and solve for the corresponding
     * velocity in joint-space \f$\dot q\f$.
     *
     * The implementation includes a drift-compensation with effective desired task-space velocity
     * \f$\dot w_{d,eff} = \dot w_d + K \frac{1}{dt} (w_d - w))\f$, where \f$K\f$ is a scalar gain and \f$dt\f$ is the
     * sample time.
     *
     * \tparam TaskSpaceSize The size of the task-space
     * \tparam ConfigurationSpaceSize The size of the configuration or joint-space
     */
    template <std::size_t TaskSpaceSize, std::size_t ConfigurationSpaceSize>
    class VelocityLevelInverseKinematics {
    public:
        using TaskSpaceType = Eigen::Matrix<double, TaskSpaceSize, 1>;
        using ConfigurationSpaceType = Eigen::Matrix<double, ConfigurationSpaceSize, 1>;

        virtual ~VelocityLevelInverseKinematics() {}

        /*!
         * \brief Set the target position and velocity in task space.
         * \param targetPosition The target position \f$w_d\f$ in task space
         * \param targetVelocity The target velocity \f$\dot w_d\f$ in task space
         */
        void setTarget(const TaskSpaceType& targetPosition, const TaskSpaceType& targetVelocity)
        {
            m_targetPosition = targetPosition;
            m_targetVelocity = targetVelocity;
        }

        /*!
         * \brief Sets the drift compensation gain
         * \param gain The drift compensation gain \f$0.0 \le K \le 1.0\f$
         */
        void setDriftCompensationGain(const double& gain)
        {
            assert(gain <= 1.0 && gain >= 0.0 && "Drift compensation gain must obey 0 <= gain <= 1.0 for stability!");
            m_driftCompensationGain = broccoli::core::math::clamp(gain, 0.0, 1.0);
        }

        //! Returns the output velocity \f$\dot q\f$ in configuration space as Signal
        virtual const Signal<ConfigurationSpaceType>& outputVelocity() const = 0;

    protected:
        /*!
         * \brief Calculates the effective desired velocity (desired velocity + drift compensation)
         * \param actualPosition The actual task-space position \f$w\f$.
         * \param dt The sampling time in seconds
         * \return \f$\dot w_{d,eff}\f$
         */
        virtual TaskSpaceType effectiveDesiredVelocity(const TaskSpaceType& actualPosition, const double& dt) const
        {
            return m_driftCompensationGain / dt * (m_targetPosition - actualPosition) + m_targetVelocity;
        }

    private:
        //! The target position \f$w_d\f$
        TaskSpaceType m_targetPosition = TaskSpaceType::Zero();

        //! The target velocity \f$\dot w_d\f$
        TaskSpaceType m_targetVelocity = TaskSpaceType::Zero();

        //! Drift compensation gain \f$K\f$
        double m_driftCompensationGain = 1.0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace control
} // namespace broccoli
