/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "NullSpaceGradient.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace control {
    /*!
     * \brief Implements a joint limit avoidance null-space gradient
     * \ingroup broccoli_control_kinematics
     *
     * The algorithm uses inner safety margins \f$\Delta_{i,s}\f$ within the range from
     * minimum to maximum allowed joint angles. The safety margin defines
     * two ranges. One above the minimum joint angles and one below the maximum joint angles.
     * Within these regions the cost function \f$H\f$ increases (the joints are decelerated).
     *
     * The null-space gradient \f$\frac{\partial H}{\partial q}\f$ corresponds to the scalar cost function
     * \f$H = \frac{1}{2} \sum_{i=1}^n h_i^2\f$ with
     * \f[
     * h_i = \begin{cases}
     * \frac{q_i - q_{i,max} + \Delta_{i,s}}{\Delta_{i,s}} & q_i > q_{i,max} - \Delta_{i,s}\\
     * \frac{q_i - q_{i,min} - \Delta_{i,s}}{\Delta_{i,s}} & q_i < q_{i,min} + \Delta_{i,s}
     * \end{cases}
     * \f]
     * This algorithm was proposed in "T. Buschmann, Simulation and Control of Biped Walking Robots. Dissertation. Technical University of Munich, 2010"
     *
     * \tparam NrOfDofs The number of joint-space degrees of freedom \f$n\f$
     */
    template <std::size_t NrOfDofs>
    class JointLimitGradient : public NullSpaceGradient<NrOfDofs> {
    public:
        using JointSpaceType = typename NullSpaceGradient<NrOfDofs>::JointSpaceType;

        /*!
         * \brief A constructor
         * \param safetyMargins The safety margins \f$\Delta_{i,s} > 0\f$
         * \param minimumJointAngles The minimum joint angles \f$q_{i,min}\f$
         * \param maximumJointAngles The maximum joint angles \f$q_{i,max}\f$
         */
        JointLimitGradient(const JointSpaceType& safetyMargins,
            const JointSpaceType& minimumJointAngles,
            const JointSpaceType& maximumJointAngles)
        {
            m_safetyMargin = safetyMargins.cwiseAbs();
            m_minimumJointAngles = minimumJointAngles;
            m_maximumJointAngles = maximumJointAngles;

            for (std::size_t i = 0; i < NrOfDofs; i++) {
                if (m_safetyMargin(i) == 0.0) {
                    assert(false && "Safety margin entries must not be zero!");
                    m_safetyMargin(i) = 0.001;
                }

                if (m_maximumJointAngles(i) < m_minimumJointAngles(i)) {
                    assert(false && "Maximum joint angles must be greater than minimum joint angles!");
                    m_maximumJointAngles(i) = m_minimumJointAngles(i);
                }
            }
        }

    protected:
        JointSpaceType calculateGradient(const JointSpaceType& actualJointPosition, const JointSpaceType&) override
        {
            JointSpaceType result;
            for (size_t i = 0; i < NrOfDofs; i++) {
                const double q0 = m_maximumJointAngles(i) - m_safetyMargin(i);
                const double q1 = m_minimumJointAngles(i) + m_safetyMargin(i);

                if (actualJointPosition(i) > q0) {
                    result(i) = (actualJointPosition(i) - q0) / m_safetyMargin(i);
                } else if (actualJointPosition(i) < q1) {
                    result(i) = (actualJointPosition(i) - q1) / m_safetyMargin(i);
                } else {
                    result(i) = 0.0;
                }
            }
            return result;
        }

        JointSpaceType m_safetyMargin;
        JointSpaceType m_minimumJointAngles;
        JointSpaceType m_maximumJointAngles;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace control
} // namespace broccoli
