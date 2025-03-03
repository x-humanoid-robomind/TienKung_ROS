/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/math.hpp"
#include "../Signal.hpp"
#include "VelocityLevelInverseKinematics.hpp"
#include <Eigen/Dense>

namespace broccoli {
namespace control {
    /*!
     * \brief Implements an Automatic Supervisory Control based inverse kinematics
     * \ingroup broccoli_control_kinematics
     *
     * This velocity-level inverse kinematics algorithm is a solution
     * to the following optimization problem:
     * \f{align*}{
     * \Phi &= \frac{1}{2} \dot q^T W \dot q + \frac{\partial H}{\partial q} \dot q \rightarrow \text{min} \\
     * & \gamma(\dot w_{d,eff} - J \dot q) = 0, \\
     * \f}
     * with the joint-space velocity \f$\dot q\f$, the effective desired task-space velocity \f$\dot w_{d,eff}\f$,
     * the task-space Jacobian \f$J\f$, the diagonal weighing matrix \f$W\f$,
     * the nullspace gradient \f$\left (\frac{\partial H}{\partial q} \right )^T\f$, and a selection factor \f$0 \le \gamma \le 1.0\f$.
     * This algorithm locally minimizes the scalar cost function \f$H(q)\f$.
     *
     * The implementation is based on this equation:
     * \f$\dot q = J^\#_W \dot w_{d,eff} + (E - J^\#_W J) \underbrace{W^{-1} - \left (\frac{\partial H}{\partial q} \right )^T}_u \f$
     * and uses the \f$W\f$-weighed pseudoinverse \f$J^\#_W\f$. When the selection factor \f$\gamma = 0.0\f$, the task-space
     * constraint is completely lifted, which results in the solution \f$\dot q_n = u\f$. This algorithm implements an efficient solution of the
     * IK problem without explicit calculation of the pseudoinverse.
     * See \ref VelocityLevelInverseKinematics for details on the computation of the effective desired velocity
     * \f$\dot w_{d,eff} \f$.
     *
     * \tparam TaskSpaceSize The size of the task space
     * \tparam ConfigurationSpaceSize The size of the configuration space
     */
    template <std::size_t TaskSpaceSize, std::size_t ConfigurationSpaceSize>
    class AutomaticSupervisoryControl : public VelocityLevelInverseKinematics<TaskSpaceSize, ConfigurationSpaceSize> {
    public:
        using TaskSpaceType = Eigen::Matrix<double, TaskSpaceSize, 1>;
        using ConfigurationSpaceType = Eigen::Matrix<double, ConfigurationSpaceSize, 1>;
        using JacobianType = Eigen::Matrix<double, TaskSpaceSize, ConfigurationSpaceSize>;
        using WeighingType = Eigen::Matrix<double, ConfigurationSpaceSize, ConfigurationSpaceSize>;

        /*!
         * \brief Sets the weighing matrix entries
         * \param weighingFactors The diagonal entries of \f$W\f$
         */
        void setWeighingMatrix(const ConfigurationSpaceType& weighingFactors)
        {
            m_inverseWeighing = weighingFactors.cwiseInverse().asDiagonal();
        }

        /*!
         * \brief Sets the task-space constraint scaling factor \f$\gamma\f$.
         *
         * This factor may be used to deactivate the task-space constraint and
         * pass-through the null-space output \f$u\f$.
         *
         * \attention The factor must be \f$\mathcal{C}^n\f$-smooth in order to get
         * \f$\mathcal{C}^n\f$-smooth output velocities from this inverse kinematics algorithm.
         *
         * \param activationFactor Task-space constraint activation factor [0...1]
         */
        void setTaskSpaceConstraintFactor(const double& activationFactor)
        {
            m_activationFactorTaskSpace = activationFactor;
        }

        /*!
         * \brief Calculates inverse kinematics with given nullspace gradient
         * \param taskSpaceJacobian The jacobian \f$J\f$
         * \param actualPosition The current task-space position \f$w\f$
         * \param nullSpaceGradient The null-space gradient \f$\left (\frac{\partial H}{\partial q} \right )^T\f$
         * \param dt The sample time in seconds
         * \return The configuration-space velocity \f$\dot q\f$
         */
        const Signal<ConfigurationSpaceType>& process(const JacobianType& taskSpaceJacobian, const TaskSpaceType& actualPosition, const ConfigurationSpaceType& nullSpaceGradient, const double& dt)
        {
            calculateOutputVelocity(taskSpaceJacobian, actualPosition, -m_inverseWeighing * nullSpaceGradient, dt);
            return m_outputVelocity;
        }

        /*!
         * \brief Calculates inverse kinematics with an additional lower-priority task
         *
         * This method can be used to realize a task-priority order with different
         * task-space vectors. The implementation follows the method described in "S. Chiaverini, Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators, Robot. Autom. IEEE Trans., vol. 13, no. 3, pp. 398–410, 1997."
         * and is realized by chaining separate instances of AutomaticSupervisoryControl.
         * Using this method, a slave instance --- whose output is projected in the nullspace of this instance ---
         * can be supplied instead of a nullspace gradient. The output of the slave replaces \f$u\f$. The slave's process() method must have been called beforehand.
         * In combination with the selection factor \f$\gamma\f$ this feature can be used to change task-priority orders online.
         *
         * \param taskSpaceJacobian The jacobian \f$J\f$
         * \param actualPosition The current task-space position \f$w\f$
         * \param slave The slave instance with valid output velocity in the current step. The output of this IK object will be projected into the nullspace.
         * \return The configuration-space velocity \f$\dot q\f$
         */
        template <std::size_t TaskSpaceSizeSlave>
        const Signal<ConfigurationSpaceType>& process(const JacobianType& taskSpaceJacobian, const TaskSpaceType& actualPosition, const VelocityLevelInverseKinematics<TaskSpaceSizeSlave, ConfigurationSpaceSize>& slave)
        {
            calculateOutputVelocity(taskSpaceJacobian, actualPosition, slave.outputVelocity().value(), slave.outputVelocity().sampleTime());
            return m_outputVelocity;
        }

        const Signal<ConfigurationSpaceType>& outputVelocity() const override
        {
            return m_outputVelocity;
        }

    protected:
        using VelocityLevelInverseKinematics<TaskSpaceSize, ConfigurationSpaceSize>::effectiveDesiredVelocity;

        /*!
         * \brief Calculates the joint-space velocity output for the given task-space and null-space objectives
         * \param taskSpaceJacobian The jacobian \f$J\f$
         * \param actualPosition The current task-space position \f$w\f$
         * \param nullSpaceVelocity The desired joint-velocities in null-space
         * \param dt The sample time in seconds
         */
        virtual void calculateOutputVelocity(const JacobianType& taskSpaceJacobian, const TaskSpaceType& actualPosition, const ConfigurationSpaceType& nullSpaceVelocity, const double& dt)
        {
            m_outputVelocity.value() = broccoli::core::math::solvePseudoInverseEquation(taskSpaceJacobian, m_inverseWeighing, effectiveDesiredVelocity(actualPosition, dt), nullSpaceVelocity, m_activationFactorTaskSpace);
            m_outputVelocity.sampleTime() = dt;
        }

    private:
        //! Inverse nullspace weighing matrix \f$W^{-1}\f$
        WeighingType m_inverseWeighing = WeighingType::Identity();

        //! Activation factor \f$\gamma\f$ [0...1] may be used to deactivate the task-space constraints
        double m_activationFactorTaskSpace = 1.0;

        //! The output velocity \f$\dot q\f$ in joint-space
        Signal<ConfigurationSpaceType> m_outputVelocity;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace control
} // namespace broccoli
