/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../io/logging/ColumnBasedLogFileData.hpp"

namespace broccoli {
namespace analysis {
    //! Container for taskspace-data computed during evaluation of a single sample (configuration of kinematic chain)
    /*! \ingroup broccoli_analysis_taskspace */
    class TaskSpaceSample : public io::ColumnBasedLogFileData {
    public:
        friend class TaskSpaceSampleEvaluator;

        // Members
        // -------
    protected:
        uint64_t m_sampleIndex = 0; //!< \copybrief sampleIndex()
        Eigen::VectorXd m_configuration; //!< \copybrief configuration()
        Eigen::Vector3d m_position = Eigen::Vector3d::Zero(); //!< \copybrief position()
        Eigen::Quaterniond m_orientation = Eigen::Quaterniond::Identity(); //!< \copybrief orientation()
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_jacobianTranslation; //!< \copybrief jacobianTranslation()
        Eigen::Matrix<double, 3, Eigen::Dynamic> m_jacobianRotation; //!< \copybrief jacobianRotation()
        Eigen::MatrixXd m_jacobianTaskSpace; //!< \copybrief jacobianTaskSpace()
        double m_conditionIndex = 0.0; //!< \copybrief conditionIndex()
        double m_manipulabilityMeasure = 0.0; //!< \copybrief manipulabilityMeasure()
        double m_jointRangeAvailability = 0.0; //!< \copybrief jointRangeAvailability()

        // Logging
        // -------
    public:
        // Encodes the column-based log data to the log stream (see base class for details)
        void encodeColumnDataToLogStream(io::ColumnBasedLogStream& stream) const
        {
            stream.addData(m_sampleIndex, "SampleIndex");
            stream.addData(m_configuration, "Configuration");
            stream.addData(m_position, "Position");
            stream.addData(m_orientation, "Orientation");
            stream.addData(m_jacobianTranslation, "JacobianTranslation");
            stream.addData(m_jacobianRotation, "JacobianRotation");
            stream.addData(m_jacobianTaskSpace, "JacobianTaskSpace");
            stream.addData(m_conditionIndex, "ConditionIndex");
            stream.addData(m_manipulabilityMeasure, "ManipulabilityMeasure");
            stream.addData(m_jointRangeAvailability, "JointRangeAvailability");
        }

        // Getters
        // -------
    public:
        //! Unique index of this sample
        const uint64_t& sampleIndex() const { return m_sampleIndex; }

        //! Configuration of kinematic chain [rad for rotational DoFs, m for translational DoFs]
        const Eigen::VectorXd& configuration() const { return m_configuration; }

        //! Absolute position of the tcp frame (relative to world frame)
        const Eigen::Vector3d& position() const { return m_position; }

        //! Absolute orientation of the tcp frame (relative to world frame)
        const Eigen::Quaterniond& orientation() const { return m_orientation; }

        //! Absolute jacobian of translation of tcp frame (relative to world frame)
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobianTranslation() const { return m_jacobianTranslation; }

        //! Absolute jacobian of rotation of tcp frame (relative to world frame)
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& jacobianRotation() const { return m_jacobianRotation; }

        //! Absolute jacobian of task space of tcp frame (relative to world frame)
        const Eigen::MatrixXd& jacobianTaskSpace() const { return m_jacobianTaskSpace; }

        //! Condition index
        /*!
         * The (reciprocal) condition of the task-space jacobian.
         * \f[ CI(q) = \frac{\sigma_{J,min}}{\sigma_{J,max}} \f]
         * According to Neuburger, N., Kinematic Structure Optimization for Humanoid Robots, Technical University of Munich, 2019, Master's thesis, https://mediatum.ub.tum.de/1523789
         */
        const double& conditionIndex() const { return m_conditionIndex; }

        //! Manipulability measure
        /*!
         * The manipulability measure with respect to the task-space jacobian.
         * \f[ MM(q) = \left\lbrace \begin{array}{ll}\sqrt{ \det(J J^T) } = \prod_i \sigma_{J,i} & \mbox{for } m \leq n\\ 0 & \mbox{for } m > n \end{array} \right. \f]
         * According to Yoshikawa, T., Manipulability of Robotic Mechanisms, The International Journal of Robotics Research, 1985, doi: 10.1177/027836498500400201
         */
        const double& manipulabilityMeasure() const { return m_manipulabilityMeasure; }

        //! Joint range availability
        /*!
         * \f[ JRA(q) = \sqrt[n]{\prod_{i=1}^n \frac{\min(q_{max,i}-q_i,\,q_i-q_{min,i})}{\frac{1}{2}(q_{max,i}-q_{min,i})}} \f]
         * According to Neuburger, N., Kinematic Structure Optimization for Humanoid Robots, Technical University of Munich, 2019, Master's thesis, https://mediatum.ub.tum.de/1523789
         */
        const double& jointRangeAvailability() const { return m_jointRangeAvailability; }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace analysis
} // namespace broccoli

#endif // HAVE_EIGEN3
