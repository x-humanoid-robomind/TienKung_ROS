/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../core/type_traits.hpp"
#include <Eigen/Dense>
#include <assert.h>

namespace broccoli {
namespace control {
    /*!
     * \brief Represents a 6D wrench (3D torque and 3D Force) at a specified reference point
     * \ingroup broccoli_control
     *
     * The reference point \f$r\f$ and the torque-/force vectors \f$T, F\f$ must be expressed in the same
     * coordinate system.
     *
     * \attention The reference point resulting from an addition or subtraction of wrenches with different
     * reference points is always the reference point of the left operand. Bear in mind that such operations are in general incorrect.
     */
    class Wrench {
    public:
        //! Default constructor
        Wrench()
        {
        }

        /*!
         * \brief Construct from 6d force/torque vector
         *
         * The reference point is initialized to zero.
         * \param vector Force/torque vector (fx, fy, fz, tx, ty, tz)
         */
        template <typename Derived>
        explicit Wrench(const Eigen::MatrixBase<Derived>& vector)
            : m_vector(vector)
            , m_referencePoint(Eigen::Vector3d::Zero())
        {
        }

        /*!
         * \brief Construct from separate force and torque vectors
         *
         * The reference point is initialized to zero.
         * \param forceVector Force vector (fx, fy, fz)
         * \param torqueVector Force vector (tx, ty, tz)
         */
        template <typename ForceDerived, typename TorqueDerived>
        explicit Wrench(const Eigen::MatrixBase<ForceDerived>& forceVector, const Eigen::MatrixBase<TorqueDerived>& torqueVector)
            : m_referencePoint(Eigen::Vector3d::Zero())
        {
            this->forceVector() = forceVector;
            this->torqueVector() = torqueVector;
        }

        //! Returns a zeroed wrench instance
        static Wrench Zero()
        {
            Wrench result;
            result.setZero();
            return result;
        }

        bool operator==(const Wrench& rhs) const
        {
            return m_vector == rhs.m_vector && m_referencePoint == rhs.m_referencePoint;
        }

        bool operator!=(const Wrench& rhs) const
        {
            return !(rhs == *this);
        }

        Wrench& operator+=(const Wrench& other)
        {
            this->vector() += other.vector();
            return *this;
        }

        Wrench& operator-=(const Wrench& other)
        {
            this->vector() -= other.vector();
            return *this;
        }

        Wrench& operator*=(const double& other)
        {
            this->vector() *= other;
            return *this;
        }

        Wrench& operator/=(const double& other)
        {
            this->vector() /= other;
            return *this;
        }

        Wrench operator-(const Wrench& other) const
        {
            Wrench result;
            result.vector() = this->vector() - other.vector();
            result.referencePoint() = this->referencePoint();
            return result;
        }

        Wrench operator+(const Wrench& other) const
        {
            Wrench result;
            result.vector() = this->vector() + other.vector();
            result.referencePoint() = this->referencePoint();
            return result;
        }

        Wrench operator*(const double& other) const
        {
            Wrench result;
            result.vector() = this->vector() * other;
            result.referencePoint() = this->referencePoint();
            return result;
        }

        Wrench operator/(const double& other) const
        {
            Wrench result;
            result.vector() = this->vector() / other;
            result.referencePoint() = this->referencePoint();
            return result;
        }

        //! Resets the value and reference point of this wrench to zero
        void setZero()
        {
            m_vector.setZero();
            m_referencePoint.setZero();
        }

        /*!
         * \brief Shifts the reference point of the wrench to the given new reference position
         *
         * This conserves the total effect of the wrench by calculating a new torque
         * \f$T = T + F \times (r_n - r)\f$ based on the old \f$r\f$ and new \f$r_n\f$ reference point.
         *
         * \param newReferencePoint The new reference point \f$r_n\f$.
         */
        template <typename Derived>
        Wrench& shiftReferenceTo(const Eigen::MatrixBase<Derived>& newReferencePoint)
        {
            torqueVector() += forceVector().cross(newReferencePoint - m_referencePoint);
            m_referencePoint = newReferencePoint;
            return *this;
        }

        /*!
         * \brief Creates a new Wrench by shifting the reference point of this wrench to the given new reference position
         *
         * This conserves the total effect of the wrench by calculating a new torque
         * \f$T = T + F \times (r_n - r)\f$ based on the old \f$r\f$ and new \f$r_n\f$ reference point.
         *
         * \param newReferencePoint The new reference point \f$r_n\f$.
         * \returns A copied wrench with shifted reference
         */
        template <typename Derived>
        Wrench shiftedTo(const Eigen::MatrixBase<Derived>& newReferencePoint) const
        {
            Wrench result(*this);
            result.shiftReferenceTo(newReferencePoint);
            return result;
        }

        /*!
         * \brief Shifts the reference point of the wrench by the given delta-vector \f$\Delta r\f$
         *
         * This conserves the total effect of the wrench by calculating a new torque
         * \f$T = T + F \times \Delta r\f$.
         *
         * \param shiftBy The delta vector for the reference point location \f$\Delta r\f$ (new - old)
         */
        template <typename Derived>
        Wrench& shiftReferenceBy(const Eigen::MatrixBase<Derived>& shiftBy)
        {
            torqueVector() += forceVector().cross(shiftBy);
            m_referencePoint += shiftBy;
            return *this;
        }

        /*!
         * \brief Creates a new Wrench by shifting the reference point of this wrench by the given delta-vector \f$\Delta r\f$
         *
         * This conserves the total effect of the wrench by calculating a new torque
         * \f$T = T + F \times \Delta r\f$.
         *
         * \param shiftBy The delta vector for the reference point location \f$\Delta r\f$ (new - old)
         * \returns A copied wrench with shifted reference
         */
        template <typename Derived>
        Wrench shiftedBy(const Eigen::MatrixBase<Derived>& shiftBy) const
        {
            Wrench result(*this);
            result.shiftReferenceBy(shiftBy);
            return result;
        }

        /*!
         * \brief Transforms the wrench to a new frame of reference
         * \param rotationMatrix The rotational matrix, which defines the rotation from the old to the new frame of reference.
         */
        template <typename Derived>
        Wrench& transformWith(const Eigen::MatrixBase<Derived>& rotationMatrix)
        {
            forceVector() = rotationMatrix * forceVector();
            torqueVector() = rotationMatrix * torqueVector();
            referencePoint() = rotationMatrix * referencePoint();
            return *this;
        }

        //! Returns a reference to the force vector of the wrench (x,y,z)
        Eigen::VectorBlock<Eigen::Matrix<double, 6, 1>, 3> forceVector()
        {
            return m_vector.segment<3>(0);
        }

        //! Returns a const reference to the force vector of the wrench (x,y,z)
        const Eigen::VectorBlock<const Eigen::Matrix<double, 6, 1>, 3> forceVector() const
        {
            return m_vector.segment<3>(0);
        }

        //! Returns a reference to the torque vector of the wrench (x,y,z)
        Eigen::VectorBlock<Eigen::Matrix<double, 6, 1>, 3> torqueVector()
        {
            return m_vector.segment<3>(3);
        }

        //! Returns a const reference to the torque vector of the wrench (x,y,z)
        const Eigen::VectorBlock<const Eigen::Matrix<double, 6, 1>, 3> torqueVector() const
        {
            return m_vector.segment<3>(3);
        }

        //! Returns a reference to the 6D vector of the wrench (Fx, Fy, Fz, Tx, Ty, Tz)
        Eigen::Matrix<double, 6, 1>& vector()
        {
            return m_vector;
        }

        //! Returns a const reference to the 6D vector of the wrench (Fx, Fy, Fz, Tx, Ty, Tz)
        const Eigen::Matrix<double, 6, 1>& vector() const
        {
            return m_vector;
        }

        //! Returns a reference to the reference point of the wrench (x,y,z)
        Eigen::Vector3d& referencePoint()
        {
            return m_referencePoint;
        }

        //! Returns a const reference to the reference point of the wrench (x,y,z)
        const Eigen::Vector3d& referencePoint() const
        {
            return m_referencePoint;
        }

    private:
        //! 6D force / torque vector (Fx, Fy, Fz, Tx, Ty, Tz)
        Eigen::Matrix<double, 6, 1> m_vector;

        //! 3D Reference point for the wrench (x,y,z)
        Eigen::Vector3d m_referencePoint;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline Wrench operator*(const double& scalar, const Wrench& wrench)
    {
        return wrench * scalar;
    }
} // namespace control

namespace core {
    //! Specialization of broccoli traits for Wrench
    template <>
    struct Traits<control::Wrench, void> {
        static auto zero()
        {
            return control::Wrench::Zero();
        }
    };
} // namespace core
} // namespace broccoli
