/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../core/floats.hpp"
#include <Eigen/Dense>
#include <math.h>

namespace broccoli {
namespace geometry {
    /*!
     * \addtogroup broccoli_geometry
     * \{
     */

    //! Representation of a point in spherical coordinates
    class SphericalCoordinates {
        // Construction
        // ------------
    public:
        //! Default constructor
        SphericalCoordinates() = default;

        //! Specialized constructor
        /*!
         * \param [in] radius Initializes \ref radius() - \copybrief radius()
         * \param [in] phi Initializes \ref phi() - \copybrief phi()
         * \param [in] theta Initializes \ref theta() - \copybrief theta()
         * \param [in] normalized If `true` \ref normalize() is called
         */
        SphericalCoordinates(const double& radius, const double& phi, const double& theta, const bool& normalized = false)
            : m_radius(radius)
            , m_phi(phi)
            , m_theta(theta)
        {
            if (normalized == true)
                normalize();
            else {
                m_cosPhi = cos(m_phi);
                m_sinPhi = sin(m_phi);
                m_cosTheta = cos(m_theta);
                m_sinTheta = sin(m_theta);
            }
        }

        //! Comparison operator: **equality**
        inline bool operator==(const SphericalCoordinates& reference) const { return m_radius == reference.m_radius && m_phi == reference.m_phi && m_theta == reference.m_theta; }

        //! Comparison operator: **inequality**
        inline bool operator!=(const SphericalCoordinates& reference) const { return !(*this == reference); }

        //! Comparison operator: **equivalence** (compares normalized coordinates)
        inline bool isEquivalent(const SphericalCoordinates& reference) const { return (this->normalized() == reference.normalized()); }

        // Members
        // -------
    protected:
        // Core
        double m_radius = 0.0; //!< \copybrief radius()
        double m_phi = 0.0; //!< \copybrief phi()
        double m_theta = 0.0; //!< \copybrief theta()
        // Buffers
        double m_cosPhi = 1.0; //!< \copybrief cosPhi()
        double m_sinPhi = 0.0; //!< \copybrief sinPhi()
        double m_cosTheta = 1.0; //!< \copybrief cosTheta()
        double m_sinTheta = 0.0; //!< \copybrief sinTheta()

        // Getters
        // -------
    public:
        //! Radius \f$ r \f$ (typically \f$ r \geq 0 \f$)
        inline const double& radius() const { return m_radius; }
        //! Angle (rad) \f$ \varphi \f$ (in x-y plane, starting from positive x-axis, around positive z-axis) (typically \f$ \varphi \in [0, 2\pi[ \f$)
        inline const double& phi() const { return m_phi; }
        //! Angle (rad) \f$ \vartheta \f$ (between z-axis and vector from origin to point \f$x\f$) (typically \f$ \vartheta \in [0, \pi] \f$)
        inline const double& theta() const { return m_theta; }
        //! \f$ \cos(\varphi) \f$
        inline const double& cosPhi() const { return m_cosPhi; }
        //! \f$ \sin(\varphi) \f$
        inline const double& sinPhi() const { return m_sinPhi; }
        //! \f$ \cos(\vartheta) \f$
        inline const double& cosTheta() const { return m_cosTheta; }
        //! \f$ \sin(\vartheta) \f$
        inline const double& sinTheta() const { return m_sinTheta; }

        // Setters
        // -------
    public:
        //! Setter for \ref radius()
        inline void setRadius(const double& value) { m_radius = value; }

        //! Setter for \ref phi()
        inline void setPhi(const double& value)
        {
            m_phi = value;
            m_cosPhi = cos(m_phi);
            m_sinPhi = sin(m_phi);
        }

        //! Setter for \ref theta()
        inline void setTheta(const double& value)
        {
            m_theta = value;
            m_cosTheta = cos(m_theta);
            m_sinTheta = sin(m_theta);
        }

        //! Normalizes the coordinates such that all parameters remain within their typical bounds
        /*! \note The represented point stays the same! */
        inline void normalize()
        {
            // Special case: zero radius
            if (core::isZero(m_radius)) {
                m_radius = 0.0;
                m_phi = 0.0;
                m_cosPhi = 1.0;
                m_sinPhi = 0.0;
                m_theta = 0.0;
                m_cosTheta = 1.0;
                m_sinTheta = 0.0;
                return;
            }

            // Normalize radius
            if (m_radius < 0.0) {
                m_radius = -m_radius;
                m_theta += M_PI;
            }

            // Normalize theta
            m_theta = fmod(m_theta, 2.0 * M_PI);
            if (m_theta < 0.0)
                m_theta += 2.0 * M_PI;
            if (m_theta > M_PI) {
                m_theta = 2.0 * M_PI - m_theta;
                m_phi += M_PI;
            }
            m_cosTheta = cos(m_theta);
            m_sinTheta = sin(m_theta);

            // Special case: point on z-axis
            if (core::isZero(m_theta) || core::isZero(m_theta - M_PI)) {
                m_phi = 0.0;
                m_cosPhi = 1.0;
                m_sinPhi = 0.0;
                return;
            }

            // Normalize phi
            m_phi = fmod(m_phi, 2.0 * M_PI);
            if (m_phi < 0.0)
                m_phi += 2.0 * M_PI;
            m_cosPhi = cos(m_phi);
            m_sinPhi = sin(m_phi);
        }

        //! Returns a normalized copy of the coordinates such that all parameters remain within their typical bounds
        /*! \note The represented point stays the same! */
        inline SphericalCoordinates normalized() const { return SphericalCoordinates(m_radius, m_phi, m_theta, true); }

        // Transformation
        // --------------
    public:
        //! Computes the equivalent cartesian coordinates (base)
        inline Eigen::Vector3d toCartesian() const
        {
            const double radius_sinTheta = m_radius * m_sinTheta;
            return { radius_sinTheta * m_cosPhi, radius_sinTheta * m_sinPhi, m_radius * m_cosTheta };
        }

        //! Computes the equivalent cartesian coordinates (**first** derivative with respect to \f$r\f$)
        inline Eigen::Vector3d toCartesianFirstDerivativeRadius() const { return { m_cosPhi * m_sinTheta, m_sinPhi * m_sinTheta, m_cosTheta }; }

        //! Computes the equivalent cartesian coordinates (**first** **normalized** derivative with respect to \f$r\f$) (provides proper normal even for \f$ r = 0 \f$)
        inline Eigen::Vector3d toCartesianFirstDerivativeRadiusNormalized() const { return toCartesianFirstDerivativeRadius(); /* <-- length 1 already guaranteed! */ }

        //! Computes the equivalent cartesian coordinates (**first** derivative with respect to \f$\varphi\f$)
        inline Eigen::Vector3d toCartesianFirstDerivativePhi() const
        {
            const double radius_sinTheta = m_radius * m_sinTheta;
            return { -radius_sinTheta * m_sinPhi, radius_sinTheta * m_cosPhi, 0.0 };
        }

        //! Computes the equivalent cartesian coordinates (**first** **normalized** derivative with respect to \f$\varphi\f$) (provides proper normal even for \f$ r = 0 \f$)
        inline Eigen::Vector3d toCartesianFirstDerivativePhiNormalized() const { return { -m_sinPhi, m_cosPhi, 0.0 }; }

        //! Computes the equivalent cartesian coordinates (**first** derivative with respect to \f$\vartheta\f$)
        inline Eigen::Vector3d toCartesianFirstDerivativeTheta() const
        {
            const double radius_cosTheta = m_radius * m_cosTheta;
            return { radius_cosTheta * m_cosPhi, radius_cosTheta * m_sinPhi, -m_radius * m_sinTheta };
        }

        //! Computes the equivalent cartesian coordinates (**first** **normalized** derivative with respect to \f$\vartheta\f$) (provides proper normal even for \f$ r = 0 \f$)
        inline Eigen::Vector3d toCartesianFirstDerivativeThetaNormalized() const { return { m_cosTheta * m_cosPhi, m_cosTheta * m_sinPhi, -m_sinTheta }; }

        //! Computes the equivalent spherical coordinates (normalized)
        static inline SphericalCoordinates fromCartesian(const Eigen::Vector3d& cartesian)
        {
            const double radius = cartesian.norm();
            double phi = 0.0;
            double theta = 0.0;
            if (!core::isZero(radius)) {
                theta = acos(cartesian.z() / radius);
                if (!core::isZero(theta) && !core::isZero(theta - M_PI)) {
                    phi = atan2(cartesian.y(), cartesian.x());
                    if (phi < 0.0)
                        phi += 2.0 * M_PI;
                }
            }
            return SphericalCoordinates(radius, phi, theta, false);
        }
    };
    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
