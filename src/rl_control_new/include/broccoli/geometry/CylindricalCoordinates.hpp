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

    //! Representation of a point in cylindrical coordinates
    class CylindricalCoordinates {
        // Construction
        // ------------
    public:
        //! Default constructor
        CylindricalCoordinates() = default;

        //! Specialized constructor
        /*!
         * \param [in] radius Initializes \ref radius() - \copybrief radius()
         * \param [in] phi Initializes \ref phi() - \copybrief phi()
         * \param [in] z Initializes \ref z() - \copybrief z()
         * \param [in] normalized If `true` \ref normalize() is called
         */
        CylindricalCoordinates(const double& radius, const double& phi, const double& z, const bool& normalized = false)
            : m_radius(radius)
            , m_phi(phi)
            , m_z(z)
        {
            if (normalized == true)
                normalize();
            else {
                m_cosPhi = cos(m_phi);
                m_sinPhi = sin(m_phi);
            }
        }

        //! Comparison operator: **equality**
        inline bool operator==(const CylindricalCoordinates& reference) const { return m_radius == reference.m_radius && m_phi == reference.m_phi && m_z == reference.m_z; }

        //! Comparison operator: **inequality**
        inline bool operator!=(const CylindricalCoordinates& reference) const { return !(*this == reference); }

        //! Comparison operator: **equivalence** (compares normalized coordinates)
        inline bool isEquivalent(const CylindricalCoordinates& reference) const { return (this->normalized() == reference.normalized()); }

        // Members
        // -------
    protected:
        // Core
        double m_radius = 0.0; //!< \copybrief radius()
        double m_phi = 0.0; //!< \copybrief phi()
        double m_z = 0.0; //!< \copybrief z()
        // Buffers
        double m_cosPhi = 1.0; //!< \copybrief cosPhi()
        double m_sinPhi = 0.0; //!< \copybrief sinPhi()

        // Getters
        // -------
    public:
        //! Radius \f$ r \f$ (typically \f$ r \geq 0 \f$)
        inline const double& radius() const { return m_radius; }
        //! Angle (rad) \f$ \varphi \f$ (in x-y plane, starting from positive x-axis, around positive z-axis) (typically \f$ \varphi \in [0, 2\pi[ \f$)
        inline const double& phi() const { return m_phi; }
        //! \f$ z \f$-coordinate
        inline const double& z() const { return m_z; }
        //! \f$ \cos(\varphi) \f$
        inline const double& cosPhi() const { return m_cosPhi; }
        //! \f$ \sin(\varphi) \f$
        inline const double& sinPhi() const { return m_sinPhi; }

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

        //! Setter for \ref z()
        inline void setZ(const double& value) { m_z = value; }

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
                return;
            }

            // Normalize radius
            if (m_radius < 0.0) {
                m_radius = -m_radius;
                m_phi += M_PI;
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
        inline CylindricalCoordinates normalized() const { return CylindricalCoordinates(m_radius, m_phi, m_z, true); }

        // Transformation
        // --------------
    public:
        //! Computes the equivalent cartesian coordinates (base)
        inline Eigen::Vector3d toCartesian() const { return { m_radius * m_cosPhi, m_radius * m_sinPhi, m_z }; }

        //! Computes the equivalent cartesian coordinates (**first** derivative with respect to \f$r\f$)
        /*! \attention The coordinates may be normalized with \ref normalize() before to give reasonable results! */
        inline Eigen::Vector3d toCartesianFirstDerivativeRadius() const { return { m_cosPhi, m_sinPhi, 0.0 }; }

        //! Computes the equivalent cartesian coordinates (**first** **normalized** derivative with respect to \f$r\f$) (provides proper normal even for \f$ r = 0 \f$)
        /*! \attention The coordinates may be normalized with \ref normalize() before to give reasonable results! */
        inline Eigen::Vector3d toCartesianFirstDerivativeRadiusNormalized() const { return toCartesianFirstDerivativeRadius(); /* <-- length 1 already guaranteed! */ }

        //! Computes the equivalent cartesian coordinates (**first** derivative with respect to \f$\varphi\f$)
        /*! \attention The coordinates may be normalized with \ref normalize() before to give reasonable results! */
        inline Eigen::Vector3d toCartesianFirstDerivativePhi() const { return { -m_radius * m_sinPhi, m_radius * m_cosPhi, 0.0 }; }

        //! Computes the equivalent cartesian coordinates (**first** **normalized** derivative with respect to \f$\varphi\f$) (provides proper normal even for \f$ r = 0 \f$)
        /*! \attention The coordinates may be normalized with \ref normalize() before to give reasonable results! */
        inline Eigen::Vector3d toCartesianFirstDerivativePhiNormalized() const { return { -m_sinPhi, m_cosPhi, 0.0 }; }

        //! Computes the equivalent cartesian coordinates (**first** derivative with respect to \f$z\f$)
        /*! \attention The coordinates may be normalized with \ref normalize() before to give reasonable results! */
        inline Eigen::Vector3d toCartesianFirstDerivativeZ() const { return Eigen::Vector3d::UnitZ(); }

        //! Computes the equivalent cartesian coordinates (**first** **normalized** derivative with respect to \f$z\f$) (provides proper normal even for \f$ r = 0 \f$)
        /*! \attention The coordinates may be normalized with \ref normalize() before to give reasonable results! */
        inline Eigen::Vector3d toCartesianFirstDerivativeZNormalized() const { return toCartesianFirstDerivativeZ(); /* <-- length 1 already guaranteed! */ }

        //! Computes the equivalent cylindrical coordinates (normalized)
        static inline CylindricalCoordinates fromCartesian(const Eigen::Vector3d& cartesian)
        {
            const double radius = sqrt(cartesian.x() * cartesian.x() + cartesian.y() * cartesian.y());
            double phi = 0.0;
            if (!core::isZero(radius)) {
                phi = atan2(cartesian.y(), cartesian.x());
                if (phi < 0.0)
                    phi += 2.0 * M_PI;
            }
            return CylindricalCoordinates(radius, phi, cartesian.z(), false);
        }
    };
    //! \}
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
