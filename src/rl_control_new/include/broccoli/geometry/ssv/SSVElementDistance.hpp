/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include <Eigen/Dense>
#include <array>

namespace broccoli {
namespace geometry {
    //! Contains the results of a distance evaluation between two SSV elements (see \ref broccoli::geometry::SSVDistanceEvaluator)
    /*!
     * \ingroup broccoli_geometry_ssv
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVElementDistance {
    public:
        //! Default constructor
        SSVElementDistance() = default;

        // Specialized constructor
        //! \copydoc set()
        SSVElementDistance(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint, const double& firstRadius, const double& secondRadius, const bool& flip = false)
            : m_firstPoint(flip ? secondPoint : firstPoint)
            , m_secondPoint(flip ? firstPoint : secondPoint)
            , m_firstRadius(flip ? secondRadius : firstRadius)
            , m_secondRadius(flip ? firstRadius : secondRadius)
        {
            // Compute remaining parameters
            update();
        }

        // Members
        // -------
    protected:
        Eigen::Vector3d m_firstPoint = Eigen::Vector3d::Zero(); //!< \copybrief firstPoint()
        Eigen::Vector3d m_secondPoint = Eigen::Vector3d::Zero(); //!< \copybrief secondPoint()
        double m_firstRadius = 0.0; //!< \copybrief firstRadius()
        double m_secondRadius = 0.0; //!< \copybrief secondRadius()
        Eigen::Vector3d m_connection = Eigen::Vector3d::Zero(); //!< \copybrief connection()
        double m_connectionLength = 0.0; //!< \copybrief connectionLength()
        double m_distance = 0.0; //!< \copybrief distance()

        // Getters
        // -------
    public:
        //! Closest point on base geometry (point, line, triangle) of **first** SSV element (**not** affected by the radius of the SSV element)
        inline const Eigen::Vector3d& firstPoint() const { return m_firstPoint; }

        //! Closest point on base geometry (point, line, triangle) of **second** SSV element (**not** affected by the radius of the SSV element)
        inline const Eigen::Vector3d& secondPoint() const { return m_secondPoint; }

        //! Radius of the **first** SSV element
        inline const double& firstRadius() const { return m_firstRadius; }

        //! Radius of the **second** SSV element
        inline const double& secondRadius() const { return m_secondRadius; }

        //! Connection vector from firstPoint() to secondPoint() (**not** affected by the radius of the SSV elements)
        inline const Eigen::Vector3d& connection() const { return m_connection; }

        //! Length of connection() (**not** affected by the radius of the SSV elements)
        inline const double& connectionLength() const { return m_connectionLength; }

        //! Actual distance between the two evaluated SSV elements (takes the radius of the SSV elements into account) (negative in case of intersection)
        inline const double& distance() const { return m_distance; }

        // Setters
        // -------
    public:
        //! Sets the members of this class by the given parameters (missing members are computed)
        /*!
         * \param [in] firstPoint Sets \ref firstPoint() - \copybrief firstPoint()
         * \param [in] secondPoint Sets \ref secondPoint() - \copybrief secondPoint()
         * \param [in] firstRadius Sets \ref firstRadius() - \copybrief firstRadius()
         * \param [in] secondRadius Sets \ref secondRadius() - \copybrief secondRadius()
         * \param [in] flip If `true`, the direction of the distance is flipped (swapping first and second element)
         */
        inline void set(const Eigen::Vector3d& firstPoint, const Eigen::Vector3d& secondPoint, const double& firstRadius, const double& secondRadius, const bool& flip = false)
        {
            // Copy input parameters (optional: flip)
            if (flip == false) {
                m_firstPoint = firstPoint;
                m_secondPoint = secondPoint;
                m_firstRadius = firstRadius;
                m_secondRadius = secondRadius;
            } else {
                m_firstPoint = secondPoint;
                m_secondPoint = firstPoint;
                m_firstRadius = secondRadius;
                m_secondRadius = firstRadius;
            }

            // Compute remaining parameters
            update();
        }

        //! Replaces *this* by *other*, if *other* (distance) is smaller (to get the minimum of both)
        inline void minimum(const SSVElementDistance& other)
        {
            if (other.distance() < distance())
                *this = other;
        }

        // Helpers
        // -------
    protected:
        //! Computes the "internal" parameters
        inline void update()
        {
            m_connection = m_secondPoint - m_firstPoint;
            m_connectionLength = m_connection.norm();
            m_distance = m_connectionLength - m_firstRadius - m_secondRadius;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
