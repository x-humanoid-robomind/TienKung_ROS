/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/floats.hpp"
#include "../../io/serialization/serialization.hpp"
#include <Eigen/Dense>
#include <array>
#include <assert.h>

namespace broccoli {
namespace geometry {
    //! Representation of a generic SSV element
    /*!
     * \ingroup broccoli_geometry_ssv
     * In general, a SSVElement is described by a 2-dimensional geometry and a sphere.\n
     * A SSVElement contains, depending on the geometry, one to three points \f$ p \in \mathbb{R}^{3 \times 1} \f$ and a radius \f$ r >= 0 \f$ defining the sphere.
     * The common members and functions for all three elements, as well as a function to check for valid element construction are defined in this class.
     *
     * Contains common properties of:
     *  * \ref SSVPointElement \f$ (P=1,\,E=0) \f$
     *  * \ref SSVLineElement \f$ (P=2,\,E=1) \f$
     *  * \ref SSVTriangleElement \f$ (P=3,\,E=3) \f$
     *
     * \tparam P Number of points (1, 2 or 3)
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    template <unsigned int P>
    class SSVElement : public io::serialization::SerializableData {
        // Check point count
        static_assert((P >= 1 && P <= 3), "Point count has to be 1, 2 or 3!");
        static constexpr unsigned int E = (P == 1) ? 0 : (P == 2) ? 1 : 3; //!< Number of edges

    public:
        //! Default constructor (members remain uninitialized!)
        SSVElement() = default;

        //! Constructor for a generic SSV element
        /*! \param [in] radius Initializes \ref radius() - \copybrief radius() */
        SSVElement(const double& radius)
            : m_radius(radius)
        {
            assert(isValid());
        }

        //! Comparison operator: **equality**
        inline bool operator==(const SSVElement& reference) const
        {
            // Compare members
            for (size_t i = 0; i < m_points.size(); i++) {
                if (core::isEqual(m_points[i], reference.m_points[i]) == false)
                    return false;
            }
            if (core::isEqual(m_radius, reference.m_radius) == false)
                return false;
            // Note: no comparison of the edges, since they are implicitly given through the other members

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const SSVElement& reference) const { return !(*this == reference); }

        // Members
        // -------
    protected:
        std::array<Eigen::Vector3d, P> m_points; //!< Array containing \f$ P \f$ vectors \f$ \in \mathbb{R}^{3 \times 1} \f$ representing the points
        std::array<Eigen::Vector3d, E> m_edges; //!< Array containing \f$ E \f$ vectors \f$ \in \mathbb{R}^{3 \times 1} \f$ representing the edges
        double m_radius; //!< \copybrief radius()

        // Getters
        // -------
    public:
        //! Returns a reference to the first point (point "0") of the element
        inline const Eigen::Vector3d& point0() const
        {
            return m_points[0];
        }

        //! Returns a reference to the second point (point "1") of the element (only for SSVLineElement and SSVTriangleElement)
        inline const Eigen::Vector3d& point1() const
        {
            static_assert(P > 1, "Point count has to be greater than 1!");
            return m_points[1];
        }

        //! Returns a reference to the third point (point "2") of the element (only for SSVTriangleElement)
        inline const Eigen::Vector3d& point2() const
        {
            static_assert(P > 2, "Point count has to be greater than 2!");
            return m_points[2];
        }

        //! Returns a reference to the first edge (edge "0") of the element (only for SSVLineElement and SSVTriangleElement)
        inline const Eigen::Vector3d& edge0() const
        {
            return m_edges[0];
        }

        //! Returns a reference to the second edge (edge "1") of the element (only for SSVTriangleElement)
        inline const Eigen::Vector3d& edge1() const
        {
            static_assert(E > 1, "Edge count has to be greater than 1!");
            return m_edges[1];
        }

        //! Returns a reference to the third edge (edge "2") of the element (only for SSVTriangleElement)
        inline const Eigen::Vector3d& edge2() const
        {
            static_assert(E > 2, "Edge count has to be greater than 2!");
            return m_edges[2];
        }

        //! Radius of the swept sphere volume (has to be >= 0)
        inline const double& radius() const { return m_radius; }

        // Setters
        // -------
    public:
        //! Setter for \ref radius() - \copybrief radius()
        inline void setRadius(const double& radius)
        {
            assert(radius >= 0.0);
            m_radius = radius;
        }

        //! Applies the given **uniform** scaling to the element
        /*!
         * \f[ x_{new} = s \cdot x_{old} \f]
         *
         * \note The radius of the element will also be scaled. In case of a negative scaling factor, the radius will be scaled by the positive factor.
         *
         * \param [in] scaling The scaling factor \f$ s \neq 0 \f$
         */
        inline void scale(const double& scaling)
        {
            // Scaling factor equal to zero is not valid
            assert(core::isZero(scaling) == false);

            // Scale the radius (avoid negative scaling)
            m_radius *= fabs(scaling);

            // Check validity of element after scaling
            assert(isValid());
        }

        //! Applies the given arbitrary scaling to the element
        /*!
         * \f[ x_{new} = \left[\begin{array}{ccc} s_x & 0 & 0 \\ 0 & s_y & 0\\ 0 & 0 & s_z \end{array}\right] \cdot x_{old}\f]
         *
         * \note The radius of the element will **NOT** be scaled!
         *
         * \param [in] scaling The scaling vector \f$ [s_x,\,s_y,\,s_z] \f$
         */
        inline void scale(const Eigen::Vector3d& scaling) { (void)scaling; }

        //! Applies the given rotation to the element
        /*!
         * \f[ x_{new} = A \cdot x_{old} \f]
         *
         * \param [in] rotation The rotation matrix \f$ A \in \mathbb{R}^{3\times 3}\f$
         */
        inline void rotate(const Eigen::Matrix3d& rotation) { (void)rotation; }

        //! Applies the given translation to the element
        /*!
         * \f[ x_{new} = x_{old} + t \f]
         *
         * \param [in] translation The translation vector \f$ t \f$
         */
        inline void translate(const Eigen::Vector3d& translation) { (void)translation; }

        // Helpers
        // -------
    public:
        //! Checks if the element is valid
        /*! \return `true`, if the element is valid, `false` otherwise. */
        inline bool isValid() const
        {
            // Check, if radius is valid
            if (m_radius < 0.0)
                return false;

            // No errors found -> valid
            return true;
        }

        // Serialization
        // -------------
    protected:
        // Serialization of payload (see base class for details)
        io::serialization::BinaryStreamSize serializePayload(io::serialization::BinaryStream& stream, const io::serialization::Endianness& endianness) const
        {
            return io::serialization::serialize(stream, endianness, m_points, m_radius);
        }

        // Deserialization of payload (see base class for details)
        io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            (void)payloadSize; // unused
            return io::serialization::deSerialize(stream, index, endianness, m_points, m_radius);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
