/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../../core/math.hpp"
#include "../CGMeshFactory.hpp"
#include "../CGMeshTools.hpp"
#include "SSVElement.hpp"

namespace broccoli {
namespace geometry {
    //! Representation of a SSV **line** element
    /*!
     * \ingroup broccoli_geometry_ssv
     * It is defined by: \f[ v_{0} = \left[\begin{array}{c} x_{0} \\ y_{0} \\ z_{0} \end{array}\right] \quad \mbox{,} \quad v_{1} = \left[\begin{array}{c} x_{1} \\ y_{1} \\ z_{1} \end{array}\right]
     * \quad \mbox{,} \quad e_{0} = v_{1} - v_{0} \quad \mbox{and} \quad r\f].
     *
     * Definition of a line element:
     * -----------------------------
     * \verbatim
     * +---------------------+
     * |                     |
     * |    v0---e0---v1     |
     * |                     |
     * +---------------------+
     * \endverbatim
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVLineElement : public SSVElement<2> {
    public:
        //! Default constructor (members remain uninitialized!)
        SSVLineElement() = default;

        //! Constructor
        /*!
         * \attention You can not change the points **after** initialization, just the radius can be changed.
         * You can change the whole element by scaling, rotating and translating.
         *
         * \param [in] point0 Position of the first point \f[ v_{0} = \left[\begin{array}{c} x_{0} \\ y_{0} \\ z_{0} \end{array}\right]\f]
         * \param [in] point1 Position of the second point \f[ v_{1} = \left[\begin{array}{c} x_{1} \\ y_{1} \\ z_{1} \end{array}\right]\f]
         * \param [in] radius Initializes \ref radius() - \copybrief radius()
         */
        SSVLineElement(const Eigen::Vector3d& point0, const Eigen::Vector3d& point1, const double& radius)
            : SSVElement(radius)
        {
            m_points[0] = point0;
            m_points[1] = point1;
            m_edges[0] = point1 - point0;
            assert(isValid());
        }

        //! Comparison operator: **equality**
        inline bool operator==(const SSVLineElement& reference) const
        {
            // Compare base class
            if (SSVElement::operator==(reference) == false)
                return false;

            // Compare members
            // ...

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const SSVLineElement& reference) const { return !(*this == reference); }

        // Setters
        // -------
    public:
        //! \copydoc SSVElement::scale(const double&)
        inline void scale(const double& scaling)
        {
            // Scale the base element
            SSVElement::scale(scaling);

            // Scale points
            m_points[0] *= scaling;
            m_points[1] *= scaling;

            // Scale edge
            m_edges[0] *= scaling;

            // Check validity of element after scaling
            assert(isValid());
        }

        //! \copydoc SSVElement::scale(const Eigen::Vector3d&)
        inline void scale(const Eigen::Vector3d& scaling)
        {
            // Scale the base element
            SSVElement::scale(scaling);

            // Component-wise scaling of points
            m_points[0].array() *= scaling.array();
            m_points[1].array() *= scaling.array();

            // Component-wise scaling of edge
            m_edges[0].array() *= scaling.array();

            // Check validity of element after scaling
            assert(isValid());
        }

        //! \copydoc SSVElement::rotate(const Eigen::Matrix3d&)
        inline void rotate(const Eigen::Matrix3d& rotation)
        {
            // Rotate the base element
            SSVElement::rotate(rotation);

            // Rotate points
            m_points[0] = (rotation * m_points[0]).eval();
            m_points[1] = (rotation * m_points[1]).eval();

            // Rotate edge
            m_edges[0] = (rotation * m_edges[0]).eval();
        }

        //! \copydoc SSVElement::translate(const Eigen::Vector3d&)
        inline void translate(const Eigen::Vector3d& translation)
        {
            // Translate the base element
            SSVElement::translate(translation);

            // Translate points
            m_points[0] += translation;
            m_points[1] += translation;

            // Edges are invariant to translation
        }

        // Helpers
        // -------
    public:
        //! Checks if the element is valid
        /*! \return `true`, if the element is valid, `false` otherwise. */
        inline bool isValid() const
        {
            // Check base class
            if (SSVElement::isValid() == false)
                return false;

            // Check, if both vertices coincide
            if (core::isEqual(m_points[0], m_points[1]) == true)
                return false;

            // No errors found -> valid
            return true;
        }

        // Serialization
        // -------------
    protected:
        // Deserialization of payload (see base class for details)
        io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            const io::serialization::BinaryStreamSize returnValue = SSVElement::deSerializePayload(stream, index, payloadSize, endianness);
            m_edges[0] = m_points[1] - m_points[0];
            return returnValue;
        }

        // Triangularization
        // -----------------
    public:
        //! Creates a triangle mesh representing this element
        /*!
         * \param [in] stepsPerPi Count of discretizing steps for approximating the angle \f$ \pi \f$, constraint: has to be greater than 1
         * \return Created mesh (empty mesh in case of an error, e.g. invalid input)
         */
        CGMesh createMesh(const uint64_t& stepsPerPi) const
        {
            // Check input
            if (stepsPerPi < 2) {
                assert(false);
                return CGMesh();
            }

            // Precompute helpers
            const double connectionLength = m_edges[0].norm();
            const Eigen::Vector3d connectionAxis = m_edges[0] / connectionLength;
            const Eigen::Vector3d perpendicularAxis = core::math::findPerpendicularVector(connectionAxis).normalized();
            CGMesh spherePoint0 = CGMeshFactory::createSphereSlice(m_radius, m_radius, 0, 0.5 * M_PI, 1.5 * M_PI, stepsPerPi, 0.0, M_PI, stepsPerPi);
            CGMesh spherePoint1 = spherePoint0; // Copy to reuse

            // Create mesh for sphere at point 0
            spherePoint0.changeCoordinateSystem(m_points[0], connectionAxis, perpendicularAxis);

            // Create mesh for sphere at point 1
            spherePoint1.changeCoordinateSystem(m_points[1], -connectionAxis, perpendicularAxis);

            // Create cylinder for edge between 0 and 1
            CGMesh cylinderPoint01 = CGMeshFactory::createCylinderSlice(m_radius, m_radius, 0, 0.0, 2.0 * M_PI, 2 * stepsPerPi, 0.0, connectionLength, 1);
            cylinderPoint01.changeCoordinateSystem(m_points[0], perpendicularAxis, connectionAxis);

            // Merge meshes
            std::vector<const CGMesh*> subMeshList;
            subMeshList.reserve(3);
            subMeshList.push_back(&spherePoint0);
            subMeshList.push_back(&spherePoint1);
            subMeshList.push_back(&cylinderPoint01);
            return CGMeshTools::merge(subMeshList);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
