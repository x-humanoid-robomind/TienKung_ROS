/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../CGMeshFactory.hpp"
#include "../CGMeshTools.hpp"
#include "SSVElement.hpp"

namespace broccoli {
namespace geometry {
    //! Representation of a SSV **triangle** element
    /*!
     * \ingroup broccoli_geometry_ssv
     * It is defined by: \f[ v_{0} = \left[\begin{array}{c} x_{0} \\ y_{0} \\ z_{0} \end{array}\right] \quad \mbox{,} \quad v_{1} = \left[\begin{array}{c} x_{1} \\ y_{1} \\ z_{1} \end{array}\right]  \quad \mbox{,} \quad v_{2} = \left[\begin{array}{c} x_{2} \\ y_{2} \\ z_{2} \end{array}\right]
     * \quad \mbox{,} \quad e_{0} = v_{1} - v_{0} \quad \mbox{,} \quad e_{1} = v_{2} - v_{0} \quad \mbox{,} \quad e_{2} = v_{2} - v_{1} \quad \mbox{and} \quad r.\f]
     *
     * Definition of a triangle element:
     * ---------------------------------
     * \verbatim
     * +---------------------+
     * |                     |
     * |         v2          |
     * |         /\          |
     * |        /  \         |
     * |      e1    e2       |
     * |      /      \       |
     * |     /        \      |
     * |    v0---e0---v1     |
     * |                     |
     * +---------------------+
     * \endverbatim
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVTriangleElement : public SSVElement<3> {
    public:
        //! Default constructor (members remain uninitialized!)
        SSVTriangleElement() = default;

        //! Constructor
        /*!
         * \attention You can not change the points **after** initialization, just the radius can be changed.
         * You can change the whole element by scaling, rotating and translating.
         *
         * \param [in] point0 Position of the first point \f[ v_{0} = \left[\begin{array}{c} x_{0} \\ y_{0} \\ z_{0} \end{array}\right]\f]
         * \param [in] point1 Position of the second point \f[ v_{1} = \left[\begin{array}{c} x_{1} \\ y_{1} \\ z_{1} \end{array}\right]\f]
         * \param [in] point2 Position of the third point \f[ v_{2} = \left[\begin{array}{c} x_{2} \\ y_{2} \\ z_{2} \end{array}\right]\f]
         * \param [in] radius Initializes \ref radius() - \copybrief radius()
         */
        SSVTriangleElement(const Eigen::Vector3d& point0, const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const double& radius)
            : SSVElement(radius)
        {
            m_points[0] = point0;
            m_points[1] = point1;
            m_points[2] = point2;
            m_edges[0] = point1 - point0;
            m_edges[1] = point2 - point0;
            m_edges[2] = point2 - point1;
            computeNormal();
            computeMatrixS();
            assert(isValid());
        }

        //! Comparison operator: **equality**
        inline bool operator==(const SSVTriangleElement& reference) const
        {
            // Compare base class
            if (SSVElement::operator==(reference) == false)
                return false;

            // Compare members
            // Note: no comparison of the matrix S and the normal, since they are implicitly given through the other members

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const SSVTriangleElement& reference) const { return !(*this == reference); }

        // Members
        // -------
    protected:
        Eigen::Matrix<double, 2, 3> m_matrixS; //!< \copybrief matrixS()
        Eigen::Vector3d m_normal; //!< \copybrief normal()

        // Getters
        // -------
    public:
        //! Matrix \f$ S \in \mathbb{R}^{2\times 3}\f$
        /*!
         * The matrix \f$ S \f$ is used for the distance evaluation as an indicator for the different regions.
         * \f[S = (D^{T}D)^{-1}D^{T} \quad \mbox{with} \quad D = \left[\begin{array}{cc} e_{0} & e_{1} \end{array}\right]\f]
         */
        const Eigen::Matrix<double, 2, 3>& matrixS() const { return m_matrixS; }

        //! Normal vector \f$ n\f$
        /*! The normal vector is required for the distance evaluation. */
        const Eigen::Vector3d& normal() const { return m_normal; }

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
            m_points[2] *= scaling;

            // Scale edges
            m_edges[0] *= scaling;
            m_edges[1] *= scaling;
            m_edges[2] *= scaling;

            // The normal vector is invariant to uniform scaling (even it the scaling factor is negative!)

            // Scale the matrix S (because of its definition, the matrix S gets scaled by the inverse of the scaling factor)
            double scaling_inv = 1e9; // Reciprocal of scaling
            if (fabs(scaling) > 1e-9) // Avoid division by zero
                scaling_inv = 1.0 / scaling;
            m_matrixS *= scaling_inv;

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
            m_points[2].array() *= scaling.array();

            // Component-wise scaling of edges
            m_edges[0].array() *= scaling.array();
            m_edges[1].array() *= scaling.array();
            m_edges[2].array() *= scaling.array();

            // Recompute normal vector
            computeNormal();

            // Recompute matrix S
            computeMatrixS();

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
            m_points[2] = (rotation * m_points[2]).eval();

            // Rotate edges
            m_edges[0] = (rotation * m_edges[0]).eval();
            m_edges[1] = (rotation * m_edges[1]).eval();
            m_edges[2] = (rotation * m_edges[2]).eval();

            // Rotate the normal vector
            m_normal = (rotation * m_normal).eval();

            // Rotate matrix S
            /* Explanation
             * D   -> A * D
             * D^T -> (A * D)^T = D^T * A^T = D^T * A^(-1)
             * S -> (D^T * A^(-1) * A * D) * D^T * A^T = (D^T * D) * D^T * A^T = S * A^T
             */
            m_matrixS = (m_matrixS * rotation.transpose()).eval();
        }

        //! \copydoc SSVElement::translate(const Eigen::Vector3d&)
        inline void translate(const Eigen::Vector3d& translation)
        {
            // Translate the base element
            SSVElement::translate(translation);

            // Translate points
            m_points[0] += translation;
            m_points[1] += translation;
            m_points[2] += translation;

            // Edges are invariant to translation

            // The normal vector is invariant to translation

            // The matrix S is invariant to translation
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

            // Check, if two of the three points coincide
            if (m_edges[0].isZero() == true || m_edges[1].isZero() == true || m_edges[2].isZero() == true)
                return false;

            // Check, if all three points lie on one line
            const Eigen::Vector3d firstEdgeNormalized = m_edges[0].normalized();
            const Eigen::Vector3d neg_firstEdgeNormalized = -firstEdgeNormalized;
            const Eigen::Vector3d secondEdgeNormalized = m_edges[1].normalized();
            if (core::isEqual(firstEdgeNormalized, secondEdgeNormalized) == true || core::isEqual(neg_firstEdgeNormalized, secondEdgeNormalized) == true)
                return false;

            // No errors found -> valid
            return true;
        }

        // Internals
        // ---------
    private:
        //! (Re-)computes the normal vector
        /*! \f[n = \frac{e_{0} \times e_{1}}{||e_{0} \times e_{1}||} \f] */
        inline void computeNormal()
        {
            m_normal = (m_edges[0].cross(m_edges[1])).normalized();
        }

        //! Computes the matrix \f$ S \f$
        /*! \f[S = (D^{T}D)^{-1}D^{T} \quad \mbox{with} \quad D = \left[\begin{array}{cc} e_{0} & e_{1} \end{array}\right]\f] */
        inline void computeMatrixS()
        {
            // Define intermediate matrix X
            /* X = D^T * D
             *   = | a, b |
             *     | c, d |
             *
             * ...with a = e0^T * e0
             *         b = e0^T * e1
             *         c = e1^T * e0 = e0^T * e1 = b
             *         d = e1^T * e1
             */
            const double a = m_edges[0].dot(m_edges[0]);
            const double b = m_edges[0].dot(m_edges[1]);
            const double d = m_edges[1].dot(m_edges[1]);

            // Compute inverse of X
            /*
             * X^(-1) = 1 / (ad-bc) * |  d, -b | = | x1, x2 |
             *                        | -c,  a | = | x3, x4 |
             * ...with x1 = d / (ad-bc)
             *         x2 = -b / (ad-bc)
             *         x3 = -c / (ad-bc) = x2
             *         x4 = a / (ad-bc)
             */
            const double detX = a * d - b * b;
            const double x1 = d / detX;
            const double x2 = -b / detX;
            const double x4 = a / detX;

            // Explicit multiplication of X^(-1) * D^T to obtain S
            m_matrixS.row(0) = x1 * m_edges[0] + x2 * m_edges[1];
            m_matrixS.row(1) = x2 * m_edges[0] + x4 * m_edges[1];
        }

        // Serialization
        // -------------
    protected:
        // Deserialization of payload (see base class for details)
        io::serialization::BinaryStreamSize deSerializePayload(const io::serialization::BinaryStream& stream, const io::serialization::BinaryStreamSize& index, const io::serialization::BinaryStreamSize& payloadSize, const io::serialization::Endianness& endianness)
        {
            const io::serialization::BinaryStreamSize returnValue = SSVElement::deSerializePayload(stream, index, payloadSize, endianness);
            m_edges[0] = m_points[1] - m_points[0];
            m_edges[1] = m_points[2] - m_points[0];
            m_edges[2] = m_points[2] - m_points[1];
            computeNormal();
            computeMatrixS();
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
            std::array<double, 3> edgeLength;
            std::array<Eigen::Vector3d, 3> edgeNormalized;
            for (size_t i = 0; i < 3; i++) {
                edgeLength[i] = m_edges[i].norm();
                edgeNormalized[i] = m_edges[i] / edgeLength[i];
            }
            const std::array<double, 3> pointAngle{ { acos(edgeNormalized[0].dot(edgeNormalized[1])), acos(-edgeNormalized[0].dot(edgeNormalized[2])), acos(edgeNormalized[1].dot(edgeNormalized[2])) } };
            static constexpr double sphereMinimumPhi = 0.5 * M_PI;
            std::array<double, 3> sphereMaximumPhi;
            std::array<uint64_t, 3> sphereStepsPhi;
            for (size_t i = 0; i < 3; i++) {
                sphereMaximumPhi[i] = 1.5 * M_PI - pointAngle[i];
                sphereStepsPhi[i] = std::round((sphereMaximumPhi[i] - sphereMinimumPhi) / M_PI * stepsPerPi);
                if (sphereStepsPhi[i] < 2)
                    sphereStepsPhi[i] = 2;
            }

            // Create mesh for sphere at point 0
            CGMesh spherePoint0 = CGMeshFactory::createSphereSlice(m_radius, m_radius, 0, sphereMinimumPhi, sphereMaximumPhi[0], sphereStepsPhi[0], 0.0, M_PI, stepsPerPi);
            spherePoint0.changeCoordinateSystem(m_points[0], edgeNormalized[1], m_normal);

            // Create mesh for sphere at point 1
            CGMesh spherePoint1 = CGMeshFactory::createSphereSlice(m_radius, m_radius, 0, sphereMinimumPhi, sphereMaximumPhi[1], sphereStepsPhi[1], 0.0, M_PI, stepsPerPi);
            spherePoint1.changeCoordinateSystem(m_points[1], -edgeNormalized[0], m_normal);

            // Create mesh for sphere at point 2
            CGMesh spherePoint2 = CGMeshFactory::createSphereSlice(m_radius, m_radius, 0, sphereMinimumPhi, sphereMaximumPhi[2], sphereStepsPhi[2], 0.0, M_PI, stepsPerPi);
            spherePoint2.changeCoordinateSystem(m_points[2], -edgeNormalized[2], m_normal);

            // Create cylinder around edge 0
            CGMesh cylinderEdge0 = CGMeshFactory::createCylinderSlice(m_radius, m_radius, 0, 0.0, M_PI, stepsPerPi, 0.0, edgeLength[0], 1);
            cylinderEdge0.changeCoordinateSystem(m_points[0], m_normal, edgeNormalized[0]);

            // Create cylinder around edge 1
            CGMesh cylinderEdge1 = CGMeshFactory::createCylinderSlice(m_radius, m_radius, 0, 0.0, M_PI, stepsPerPi, 0.0, edgeLength[1], 1);
            cylinderEdge1.changeCoordinateSystem(m_points[2], m_normal, -edgeNormalized[1]);

            // Create cylinder around edge 2
            CGMesh cylinderEdge2 = CGMeshFactory::createCylinderSlice(m_radius, m_radius, 0, 0.0, M_PI, stepsPerPi, 0.0, edgeLength[2], 1);
            cylinderEdge2.changeCoordinateSystem(m_points[1], m_normal, edgeNormalized[2]);

            // Create top and bottom caps (triangles)
            CGMesh topBottomCaps;
            topBottomCaps.m_vertexBuffer.resize(Eigen::NoChange, 6);
            topBottomCaps.m_normalBuffer.resize(Eigen::NoChange, 6);
            for (int i = 0; i < 3; i++) {
                topBottomCaps.m_vertexBuffer.col(i) = m_points[i] + m_radius * m_normal;
                topBottomCaps.m_normalBuffer.col(i) = m_normal;
                topBottomCaps.m_vertexBuffer.col(3 + i) = m_points[i] - m_radius * m_normal;
                topBottomCaps.m_normalBuffer.col(3 + i) = -m_normal;
            }
            topBottomCaps.m_indexBuffer.resize(3, 2);
            topBottomCaps.m_indexBuffer << 0, 3,
                1, 5, //
                2, 4;

            // Merge meshes
            std::vector<const CGMesh*> subMeshList;
            subMeshList.reserve(7);
            subMeshList.push_back(&spherePoint0);
            subMeshList.push_back(&spherePoint1);
            subMeshList.push_back(&spherePoint2);
            subMeshList.push_back(&cylinderEdge0);
            subMeshList.push_back(&cylinderEdge1);
            subMeshList.push_back(&cylinderEdge2);
            subMeshList.push_back(&topBottomCaps);
            return CGMeshTools::merge(subMeshList);
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
