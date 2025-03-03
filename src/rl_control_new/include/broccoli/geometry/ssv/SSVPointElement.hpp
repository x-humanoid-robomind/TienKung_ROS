/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../CGMeshFactory.hpp"
#include "SSVElement.hpp"

namespace broccoli {
namespace geometry {
    //! Representation of a SSV **point** element
    /*!
     * \ingroup broccoli_geometry_ssv
     * It is defined by:
     * \f[ p = \left[\begin{array}{c} x \\ y \\ z \end{array}\right] \quad \mbox{and} \quad r \f]
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVPointElement : public SSVElement<1> {
    public:
        //! Default constructor (members remain uninitialized!)
        SSVPointElement() = default;

        //! Constructor
        /*!
         * \attention You can not change the point **after** initialization, just the radius can be changed.
         * You can change the whole element by scaling, rotating and translating.
         *
         * \param [in] point0 Position of the point \f[ p = \left[\begin{array}{c} x \\ y \\ z \end{array}\right]\f]
         * \param [in] radius Initializes \ref radius() - \copybrief radius()
         */
        SSVPointElement(const Eigen::Vector3d& point0, const double& radius)
            : SSVElement(radius)
        {
            m_points[0] = point0;
            assert(isValid());
        }

        //! Comparison operator: **equality**
        inline bool operator==(const SSVPointElement& reference) const
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
        inline bool operator!=(const SSVPointElement& reference) const { return !(*this == reference); }

        // Setters
        // -------
    public:
        //! \copydoc SSVElement::scale(const double&)
        inline void scale(const double& scaling)
        {
            // Scale the base element
            SSVElement::scale(scaling);

            // Scale point
            m_points[0] *= scaling;

            // Check validity of element after scaling
            assert(isValid());
        }

        //! \copydoc SSVElement::scale(const Eigen::Vector3d&)
        inline void scale(const Eigen::Vector3d& scaling)
        {
            // Scale the base element
            SSVElement::scale(scaling);

            // Component-wise scaling of point
            m_points[0].array() *= scaling.array();

            // Check validity of element after scaling
            assert(isValid());
        }

        //! \copydoc SSVElement::rotate(const Eigen::Matrix3d&)
        inline void rotate(const Eigen::Matrix3d& rotation)
        {
            // Rotate the base element
            SSVElement::rotate(rotation);

            // Rotate point
            m_points[0] = (rotation * m_points[0]).eval();
        }

        //! \copydoc SSVElement::translate(const Eigen::Vector3d&)
        inline void translate(const Eigen::Vector3d& translation)
        {
            // Translate the base element
            SSVElement::translate(translation);

            // Translate point
            m_points[0] += translation;
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

            // No errors found -> valid
            return true;
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

            // Create mesh
            CGMesh mesh = CGMeshFactory::createSphere(m_radius, 2 * stepsPerPi, stepsPerPi);
            mesh.translate(m_points[0]);

            // Pass back created mesh
            return mesh;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
