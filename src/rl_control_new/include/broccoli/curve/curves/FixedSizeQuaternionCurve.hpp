/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "QuaternionCurve.hpp"
#include <array>
#include <math.h>

namespace broccoli {
namespace curve {
    //! Class abstracting quaternion cuves of **fixed size** (=fixed count of control "point" quaternions)
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \warning This class is restricted to **unit-quaternions**!
     *
     * \tparam ControlPointCount Count of (quaternion) control points.
     */
    template <unsigned int ControlPointCount>
    class FixedSizeQuaternionCurve : public QuaternionCurve {
    public:
        //! Default constructor
        /*!
         * Initializes all control points with the given quaternion as initial value
         * \param [in] initialValue Quaternion used to initialize **all** control point quaternions
         */
        FixedSizeQuaternionCurve(const Eigen::Quaterniond& initialValue = Eigen::Quaterniond(1, 0, 0, 0))
        {
            // Set all control points to initial value
            m_controlPoints.fill(initialValue);
        }

        //! Destructor
        virtual ~FixedSizeQuaternionCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const FixedSizeQuaternionCurve& reference) const
        {
            // Compare base class
            if (QuaternionCurve::operator!=(reference))
                return false;

            // Compare control points
            for (size_t i = 0; i < m_controlPoints.size(); i++)
                if (fabs(2.0 * acos(m_controlPoints[i].dot(reference.m_controlPoints[i]))) > 1e-6 /* <- tolerance in [rad] */)
                    return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const FixedSizeQuaternionCurve& reference) const { return !(*this == reference); }

        // Member data
        std::array<Eigen::Quaterniond, ControlPointCount> m_controlPoints; //!< List of quaternion control points (fixed count)

        // Encoding
        // --------
        //! Encode XML attributes and add them to the specified stream
        /*!
         * \param [in,out] stream The stream to which the attributes should be appended to.
         * \param [in] numericFormat C-format specifier for numeric values.
         * \return Count of appended characters (elements in stream).
         */
        virtual io::encoding::CharacterStreamSize encodeXMLAttributes(broccoli::io::encoding::CharacterStream& stream, const std::string& numericFormat = "%.7g") const
        {
            // Write control point data
            io::encoding::CharacterStreamSize addedElements = io::encoding::encode(stream, " ControlPointsWXYZ=\"");
            for (size_t i = 0; i < m_controlPoints.size(); i++) {
                if (i > 0)
                    addedElements += io::encoding::encode(stream, (char)';');
                addedElements += io::encoding::encode(stream, m_controlPoints[i], numericFormat);
            }
            addedElements += io::encoding::encode(stream, (char)'\"');

            // Pass back added elements in stream
            return addedElements;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
