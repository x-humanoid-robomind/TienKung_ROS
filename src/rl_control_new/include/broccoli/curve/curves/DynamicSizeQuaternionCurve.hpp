/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "QuaternionCurve.hpp"
#include <Eigen/StdVector>
#include <math.h>

namespace broccoli {
namespace curve {
    //! Class abstracting quaternion cuves of **dynamic size** (=varying count of control "point" quaternions)
    /*!
     * \ingroup broccoli_curve_curves
     *
     * \warning This class is restricted to **unit-quaternions**!
     */
    class DynamicSizeQuaternionCurve : public QuaternionCurve {
    public:
        //! Default constructor
        /*!
         * Initializes with pre-allocated list of quaternion control points set to the given initial value
         * \param [in] controlPointCount Count of control points to initialize (use 0 for empty list)
         * \param [in] initialValue Quaternion used to initialize **all** control point quaternions in the list
         */
        DynamicSizeQuaternionCurve(const size_t& controlPointCount = 0, const Eigen::Quaterniond& initialValue = Eigen::Quaterniond(1, 0, 0, 0))
        {
            // Check, if list of control point quaternions should be empty
            if (controlPointCount == 0) {
                // ...yes -> do nothing
            } else {
                // Allocate memory for control points
                m_controlPoints.resize(controlPointCount, initialValue);
            }
        }

        //! Destructor
        virtual ~DynamicSizeQuaternionCurve()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const DynamicSizeQuaternionCurve& reference) const
        {
            // Compare base class
            if (QuaternionCurve::operator!=(reference))
                return false;

            // Compare control point counts
            if (m_controlPoints.size() != reference.m_controlPoints.size()) {
                assert(false);
                return false;
            }

            // Compare control points
            for (size_t i = 0; i < m_controlPoints.size(); i++)
                if (fabs(2.0 * acos(m_controlPoints[i].dot(reference.m_controlPoints[i]))) > 1e-6 /* <- tolerance in [rad] */)
                    return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const DynamicSizeQuaternionCurve& reference) const { return !(*this == reference); }

        // Checks, if the curve is properly defined (see base class for details)
        virtual bool isValid() const
        {
            // Check base class validity
            if (QuaternionCurve::isValid() == false)
                return false;

            // Check, if there is at least one control point in the list
            if (m_controlPoints.size() == 0)
                return false;

            // No error -> valid
            return true;
        }

        // Member data
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> m_controlPoints; //!< List of quaternion control points (dynamic size)

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
