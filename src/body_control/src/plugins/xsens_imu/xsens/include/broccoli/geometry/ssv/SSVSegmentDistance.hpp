/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "SSVElementDistance.hpp"
#include "SSVSegment.hpp"

namespace broccoli {
namespace geometry {
    //! \copydoc broccoli::geometry::SSVElementDistance
    /*!
     * \ingroup broccoli_geometry_ssv
     * This class additionally stores the IDs of the corresponding SSVSegment%s
     *
     * \note
     * Original implementation by Poscher, Reinhold, "Effiziente Abstandsberechnungen mit Swept-Sphere Volumen für Echtzeit Kollisionsvermeidung in der Robotik", Technical University of Munich, 2020, Bachelor's thesis, https://mediatum.ub.tum.de/1580089
     */
    class SSVSegmentDistance : public SSVElementDistance {
    public:
        //! Default constructor
        SSVSegmentDistance() = default;

        //! Specialized constructor
        SSVSegmentDistance(const SSVElementDistance& elementDistance, const SSVSegment::ID& firstID, const SSVSegment::ID& secondID)
            : SSVElementDistance(elementDistance)
            , m_firstID(firstID)
            , m_secondID(secondID)
        {
        }

        //! Copy assignment operator (from base class)
        inline SSVSegmentDistance& operator=(const SSVElementDistance& reference)
        {
            SSVElementDistance::operator=(reference);
            return *this;
        }

        // Members
        // -------
    protected:
        SSVSegment::ID m_firstID = 0; //!< \copybrief firstID()
        SSVSegment::ID m_secondID = 0; //!< \copybrief secondID()

        // Getters
        // -------
    public:
        //! ID of the first segment
        inline const SSVSegment::ID& firstID() const { return m_firstID; }

        //! ID of the second segment
        inline const SSVSegment::ID& secondID() const { return m_secondID; }

        // Setters
        // -------
    public:
        //! Setter for firstID()
        inline void setFirstID(const SSVSegment::ID& firstID) { m_firstID = firstID; }

        //! Setter for secondID()
        inline void setSecondID(const SSVSegment::ID& secondID) { m_secondID = secondID; }

        //! Replaces *this* by *other*, if *other* (distance) is smaller (to get the minimum of both)
        inline void minimum(const SSVSegmentDistance& other)
        {
            if (other.distance() < distance())
                *this = other;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace geometry
} // namespace broccoli

#endif // HAVE_EIGEN3
