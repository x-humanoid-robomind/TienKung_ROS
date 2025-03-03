/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "PolynomialTrajectory.hpp"
#include "TrajectorySplitter.hpp"

namespace broccoli {
namespace curve {
    //! Helper class which allows to split and join **polynomial trajectories**
    /*!
     * \ingroup broccoli_curve_splines
     *
     * \tparam Degree (Maximum) degree \f$ p \geq 0 \f$
     * \tparam Dimension The count of splines related to this trajectory (size of spline-array).
     */
    template <unsigned int Degree, unsigned int Dimension>
    class PolynomialTrajectorySplitter : public TrajectorySplitter<PolynomialTrajectory<Degree, Dimension>, PolynomialSpline<Degree>, PolynomialCurve<Degree>> {
    public:
        //! Constructor
        PolynomialTrajectorySplitter()
        {
        }

        //! Destructor
        virtual ~PolynomialTrajectorySplitter()
        {
        }
    };
} // namespace curve
} // namespace broccoli
