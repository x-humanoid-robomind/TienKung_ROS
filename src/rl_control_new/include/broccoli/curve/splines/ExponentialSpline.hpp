/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../curves/ExponentialCurve.hpp"
#include "Spline.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a spline as concatenation of exponential curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref Spline and \ref ExponentialCurve for details.
     */
    class ExponentialSpline : public Spline<ExponentialCurve> {
    public:
        //! Constructor
        ExponentialSpline()
        {
        }

        //! Destructor
        virtual ~ExponentialSpline()
        {
        }
    };

} // namespace curve
} // namespace broccoli
