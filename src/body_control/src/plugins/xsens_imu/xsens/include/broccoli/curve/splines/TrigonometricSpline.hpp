/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../curves/TrigonometricCurve.hpp"
#include "Spline.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a spline as concatenation of trigonometric curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref Spline and \ref TrigonometricCurve for details.
     */
    class TrigonometricSpline : public Spline<TrigonometricCurve> {
    public:
        //! Constructor
        TrigonometricSpline()
        {
        }

        //! Destructor
        virtual ~TrigonometricSpline()
        {
        }
    };

} // namespace curve
} // namespace broccoli
