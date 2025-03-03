/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "PolynomialSpline.hpp"
#include "SplineSplitter.hpp"

namespace broccoli {
namespace curve {
    //! Helper class which allows to split and join **polynomial splines**
    /*!
     * \ingroup broccoli_curve_splines
     *
     * \tparam Degree (Maximum) degree \f$ p \geq 0 \f$
     */
    template <unsigned int Degree>
    class PolynomialSplineSplitter : public SplineSplitter<PolynomialSpline<Degree>, PolynomialCurve<Degree>> {
    public:
        //! Constructor
        PolynomialSplineSplitter()
        {
        }

        //! Destructor
        virtual ~PolynomialSplineSplitter()
        {
        }
    };
} // namespace curve
} // namespace broccoli
