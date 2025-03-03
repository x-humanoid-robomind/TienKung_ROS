/*
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

// This class requires Eigen library
#ifdef HAVE_EIGEN3

#include "AbstractSolvableEntity.hpp"
#include <Eigen/Dense>
#include <stdexcept>

namespace broccoli {
namespace ode {
    /*!
     * \brief Abstract base class for purely time-discrete solvable entities.
     * \ingroup broccoli_ode_integration
     */
    class AbstractTimeDiscreteSolvableEntity : public AbstractSolvableEntity {
    public:
        //! Unused for this purely time-discrete entity
        Eigen::VectorXd& state() override
        {
            throw std::runtime_error("Internal Error");
        }
        //! Unused for this purely time-discrete entity
        const Eigen::VectorXd& stateGradient() const override
        {
            throw std::runtime_error("Internal Error");
        }
        //! Unused for this purely time-discrete entity
        void evaluateRHS() override
        {
            throw std::runtime_error("Internal Error");
        }
        //! Returns zero to indicate purely time-discrete behavior
        std::size_t stateVectorSize() const override
        {
            return 0;
        }
    };
} // namespace ode
} // namespace broccoli

#endif // HAVE_EIGEN3
