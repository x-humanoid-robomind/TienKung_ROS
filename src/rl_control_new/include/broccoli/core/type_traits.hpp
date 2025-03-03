/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include "../control/ForwardDeclarations.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif // HAVE_EIGEN3
#include <type_traits>

namespace broccoli {
namespace core {
    /*!
     * \addtogroup broccoli_core
     * \{
     */

    //! If T is an Eigen vector or matrix, this provides the member constant is_eigen_matrix::value equal true. For any other type, is_eigen_matrix::value is false.
    template <typename T>
#ifdef HAVE_EIGEN3
    struct is_eigen_matrix
        : std::is_base_of<Eigen::MatrixBase<typename std::decay<T>::type>, typename std::decay<T>::type> {
    };
#else
    struct is_eigen_matrix {
        static constexpr bool value = false;
    };
#endif

    //! If T is a broccoli Signal, this provides the member constant is_signal::value equal true. For any other type, is_signal::value is false.
    template <typename T>
    struct is_signal
        : std::is_base_of<control::internal::SignalBaseTrait, typename std::decay_t<T>> {
    };

    /*!
     * \brief Type traits for arithmetic types.
     *
     * \tparam T The Type
     * \tparam Enabler An enabler type for selective enabling of template specializations
     */
    template <typename T, typename Enabler = void>
    struct Traits {
        //! Returns an instance of T corresponding to a zero (0) value.
        static auto zero();
    };
    //! \}

    template <typename T>
    struct Traits<T, typename std::enable_if_t<std::is_fundamental<T>::value>> {
        static auto zero()
        {
            return T(0);
        }
    };

    template <typename T>
    struct Traits<T, typename std::enable_if_t<broccoli::core::is_eigen_matrix<T>::value && T::SizeAtCompileTime != Eigen::Dynamic>> {
        static auto zero()
        {
            return T::Zero();
        }
    };

    template <typename T>
    struct Traits<T, typename std::enable_if_t<broccoli::core::is_eigen_matrix<T>::value && T::SizeAtCompileTime == Eigen::Dynamic>> {
        static auto zero()
        {
            // For dynamic Eigen matrices the unknown dimension is set
            // to one to make zero() independent of the matrix type and
            // support basic properties of a null object.
            T result;

            // Preserves static dimensions
            if (T::RowsAtCompileTime != Eigen::Dynamic) {
                result.resize(T::RowsAtCompileTime, 1);
            } else if (T::ColsAtCompileTime != Eigen::Dynamic) {
                result.resize(1, T::ColsAtCompileTime);
            } else {
                result.resize(1, 1);
            }

            return result;
        }
    };
} // namespace core
} // namespace broccoli
