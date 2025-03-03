/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../core/type_traits.hpp"
#include "../signal_helpers.hpp"
#include <cassert>
#include <utility>

namespace broccoli {
namespace control {

    namespace internal {

        //! A functor for a sum
        template <typename LHS, typename RHS, typename Enabler = void>
        struct SumFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return a + b;
            }
        };

        //! A functor for a difference
        template <typename LHS, typename RHS, typename Enabler = void>
        struct DifferenceFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return a - b;
            }
        };

        //! A functor for a product
        template <typename LHS, typename RHS, typename Enabler = void>
        struct ProductFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return a * b;
            }
        };

        //! A functor for a quotient
        template <typename LHS, typename RHS, typename Enabler = void>
        struct QuotientFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return a / b;
            }
        };

        //! A functor for an element-wise product
        template <typename LHS, typename RHS, typename Enabler = void>
        struct ElementwiseProductFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return a * b;
            }
        };

        //! A functor for an element-wise product - Eigen specialization
        template <typename LHS, typename RHS>
        struct ElementwiseProductFunctor<LHS, RHS, typename std::enable_if_t<core::is_eigen_matrix<LHS>::value || core::is_eigen_matrix<RHS>::value>> {
            static auto result(const LHS& a, const RHS& b)
            {
                return a.cwiseProduct(b);
            }
        };

        //! A functor for min()
        template <typename LHS, typename RHS, typename Enabler = void>
        struct MinFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return std::min(a, b);
            }
        };

        //! A functor for min() - Eigen specialization
        template <typename LHS, typename RHS>
        struct MinFunctor<LHS, RHS, typename std::enable_if_t<core::is_eigen_matrix<LHS>::value || core::is_eigen_matrix<RHS>::value>> {

            template <typename Derived>
            static auto result(const Eigen::MatrixBase<Derived>& a, const RHS& b)
            {
                return a.cwiseMin(b);
            }

            template <typename DerivedLHS, typename DerivedRHS, typename = std::enable_if_t<!core::is_eigen_matrix<DerivedLHS>::value>>
            static auto result(const DerivedLHS& a, const Eigen::MatrixBase<DerivedRHS>& b)
            {
                return b.cwiseMin(a);
            }
        };

        //! A functor for max()
        template <typename LHS, typename RHS, typename Enabler = void>
        struct MaxFunctor {
            static auto result(const LHS& a, const RHS& b)
            {
                return std::max(a, b);
            }
        };

        //! A functor for max() - Eigen specialization
        template <typename LHS, typename RHS>
        struct MaxFunctor<LHS, RHS, typename std::enable_if_t<core::is_eigen_matrix<LHS>::value || core::is_eigen_matrix<RHS>::value>> {

            template <typename Derived>
            static auto result(const Eigen::MatrixBase<Derived>& a, const RHS& b)
            {
                return a.cwiseMax(b);
            }

            template <typename DerivedLHS, typename DerivedRHS, typename = std::enable_if_t<!core::is_eigen_matrix<DerivedLHS>::value>>
            static auto result(const DerivedLHS& a, const Eigen::MatrixBase<DerivedRHS>& b)
            {
                return b.cwiseMax(a);
            }
        };

        /*!
         * \brief Signal expression for a binary operation on LHS, RHS
         *
         * \tparam Functor A callable functor template for the binary operation
         * \tparam LHS Type of the left hand side signal
         * \tparam RHS Type of the right hand side signal
         */
        template <template <typename, typename...> class Functor, typename LHS, typename RHS>
        class BinaryOperator : public SignalBase<BinaryOperator<Functor, LHS, RHS>> {
        public:
            /*!
             * \brief A constructor
             * \param lhs The left hand side
             * \param rhs The right hand side
             */
            BinaryOperator(LHS&& lhs, RHS&& rhs)
                : m_lhs(std::forward<LHS>(lhs))
                , m_rhs(std::forward<RHS>(rhs))
            {
                assert(m_lhs.sampleTime() < 0.0 || m_rhs.sampleTime() < 0.0 || m_lhs.sampleTime() == m_rhs.sampleTime() && "You are mixing signals with different sample times!");
            }

            //! \copydoc SignalBase::value()
            auto value() const
            {
                return Functor<decltype(m_lhs.value()), decltype(m_rhs.value())>::result(m_lhs.value(), m_rhs.value());
            }

            //! \copydoc SignalBase::sampleTime()
            double sampleTime() const
            {
                return m_lhs.sampleTime() < 0.0 ? m_rhs.sampleTime() : m_lhs.sampleTime();
            }

        protected:
            const LHS m_lhs;
            const RHS m_rhs;
        };

        template <template <typename, typename...> class Functor, typename TypeA, typename TypeB>
        auto binaryOpHelper(TypeA&& a, TypeB&& b)
        {
            return internal::BinaryOperator<Functor, TypeA, TypeB>(std::forward<TypeA>(a), std::forward<TypeB>(b));
        }

    } // namespace internal

    template <typename TypeA, typename TypeB, typename = std::enable_if_t<core::is_signal<TypeA>::value || core::is_signal<TypeB>::value>>
    auto operator+(TypeA&& a, TypeB&& b)
    {
        return internal::binaryOpHelper<internal::SumFunctor>(mapSignal(std::forward<TypeA>(a)), mapSignal(std::forward<TypeB>(b)));
    }

    template <typename TypeA, typename TypeB, typename = std::enable_if_t<core::is_signal<TypeA>::value || core::is_signal<TypeB>::value>>
    auto operator-(TypeA&& a, TypeB&& b)
    {
        return internal::binaryOpHelper<internal::DifferenceFunctor>(mapSignal(std::forward<TypeA>(a)), mapSignal(std::forward<TypeB>(b)));
    }

    template <typename TypeA, typename TypeB, typename = std::enable_if_t<core::is_signal<TypeA>::value || core::is_signal<TypeB>::value>>
    auto operator*(TypeA&& a, TypeB&& b)
    {
        return internal::binaryOpHelper<internal::ProductFunctor>(mapSignal(std::forward<TypeA>(a)), mapSignal(std::forward<TypeB>(b)));
    }

    template <typename TypeA, typename TypeB, typename = std::enable_if_t<core::is_signal<TypeA>::value || core::is_signal<TypeB>::value>>
    auto operator/(TypeA&& a, TypeB&& b)
    {
        return internal::binaryOpHelper<internal::QuotientFunctor>(mapSignal(std::forward<TypeA>(a)), mapSignal(std::forward<TypeB>(b)));
    }

} // namespace control
} // namespace broccoli
