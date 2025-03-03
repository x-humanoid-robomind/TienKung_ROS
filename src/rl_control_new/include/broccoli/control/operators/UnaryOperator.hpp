/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../signal_helpers.hpp"
#include <utility>

namespace broccoli {
namespace control {

    namespace internal {

        //! A functor for an unary minus
        template <typename Operand, typename Enabler = void>
        struct MinusFunctor {
            static auto result(const Operand& a)
            {
                return -a;
            }
        };

        /*!
         * \brief Signal expression for a unary operation on Operand
         *
         * \tparam Functor A callable functor template for the unary operation
         * \tparam Operand Type of the signal operand
         */
        template <template <typename...> class Functor, typename Operand>
        class UnaryOperator : public SignalBase<UnaryOperator<Functor, Operand>> {
        public:
            /*!
             * \brief A constructor
             * \param op The operand to modify
             */
            UnaryOperator(Operand&& op)
                : m_op(std::forward<Operand>(op))
            {
            }

            //! \copydoc SignalBase::value()
            auto value() const
            {
                return Functor<decltype(m_op.value())>::result(m_op.value());
            }

            //! \copydoc SignalBase::sampleTime()
            double sampleTime() const
            {
                return m_op.sampleTime();
            }

        private:
            const Operand m_op;
        };
    } // namespace internal

    template <typename TypeA, typename = std::enable_if_t<core::is_signal<TypeA>::value>>
    auto operator-(TypeA&& a)
    {
        return internal::UnaryOperator<internal::MinusFunctor, TypeA>(std::forward<TypeA>(a));
    }

    template <typename TypeA, typename = std::enable_if_t<core::is_signal<TypeA>::value>>
    TypeA&& operator+(TypeA&& a)
    {
        return std::forward<TypeA>(a);
    }

} // namespace control
} // namespace broccoli
