/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "ForwardDeclarations.hpp"
#include "operators/BinaryOperator.hpp"
#include "operators/UnaryOperator.hpp"
#include "signal_helpers.hpp"
#include <ostream>

namespace broccoli {
namespace control {

    namespace internal {
        //! A trait used to recognize signals
        struct SignalBaseTrait {
        };
    }

    /*!
     * \brief A base class for a Signal
     * \tparam Derived The derived class type
     * \ingroup broccoli_control
     */
    template <typename Derived>
    class SignalBase : public internal::SignalBaseTrait {
    public:
        //! Returns a reference to the derived instance
        Derived& derived()
        {
            return *static_cast<Derived*>(this);
        }

        //! Returns a const reference to the derived instance
        const Derived& derived() const
        {
            return *static_cast<const Derived*>(this);
        }

        //! Returns the value of the signal
        auto value() const
        {
            return derived().value();
        }

        //! Returns the sample time of the signal in seconds
        auto sampleTime() const
        {
            return derived().sampleTime();
        }

        //! Force evaluation and return new Signal from expression
        auto evaluate() const
        {
            return mapSignal(value(), sampleTime());
        }

        /*!
         * \brief Returns the minimum of this and other as signal expression
         * \tparam TypeB Other type
         * \param other The other signal
         * \return min(this, other) as signal expression.
         */
        template <typename TypeB>
        auto min(TypeB&& other) const&
        {
            return internal::binaryOpHelper<internal::MinFunctor>(derived(), mapSignal(std::forward<TypeB>(other)));
        }

        /*!
         * \brief Returns the minimum of this and other as signal expression
         * \tparam TypeB Other type
         * \param other The other signal
         * \return min(this, other) as signal expression.
         */
        template <typename TypeB>
        auto min(TypeB&& other) const&&
        {
            return internal::binaryOpHelper<internal::MinFunctor>(std::move(derived()), mapSignal(std::forward<TypeB>(other)));
        }

        /*!
         * \brief Returns the maximum of this and other as signal expression
         * \tparam TypeB Other type
         * \param other The other signal
         * \return max(this, other) as signal expression.
         */
        template <typename TypeB>
        auto max(TypeB&& other) const&
        {
            return internal::binaryOpHelper<internal::MaxFunctor>(derived(), mapSignal(std::forward<TypeB>(other)));
        }

        /*!
         * \brief Returns the maximum of this and other as signal expression
         * \tparam TypeB Other type
         * \param other The other signal
         * \return max(this, other) as signal expression.
         */
        template <typename TypeB>
        auto max(TypeB&& other) const&&
        {
            return internal::binaryOpHelper<internal::MaxFunctor>(std::move(derived()), mapSignal(std::forward<TypeB>(other)));
        }

        /*!
         * \brief Returns the element-wise product of this and other as signal expression.
         * \tparam TypeB Other type
         * \param other The other signal
         * \return Element-wise product as signal expression.
         */
        template <typename TypeB>
        auto elementWiseProduct(TypeB&& other) const&
        {
            return internal::binaryOpHelper<internal::ElementwiseProductFunctor>(derived(), mapSignal(std::forward<TypeB>(other)));
        }

        /*!
         * \brief Returns the element-wise product of this and other as signal expression.
         * \tparam TypeB Other type
         * \param other The other signal
         * \return Element-wise product as signal expression.
         */
        template <typename TypeB>
        auto elementWiseProduct(TypeB&& other) const&&
        {
            return internal::binaryOpHelper<internal::ElementwiseProductFunctor>(std::move(derived()), mapSignal(std::forward<TypeB>(other)));
        }

        //! Outputs the signal value to the given stream
        friend std::ostream& operator<<(std::ostream& os, const SignalBase& base)
        {
            os << "<broccoli signal dt=" << base.sampleTime() << ">\n"
               << base.value() << "\n</broccoli signal>";
            return os;
        }
    };

} // namespace control
} // namespace broccoli
