/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "ForwardDeclarations.hpp"

namespace broccoli {
namespace control {

    namespace internal {

        template <typename ValueType>
        auto makeSignal(ValueType&& value, const double& sampleTime)
        {
            return Signal<ValueType>(std::forward<ValueType>(value), sampleTime);
        }

        template <typename ValueType>
        auto makeSignal(ValueType&& value)
        {
            return internal::SignalNoSampleTime<ValueType>(std::forward<ValueType>(value));
        }

        template <typename EigenType>
        auto evaluateEigenExpression(std::remove_reference_t<EigenType>& value) -> decltype(value.eval())
        {
            return value.eval();
        }

        template <typename EigenType>
        auto evaluateEigenExpression(std::remove_reference_t<EigenType>&& value)
        {
            // We need to make sure eval() on an rvalue always returns an rvalue
            // (even if it is a plain object)
            return std::move(std::move(value).eval());
        }

        template <typename ValueType, typename Enabler = void>
        struct MappingTraits {
            static auto mapSignal(ValueType&& value)
            {
                return internal::makeSignal(std::forward<ValueType>(value));
            }

            static auto mapSignal(ValueType&& value, const double& sampleTime)
            {
                return internal::makeSignal(std::forward<ValueType>(value), sampleTime);
            }
        };

        template <typename ValueType>
        struct MappingTraits<ValueType, typename std::enable_if_t<core::is_eigen_matrix<ValueType>::value>> {
            static auto mapSignal(ValueType&& value)
            {
                return internal::makeSignal(internal::evaluateEigenExpression<ValueType>(std::forward<ValueType>(value)));
            }

            static auto mapSignal(ValueType&& value, const double& sampleTime)
            {
                return internal::makeSignal(internal::evaluateEigenExpression<ValueType>(std::forward<ValueType>(value)), sampleTime);
            }
        };
    }

    /*!
    * \addtogroup broccoli_control
    * \{
    */

    /*!
     * \brief Maps a value as a Signal expression.
     *
     * Uses perfect forwarding. For lvalues only a reference to the
     * data is stored. If value already is a Signal expression, this template
     * returns it unchanged. Eigen expressions being mapped to a Signal are evaluated
     * first as Eigen does internally not use move semantics for its expressions.
     *
     * \tparam Type The deduced storage type
     * \param value The value to map
     * \return A signal expression mapping value by universal reference
     */
    template <typename Type, typename = std::enable_if_t<!core::is_signal<Type>::value>>
    auto mapSignal(Type&& value)
    {
        return internal::MappingTraits<Type>::mapSignal(std::forward<Type>(value));
    }

    template <typename Type, typename std::enable_if_t<core::is_signal<Type>::value, int> = 0>
    Type&& mapSignal(Type&& value)
    {
        return std::forward<Type>(value);
    }

    /*!
     * \brief Maps value and sample time as Signal expression.
     *
     * Uses perfect forwarding. For lvalues only a reference to the
     * data is stored.
     *
     * \tparam Type The deduced storage type
     * \param value The value to map
     * \param sampleTime The sample time of the new signal
     * \return A signal expression mapping value by universal reference
     */
    template <typename Type>
    auto mapSignal(Type&& value, const double& sampleTime)
    {
        return internal::MappingTraits<Type>::mapSignal(std::forward<Type>(value), sampleTime);
    }

    //!\}
} // namespace control
} // namespace broccoli
