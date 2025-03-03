/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../core/type_traits.hpp"
#include "SignalBase.hpp"

#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace control {

    namespace internal {

        /*!
         * \brief Internal storage type for Signal
         * \tparam Derived The derived type
         * \tparam Type The storage type
         */
        template <typename Derived, typename Type>
        class SignalStorage : public SignalBase<Derived> {
        public:
            /*!
             * \brief Created mapped (reference) signal from value
             * \param value The desired signal value
             */
            explicit SignalStorage(std::remove_reference_t<Type>&& value)
                : m_value(std::move(value))
            {
            }

            explicit SignalStorage(std::remove_reference_t<Type>& value)
                : m_value(value)
            {
            }

            /*!
             * \brief Copy-assign from other signal
             * \tparam Derived Derived type of the other signal
             * \param other The source signal
             * \return A reference to this signal instance
             */
            template <typename DerivedOther>
            SignalStorage<Derived, Type>& operator=(const SignalBase<DerivedOther>& other)
            {
                m_value = other.value();
                return *this;
            }

            //! Returns the value of the signal
            const std::decay_t<Type>& value() const
            {
                return m_value;
            }

            //! Returns the value of the signal
            std::remove_reference_t<Type>& value()
            {
                return m_value;
            }

            //! Returns the sample time of the signal in seconds
            double sampleTime() const
            {
                return -1.0;
            }

        private:
            //! Reference / copy of the signal value
            Type m_value;

#ifdef HAVE_EIGEN3
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
        };

        /*!
         * \brief Internal type for a Signal without explicit sample time. Only used from mapSignal().
         * \tparam Type The value storage type
         */
        template <typename Type>
        class SignalNoSampleTime : public SignalStorage<SignalNoSampleTime<Type>, Type> {
        public:
            //! \copydoc SignalStorage::SignalStorage
            explicit SignalNoSampleTime(Type&& value)
                : SignalStorage<SignalNoSampleTime<Type>, Type>(std::forward<Type>(value))
            {
            }
        };
    }

    /*!
     * \brief A class template for signals in broccoli
     * \ingroup broccoli_control
     *
     * Represents the signal type for all signal-processing modules in the \ref broccoli_control module.
     * Stores a signal value and sample time. When a Signal is constructed without explicitly given
     * sample time (or a negative sample time), the sample time is derived from other signals of an expression.
     * Note that only a forward evaluation is done to calculate the sample time, no back propagation. At least one
     * signal in an expression must have an explicitly set sample time for all expressions to work correctly.
     *
     * \tparam Type The signal value type
     */
    template <typename Type>
    class Signal : public internal::SignalStorage<Signal<Type>, Type> {
    public:
        using Base = internal::SignalStorage<Signal<Type>, Type>;

        /*!
         * \brief Default constructor. Primitive data types are initialized with 0
         */
        Signal()
            : Base(core::Traits<Type>::zero())
            , m_sampleTime(-1.0)
        {
        }

        /*!
         * \brief Copy construct from value with implicit sample time.
         * \param value The desired signal value
         */
        explicit Signal(std::remove_reference_t<Type>& value)
            : Base(value)
            , m_sampleTime(-1.0)
        {
        }

        /*!
         * \brief Move construct from rvalue with implicit sample time.
         * \param value The desired signal value
         */
        explicit Signal(std::remove_reference_t<Type>&& value)
            : Base(std::move(value))
            , m_sampleTime(-1.0)
        {
        }

        /*!
         * \brief Copy construct from rvalue with explicit sample time.
         * \param value The desired signal value
         * \param sampleTime The sample time in seconds
         */
        Signal(std::remove_reference_t<Type>& value, const double& sampleTime)
            : Base(value)
            , m_sampleTime(sampleTime)
        {
        }

        /*!
         * \brief Move construct from rvalue with explicit sample time.
         * \param value The desired signal value
         * \param sampleTime The sample time in seconds
         */
        Signal(std::remove_reference_t<Type>&& value, const double& sampleTime)
            : Base(std::move(value))
            , m_sampleTime(sampleTime)
        {
        }

        /*!
         * \brief Copy construct from other Signal
         * \tparam Derived Derived type of the other signal
         * \param otherSignal The signal to copy
         */
        template <typename Derived>
        Signal(const SignalBase<Derived>& otherSignal)
            : Base(otherSignal.value())
            , m_sampleTime(otherSignal.sampleTime())
        {
        }

        /*!
         * \brief Move construct from other Signal
         * \tparam Derived Derived type of the other signal
         * \param otherSignal The signal to move
         */
        template <typename Derived>
        Signal(SignalBase<Derived>&& otherSignal)
            : Base(std::move(otherSignal.value()))
            , m_sampleTime(std::move(otherSignal.sampleTime()))
        {
        }

        /*!
         * \brief Copy-assign from other signal
         * \tparam Derived Derived type of the other signal
         * \param other The source signal
         * \return A reference to this signal instance
         */
        template <typename Derived>
        Signal<Type>& operator=(const SignalBase<Derived>& other)
        {
            Base::operator=(other);
            m_sampleTime = other.sampleTime();
            return *this;
        }

        //! Returns the sample time of the signal in seconds
        const double& sampleTime() const
        {
            return m_sampleTime;
        }

        //! Returns the sample time of the signal in seconds
        double& sampleTime()
        {
            return m_sampleTime;
        }

    private:
        //! The sample time of the signal
        double m_sampleTime;
    };

} // namespace control
} // namespace broccoli
