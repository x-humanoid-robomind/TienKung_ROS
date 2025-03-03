/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif
#include "../Signal.hpp"
#include <cassert>
#include <utility>

namespace broccoli {
namespace control {

    namespace internal {
        /*!
         * \brief Implementation of \ref BlendingScheme
         * \tparam AlphaType Derived type of the \f$\alpha\f$ signal
         * \tparam AType Derived type of the A signal
         * \tparam BType Derived type of the B signal
         */
        template <typename AlphaType, typename AType, typename BType>
        class BlendingSchemeImpl : public SignalBase<BlendingSchemeImpl<AlphaType, AType, BType>> {
        public:
            /*!
             * \brief Construct from signals
             * \param alpha Blending factor [0...1]
             * \param inputA A input
             * \param inputB B input
             */
            BlendingSchemeImpl(AlphaType&& alpha, AType&& inputA, BType&& inputB)
                : m_alpha(std::forward<AlphaType>(alpha))
                , m_A(std::forward<AType>(inputA))
                , m_B(std::forward<BType>(inputB))
            {

                assert(m_alpha.sampleTime() < 0.0 || m_A.sampleTime() < 0.0 || m_alpha.sampleTime() == m_A.sampleTime() && "You are mixing signals with different sample times! (alpha and operand A)");

                assert(m_alpha.sampleTime() < 0.0 || m_B.sampleTime() < 0.0 || m_alpha.sampleTime() == m_B.sampleTime() && "You are mixing signals with different sample times! (alpha and operand B)");

                assert(std::min(m_alpha.sampleTime(), m_A.sampleTime()) < 0.0 || std::min(m_alpha.sampleTime(), m_B.sampleTime()) < 0.0 || std::min(m_alpha.sampleTime(), m_A.sampleTime()) == std::min(m_alpha.sampleTime(), m_B.sampleTime()) && "You are mixing signals with different sample times!");
            }

            //! \copydoc SignalBase::value()
            auto value() const
            {
                const auto alphaEval = m_alpha.value();
                return alphaEval * m_A.value() + (1.0 - alphaEval) * m_B.value();
            }

            //! \copydoc SignalBase::sampleTime()
            double sampleTime() const
            {
                return m_alpha.sampleTime() < 0.0 ? (m_A.sampleTime() < 0.0 ? m_B.sampleTime() : m_A.sampleTime()) : m_alpha.sampleTime();
            }

        protected:
            const AlphaType m_alpha;
            const AType m_A;
            const BType m_B;
        };

        template <typename Alpha, typename TypeA, typename TypeB>
        auto blendingSchemeHelper(Alpha&& alpha, TypeA&& a, TypeB&& b)
        {
            return internal::BlendingSchemeImpl<Alpha, TypeA, TypeB>(std::forward<Alpha>(alpha), std::forward<TypeA>(a), std::forward<TypeB>(b));
        }
    } // namespace internal

    /*!
     * \brief Continuously blends between two signals.
     * \ingroup broccoli_control_operators
     *
     * Returns a signal corresponding to
     * \f{align*}{
     * \alpha \cdot A + (1-\alpha) \cdot B
     * \f}
     * where \f$\alpha \in [0, 1]\f$ and A, B are arbitrary signals.
     *
     * \param alpha The Signal for \f$\alpha\f$
     * \param inputA The Signal for A
     * \param inputB The Signal for B
     * \returns Signal for \f$\alpha * A + (1-\alpha) * B\f$
     */
    template <typename Alpha, typename A, typename B>
    auto BlendingScheme(Alpha&& alpha, A&& inputA, B&& inputB)
    {
        return internal::blendingSchemeHelper(mapSignal(std::forward<Alpha>(alpha)), mapSignal(std::forward<A>(inputA)), mapSignal(std::forward<B>(inputB)));
    }
} // namespace control
} // namespace broccoli
