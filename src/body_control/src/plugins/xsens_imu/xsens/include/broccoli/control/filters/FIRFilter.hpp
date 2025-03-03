/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#include "../../core/type_traits.hpp"
#include "../../memory/CircularAccessor.hpp"
#include "SignalFilter.hpp"
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif
#include <array>
#include <vector>

#pragma once

namespace broccoli {
namespace control {
    /*!
     * \brief Finite Impulse Response (FIR) Filter implementation
     * \ingroup broccoli_control_filters
     *
     * This digital FIR filter of length \f$n\f$ (order \f$n-1\f$) stores the last \f$n\f$ samples \f$[u_k \; u_{k-1} \; u_{k-2} \; ... \; u_{k-n+1}]\f$ and weighs them with
     * the filter coefficients \f$b \in \mathcal{R}^n\f$ to get the output value for the current step \f$k\f$:
     * \f[
     * y_k = b^T \, [u_k \; u_{k-1} \; u_{k-2} \; ... \; u_{k-n+1}]
     * \f]
     * \tparam T The signal data type
     * \tparam filterLength The length of the filter in samples. This is also the number of filter coefficients and equals the filter order + 1.
     */
    template <typename T, std::size_t filterLength>
    class FIRFilter : public SignalFilter<FIRFilter<T, filterLength>> {
    public:
        //! The length of the filter (in samples)
        static constexpr std::size_t length = filterLength;

        FIRFilter()
            : m_value(core::Traits<T>::zero())
        {
            m_samples.fill(0);
            assert(filterLength > 0);
        }

        /*!
         * \brief Prototype initialization constructor for complex data types
         * \param prototype Initial output value of the filter
         */
        explicit FIRFilter(const T& prototype)
            : m_value(prototype)
        {
            m_samples.fill(prototype);
            assert(filterLength > 0);
        }

        /*!
         * \brief (Re)-initializes the FIR filter
         * \param filterCoefficients FIR coefficients array. Start with the coefficient for the newest data sample
         * \returns A reference to this instance
         */
        auto& init(const std::array<double, filterLength>& filterCoefficients)
        {
            m_filterCoefficients = filterCoefficients;
            m_accessor.reset(0, 0);
            return *this;
        }

        /*!
         * \brief Resets the filter state and output to the given value
         * \param value The desired value for the filter
         * \returns A reference to this instance
         */
        auto& reset(const T& value)
        {
            m_value = value;
            m_accessor.reset(0, 0);
            m_samples.fill(value);
            return *this;
        }

        //! \copydoc SignalFilter::process()
        auto& process(const Signal<T>& input)
        {
            m_accessor.access(m_samples).pushFront(input.value());
            m_sampleTime = input.sampleTime();

            calculateFilterResponse();
            return *this;
        }

        //! \copydoc SignalBase::value()
        const T& value() const
        {
            return m_value;
        }

        //! \copydoc SignalBase::sampleTime()
        const double& sampleTime() const
        {
            return m_sampleTime;
        }

    protected:
        //! Calculate the filter response based on the last filterLength samples and their corresponding coefficients
        void calculateFilterResponse()
        {
            // There is always at least one element valid
            m_value = m_accessor.access(m_samples)[0] * m_filterCoefficients[0];
            for (size_t i = 1; i < m_accessor.access(m_samples).count(); i++) {
                m_value += m_accessor.access(m_samples)[i] * m_filterCoefficients[i];
            }
        }

        //! The current output value
        T m_value;

        //! The output sample time
        double m_sampleTime = -1.0;

        //! The filter coefficients. The coefficient for the oldest element comes first
        std::array<double, filterLength> m_filterCoefficients;

        //! The ring array for all sampled values of the given order
        std::array<T, filterLength> m_samples;

        //! Circular accessor for the stored samples
        memory::CircularAccessor m_accessor{ 0, 0 };

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
    };
} // namespace control
} // namespace broccoli
