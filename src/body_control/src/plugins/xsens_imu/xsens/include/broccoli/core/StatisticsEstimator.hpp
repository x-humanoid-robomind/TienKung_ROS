/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "type_traits.hpp"
#include <cmath>
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace core {

    /*!
     * \brief Abstract base class of StatisticsEstimator
     * \ingroup broccoli_core
     * \tparam T The signal type
     */
    template <typename T>
    class BaseStatisticsEstimator {
    public:
        /*!
         * \brief A constructor
         * \param zeroPrototype A prototype corresponding to a zero value
         */
        explicit BaseStatisticsEstimator(T&& zeroPrototype)
            : m_prototype(std::forward<T>(zeroPrototype))
            , m_mean(zeroPrototype)
            , m_standardDeviation(zeroPrototype)
            , m_max(zeroPrototype)
            , m_min(zeroPrototype)
            , m_sumOfSquaredDifferences(zeroPrototype)
        {
        }

        virtual ~BaseStatisticsEstimator() = default;

        /*!
         * \brief Updates stored statistical information by adding a new sample to the population
         * \param signal The new signal sample
         */
        void process(const T& signal)
        {
            m_numberOfSamples++;
            T lastAverage(m_mean);
            updateRunningAverage(signal);
            updateVarianceEstimate(signal, lastAverage);
            updateMinMax(signal);
        }

        //! Resets all statistics to the supplied prototype value (or zero)
        void reset()
        {
            m_min = m_prototype;
            m_max = m_prototype;
            m_mean = m_prototype;
            m_standardDeviation = m_prototype;
            m_sumOfSquaredDifferences = m_prototype;
            m_numberOfSamples = 0;
        }

        //! Returns the sample standard deviation estimation
        const T& standardDeviation() const
        {
            return m_standardDeviation;
        }

        //! Returns the mean of all samples
        const T& mean() const
        {
            return m_mean;
        }

        //! Returns the number of samples seen so far
        const std::size_t& numberOfSamples() const
        {
            return m_numberOfSamples;
        }

        //! Returns the max observed value
        const T& maximum() const
        {
            return m_max;
        }

        //! Returns the min observed value
        const T& minimum() const
        {
            return m_min;
        }

        //! Returns a human readable string representation of the current statistical data
        std::string toString() const
        {
            std::ostringstream stream;
            stream << std::scientific << "mean: " << mean() << ", std_dev: " << standardDeviation()
                   << ", min: " << minimum() << ", max: " << maximum() << ", samples: " << numberOfSamples();
            return stream.str();
        }

    protected:
        /*!
         * \brief Updates the running average for added new sample
         * \param signal The new sample
         */
        virtual void updateRunningAverage(const T& signal)
        {
            m_mean = (m_mean * (m_numberOfSamples - 1) + signal) / m_numberOfSamples;
        }

        /*!
         * \brief Updates the variance estimation
         *
         * Uses Welford's online algorithm, doi:10.2307/1266577
         * \param signal The new sample
         * \param lastAverage Mean value of the last update cycle (not including the new sample)
         */
        virtual void updateVarianceEstimate(const T& signal, const T& lastAverage) = 0;

        /*!
         * \brief Updates the min and max values
         * \param signal The new sample
         */
        virtual void updateMinMax(const T& signal) = 0;

        //! A prototype instance corresponding to a zero value
        T m_prototype;

        //! The current mean
        T m_mean;

        //! Estimated standard deviation of all samples
        T m_standardDeviation;

        //! The maximum observed value
        T m_max;

        //! The minimum observed value
        T m_min;

        //! Aggregate used for welford's online algorithm
        T m_sumOfSquaredDifferences;

        //! The number of samples seen
        std::size_t m_numberOfSamples = 0;
    };

    /*!
     * \brief Estimates statistical information on a signal
     * \tparam T A non-Eigen arithmetic signal type
     * \tparam isEigenVector Internal use only - defaults to true if T is an Eigen type
     * \ingroup broccoli_core
     */
    template <typename T, bool isEigenVector = core::is_eigen_matrix<T>::value>
    class StatisticsEstimator : public BaseStatisticsEstimator<T> {
    public:
        //! Default constructor. Initializes with 0
        StatisticsEstimator()
            : BaseStatisticsEstimator<T>(0)
        {
        }

    protected:
        void updateVarianceEstimate(const T& signal, const T& lastAverage) override
        {
            m_sumOfSquaredDifferences += (signal - lastAverage) * (signal - m_mean);

            if (m_numberOfSamples > 1) {
                m_standardDeviation = std::sqrt(m_sumOfSquaredDifferences / (m_numberOfSamples - 1));
            }
        }

        void updateMinMax(const T& signal) override
        {
            if (signal > m_max) {
                m_max = signal;
            }

            if (signal < m_min) {
                m_min = signal;
            }
        }

        using BaseStatisticsEstimator<T>::m_numberOfSamples;
        using BaseStatisticsEstimator<T>::m_standardDeviation;
        using BaseStatisticsEstimator<T>::m_sumOfSquaredDifferences;
        using BaseStatisticsEstimator<T>::m_mean;
        using BaseStatisticsEstimator<T>::m_min;
        using BaseStatisticsEstimator<T>::m_max;
    };

#ifdef HAVE_EIGEN3
    /*!
     * \brief Estimates coefficient-wise statistical information on an Eigen-typed signal
     * \tparam T An Eigen arithmetic signal type
     * \ingroup broccoli_core
     */
    template <typename T>
    class StatisticsEstimator<T, true> : public BaseStatisticsEstimator<T> {
    public:
        //! \brief Default Constructor. Initializes with T::Zero()
        StatisticsEstimator()
            : StatisticsEstimator(T::Zero())
        {
        }

        /*!
         * \brief A constructor
         * \param zeroPrototype A prototype corresponding to a zero value
         */
        explicit StatisticsEstimator(T&& zeroPrototype)
            : BaseStatisticsEstimator<T>(std::forward<T>(zeroPrototype))
        {
        }

    protected:
        void updateVarianceEstimate(const T& signal, const T& lastAverage) override
        {
            // Uses Welford's online algorithm
            m_sumOfSquaredDifferences += (signal - lastAverage).cwiseProduct((signal - m_mean));

            if (m_numberOfSamples > 1) {
                m_standardDeviation = (m_sumOfSquaredDifferences / (m_numberOfSamples - 1)).cwiseSqrt();
            }
        }

        void updateMinMax(const T& signal) override
        {
            m_max = signal.cwiseMax(m_max);
            m_min = signal.cwiseMin(m_min);
        }

        using BaseStatisticsEstimator<T>::m_numberOfSamples;
        using BaseStatisticsEstimator<T>::m_standardDeviation;
        using BaseStatisticsEstimator<T>::m_sumOfSquaredDifferences;
        using BaseStatisticsEstimator<T>::m_mean;
        using BaseStatisticsEstimator<T>::m_min;
        using BaseStatisticsEstimator<T>::m_max;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
#endif

} // namespace core
} // namespace broccoli
