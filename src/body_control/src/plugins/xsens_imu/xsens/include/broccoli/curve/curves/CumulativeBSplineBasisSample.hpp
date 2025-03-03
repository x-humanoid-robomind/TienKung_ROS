/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../memory/SmartVector.hpp"
#include "BSplineBasisSample.hpp"
#include <assert.h>

namespace broccoli {
namespace curve {
    //! *Evaluated* (derivative of) **cumulative** B-spline basis **sample** at a certain position \f$ x \f$ according to Kim 1995
    /*!
     * \ingroup broccoli_curve_curves
     *
     * (see also \ref BSplineBasis and \ref BSplineBasisSample)
     *
     * \tparam Degree (Maximum) degree \f$ p \geq 0 \f$
     * \tparam MaximumBSplineCount (Expected) maximum count of B-Splines (a higher B-Spline count at runtime is also supported, but less efficient)
     *
     * \warning Note that there is a difference between the terms *degree* \f$ p \f$ and *order* \f$ k \f$ (\f$ k = p + 1 \f$).
     *
     * Definition
     * ----------
     * The cumulative basis is defined by
     * \f[
     * \widetilde{B}_{i,k}(x) = \sum_{c=i}^{m-1} {B}_{c,k}(x)
     * \f]
     * The vector has the following form
     * \f[
     * \left[\widetilde{B}_{i,k}(x)\right] = [\underbrace{1,\,\dots\,,\,1}_{f_1-elem.},\,\underbrace{\otimes,\,\dots\,,\,\otimes}_{f_2-elem.},\,\underbrace{0,\,\dots\,,\,0}_{f_3-elem.}]^T
     * \f]
     * where \f$\otimes\f$ is a placeholder for floating point numbers within the interval \f$[0,\,1]\f$ and \f$f_1\f$, \f$f_2\f$ and \f$f_3\f$ are the element counts of the corresponding blocks:
     *  * \f$f_1 = \f$ count of \f$\widetilde{B}_{i,k}(x)=1\f$ elements at the beginning
     *  * \f$f_2 = \f$ count of \f$\widetilde{B}_{i,k}(x) \in\; [0,\,1]\f$ elements
     *  * \f$f_3 = \f$ count of \f$\widetilde{B}_{i,k}(x)=0\f$ elements at the end
     *
     * Derivatives
     * -----------
     * The \f$d\f$-th derivative of the cumulative basis is defined by
     * \f[
     * \frac{\partial^d \widetilde{B}_{i,k}(x)}{\partial x^d} = \sum_{c=i}^{m-1} \frac{\partial^d {B}_{c,k}(x)}{\partial x^d}
     * \f]
     * The vector has the following form
     * \f[
     * \left[\frac{\partial^d \widetilde{B}_{i,k}(x)}{\partial x^d}\right] = [\underbrace{0,\,\dots\,,\,0}_{f_1-elem.},\,\underbrace{\otimes,\,\dots\,,\,\otimes}_{f_2-elem.},\,\underbrace{0,\,\dots\,,\,0}_{f_3-elem.}]^T
     * \f]
     * where \f$\otimes\f$ is a placeholder for (possible) non-zero floating point numbers and \f$f_1\f$, \f$f_2\f$ and \f$f_3\f$ are the element counts of the corresponding blocks:
     *  * \f$f_1 = \f$ count of \f$\widetilde{B}_{i,k}(x)=0\f$ elements at the beginning
     *  * \f$f_2 = \f$ count of \f$\widetilde{B}_{i,k}(x) \in\;\mathbb{R}\f$ elements
     *  * \f$f_3 = \f$ count of \f$\widetilde{B}_{i,k}(x)=0\f$ elements at the end
     *
     * References
     * ----------
     * * Myoung-Jun Kim et al., "A General Construction Scheme for Unit Quaternion Curves with Simple High Order Derivatives", Proceedings of the 22nd Annual Conference on Computer Graphics and Interactive Techniques, 1995, SIGGRAPH '95, ACM, pages = 369--376, DOI:[10.1145/218380.218486](https://www.doi.org/10.1145/218380.218486)
     */
    template <unsigned int Degree, unsigned int MaximumBSplineCount = Degree + 1>
    class CumulativeBSplineBasisSample {
    public:
        // Type definitions
        using BType = memory::SmartVector<double, MaximumBSplineCount, std::allocator<double>>;
        using fType = std::array<int, 3>;

        //! Default constructor
        /*!
         * Initializes
         *  * \ref m_B with 0 for each element (**Warning:** Only if \p BSplineCount is non-zero!),
         *  * \ref m_f with \f$[0,\,0,\,0]\f$.
         *
         * \warning If \p BSplineCount is 0, \ref m_B remains uninitialized (invalid row count)!
         *
         * \param [in] BSplineCount Count \f$m\f$ of B-splines in corresponding basis (=count of control points in corresponding B-spline curve)
         */
        CumulativeBSplineBasisSample(const unsigned int& BSplineCount = 0)
        {
            // Initialize f
            m_f.fill(0);

            // Initialize B if possible
            if (BSplineCount > 0) {
                m_B.resize(BSplineCount);
                m_B.fill(0);
            }
        }

        //! Destructor
        virtual ~CumulativeBSplineBasisSample()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const CumulativeBSplineBasisSample& reference) const
        {
            // Compare elements of B
            if (m_B != reference.m_B)
                return false;

            // Compare auxillary parameters
            if (m_f != reference.m_f)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const CumulativeBSplineBasisSample& reference) const { return !(*this == reference); }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of cumulative B-spline basis sample
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of cumulative B-spline basis sample
        static unsigned int order() { return Degree + 1; }

        // Member data
        BType m_B; //!< *Evaluated* (derivative of) **cumulative** B-spline basis **sample** \f$ \widetilde{B}_{i,k}(x) \in \mathbb{R}^{m\times 1}\f$ at a certain position \f$ x \f$
        fType m_f; //!< Tuple \f$ (f_1,\,f_2,\,f_3)\f$ where \f$f_i\f$ denotes the element count of each block in \f$ [\widetilde{B}_{i,k}(x)]\f$ (see "Definition" in \ref CumulativeBSplineBasisSample).

        //! Computes (derivative of) the cumulative basis \f$\widetilde{B}_{i,k}(x)\f$ given a certain (derivative of) B-spline basis \f$B_{i,k}(x)\f$ sample.
        /*!
         * \param [in] basisSample (Regular) (derivative of) B-spline basis sample containing \f$B_{i,k}(x)\f$ (or its \f$d\f$-th derivative)
         * \return `true` on success, `false` in case of an invalid input
         */
        virtual bool evaluate(const BSplineBasisSample<Degree, MaximumBSplineCount>& basisSample)
        {
            // Check input
            if (basisSample.m_B.size() == 0 || basisSample.m_B[0].size() != order() || basisSample.m_X.size() != 2 || basisSample.m_X[0].size() != order()) {
                assert(false);
                return false;
            }

            // Re-initialize cumulative basis
            m_B.resize(basisSample.m_B.size());
            m_B.fill(0);
            m_f.fill(0);

            // Analyse structure of cumulative basis
            // -------------------------------------
            const int& basisSample_FirstNonZeroElement = basisSample.m_X[0][order() - 1]; // Index i of (possible) first non-zero element in k-th column of basis (if -1 -> k-th column in basis is all-zero)
            const int& basisSample_LastNonZeroElement = basisSample.m_X[1][order() - 1]; // Index i of (possible) last non-zero element in k-th column of basis (if -1 -> k-th column in basis is all-zero)
            if (basisSample_FirstNonZeroElement == -1 || basisSample_LastNonZeroElement == -1) {
                // Structure: [0,...,0]
                // k-th column in basis is all-zero -> cumulative basis is zero too
                m_f[2] = m_B.size();
                return true;
            }
            // else: k-th column of basis contains at least one (possible) non-zero element
            // Structure for d=0: [1,...,1|x,...,x|0,...,0]
            // Structure for d>0: [0,...,0|x,...,x|0,...,0]
            //                        ^       ^       ^
            //                        |       |       |
            //                       f_1     f_2     f_3
            m_f[0] = basisSample_FirstNonZeroElement; // f1
            m_f[1] = basisSample_LastNonZeroElement - basisSample_FirstNonZeroElement + 1; // f2
            m_f[2] = m_B.size() - m_f[0] - m_f[1]; // f3 = m - f1 - f2

            // Compute cumulative basis
            // ------------------------
            // Last block (f3): B_ik=0 -> tildeB_ik = 0 -> nothing to do

            // Middle block (f2): B_ik != 0 -> recursively compute previous elements (as sum of all following elements)
            m_B[basisSample_LastNonZeroElement] = basisSample.m_B[basisSample_LastNonZeroElement][order() - 1];
            for (int i = basisSample_LastNonZeroElement - 1; i >= basisSample_FirstNonZeroElement; i--)
                m_B[i] = basisSample.m_B[i][order() - 1] + m_B[i + 1];

            // First block (f1): B_ik=0 -> tildeB_ik = first element of f2-block (same for all elements)
            for (int i = basisSample_FirstNonZeroElement - 1; i >= 0; i--)
                m_B[i] = m_B[i + 1];

            // Success!
            return true;
        }
    };
} // namespace curve
} // namespace broccoli
