/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../memory/SmartVector.hpp"
#include <assert.h>

namespace broccoli {
namespace curve {
    //! *Evaluated* (derivative of) B-spline basis **sample** at a certain position \f$ x \f$
    /*!
     * \ingroup broccoli_curve_curves
     *
     * (see also \ref BSplineBasis)
     *
     * \tparam Degree (Maximum) degree \f$ p \geq 0 \f$
     * \tparam MaximumBSplineCount (Expected) maximum count of B-Splines (a higher B-Spline count at runtime is also supported, but less efficient)
     *
     * \warning Note that there is a difference between the terms *degree* \f$ p \f$ and *order* \f$ k \f$ (\f$ k = p + 1 \f$).
     *
     * \remark The auxillary integer matrix \ref m_X indicates which elements in the sample \ref m_B are **guaranteed** to be zero.
     * This allows to easily skip zero-elements in subsequent computations.
     */
    template <unsigned int Degree, unsigned int MaximumBSplineCount = Degree + 1>
    class BSplineBasisSample {
    public:
        // Type definitions
        using BType = memory::SmartVector<std::array<double, Degree + 1>, MaximumBSplineCount, std::allocator<std::array<double, Degree + 1>>>;
        using XType = std::array<std::array<int, Degree + 1>, 2>;
        using KnotSequenceType = memory::SmartVector<double, Degree + 1 + MaximumBSplineCount, std::allocator<double>>;

        //! Default constructor
        /*!
         * Initializes
         *  * \ref m_B with 0 for each element (**Warning:** Only if \p BSplineCount is non-zero!).
         *  * \ref m_X with -1 for each element.
         *
         * \warning If \p BSplineCount is 0, \ref m_B remains uninitialized (invalid row count)!
         *
         * \param [in] BSplineCount Count \f$m\f$ of B-splines in basis (=count of control points in corresponding B-spline curve)
         */
        BSplineBasisSample(const unsigned int& BSplineCount = 0)
        {
            // Initialize B if possible
            if (BSplineCount > 0) {
                m_B.resize(BSplineCount);
                for (size_t i = 0; i < m_B.size(); i++)
                    m_B[i].fill(0);
            }

            // Set all elements of X to -1
            for (int i = 0; i < 2; i++)
                for (unsigned int j = 1; j <= order(); j++)
                    m_X[i][j - 1] = -1;
        }

        //! Specialized constructor
        /*!
         * Initializes
         *  * \ref m_B with 0 for each element (**Warning:** Only if \f$n>k\f$!).
         *  * \ref m_X by calling \ref evaluateX().
         *
         * \warning If \f$n\leq k\f$, the sample \ref m_B remains uninitialized (invalid row count)!
         *
         * \param [in] knotSequence Knot sequence \f$\tau\f$ with \f$n\f$ knots of related B-spline basis
         * \param [in] position Position \f$x\f$ at which the sample is evaluated at
         * \param [in] derivationOrder Derivation order \f$d\f$ of sample \f$ B_{\tau}(x) \f$ (first \f$d\f$ columns of \f$X_{\tau}(x)\f$ are null-columns).
         */
        BSplineBasisSample(const KnotSequenceType& knotSequence, const double& position, const unsigned int& derivationOrder = 0)
        {
            // Initialize helpers
            const int m = knotSequence.size() - order();

            // Check, if input is valid
            if (m > 0) {
                // Initialize B
                m_B.resize(m);
                for (size_t i = 0; i < m_B.size(); i++)
                    m_B[i].fill(0);

                // Compute X
                evaluateX(knotSequence, position, derivationOrder);
            } else {
                // Set all elements of X to -1
                for (int i = 0; i < 2; i++)
                    for (unsigned int j = 1; j <= order(); j++)
                        m_X[i][j - 1] = -1;
            }
        }

        //! Destructor
        virtual ~BSplineBasisSample()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const BSplineBasisSample& reference) const
        {
            // Compare elements of B
            if (m_B != reference.m_B)
                return false;

            // Compare elements of X
            if (m_X != reference.m_X)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const BSplineBasisSample& reference) const { return !(*this == reference); }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of B-spline basis sample
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of B-spline basis sample
        static unsigned int order() { return Degree + 1; }

        // Member data
        //! *Evaluated* (derivative of) B-spline basis **sample** \f$ B_{\tau}(x) \in \mathbb{R}^{m\times k}\f$ at a certain position \f$ x \f$
        /*!
         * Definition
         * ----------
         * The sample \f$ B_{\tau}(x) \f$ is defined as
         * \f[
         * B_{\tau}(x) :=
         * \left[
         * \begin{array}{ccccc}
         * B_{0,1,\tau}(x) & \cdots & B_{0,j,\tau}(x) & \cdots & B_{0,k,\tau}(x)\\
         * \vdots & \ddots & & & \vdots\\
         * B_{i,1,\tau}(x) & \cdots & B_{i,j,\tau}(x) & \cdots & B_{i,k,\tau}(x)\\
         * \vdots & & & \ddots & \vdots\\
         * B_{m-1,1,\tau}(x) & \cdots & B_{m-1,j,\tau}(x) & \cdots & B_{m-1,k,\tau}(x)
         * \end{array}
         * \right]
         * \f]
         * where \f$ B_{i,j,\tau}(x)\f$ is the value of the \f$ i\f$-th *normalized* B-spline of order \f$ j \f$ for the knot sequence \f$ \tau \f$ at the position \f$x\f$.
         *
         * Derivatives
         * -----------
         * The \f$d\f$-th derivative of the sample \f$ B_{\tau}(x) \f$ is defined as
         * \f[
         * \frac{\partial^d B_{\tau}(x)}{\partial x^d} :=
         * \left[
         * \begin{array}{ccccc}
         * \frac{\partial^d B_{0,1,\tau}(x)}{\partial x^d} & \cdots & \frac{\partial^d B_{0,j,\tau}(x)}{\partial x^d} & \cdots & \frac{\partial^d B_{0,k,\tau}(x)}{\partial x^d}\\
         * \vdots & \ddots & & & \vdots\\
         * \frac{\partial^d B_{i,1,\tau}(x)}{\partial x^d} & \cdots & \frac{\partial^d B_{i,j,\tau}(x)}{\partial x^d} & \cdots & \frac{\partial^d B_{i,k,\tau}(x)}{\partial x^d}\\
         * \vdots & & & \ddots & \vdots\\
         * \frac{\partial^d B_{m-1,1,\tau}(x)}{\partial x^d} & \cdots & \frac{\partial^d B_{m-1,j,\tau}(x)}{\partial x^d} & \cdots & \frac{\partial^d B_{m-1,k,\tau}(x)}{\partial x^d}
         * \end{array}
         * \right]
         * \f]
         * i.e. the derivation is performed simply element-wise.
         */
        BType m_B;

        //! Auxillary **integer** matrix \f$X_{\tau}(x) \in \mathbb{N}^{2\times k}\f$
        /*! Contains the integers \f$\alpha_{j,\tau}(x),\,\beta_{j,\tau}(x) \in \lbrace 0,\,\dots\,,\, m-1\rbrace\f$ indicating which elements of the \f$j\f$-th column in \f$B_{\tau}(x)\f$ have **guaranteed** zero elements.
         *  * The **first** row (\f$\alpha_{j,\tau}(x) \f$) indicates the index (starting with 0) of the **first** row containing a (possible) non-zero element in the \f$j\f$-th column of \f$B_{\tau}(x)\f$ (**minimum** row-index of (possible) non-zero elements).
         *  * The **last** row (\f$\beta_{j,\tau}(x) \f$) indicates the index (starting with 0) of the **last** row containing a (possible) non-zero element in the \f$j\f$-th column of \f$B_{\tau}(x)\f$ (**maximum** row-index of (possible) non-zero elements).
         *
         * \f[
         * X_{\tau}(x) :=
         * \left[
         * \begin{array}{ccccc}
         * \alpha_{1,\tau}(x) & \cdots & \alpha_{j,\tau}(x) & \cdots & \alpha_{k,\tau}(x)\\
         * \beta_{1,\tau}(x) & \cdots & \beta_{j,\tau}(x) & \cdots & \beta_{k,\tau}(x)
         * \end{array}
         * \right]
         * \f]
         *
         * \attention If \f$\alpha_{j,\tau}(x)= \beta_{j,\tau}(x) =-1 \not\in \lbrace 0,\,\dots\,,\, m-1\rbrace \f$, then the \f$j\f$-th column in \f$ B_{\tau}(x) \f$ is guaranteed to have only zero elements ("null-column").
         *
         * \remark Note that \f$X_{\tau}(x)\f$ is the same for the \f$d\f$-th derivative of \f$ B_{\tau}(x) \f$, except for the first \f$d\f$ columns which are null-columns.
         */
        XType m_X;

        // Helpers
        // -------
        //! (Re-)computes \ref m_X from the given knot sequence \f$\tau\f$ of a \ref BSplineBasis and a given position \f$x\f$
        /*!
         * \param [in] knotSequence Knot sequence \f$\tau\f$ with \f$n\f$ knots of related B-spline basis
         * \param [in] position Position \f$x\f$ at which the sample is evaluated at
         * \param [in] derivationOrder Derivation order \f$d\f$ of sample \f$ B_{\tau}(x) \f$ (first \f$d\f$ columns of \f$X_{\tau}(x)\f$ are null-columns).
         * \return `true` on success, `false` if the input is invalid (i.e. if \f$n \leq k\f$)
         */
        bool evaluateX(const KnotSequenceType& knotSequence, const double& position, const unsigned int& derivationOrder = 0)
        {
            // Initialize helpers
            const int m = knotSequence.size() - order();

            // Check input
            if (m <= 0) {
                assert(false);
                return false;
            }

            // Reset all elements of X to -1
            for (int i = 0; i < 2; i++)
                for (unsigned int j = 1; j <= order(); j++)
                    m_X[i][j - 1] = -1;

            // Compute possible non-zero elements of first column (with d=0)
            int firstColumn_imin = m;
            int firstColumn_imax = -1;
            for (int i = 0; i < m; i++) {
                // Check, if t_i <= x < t_{i+1}
                if (knotSequence[i] <= position && position < knotSequence[i + 1]) {
                    // Non-zero B-spline element (B_i0 = 1 for t_i <= x < t_{i+1})
                    if (i < firstColumn_imin)
                        firstColumn_imin = i;
                    if (i > firstColumn_imax)
                        firstColumn_imax = i;
                }
            }

            // Abort, if there are only zero elements in the first column (all other columns are zero too)
            if (firstColumn_imin > m - 1 || firstColumn_imax < 0)
                return true;

            // Compute pattern of possible non-zero elements for each column (first d-columns are null-columns)
            for (unsigned int j = derivationOrder + 1; j <= order(); j++) {
                // Compute lower and upper bound for current column
                int currentColumn_imin = firstColumn_imin - (j - 1); // 1 step "down" for each column
                if (currentColumn_imin < 0) // Limit "lower" bound
                    currentColumn_imin = 0;
                int& currentColumn_imax = firstColumn_imax; // Stays the same

                // Save bounds to X
                m_X[0][j - 1] = currentColumn_imin;
                m_X[1][j - 1] = currentColumn_imax;
            }

            // Otherwise: success
            return true;
        }
    };
} // namespace curve
} // namespace broccoli
