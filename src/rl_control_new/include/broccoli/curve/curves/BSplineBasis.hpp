/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../io/encoding.hpp"
#include "../../memory/SmartVector.hpp"
#include "BSplineBasisSample.hpp"
#include <array>
#include <cmath>

namespace broccoli {
namespace curve {
    //! Abstraction of a generic **B-spline basis** according to *de Boor 2001*.
    /*!
     * \ingroup broccoli_curve_curves
     * \tparam Degree (Maximum) degree \f$ p \geq 0 \f$
     * \tparam MaximumBSplineCount (Expected) maximum count of B-Splines (a higher B-Spline count at runtime is also supported, but less efficient)
     *
     * \warning Note that there is a difference between the terms *degree* \f$ p \f$ and *order* \f$ k \f$ (\f$ k = p + 1 \f$).
     *
     * Definition
     * ----------
     * Given the non-decreasing *knot sequence* \f$ \tau = [t_0,\,\dots\,,\,t_{n-1}] \f$ (stored in \ref m_knotSequence) with \f$ t_i \leq t_{i+1}\f$ and
     *   * \f$ n \f$ as *count* of knots in the knot sequence \f$ \tau \f$,
     *
     * the \f$ i\f$-th *normalized* B-spline of order \f$ j \f$ for the knot sequence \f$ \tau \f$ denoted by \f$ B_{i,j,\tau} \f$ is defined by the recurrence relation (de Boor 2001, p. 87ff)
     * \f[
     * B_{i,j=1,\tau}(x) := \left\lbrace \begin{array}{ll} 1,&\mbox{for}\; t_i \leq x < t_{i+1}\\ 0,&\mbox{else} \end{array}\right.
     * \f]
     * and
     * \f[
     * B_{i,j>1,\tau}(x) := \frac{x-t_i}{t_{i+j-1} - t_i}\,B_{i,j-1,\tau}(x) + \frac{t_{i+j} - x}{t_{i+j} - t_{i+1}}\,B_{i+1,j-1,\tau}(x)
     * \f]
     * with \f$ i = 0,\,\dots\,,\,m-1 \f$ and \f$ j = 1,\,\dots\,,\,k \f$ and
     *   * \f$ m = n - k > 0 \f$ as *count* of basis splines,
     *   * \f$ k = p + 1 \f$ as the (maximum) *order* of the B-spline basis,
     *   * \f$ p \geq 0 \f$ as the (maximum) *degree* of the B-spline basis.
     *
     * Derivatives
     * -----------
     * The first order derivative of \f$ B_{i,j,\tau} \f$ is given by (de Boor 2001, p. 115)
     * \f[
     * \frac{\partial B_{i,j=1,\tau}(x)}{\partial x} = 0
     * \f]
     * \f[
     * \frac{\partial B_{i,j>1,\tau}(x)}{\partial x} = \left(\frac{j-1}{t_{i+j-1} - t_i}\right)B_{i,j-1,\tau}(x) - \left(\frac{j-1}{t_{i+j} - t_{i+1}}\right)B_{i+1,j-1,\tau}(x)
     * \f]
     *
     * Simple derivation leads to the second order derivative of \f$ B_{i,j,\tau} \f$
     * \f[
     * \frac{\partial^2 B_{i,j\leq 2,\tau}(x)}{\partial x^2} = 0
     * \f]
     * \f[
     * \frac{\partial^2 B_{i,j>2,\tau}(x)}{\partial x^2} = \frac{\partial}{\partial x}\left(\frac{\partial B_{i,j>2,\tau}(x)}{\partial x}\right) = \left(\frac{j-1}{t_{i+j-1} - t_i}\right)\frac{\partial B_{i,j-1,\tau}(x)}{\partial x} - \left(\frac{j-1}{t_{i+j} - t_{i+1}}\right)\frac{\partial B_{i+1,j-1,\tau}(x)}{\partial x}
     * \f]
     *
     * Finally the \f$d\f$-th order derivative of \f$ B_{i,j,\tau} \f$ follows as
     * \f[
     * \frac{\partial^d B_{i,j\leq d,\tau}(x)}{\partial x^d} = 0
     * \f]
     * \f[
     * \frac{\partial^d B_{i,j>d,\tau}(x)}{\partial x^d} = \frac{\partial}{\partial x}\left(\frac{\partial^{d-1} B_{i,j>d,\tau}(x)}{\partial x^{d-1}}\right) = \left(\frac{j-1}{t_{i+j-1} - t_i}\right)\frac{\partial^{d-1} B_{i,j-1,\tau}(x)}{\partial x^{d-1}} - \left(\frac{j-1}{t_{i+j} - t_{i+1}}\right)\frac{\partial^{d-1} B_{i+1,j-1,\tau}(x)}{\partial x^{d-1}}
     * \f]
     *
     * References
     * ----------
     * * Carl de Boor, "A Practical Guide to Splines", Springer, New York, ISBN: 978-0387953663, 2001, p.87ff
     */
    template <unsigned int Degree, unsigned int MaximumBSplineCount = Degree + 1>
    class BSplineBasis {
    public:
        // Type definitions
        using KnotSequenceType = typename BSplineBasisSample<Degree, MaximumBSplineCount>::KnotSequenceType;

        //! Default constructor
        /*!
         * Initializes
         *  * for \f$m \geq k\f$: with a *clamped* and *uniform* knot sequence (see \ref setClampedUniformKnots)
         *  * for \f$m<k\f$ and \f$m>0\f$: with an all-zero knot sequence \f$\tau = [0,\,\dots\,,\,0]\f$
         *  * for \f$m=0\f$: nothing
         *
         * \warning If \p BSplineCount is 0, \ref m_knotSequence remains uninitialized (invalid element count)!
         *
         * \param [in] BSplineCount Count \f$m\f$ of B-splines in basis (=count of control points in corresponding B-spline curve)
         */
        BSplineBasis(const unsigned int& BSplineCount = 0)
        {
            // Clamped and uniform initialization
            if (BSplineCount >= order())
                setClampedUniformKnots(BSplineCount);
            // All-zero initialization
            else if (BSplineCount > 0) {
                m_knotSequence.resize(BSplineCount + order());
                m_knotSequence.fill(0);
            }
            // else: no initialization
        }

        //! Destructor
        virtual ~BSplineBasis()
        {
        }

        //! Comparison operator: **equality**
        virtual bool operator==(const BSplineBasis& reference) const
        {
            // Compare knot sequence elements
            if (m_knotSequence != reference.m_knotSequence)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        virtual bool operator!=(const BSplineBasis& reference) const { return !(*this == reference); }

        //! Checks, if the B-spline basis is properly defined
        /*! \return `true`, if the B-spline basis is valid, `false` otherwise. */
        virtual bool isValid() const
        {
            // Check, if knot sequence is initialized
            if (m_knotSequence.size() == 0)
                return false;

            // Check, if dimension of knot sequence is large enough
            if (m_knotSequence.size() < Degree + 2)
                return false;

            // Check, if knot sequence is non-decreasing
            for (size_t i = 0; i < m_knotSequence.size() - 1; i++)
                if (m_knotSequence[i] > m_knotSequence[i + 1])
                    return false;

            // Otherwise -> valid
            return true;
        }

        //! Returns (maximum) degree \f$ p = k - 1 \f$ of B-spline basis
        static unsigned int degree() { return Degree; }

        //! Returns (maximum) order \f$ k = p + 1 \f$ of B-spline basis
        static unsigned int order() { return Degree + 1; }

        // Member data
        /*! Knot sequence vector \f$ \tau = [t_0,\,...\,,\,t_{n-1}] \f$
         * \remark For best numerical stability, use a **normalized** knot sequence vector, i.e. \f$t_0 = 0\f$ and \f$t_{n-1} = 1\f$.
         */
        KnotSequenceType m_knotSequence;

        //! Evaluation of (derivative of) B-spline basis at a certain position \f$ x \f$
        /*!
         * \warning You may call \ref isValid() before to check, if the B-spline basis is valid! If the B-spline basis is invalid the behaviour of this method is not defined!
         *
         * \warning The derivative is given with respect to the \p position (interpolation parameter \f$ x \f$), **not** time in general!
         *
         * \param [in] position Position to evaluate (derivative) at (should - but has no to be - within \f$[t_0,\,t_m]\f$)
         * \param [in] derivationOrder Order \f$d\f$ of the derivation (use 0 for base function, i.e. no derivation)
         * \return Sample at specified position
         *
         * Evaluation of base
         *  ------------------
         * The "base" is evaluated with the recurrence relation given in the definition (see main description of this class in \ref BSplineBasis).
         *
         * Evaluation of derivatives
         *  -------------------------
         * Computes **analytic** derivative with respect to the interpolation parameter \f$ x \f$ (**not** time in general).
         *
         * The \f$d\f$-th derivative is evaluated with a recursion using the \f$(d-1)\f$-th derivative (see main description of this class in \ref BSplineBasis).
         *
         * \remark Note that the derivative \f$\frac{\partial^d S_\tau(x)}{\partial x^d}\f$ has the same "pattern" (triangular array) as \f$S_\tau(x)\f$ (see Figure 1 in de Boor p. 110). However, the first \f$d\f$ columns are zero.
         */
        virtual BSplineBasisSample<Degree, MaximumBSplineCount> evaluate(const double& position, const unsigned int& derivationOrder = 0) const
        {
            // Validity check
            assert(isValid());

            // Initialize sample to return
            BSplineBasisSample<Degree, MaximumBSplineCount> sample;

            // Compute value/derivative in a recursive way
            for (unsigned int d = 0; d <= derivationOrder; d++)
                sample = evaluateRecursive(position, d, sample);

            // Pass back return value
            return sample;
        }

        //! Evaluation of value (D0) up to N-th (DN) derivative at certain position \f$x\f$ (improved performance over calling \ref evaluate() N+1 times)
        /*!
         * \warning You may call \ref isValid() before to check, if the B-spline basis is valid! If the B-spline basis is invalid the behaviour of this method is not defined!
         *
         * \warning The derivatives are given with respect to the \p position (interpolation parameter \f$ x \f$), **not** time in general!
         *
         * \tparam N Order of highest derivative
         * \param [in] position Position \f$x\f$ to evaluate (derivatives) at (should - but has no to be - within [t_0,\,t_m])
         * \return List of value (D0) up to N-th derivative [D0, D1, ... , Di, ... , DN] at specified position
         */
        template <unsigned int N>
        std::array<BSplineBasisSample<Degree, MaximumBSplineCount>, N + 1> evaluateD0ToDN(const double& position) const
        {
            // Validity check
            assert(isValid());

            // Initialize samples to return
            std::array<BSplineBasisSample<Degree, MaximumBSplineCount>, N + 1> Di;

            // Recursive evaluation of Di's
            Di[0] = evaluateRecursive(position, 0, Di[0]);
            for (unsigned int d = 1; d <= N; d++)
                Di[d] = evaluateRecursive(position, d, Di[d - 1]);

            // Pass back list
            return Di;
        }

        // Helpers
        // -------
    public:
        //! Generates a **clamped** and **uniform** knot sequence \f$\tau\f$ for a given count of B-splines
        /*!
         *  * "Clamped": The first and last \f$k\f$ knots are equal (\f$t_0=\dots=t_p=0\f$ and \f$t_{n-1}=\dots=t_{n-p-1} = 1\f$) such that a corresponding B-spline curve interpolates the first and last control point.
         *  * "Uniform": The remaining "interior" knots (i.e. \f$t_k,\,\dots\,,\,t_{n-k-1}\f$) are distributed equally within the interval \f$]0,\,1[\f$.
         *
         * \param [in] BSplineCount Count \f$m \geq k\f$ of B-splines in basis (=count of control points in corresponding B-spline curve)
         * \return `true` on success, `false` in case of an invalid input (i.e. for \f$ m < k\f$)
         */
        bool setClampedUniformKnots(const unsigned int& BSplineCount)
        {
            // Check, if input is valid (m >= k such that n = m + k >= 2*k)
            if (BSplineCount < order()) {
                assert(false);
                return false;
            }

            // Resize knot sequence n = m + k
            m_knotSequence.resize(BSplineCount + order());

            // Clamped: set first k knots to 0 and last k knots to 1
            for (unsigned int i = 0; i < order(); i++) {
                m_knotSequence[i] = 0;
                m_knotSequence[m_knotSequence.size() - 1 - i] = 1;
            }

            // Compute uniform distribution of "interior" knots
            const int interiorKnotCount = m_knotSequence.size() - 2 * order(); // Count of "interior" knots
            if (interiorKnotCount > 0) {
                const double interiorKnotStepSize = 1.0 / ((double)(interiorKnotCount + 1)); // Stepsize between two "interior" knots
                for (int i = 0; i < interiorKnotCount; i++)
                    m_knotSequence[order() + i] = m_knotSequence[order() + i - 1] + interiorKnotStepSize;
            }

            // Success!
            return true;
        }

    protected:
        //! **Recursive** evaluation of (derivative) of B-spline basis at a certain position \f$ x \f$ based on the previous derivative (if available)
        /*!
         * \warning You may call \ref isValid() before to check, if the B-spline basis is valid! If the B-spline basis is invalid the behaviour of this method is not defined!
         *
         * \warning The derivative is given with respect to the \p position (interpolation parameter \f$ x \f$), **not** time in general!
         *
         * \remark If \p derivationOrder is zero (base value), the parameter \p previousDerivative is ignored
         *
         * \param [in] position Position to evaluate (derivative) at (should - but has no to be - within [t_0,\,t_m])
         * \param [in] derivationOrder Order \f$d\f$ of the derivation (use 0 for base function, i.e. no derivation)
         * \param [in] previousDerivative Derivative of order \f$d-1\f$ (also evaluated at \p position) used to compute the derivative of order \f$d\f$ in a recursive way (ignored if \p derivationOrder is zero)
         * \return Sample at specified position
         */
        virtual BSplineBasisSample<Degree, MaximumBSplineCount> evaluateRecursive(const double& position, const unsigned int& derivationOrder, const BSplineBasisSample<Degree, MaximumBSplineCount>& previousDerivative) const
        {
            // Validity check
            assert(isValid());

            // Initialize sample to return
            BSplineBasisSample<Degree, MaximumBSplineCount> sample(m_knotSequence, position, derivationOrder);

            // Check if we only want the base function
            if (derivationOrder == 0) {
                // Iterate through columns
                // (Attention: index j starts with 1 -> index shift by 1 in column index of sample.m_B!)
                for (unsigned int j = 1; j <= order(); j++) {
                    // Skip zero-columns
                    if (sample.m_X[0][j - 1] != -1 && sample.m_X[1][j - 1] != -1) {
                        // Compute non-zero entries of j-th column
                        for (int i = sample.m_X[0][j - 1]; i <= sample.m_X[1][j - 1]; i++) {
                            // First column: value is always "1"
                            if (j == 1)
                                sample.m_B[i][j - 1] = 1;
                            else {
                                // Add first term
                                if (i >= sample.m_X[0][j - 2] && i <= sample.m_X[1][j - 2]) {
                                    // Check, if denominator is non-zero (avoid division by zero, corresponding term should be zero anyway -> should NEVER happen)
                                    const double denominator = m_knotSequence[i + j - 1] - m_knotSequence[i]; // t_{i+j-1} - t_i
                                    if (denominator != 0)
                                        sample.m_B[i][j - 1] += (position - m_knotSequence[i]) / denominator * sample.m_B[i][j - 2];
                                }

                                // Add second term
                                if (i + 1 >= sample.m_X[0][j - 2] && i + 1 <= sample.m_X[1][j - 2]) {
                                    // Check, if denominator is non-zero (avoid division by zero, corresponding term should be zero anyway -> should NEVER happen)
                                    const double denominator = m_knotSequence[i + j] - m_knotSequence[i + 1]; // t_{i+j} - t{i+1}
                                    if (denominator != 0)
                                        sample.m_B[i][j - 1] += (m_knotSequence[i + j] - position) / denominator * sample.m_B[i + 1][j - 2];
                                }
                            }
                        }
                    } else
                        return sample; // (all remaining columns are zero too)
                }
            } else {
                // Check, if derivation order is high enough, such that all columns are zero
                if (derivationOrder >= order())
                    return sample; // Return all zero-matrix

                // Iterate through columns
                // (Attention: index j starts with 1 -> index shift by 1 in column index of sample.m_B!)
                for (unsigned int j = derivationOrder + 1; j <= order(); j++) {
                    // Skip zero-columns
                    if (sample.m_X[0][j - 1] != -1 && sample.m_X[1][j - 1] != -1) {
                        // Compute non-zero entries of j-th column
                        for (int i = sample.m_X[0][j - 1]; i <= sample.m_X[1][j - 1]; i++) {
                            // Add first term
                            if (i >= previousDerivative.m_X[0][j - 2] && i <= previousDerivative.m_X[1][j - 2]) {
                                // Check, if denominator is non-zero (avoid division by zero, corresponding term should be zero anyway -> should NEVER happen)
                                const double denominator = m_knotSequence[i + j - 1] - m_knotSequence[i]; // t_{i+j-1} - t_i
                                if (denominator != 0)
                                    sample.m_B[i][j - 1] += (j - 1) / denominator * previousDerivative.m_B[i][j - 2];
                            }

                            // Add second term
                            if (i + 1 >= previousDerivative.m_X[0][j - 2] && i + 1 <= previousDerivative.m_X[1][j - 2]) {
                                // Check, if denominator is non-zero (avoid division by zero, corresponding term should be zero anyway -> should NEVER happen)
                                const double denominator = m_knotSequence[i + j] - m_knotSequence[i + 1]; // t_{i+j} - t{i+1}
                                if (denominator != 0)
                                    sample.m_B[i][j - 1] -= (j - 1) / denominator * previousDerivative.m_B[i + 1][j - 2];
                            }
                        }
                    }
                }
            }

            // Pass back return value
            return sample;
        }

        // Encoding
        // --------
    public:
        //! Encode member data as XML element and add it to the specified stream
        /*!
         * \param [in,out] stream The stream to which the member data should be appended to.
         * \param [in] XMLIndentationLevel Level of indentation in XML format (used to add spaces)
         * \param [in] XMLTabWhiteSpaces Count of whitespaces corresponding to a tab.
         * \param [in] numericFormat C-format specifier for numeric values.
         * \return Count of appended characters (elements in stream).
         */
        virtual io::encoding::CharacterStreamSize encodeToXML(io::encoding::CharacterStream& stream, const uint8_t& XMLIndentationLevel, const size_t& XMLTabWhiteSpaces, const std::string& numericFormat = "%.7g") const
        {
            io::encoding::CharacterStreamSize addedElements = 0;

            // Start XML element
            for (size_t i = 0; i < XMLIndentationLevel * XMLTabWhiteSpaces; i++)
                addedElements += io::encoding::encode(stream, (char)' ');
            addedElements += io::encoding::encode(stream, "<BSplineBasis");

            // Write attributes
            addedElements += io::encoding::encode(stream, " Degree=\"");
            addedElements += io::encoding::encode(stream, (uint64_t)Degree);
            addedElements += io::encoding::encode(stream, "\" KnotSequence=\"");
            addedElements += io::encoding::encode(stream, m_knotSequence, numericFormat);

            // End XML element
            addedElements += io::encoding::encode(stream, "\"></BSplineBasis>\n");

            // Pass back added elements in stream
            return addedElements;
        }
    };
} // namespace curve
} // namespace broccoli
