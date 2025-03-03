/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../memory/SmartVector.hpp"
#include <assert.h>
#include <cmath>
#include <stdint.h>
#include <vector>
#ifdef HAVE_EIGEN3
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#endif // HAVE_EIGEN3

namespace broccoli {
namespace core {
    namespace math {
        /*!
         * \addtogroup broccoli_core_math
         * \{
         */

        //! Specification of result types.
        /*! This enum is used to given more detailed information about the result of an algorithm and which error/warning may have occured. */
        enum class Result : uint8_t {
            UNKNOWN = 0, //!< Unknown result (this should **never** be returned by a function)
            SUCCESS, //!< Algorithm was successful
            ERROR_INVALID_INPUT, //!< An **error** occured: invalid input parameters
            ERROR_DIMENSION_MISMATCH, //!< An **error** occured: dimensions do not match
            ERROR_OVERFLOW, //!< An **error** occured: overflow
            WARNING_CLOSE_TO_SINGULAR, //!< A **warning** occured: close to singularity
            RESULT_COUNT //!< Total count of elements
        };

        //! Returns string representation of the given result type
        static inline std::string resultString(const Result& result)
        {
            // Check result
            switch (result) {
            case Result::UNKNOWN:
                return "UNKNOWN";
            case Result::SUCCESS:
                return "SUCCESS";
            case Result::ERROR_INVALID_INPUT:
                return "ERROR_INVALID_INPUT";
            case Result::ERROR_DIMENSION_MISMATCH:
                return "ERROR_DIMENSION_MISMATCH";
            case Result::ERROR_OVERFLOW:
                return "ERROR_OVERFLOW";
            case Result::WARNING_CLOSE_TO_SINGULAR:
                return "WARNING_CLOSE_TO_SINGULAR";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Factorial of a non-negative integer
        /*!
         * Computes the factorial of a natural integer as
         * \f[ n! = 1\cdot 2\cdot 3\cdot \dots = \prod_{k=1}^n k\f]
         * \param [in] integer The input variable (as unsigned integer)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return The factorial of the input value or 1 if integer > 20
         */
        static inline uint64_t factorial(const uint64_t& integer, Result* const result = nullptr)
        {
            // Check for overflow
            if (integer > 20) {
                if (result != nullptr)
                    *result = Result::ERROR_OVERFLOW;
                assert(false);
                return 1; // Return to avoid division by zero in typical applications
            } else {
                if (result != nullptr)
                    *result = Result::SUCCESS;
            }

            // Use look-up-table for speed-up
            if (integer == 0 || integer == 1)
                return 1ULL;
            else if (integer == 2)
                return 2ULL;
            else if (integer == 3)
                return 6ULL;
            else if (integer == 4)
                return 24ULL;
            else if (integer == 5)
                return 120ULL;
            else if (integer == 6)
                return 720ULL;
            else if (integer == 7)
                return 5040ULL;
            else if (integer == 8)
                return 40320ULL;
            else if (integer == 9)
                return 362880ULL;
            else if (integer == 10)
                return 3628800ULL;
            else
                return integer * factorial(integer - 1);
        }

        //! Binomial coefficient
        /*!
         * Computes the **binomial coefficient** of two positive integers \f$ n\f$ and \f$ k\f$.
         * \f[
         * \left( \begin{array}{c}n\\k\end{array}\right) = \frac{n!}{k!\left(n-k\right)!}
         * \f]
         * \param [in] n The "upper" integer \f$n\f$ with \f$ n \geq k \geq 0\f$
         * \param [in] k The "lower" integer \f$k\f$ with \f$ k \geq 0 \f$
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return The binomial coefficient of the two given integers (also positive integer)
         */
        static inline uint64_t binomialCoefficient(const uint64_t& n, const uint64_t& k, Result* const result = nullptr)
        {
            // Check, if n is greater or equal to k
            if (n < k) {
                if (result != nullptr)
                    *result = Result::ERROR_INVALID_INPUT;
                assert(false);
                return 0;
            }

            // Check for overflow
            if (n > 20) {
                if (result != nullptr)
                    *result = Result::ERROR_OVERFLOW;
                assert(false);
                return 0;
            } else if (result != nullptr)
                *result = Result::SUCCESS;

            // Compute binomial coefficient
            return (uint64_t)::round(factorial(n) / (factorial(k) * factorial(n - k)));
        }

        //! Specification of input data type for computing the multinomial coefficient
        using MultinomialCoefficientInputType = memory::SmartVector<uint64_t, 4, std::allocator<uint64_t>>;

        //! Multinomial coefficient
        /*!
         * Computes the **multinomial coefficient** of a tuple of non-negative integers \f$(k_1,\,\dots\,,\,k_m)\f$.
         * \f[
         * \left( \begin{array}{c}n\\k_1,\,\dots\,,\,k_m\end{array}\right) = \frac{n!}{k_1!\cdots k_m!}
         * \f]
         * with \f$n = k_1+\cdots + k_m\f$.
         * \param [in] k The tuple of non-negative integers \f$(k_1,\,\dots\,,\,k_m)\f$ with \f$k_i\geq 0\f$
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return The multinomial coefficient of the given tuple (positive integer)
         */
        static inline uint64_t multinomialCoefficient(const MultinomialCoefficientInputType& k, Result* const result = nullptr)
        {
            // Check, if tuple is empty
            if (k.size() == 0) {
                if (result != nullptr)
                    *result = Result::ERROR_INVALID_INPUT;
                assert(false);
                return 0;
            }

            // Compute
            uint64_t n = 0;
            for (size_t i = 0; i < k.size(); i++)
                n += k[i];

            // Check for overflow
            if (n > 20) {
                if (result != nullptr)
                    *result = Result::ERROR_OVERFLOW;
                assert(false);
                return 0;
            } else if (result != nullptr)
                *result = Result::SUCCESS;

            // Compute multinomial coefficient
            uint64_t denominator = 1;
            for (size_t i = 0; i < k.size(); i++)
                denominator *= factorial(k[i]);
            return (uint64_t)::round(factorial(n) / denominator);
        }

        //! Specification of the data type of a *Faà di Bruno* tuple
        using FaaDiBrunoTuple = memory::SmartVector<uint64_t, 5>;

        //! Specification of the data type of a list of *Faà di Bruno* tuples
        using FaaDiBrunoTuples = memory::SmartVector<FaaDiBrunoTuple, 7>;

        //! Compute coefficient tuples required by **Faà di Bruno's formula** (for \f$n\f$-th derivative of two chained functions)
        /*!
         * Computes all tuples \f$ (m_1,\ m_2,\ \dots,\ m_n)\f$ of nonnegative integers satisfying \f$ 1\cdot m_1 + 2\cdot m_2 + \dots + n\cdot m_n = n\f$
         * with \f$ n = \f$ \p derivationOrder.
         * \param [in] derivationOrder Order \f$n\geq 0\f$ of required derivation
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return List of tuples \f$ (m_1,\ m_2,\ \dots,\ m_n)\f$
         *
         * \par Structure of output data
         * The return value is structured as a two-dimensional vector. The first "level", i.e. the first index, specifies the tuple-number/index (unsorted). The
         * second "level", i.e. the second index, resembles the values \f$m_i\f$ with \f$i=1\dots n\f$ (sorted). Thus, the element [j][i] will return \f$m_i\f$
         * of the \f$j\f$-th tuple.
         */
        static inline FaaDiBrunoTuples faaDiBrunoTuples(const uint64_t& derivationOrder, Result* const result = nullptr)
        {
            // Initialize return value
            FaaDiBrunoTuples tuples;

            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Use look-up-table for speed-up
            if (derivationOrder == 0) {
                // 0 tuples...
                return tuples;
            } else if (derivationOrder == 1) {
                // Initialize tuples
                tuples.resize(1, FaaDiBrunoTuple(derivationOrder, 0));

                // Set entries
                tuples[0][0] = 1;
            } else if (derivationOrder == 2) {
                // Initialize tuples
                tuples.resize(2, FaaDiBrunoTuple(derivationOrder, 0));

                // Set entries
                tuples[0][0] = 2;
                tuples[1][1] = 1;
            } else if (derivationOrder == 3) {
                // Initialize tuples
                tuples.resize(3, FaaDiBrunoTuple(derivationOrder, 0));

                // Set entries
                tuples[0][0] = 3;
                tuples[1][0] = 1;
                tuples[1][1] = 1;
                tuples[2][2] = 1;
            } else if (derivationOrder == 4) {
                // Initialize tuples
                tuples.resize(5, FaaDiBrunoTuple(derivationOrder, 0));

                // Set entries
                tuples[0][0] = 4;
                tuples[1][0] = 2;
                tuples[1][1] = 1;
                tuples[2][1] = 2;
                tuples[3][0] = 1;
                tuples[3][2] = 1;
                tuples[4][3] = 1;
            } else if (derivationOrder == 5) {
                // Initialize tuples
                tuples.resize(7, FaaDiBrunoTuple(derivationOrder, 0));

                // Set entries
                tuples[0][0] = 5;
                tuples[1][0] = 3;
                tuples[1][1] = 1;
                tuples[2][0] = 1;
                tuples[2][1] = 2;
                tuples[3][0] = 2;
                tuples[3][2] = 1;
                tuples[4][1] = 1;
                tuples[4][2] = 1;
                tuples[5][0] = 1;
                tuples[5][3] = 1;
                tuples[6][4] = 1;
            } else {
                // Count of all possible tuple candidates: (count of possible values)^(element count) = (n+1)^n
                const size_t countOfValues = derivationOrder + 1;
                size_t tupleCandidateCount = countOfValues;
                for (size_t i = 1; i < derivationOrder; i++)
                    tupleCandidateCount *= countOfValues;

                // Check for overflow
                if (tupleCandidateCount > 1000000000ULL) {
                    if (result != nullptr)
                        *result = Result::ERROR_OVERFLOW;
                    assert(false);
                    return tuples;
                }

                // Allocate memory for maximum speed
                tuples.reserve(tupleCandidateCount);

                // Initialize tuple candidate
                FaaDiBrunoTuple tupleCandidate(derivationOrder);

                // Brute-force: compute all possible combinations with each m within [0, n] then check constraint
                for (size_t i = 0; i < tupleCandidateCount; i++) {
                    // Compute element values
                    /* Idea:
                     * -----
                     * Consider the candidate index "i" as number to the base of "n+1" (all possible values for the m's). Then we can obtain
                     * the single "digits" (m's) in the same way as we would to get every digit of a number in the dezimal system (base 10).
                     */
                    size_t number = i;
                    size_t base = derivationOrder + 1;
                    for (size_t j = 0; j < derivationOrder; j++) {
                        tupleCandidate[j] = number % base;
                        number /= base;
                    }

                    // Check constraint
                    uint64_t sum = 0;
                    for (size_t j = 1; j <= derivationOrder; j++)
                        sum += j * tupleCandidate[j - 1];
                    if (sum == derivationOrder) {
                        // Constraint is satisfied -> add candidate to tuple list
                        tuples.push_back(tupleCandidate);
                    }
                }
            }

            // Pass back computed tuples
            return tuples;
        }

        //! Specification of the data type of a *general Leibniz* tuple
        using GeneralLeibnizTuple = memory::SmartVector<uint64_t, 4, std::allocator<uint64_t>>;

        //! Specification of the data type of a list of *general Leibniz* tuples
        using GeneralLeibnizTuples = memory::SmartVector<GeneralLeibnizTuple, 20, std::allocator<GeneralLeibnizTuple>>;

        //! Compute coefficient tuples required by **general Leibniz rule** (for \f$n\f$-th derivative of product of \f$m\f$ functions)
        /*!
         * Computes all tuples \f$ (k_1,\ k_2,\ \dots,\ k_m)\f$ of nonnegative integers satisfying \f$ k_1 + k_2 + \dots + k_m = n\f$
         * with \f$ n = \f$ \p derivationOrder and \f$ m = \f$ \p factors in product.
         * \param [in] derivationOrder Order \f$n\geq 0\f$ of required derivation
         * \param [in] factors Count \f$m\geq 1\f$ of factors in function product
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return List of tuples \f$ (k_1,\ k_2,\ \dots,\ k_m)\f$
         *
         * \par Structure of output data
         * The return value is structured as a two-dimensional vector. The first "level", i.e. the first index, specifies the tuple-number/index (unsorted). The
         * second "level", i.e. the second index, resembles the values \f$k_i\f$ with \f$i=1\dots m\f$ (sorted). Thus, the element [j][i] will return \f$k_i\f$
         * of the \f$j\f$-th tuple.
         */
        static inline GeneralLeibnizTuples generalLeibnizTuples(const uint64_t& derivationOrder, const uint64_t& factors, Result* const result = nullptr)
        {
            // Initialize return value
            GeneralLeibnizTuples tuples;

            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Check count of factors
            if (factors < 1) {
                if (result != nullptr)
                    *result = Result::ERROR_INVALID_INPUT;
                assert(false);
                return tuples;
            }

            // Use look-up-table for speed-up
            if (factors == 1) {
                // Initialize tuples
                tuples.resize(1, GeneralLeibnizTuple(1, derivationOrder));
                return tuples;
            } else if (factors == 2) {
                if (derivationOrder == 0) {
                    // Initialize tuples
                    tuples.resize(1, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    return tuples;
                } else if (derivationOrder == 1) {
                    // Initialize tuples
                    tuples.resize(2, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 1;
                    tuples[1][1] = 1;
                    return tuples;
                } else if (derivationOrder == 2) {
                    // Initialize tuples
                    tuples.resize(3, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 2;
                    tuples[1][0] = 1;
                    tuples[1][1] = 1;
                    tuples[2][1] = 2;
                    return tuples;
                } else if (derivationOrder == 3) {
                    // Initialize tuples
                    tuples.resize(4, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 3;
                    tuples[1][0] = 2;
                    tuples[1][1] = 1;
                    tuples[2][0] = 1;
                    tuples[2][1] = 2;
                    tuples[3][1] = 3;
                    return tuples;
                }
            } else if (factors == 3) {
                if (derivationOrder == 0) {
                    // Initialize tuples
                    tuples.resize(1, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    return tuples;
                } else if (derivationOrder == 1) {
                    // Initialize tuples
                    tuples.resize(3, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 1;
                    tuples[1][1] = 1;
                    tuples[2][2] = 1;
                    return tuples;
                } else if (derivationOrder == 2) {
                    // Initialize tuples
                    tuples.resize(6, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 2;
                    tuples[1][0] = 1;
                    tuples[1][1] = 1;
                    tuples[2][1] = 2;
                    tuples[3][0] = 1;
                    tuples[3][2] = 1;
                    tuples[4][1] = 1;
                    tuples[4][2] = 1;
                    tuples[5][2] = 2;
                    return tuples;
                } else if (derivationOrder == 3) {
                    // Initialize tuples
                    tuples.resize(10, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 3;
                    tuples[1][0] = 2;
                    tuples[1][1] = 1;
                    tuples[2][0] = 1;
                    tuples[2][1] = 2;
                    tuples[3][1] = 3;
                    tuples[4][0] = 2;
                    tuples[4][2] = 1;
                    tuples[5][0] = 1;
                    tuples[5][1] = 1;
                    tuples[5][2] = 1;
                    tuples[6][1] = 2;
                    tuples[6][2] = 1;
                    tuples[7][0] = 1;
                    tuples[7][2] = 2;
                    tuples[8][1] = 1;
                    tuples[8][2] = 2;
                    tuples[9][2] = 3;
                    return tuples;
                }
            } else if (factors == 4) {
                if (derivationOrder == 0) {
                    // Initialize tuples
                    tuples.resize(1, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    return tuples;
                } else if (derivationOrder == 1) {
                    // Initialize tuples
                    tuples.resize(4, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 1;
                    tuples[1][1] = 1;
                    tuples[2][2] = 1;
                    tuples[3][3] = 1;
                    return tuples;
                } else if (derivationOrder == 2) {
                    // Initialize tuples
                    tuples.resize(10, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 2;
                    tuples[1][0] = 1;
                    tuples[1][1] = 1;
                    tuples[2][1] = 2;
                    tuples[3][0] = 1;
                    tuples[3][2] = 1;
                    tuples[4][1] = 1;
                    tuples[4][2] = 1;
                    tuples[5][2] = 2;
                    tuples[6][0] = 1;
                    tuples[6][3] = 1;
                    tuples[7][1] = 1;
                    tuples[7][3] = 1;
                    tuples[8][2] = 1;
                    tuples[8][3] = 1;
                    tuples[9][3] = 2;
                    return tuples;
                } else if (derivationOrder == 3) {
                    // Initialize tuples
                    tuples.resize(20, GeneralLeibnizTuple(factors, 0));

                    // Set entries
                    tuples[0][0] = 3;
                    tuples[1][0] = 2;
                    tuples[1][1] = 1;
                    tuples[2][0] = 1;
                    tuples[2][1] = 2;
                    tuples[3][1] = 3;
                    tuples[4][0] = 2;
                    tuples[4][2] = 1;
                    tuples[5][0] = 1;
                    tuples[5][1] = 1;
                    tuples[5][2] = 1;
                    tuples[6][1] = 2;
                    tuples[6][2] = 1;
                    tuples[7][0] = 1;
                    tuples[7][2] = 2;
                    tuples[8][1] = 1;
                    tuples[8][2] = 2;
                    tuples[9][2] = 3;
                    tuples[10][0] = 2;
                    tuples[10][3] = 1;
                    tuples[11][0] = 1;
                    tuples[11][1] = 1;
                    tuples[11][3] = 1;
                    tuples[12][1] = 2;
                    tuples[12][3] = 1;
                    tuples[13][0] = 1;
                    tuples[13][2] = 1;
                    tuples[13][3] = 1;
                    tuples[14][1] = 1;
                    tuples[14][2] = 1;
                    tuples[14][3] = 1;
                    tuples[15][2] = 2;
                    tuples[15][3] = 1;
                    tuples[16][0] = 1;
                    tuples[16][3] = 2;
                    tuples[17][1] = 1;
                    tuples[17][3] = 2;
                    tuples[18][2] = 1;
                    tuples[18][3] = 2;
                    tuples[19][3] = 3;
                    return tuples;
                }
            }

            // Count of all possible tuple candidates: (count of possible values)^(element count) = (n+1)^m
            size_t countOfValues = derivationOrder + 1;
            size_t tupleCandidateCount = countOfValues;
            for (size_t i = 1; i < factors; i++)
                tupleCandidateCount *= countOfValues;

            // Check for overflow
            if (tupleCandidateCount > 1000000000ULL) {
                if (result != nullptr)
                    *result = Result::ERROR_OVERFLOW;
                assert(false);
                return tuples;
            }

            // Allocate memory for maximum speed
            tuples.reserve(tupleCandidateCount);

            // Initialize tuple candidate
            GeneralLeibnizTuple tupleCandidate;
            tupleCandidate.resize(factors);

            // Brute-force: compute all possible combinations with each k within [0, n] then check constraint
            for (size_t i = 0; i < tupleCandidateCount; i++) {
                // Compute element values
                /* Idea:
                 * -----
                 * Consider the candidate index "i" as number to the base of "n+1" (all possible values for the k's). Then we can obtain
                 * the single "digits" (k's) in the same way as we would to get every digit of a number in the dezimal system (base 10).
                 */
                size_t number = i;
                size_t base = derivationOrder + 1;
                for (size_t j = 0; j < factors; j++) {
                    tupleCandidate[j] = number % base;
                    number /= base;
                }

                // Check constraint
                uint64_t sum = 0;
                for (uint64_t j = 0; j < factors; j++)
                    sum += tupleCandidate[j];
                if (sum == derivationOrder) {
                    // Constraint is satisfied -> add candidate to tuple list
                    tuples.push_back(tupleCandidate);
                }
            }

            // Pass back computed tuples
            return tuples;
        }

#ifdef HAVE_EIGEN3
        //! Solve tridiagonal system of equations.
        /*!
         * Solves a linear system of equations \f$ A \cdot X = B \f$ where \f$ A \f$ is tridiagonal by using the efficient "Thomas algorithm". The right hand side \f$ B\f$ (and thus also \f$ X \f$) may have more than one column.
         *
         * (*Algorithm according to Quarteroni, "Numerical Mathematics", Springer Berlin Heidelberg, 2007, 2nd edition, DOI:[10.1007/b98885](https://www.doi.org/10.1007/b98885), p.93ff*)
         *
         * \f[
         * \underbrace{\left[\begin{array}{ccccccc} d_1 & u_1 & & & & & 0 \\ l_2 & d_2 & u_2 & & & & \\ & \ddots & \ddots & \ddots & & & \\ & & l_i & d_i & u_i & & \\ & & & \ddots & \ddots & \ddots & \\ & & & & l_{n-1} & d_{n-1} & u_{n-1} \\ 0 & & & & & l_n & d_n \end{array}\right]}_A
         * \cdot
         * \underbrace{\left[\begin{array}{c} x_1 \\ x_2 \\ \vdots \\ x_i \\ \vdots \\ x_{n-1} \\ x_n \end{array}\right]}_X
         * =
         * \underbrace{\left[\begin{array}{c} b_1 \\ b_2 \\ \vdots \\ b_i \\ \vdots \\ b_{n-1} \\ b_n \end{array}\right]}_B
         * \f]
         *
         * \param [in] lowerDiagonalOfA Lower diagonal elements \f$ l_i \f$ of matrix \f$ A \f$ (\f$ n \f$ elements with the first element beeing zero)
         * \param [in] diagonalOfA Diagonal elements \f$ d_i \f$ of matrix \f$ A \f$ (\f$ n \f$ elements)
         * \param [in] upperDiagonalOfA Upper diagonal elements \f$ u_i \f$ of matrix \f$ A \f$ (\f$ n \f$ elements with the last element beeing zero)
         * \param [in] B Right hand side \f$ B \in \mathbb{R}^{n \times m}\f$
         * \param [out] X Solution \f$ X \in \mathbb{R}^{n \times m}\f$
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        static inline bool solveTriDiagonalSystemOfEquations(const Eigen::VectorXd& lowerDiagonalOfA, const Eigen::VectorXd& diagonalOfA, const Eigen::VectorXd& upperDiagonalOfA, const Eigen::MatrixXd& B, Eigen::MatrixXd& X, Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Check dimensions
            int n = diagonalOfA.rows();
            int m = B.cols();
            if (lowerDiagonalOfA.rows() != n || upperDiagonalOfA.rows() != n || B.rows() != n) {
                if (result != nullptr)
                    *result = Result::ERROR_DIMENSION_MISMATCH;
                assert(false);
                return false;
            }

            // Resize solution
            X.resize(n, m);

            // Forward sweep
            Eigen::VectorXd H(n - 1); // Modified upper diagonal of A (temporary helper)
            Eigen::MatrixXd P(n, m); // Modified right hand side B (temporary helper)
            double denominator = diagonalOfA(0);
            double absoluteDenominator = fabs(denominator); // Absolute value of denominator
            double minimumDenominator = absoluteDenominator; // Minimum (absolute) value of denominator
            double maximumDenominator = absoluteDenominator; // Maximum (absolute) value of denominator
            if (n > 1)
                H(0) = upperDiagonalOfA(0) / denominator;
            P.block(0, 0, 1, m) = B.block(0, 0, 1, m) / denominator;
            for (int i = 1; i < n; i++) {
                denominator = (diagonalOfA(i) - lowerDiagonalOfA(i) * H(i - 1));
                absoluteDenominator = fabs(denominator);
                if (absoluteDenominator < minimumDenominator)
                    minimumDenominator = absoluteDenominator;
                if (absoluteDenominator > maximumDenominator)
                    maximumDenominator = absoluteDenominator;
                if (i < n - 1)
                    H(i) = upperDiagonalOfA(i) / denominator;
                P.block(i, 0, 1, m) = (B.block(i, 0, 1, m) - lowerDiagonalOfA(i) * P.block(i - 1, 0, 1, m)) / denominator;
            }

            // Check for singularity (use ratio between minimum and maximum absolute value of denominator as estimate for the condition number)
            if (maximumDenominator == 0 || minimumDenominator / maximumDenominator < 1e-15 /* <-- typical machine precision for 64bit-double */)
                // Send warning to user
                if (result != nullptr)
                    *result = Result::WARNING_CLOSE_TO_SINGULAR;

            // Backwards-substitution
            X.block(n - 1, 0, 1, m) = P.block(n - 1, 0, 1, m);
            for (int i = n - 2; i >= 0; i--)
                X.block(i, 0, 1, m) = P.block(i, 0, 1, m) - H(i) * X.block(i + 1, 0, 1, m);

            // Success
            return true;
        }

        //! Solve block-tridiagonal system of equations.
        /*!
         * Solves a linear system of equations \f$ A \cdot X = B \f$ where \f$ A \f$ is block-tridiagonal (square blocks).
         *
         * (*Algorithm according to Mund, "An algorithm for the interpolation of functions using quintic splines", Journal of Computational and Applied Mathematics, 1975, Vol. 1, Nr. 4, p.279-288, DOI:[10.1016/0771-050X(75)90020-0](https://www.doi.org/10.1016/0771-050X(75)90020-0)*)
         *
         * \f[
         * \underbrace{\left[\begin{array}{ccccccc} D_1 & U_1 & & & & & 0 \\ L_2 & D_2 & U_2 & & & & \\ & \ddots & \ddots & \ddots & & & \\ & & L_i & D_i & U_i & & \\ & & & \ddots & \ddots & \ddots & \\ & & & & L_{n-1} & D_{n-1} & U_{n-1} \\ 0 & & & & & L_n & D_n \end{array}\right]}_A
         * \cdot
         * \underbrace{\left[\begin{array}{c} X_1 \\ X_2 \\ \vdots \\ X_i \\ \vdots \\ X_{n-1} \\ X_n \end{array}\right]}_X
         * =
         * \underbrace{\left[\begin{array}{c} B_1 \\ B_2 \\ \vdots \\ B_i \\ \vdots \\ B_{n-1} \\ B_n \end{array}\right]}_B
         * \f]
         *
         * \warning For numerical stability the matrix \f$ A \f$ should (but does not have to) be block-diagonally dominant, i.e. \f$ ||(D_i)^{-1}|| \cdot (||L_i|| + ||U_i||) <= 1 \f$ for all \f$ i = 1,\ \dots, n \f$
         *
         * \tparam BlockTypeA specifies the type of the blocks in \f$A\f$ (of type `Eigen::Matrix`)
         * \tparam BlockTypeXB specifies the type of the blocks in \f$ X \f$ and \f$ B \f$ (of type `Eigen::Matrix`)
         * \param [in] lowerDiagonalOfA Lower diagonal blocks \f$ L_i \f$ of matrix \f$ A \f$ (\f$ n \f$ elements with the first element as zero-matrix)
         * \param [in] diagonalOfA Diagonal blocks \f$ D_i \f$ of matrix \f$ A \f$ (\f$ n \f$ elements)
         * \param [in] upperDiagonalOfA Upper diagonal blocks \f$ U_i \f$ of matrix \f$A\f$ (\f$ n \f$ elements with the last element as zero-matrix)
         * \param [in] B Right hand side (\f$ n \f$ blocks of type \p BlockTypeXB)
         * \param [out] X Solution (\f$ n \f$ blocks of type \p BlockTypeXB)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        template <class BlockTypeA, class BlockTypeXB>
        static inline bool solveBlockTriDiagonalSystemOfEquations(const std::vector<BlockTypeA, Eigen::aligned_allocator<BlockTypeA>>& lowerDiagonalOfA, const std::vector<BlockTypeA, Eigen::aligned_allocator<BlockTypeA>>& diagonalOfA, const std::vector<BlockTypeA, Eigen::aligned_allocator<BlockTypeA>>& upperDiagonalOfA, const std::vector<BlockTypeXB, Eigen::aligned_allocator<BlockTypeXB>>& B, std::vector<BlockTypeXB, Eigen::aligned_allocator<BlockTypeXB>>& X, Result* const result = nullptr)
        {
            // Assume success
            if (result != nullptr)
                *result = Result::SUCCESS;

            // Check dimensions
            size_t n = diagonalOfA.size();
            if (lowerDiagonalOfA.size() != n || upperDiagonalOfA.size() != n || B.size() != n) {
                if (result != nullptr)
                    *result = Result::ERROR_DIMENSION_MISMATCH;
                assert(false);
                return false;
            }

            // Resize solution
            X.resize(n);

            // Forward elimination
            std::vector<BlockTypeA, Eigen::aligned_allocator<BlockTypeA>> H; // Modified upper diagonal of A (temporary helper)
            H.resize(n - 1);
            std::vector<BlockTypeXB, Eigen::aligned_allocator<BlockTypeXB>> P; // Modified right hand side B (temporary helper)
            P.resize(n);
            Eigen::ColPivHouseholderQR<BlockTypeA> QR(diagonalOfA[0]); // Compute QR decomposition of D1
            double absoluteDeterminant = QR.absDeterminant(); // The absolute value of the determinant
            double minimumDeterminant = absoluteDeterminant; // The minimum observed value for absoluteDeterminant
            double maximumDeterminant = absoluteDeterminant; // The maximum observed value for absoluteDeterminant
            if (n > 1)
                H[0] = QR.solve(upperDiagonalOfA[0]);
            P[0] = QR.solve(B[0]);
            for (size_t i = 1; i < n; i++) {
                QR.compute(diagonalOfA[i] - lowerDiagonalOfA[i] * H[i - 1]);
                absoluteDeterminant = QR.absDeterminant();
                if (absoluteDeterminant < minimumDeterminant)
                    minimumDeterminant = absoluteDeterminant;
                if (absoluteDeterminant > maximumDeterminant)
                    maximumDeterminant = absoluteDeterminant;
                if (i < n - 1)
                    H[i] = QR.solve(upperDiagonalOfA[i]);
                P[i] = QR.solve(B[i] - lowerDiagonalOfA[i] * P[i - 1]);
            }

            // Check for singularity (use ratio between minimum and maximum absolute value of block-determinant as estimate for the condition number)
            if (maximumDeterminant == 0 || minimumDeterminant / maximumDeterminant < 1e-15 /* <-- typical machine precision for 64bit-double */)
                // Send warning to user
                if (result != nullptr)
                    *result = Result::WARNING_CLOSE_TO_SINGULAR;

            // Backwards-substitution
            X[n - 1] = P[n - 1];
            for (int i = n - 2; i >= 0; i--)
                X[i] = P[i] - H[i] * X[i + 1];

            // Success
            return true;
        }

        /*!
         * \brief Efficiently computes \f$\gamma A^\#_W \, y + (I - \gamma A^\#_W \, A) \, z \f$ for given \f$z,y,A\f$.
         *
         * \f$A^\#_W = W^{-1}A^T(A W^{-1} A^T)^{-1}\f$ denotes the \f$W\f$-weighed pseudoinverse of \f$A\f$.
         * This is an implementation of the Klein and Huang Algorithm (1983), see Nakamura 1991 p. 66
         *
         * \note This algorithm assumes \f$AA^T\f$ is positive or negative semidefinite.
         *
         * \param [in] A \f$A \in \mathbb{R}^{n\times m}\f$
         * \param [in] inverseWeighingMatrix Inverse weighing matrix \f$W^{-1} \in \mathbb{R}^{m\times m}\f$
         * \param [in] y Row-vector \f$y \in \mathbb{R}^{n}\f$
         * \param [in] z Row-vector \f$z \in \mathbb{R}^{m}\f$
         * \param [in] gamma Selection factor [0...1]. For \f$\gamma=0\f$ the solution degenerates to \f$z\f$.
         * \return \f$\gamma A^\#_W \, y + (I - \gamma A^\#_W \, A) \, z \f$
         */
        template <typename DerivedA, typename DerivedY, typename DerivedZ>
        static inline Eigen::Matrix<double, Eigen::MatrixBase<DerivedZ>::RowsAtCompileTime, 1> solvePseudoInverseEquation(const Eigen::MatrixBase<DerivedA>& A, const Eigen::Matrix<double, Eigen::MatrixBase<DerivedZ>::RowsAtCompileTime, Eigen::MatrixBase<DerivedZ>::RowsAtCompileTime>& inverseWeighingMatrix, const Eigen::MatrixBase<DerivedY>& y, const Eigen::MatrixBase<DerivedZ>& z, const double& gamma = 1.0)
        {
            using BType = Eigen::Matrix<double, Eigen::MatrixBase<DerivedY>::RowsAtCompileTime, Eigen::MatrixBase<DerivedY>::RowsAtCompileTime>;
            BType B = A * inverseWeighingMatrix * A.transpose();
            auto p = y - A * z;
            Eigen::LDLT<BType> solver(B);
            return gamma * inverseWeighingMatrix * A.transpose() * solver.solve(p) + z;
        }

        /*!
         * \brief Efficiently computes \f$\gamma A^*_W \, y + (I - \gamma A^*_W \, A) \, z \f$ for given \f$z,y,A\f$.
         *
         * \f$A^*_W = W^{-1}A^T(A W^{-1} A^T + k I)^{-1}\f$ denotes the \f$k\f$-damped, \f$W\f$-weighed pseudoinverse of \f$A\f$.
         * See Nakamura 199 p. 266
         *
         * This algorithm is robust to A becoming singular and is faster than solvePseudoInverseEquation().
         * However, the accuracy of the solution is worse and influenced by the damping parameter.
         *
         * \param [in] A \f$A \in \mathbb{R}^{n\times m}\f$
         * \param [in] dampingValue \f$k > 0\f$. In the case a negative damping is passed, the dampingValue is internally set to zero.
         * \param [in] inverseWeighingMatrix Inverse weighing matrix \f$W^{-1} \mathbb{R}^{m\times m}\f$
         * \param [in] y Row-vector \f$y \in \mathbb{R}^{n}\f$
         * \param [in] z Row-vector \f$z \in \mathbb{R}^{m}\f$
         * \param [in] gamma Selection factor [0...1]. For \f$\gamma=0\f$ the solution degenerates to \f$z\f$.
         * \return \f$\gamma A^*_W \, y + (I - \gamma A^*_W \, A) \, z \f$
         */
        template <typename DerivedA, typename DerivedY, typename DerivedZ>
        static inline Eigen::Matrix<double, Eigen::MatrixBase<DerivedZ>::RowsAtCompileTime, 1> solveDampedPseudoInverseEquation(const Eigen::MatrixBase<DerivedA>& A, double dampingValue, const Eigen::Matrix<double, Eigen::MatrixBase<DerivedZ>::RowsAtCompileTime, Eigen::MatrixBase<DerivedZ>::RowsAtCompileTime>& inverseWeighingMatrix, const Eigen::MatrixBase<DerivedY>& y, const Eigen::MatrixBase<DerivedZ>& z, const double& gamma = 1.0)
        {
            if (dampingValue < 0.0) {
                dampingValue = 0.0;
                assert(false && "Damping value must not be negative!");
            }

            using BType = Eigen::Matrix<double, Eigen::MatrixBase<DerivedY>::RowsAtCompileTime, Eigen::MatrixBase<DerivedY>::RowsAtCompileTime>;
            BType B = (A * inverseWeighingMatrix * A.transpose() + dampingValue * BType::Identity());
            auto p = y - A * z;
            Eigen::LLT<BType> solver(B);
            return gamma * inverseWeighingMatrix * A.transpose() * solver.solve(p) + z;
        }
#endif // HAVE_EIGEN3

        //! Implements the Euclidean algorithm to find the greatest common divisor of two integer numbers.
        /*!
         * This follows the following recursive computation law for numbers \f$a, b \ge 0\f$
         * \f[gcd(a, a) = a\f]
         * \f[gcd(a, b) = gcd(a - b, b) \; \text{if} a > b\f]
         * \f[gcd(a, b) = gcd(a, b - a) \; \text{if} b > a\f]
         *
         * \tparam T An integer type
         * \param [in] a A value >= 0
         * \param [in] b A value >= 0
         * \return The greatest common divisor of a and b
         */
        template <typename T>
        static inline typename std::enable_if<std::numeric_limits<T>::is_integer, T>::type greatestCommonDivisor(T a, T b)
        {
            // Check input
            if (a < 0 || b < 0) {
                assert(false);
                return 1;
            }

            if (a == 0)
                return b;

            if (b == 0)
                return a;

            // The Euclidean algorithm is recursive
            // We know we have the result when the difference between a and b is zero
            while (true) {
                if (a == b) {
                    return a;
                }

                if (a > b) {
                    a = a - b;
                } else {
                    b = b - a;
                }
            }
        }

        //! Compute the least common multiple of two integer numbers
        /*!
         * \tparam T An integer type
         * \param [in] a First value >= 0
         * \param [in] b Second value >= 0
         * \return The least common multiple of a and b.
         */
        template <typename T>
        static inline typename std::enable_if<std::numeric_limits<T>::is_integer, T>::type leastCommonMultiple(const T& a, const T& b)
        {
            // Check input
            if (a < 0 || b < 0) {
                assert(false);
                return 1;
            }

            if (a == 0 && a == b)
                return 0;

            return (std::abs<T>(a) / greatestCommonDivisor(a, b)) * std::abs<T>(b);
        }

        //! Clamp a value to given min and max values
        /*!
         * \note This template is forward-compatible to the C++17 std::clamp() implementation. It does, however, not support
         * an explicitly given compare function.
         *
         * \attention The behavior is undefined if min > max.
         *
         * \param [in] value The value to clamp
         * \param [in] min The minimum boundary for value
         * \param [in] max The maximum boundary for value
         * \returns A Reference to min if value < min, a reference to max if value > max, otherwise a reference to value
         */
        template <typename T>
        constexpr const T& clamp(const T& value, const T& min, const T& max)
        {
            return value < min ? min : value > max ? max : value;
        }

#ifdef HAVE_EIGEN3
        //! Finds a vector which is perpendicular to the given **non-zero** vector
        /*!
         * \note There is an infinite set of perpendicular vectors. This method selects **one** solution.
         * \warning The returned vector is **not** normalized!
         *
         * \param [in] vector A **non-zero** input vector
         * \return A vector which is perpendicular to the given input vector (or zero-vector if input vector is the zero vector)
         */
        static inline Eigen::Vector3d findPerpendicularVector(const Eigen::Vector3d& vector)
        {
            // Initialize helpers
            const Eigen::Vector3d vectorAbsolute(fabs(vector.x()), fabs(vector.y()), fabs(vector.z()));

            // Find largest coefficient (by absolute value)
            Eigen::Index maxCoefficientIndex = 0;
            if (vectorAbsolute.y() > vectorAbsolute(maxCoefficientIndex))
                maxCoefficientIndex = 1;
            if (vectorAbsolute.z() > vectorAbsolute(maxCoefficientIndex))
                maxCoefficientIndex = 2;
            if (vectorAbsolute(maxCoefficientIndex) == 0) {
                assert(false);
                return Eigen::Vector3d::Zero();
            }

            // Select solution depending on largest coefficient
            if (maxCoefficientIndex == 0)
                return Eigen::Vector3d(-vector.y() / vector.x() /* <-- in [-1, 1] */, 1.0, 0.0);
            else if (maxCoefficientIndex == 1)
                return Eigen::Vector3d(1.0, -vector.x() / vector.y() /* <-- in [-1, 1] */, 0.0);
            else
                return Eigen::Vector3d(1.0, 0.0, -vector.x() / vector.z() /* <-- in [-1, 1] */);
        }
#endif // HAVE_EIGEN3

        //! \}
    } // namespace math
} // namespace core
} // namespace broccoli
