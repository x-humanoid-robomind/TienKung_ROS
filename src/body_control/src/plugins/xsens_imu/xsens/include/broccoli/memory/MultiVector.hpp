/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <array>
#include <assert.h>
#include <exception>
#include <vector>
#ifdef HAVE_EIGEN3
#include <Eigen/StdVector> // Necessary for 128bit alginment and STL containers (for vectorization)
#endif // HAVE_EIGEN3

namespace broccoli {
namespace memory {
    //! Abstraction of a multi-dimensional vector/array
    /*!
     * \ingroup broccoli_memory
     * Stores the multi-dimensional data as stacked vector and handles the access to elements through indices.
     * Internally uses `std::vector` for (dynamic) memory allocation.
     *
     * \tparam T Type of each element
     * \tparam N Count of dimensions
     * \tparam Allocator Allocator used to create new elements
     */
    template <typename T, unsigned int N,
#ifdef HAVE_EIGEN3
        class Allocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class Allocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    class MultiVector {
    public:
        static_assert(N > 0, "Count of dimensions has to be greater than zero!");

        //! Default constructor - constructs an **empty** container
        MultiVector()
        {
        }

        //! Specialized constructor (allocates memory for specified dimensions)
        /*! \param [in] size List of sizes for all dimensions of the vector */
        MultiVector(const std::array<size_t, N>& size)
            : MultiVector()
        {
            resize(size);
        }

        //! Specialized constructor (allocates memory for specified dimensions)
        /*!
         * \param [in] firstSize Size of first dimension
         * \param [in] remainingSizes List of sizes of remaining dimensions
         */
        template <typename... Targs>
        MultiVector(const size_t& firstSize, const Targs&... remainingSizes)
            : MultiVector()
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of sizes has to match dimension!");
            resize(firstSize, static_cast<size_t>(remainingSizes)...);
        }

    protected:
        // Members
        // -------
        std::array<size_t, N> m_size{}; //!< \copybrief size()
        std::vector<T, Allocator> m_elements; //!< Element buffer (stacked buffer for all dimensions)

    public:
        // Operators
        // ---------
        //! Comparison operator: **equality**
        bool operator==(const MultiVector& reference) const
        {
            // Compare buffers
            if (m_size != reference.m_size || m_elements != reference.m_elements)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const MultiVector& reference) const { return !(*this == reference); }

        // Element mapping
        // ---------------
        //! Computes the index in the **element buffer** from the given dimensional indices
        size_t elementBufferIndex(const std::array<size_t, N>& indices) const
        {
            // Search for element in element buffer
            size_t returnValue = 0;
            for (size_t i = 0; i < N; i++) {
                // Check boundaries
                if (indices[i] >= m_size[i]) {
                    assert(false);
                    throw std::out_of_range("index out of range");
                }

                // Compute element buffer position
                if (i + 1 == N) {
                    // Last dimension -> no sub-elements...
                    returnValue += indices[i];
                } else {
                    // Compute total count of sub-elements for one "entry" of this dimension
                    size_t totalSubElementCount = 1;
                    for (size_t j = i + 1; j < N; j++)
                        totalSubElementCount *= m_size[j];
                    returnValue += indices[i] * totalSubElementCount;
                }
            }

            // Pass back element buffer index
            return returnValue;
        }

        //! Computes the index in the **element buffer** from the given dimensional indices
        /*!
         * \param [in] firstIndex Index in first dimension
         * \param [in] remainingIndices List of indices in remaining dimensions
         */
        template <typename... Targs>
        size_t elementBufferIndex(const size_t& firstIndex, const Targs&... remainingIndices) const
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of indices has to match dimension!");
            return elementBufferIndex(std::array<size_t, N>{ { firstIndex, static_cast<size_t>(remainingIndices)... } });
        }

        //! Computes the **dimensional indices** from the given index in the element buffer
        std::array<size_t, N> indices(const size_t& elementBufferIndex) const
        {
            // Initialize return value
            std::array<size_t, N> returnValue{};

            // Check boundaries
            if (elementBufferIndex >= m_elements.size()) {
                assert(false);
                throw std::out_of_range("index out of range");
            }

            // Pass through all dimensions in the reverse order
            size_t remainder = elementBufferIndex;
            for (size_t i = N - 1; i >= 1; i--) {
                returnValue[i] = remainder % m_size[i];
                remainder /= m_size[i];
            }
            returnValue[0] = remainder;

            // Pass back indices
            return returnValue;
        }

        // Element access
        // --------------
        //! Returns a reference to the element at the specified location (**with** bounds checking)
        /*! \param [in] indices List of indices to specify the location of the desired element */
        const T& operator()(const std::array<size_t, N>& indices) const { return m_elements[elementBufferIndex(indices)]; }

        //! \copydoc operator()(const std::array<size_t, N>&) const
        T& operator()(const std::array<size_t, N>& indices) { return const_cast<T&>(static_cast<const MultiVector&>(*this)(indices)); }

        //! Returns a reference to the element at the specified location (**with** bounds checking)
        /*!
         * \param [in] firstIndex Index in first dimension
         * \param [in] remainingIndices List of indices in remaining dimensions
         */
        template <typename... Targs>
        const T& operator()(const size_t& firstIndex, const Targs&... remainingIndices) const
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of indices has to match dimension!");
            return operator()(std::array<size_t, N>{ { firstIndex, static_cast<size_t>(remainingIndices)... } });
        }

        //! Returns a reference to the element at the specified location (**with** bounds checking)
        /*!
         * \param [in] firstIndex Index in first dimension
         * \param [in] remainingIndices List of indices in remaining dimensions
         */
        template <typename... Targs>
        T& operator()(const size_t& firstIndex, const Targs&... remainingIndices) { return const_cast<T&>(static_cast<const MultiVector&>(*this)(firstIndex, remainingIndices...)); }

        //! Returns a reference to the element at the specified location **in the stacked element buffer**, **without** bounds checking.
        /*! \param [in] elementBufferIndex Position of the element **in the stacked element buffer** */
        const T& operator[](const size_t& elementBufferIndex) const { return m_elements[elementBufferIndex]; }

        //! \copydoc operator[](const size_t&) const
        T& operator[](const size_t& elementBufferIndex) { return const_cast<T&>(static_cast<const MultiVector&>(*this)[elementBufferIndex]); }

        // Capacity
        // --------
        //! List of sizes for each dimension
        const std::array<size_t, N>& size() const { return m_size; }

        //! Total count of elements in the buffer (all dimensions)
        size_t elementCount() const { return m_elements.size(); }

        // Modifiers
        // ---------
        //! Clears the vector (all dimensions)
        /*! \param [in] freeElementBuffer If `true` the reserved memory of the element buffer is freed (may be useful for large data) */
        void clear(const bool& freeElementBuffer = false)
        {
            m_size.fill(0);
            if (freeElementBuffer == false)
                m_elements.clear(); // <- keeps capacity
            else
                std::vector<T, Allocator>().swap(m_elements); // <- frees already allocated memory
        }

        //! Assigns the given value to all elements in the buffer.
        /*! \param [in] value The value to assign to the elements. */
        void fill(const T& value)
        {
            for (size_t i = 0; i < m_elements.size(); i++)
                m_elements[i] = value;
        }

        //! Resizes the vector (multi-dimension)
        /*!
         * \param [in] size List of sizes for all dimensions of the vector
         * \return `true` on success, `false` otherwise (allocation failed)
         */
        bool resize(const std::array<size_t, N>& size)
        {
            // Copy size of dimensions
            m_size = size;

            // Clear element buffer
            m_elements.clear();

            // Compute element buffer size (total count of elements)
            size_t elementBufferSize = 1;
            for (size_t i = 0; i < N; i++)
                elementBufferSize *= m_size[i];

            // Resize element buffer
            try {
                m_elements.resize(elementBufferSize);
            } catch (...) {
                // An error occured (most likely the system memory is not big enough (out of memory))
                clear();
                assert(false);
                return false;
            }

            // Success
            return true;
        }

        //! Resizes the vector (multi-dimension)
        /*!
         * \param [in] firstSize Size of first dimension
         * \param [in] remainingSizes List of remaining dimension sizes in the buffer
         * \return `true` on success, `false` otherwise (allocation failed)
         */
        template <typename... Targs>
        bool resize(const size_t& firstSize, const Targs&... remainingSizes)
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of sizes has to match dimension!");
            return resize(std::array<size_t, N>{ { firstSize, static_cast<size_t>(remainingSizes)... } });
        }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif // HAVE_EIGEN3
    };
} // namespace memory
} // namespace broccoli
