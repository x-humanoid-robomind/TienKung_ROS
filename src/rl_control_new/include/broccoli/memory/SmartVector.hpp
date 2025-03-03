/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <algorithm>
#include <array>
#include <assert.h>
#include <exception>
#include <stdint.h>
#include <vector>
#ifdef HAVE_EIGEN3
#include <Eigen/StdVector> // Necessary for 128bit alginment and STL containers (for vectorization)
#endif // HAVE_EIGEN3

namespace broccoli {
namespace memory {
    /*!
     * \brief Hybrid container combining the advantages of `std::array` (static allocation) and `std::vector` (dynamic allocation)
     * \ingroup broccoli_memory
     * This class combines
     *   * a `std::array` for statically pre-allocated elements (for efficiency) and
     *   * a `std::vector` for dynamic extension by new elements (for flexibility).
     *
     * \attention The data is **NOT** stored continuously. This means that pointers to single elements can not be incremented/decremented!
     * Instead one should use the provided iterators (random access).
     *
     * \tparam T Element type
     * \tparam MaximumStaticElementCount Count of elements of the underlying `std::array`
     * \tparam Allocator The allocator to be used for the underlying `std::vector`
     */
    template <class T, size_t MaximumStaticElementCount = 0,
#ifdef HAVE_EIGEN3
        class Allocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class Allocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    class SmartVector {
        // Iterators
        // ---------
    public:
        //! Constant iterator for iterating over smart vectors (as Random Access Iterator)
        class ConstIterator {
        public:
            // Type definitions (required by STL)
            using iterator_category = std::random_access_iterator_tag;
            using value_type = T;
            using difference_type = int64_t;
            using pointer = T*;
            using reference = T&;
            // Own type definitions
            using container_type = SmartVector<T, MaximumStaticElementCount, Allocator>;

            // Construction
            // ------------
        public:
            //! Default constructor
            ConstIterator() = default;

            //! Specialized constructor
            ConstIterator(const container_type& container, const size_t& position = 0)
                : m_container(&container)
                , m_position(position)
            {
                checkContainer();
                if (m_position > m_container->size())
                    throw std::out_of_range("SmartVector::ConstIterator: position is out of range!");
            }

            // Comparison
            // ----------
        public:
            //! Comparison operator: **equality**
            inline bool operator==(const ConstIterator& reference) const
            {
                checkSameContainers(reference);
                return m_position == reference.m_position;
            }

            //! Comparison operator: **inequality**
            inline bool operator!=(const ConstIterator& reference) const
            {
                checkSameContainers(reference);
                return m_position != reference.m_position;
            }

            //! Comparison operator: **lesser than**
            inline bool operator<(const ConstIterator& reference) const
            {
                checkSameContainers(reference);
                return m_position < reference.m_position;
            }

            //! Comparison operator: **greater than**
            inline bool operator>(const ConstIterator& reference) const
            {
                checkSameContainers(reference);
                return m_position > reference.m_position;
            }

            //! Comparison operator: **lesser or equal than**
            inline bool operator<=(const ConstIterator& reference) const
            {
                checkSameContainers(reference);
                return m_position <= reference.m_position;
            }

            //! Comparison operator: **greater or equal than**
            inline bool operator>=(const ConstIterator& reference) const
            {
                checkSameContainers(reference);
                return m_position >= reference.m_position;
            }

            // Dereferencing
            // -------------
        public:
            //! Dereferencing (reference)
            inline const T& operator*() const
            {
                checkContainer();
                return m_container->at(m_position);
            }

            //! Dereferencing (pointer)
            inline const T* operator->() const
            {
                checkContainer();
                return &m_container->at(m_position);
            }

            //! Offset dereference operator
            inline const T& operator[](const difference_type& relativePosition) const
            {
                checkContainer();
                const difference_type absolutePosition = (difference_type)m_position + relativePosition;
                if (absolutePosition < 0)
                    throw std::out_of_range("SmartVector::ConstIterator: position is out of range (lower bound)!");
                return m_container->at((size_t)absolutePosition);
            }

            // Incrementation
            // --------------
        public:
            //! Prefix increment operator
            inline ConstIterator& operator++()
            {
                checkContainer();
                if (m_position < m_container->size())
                    m_position++;
                else
                    throw std::out_of_range("SmartVector::ConstIterator: can not increment iterator! The iterator already points to the end of the container!");
                return *this;
            }

            //! Postfix increment operator
            inline ConstIterator operator++(int)
            {
                ConstIterator copy(*this);
                operator++();
                return copy;
            }

            //! Prefix decrement operator
            inline ConstIterator& operator--()
            {
                if (m_position > 0)
                    m_position--;
                else
                    throw std::out_of_range("SmartVector::ConstIterator: can not decrement iterator! The iterator already points to the beginning of the container!");
                return *this;
            }

            //! Postfix decrement operator
            inline ConstIterator operator--(int)
            {
                ConstIterator copy(*this);
                operator--();
                return copy;
            }

            //! Compound assignment / addition
            inline ConstIterator& operator+=(const difference_type& value)
            {
                // Check, if operation is valid
                checkContainer();
                if (value > 0 /* (addition) */ && m_position + (size_t)value > m_container->size())
                    throw std::out_of_range("SmartVector::ConstIterator: invalid addition (hitting upper bound)!");
                if (value < 0 /* (subtraction) */ && (size_t)(-value) > m_position)
                    throw std::out_of_range("SmartVector::ConstIterator: invalid subtraction (hitting lower bound)!");

                // Perform addition (or subtraction)
                m_position += value;
                return *this;
            }

            //! Addition (iterator + value)
            inline ConstIterator operator+(const difference_type& value) const
            {
                ConstIterator copy(*this);
                return copy.operator+=(value);
            }

            //! Addition (value + iterator)
            friend inline ConstIterator operator+(const difference_type& value, const ConstIterator& iterator)
            {
                ConstIterator copy(iterator);
                return copy.operator+=(value);
            }

            //! Compound assignment / subtraction
            inline ConstIterator& operator-=(const difference_type& value) { return operator+=(-value); }

            //! Subtraction (iterator - value)
            inline ConstIterator operator-(const difference_type& value) const { return operator+(-value); }

            //! Difference (this - iterator)
            inline difference_type operator-(const ConstIterator& iterator) const
            {
                checkSameContainers(iterator);
                return ((difference_type)m_position - (difference_type)iterator.m_position);
            }

            // Members
            // -------
        protected:
            const container_type* m_container = nullptr; //!< Pointer to linked container
            size_t m_position = 0; //!< Current position the iterator is pointing to

            // Helpers
            // -------
        protected:
            //! Checks, if the pointer to the linked container is valid
            inline void checkContainer() const
            {
                if (m_container == nullptr)
                    throw std::runtime_error("SmartVector::ConstIterator: invalid pointer to container!");
            }

            //! Checks, if the given reference iterator is linked to the same container
            inline void checkSameContainers(const ConstIterator& reference) const
            {
                if (m_container != reference.m_container)
                    throw std::runtime_error("SmartVector::ConstIterator: both iterators must be linked to the same container!");
            }
        };

        //! Iterator for iterating over smart vectors
        class Iterator : public ConstIterator {
        public:
            using typename ConstIterator::container_type;
            using typename ConstIterator::difference_type;

            //! Default constructor
            Iterator() = default;

            //! Specialized constructor
            Iterator(container_type& container, const size_t& position = 0)
                : ConstIterator(container, position)
            {
            }

            // Dereferencing
            // -------------
        public:
            //! Dereferencing (reference)
            inline T& operator*() const { return const_cast<T&>(ConstIterator::operator*()); }

            //! Dereferencing (pointer)
            inline T* operator->() const { return const_cast<T*>(ConstIterator::operator->()); }

            //! Offset dereference operator
            inline T& operator[](const difference_type& relativePosition) const { return const_cast<T&>(ConstIterator::operator[](relativePosition)); }

            // Incrementation
            // --------------
        public:
            //! Prefix increment operator
            inline Iterator& operator++()
            {
                ConstIterator::operator++();
                return *this;
            }

            //! Postfix increment operator
            inline Iterator operator++(int)
            {
                Iterator copy(*this);
                operator++();
                return copy;
            }

            //! Prefix decrement operator
            inline Iterator& operator--()
            {
                ConstIterator::operator--();
                return *this;
            }

            //! Postfix decrement operator
            inline Iterator operator--(int)
            {
                Iterator copy(*this);
                operator--();
                return copy;
            }

            //! Compound assignment / addition
            inline Iterator& operator+=(const difference_type& value)
            {
                ConstIterator::operator+=(value);
                return *this;
            }

            //! Addition (iterator + value)
            inline Iterator operator+(const difference_type& value) const
            {
                Iterator copy(*this);
                return copy.operator+=(value);
            }

            //! Addition (value + iterator)
            friend inline Iterator operator+(const difference_type& value, const Iterator& iterator)
            {
                Iterator copy(iterator);
                return copy.operator+=(value);
            }

            //! Compound assignment / subtraction
            inline Iterator& operator-=(const difference_type& value) { return operator+=(-value); }

            //! Subtraction (iterator - value)
            inline Iterator operator-(const difference_type& value) const { return operator+(-value); }

            //! Difference (this - iterator)
            inline difference_type operator-(const Iterator& iterator) const { return ConstIterator::operator-(iterator); }
        };

        // Construction
        // ------------
    public:
        //! Default constructor - constructs an **empty** container
        SmartVector()
            : m_staticElementCount(0)
        {
        }

        //! Specialized constructor - constructs the container with `count` default-inserted instances of T. No copies are made.
        /*! \param [in] count The size of the container. */
        SmartVector(const size_t& count)
            : m_staticElementCount(0)
        {
            // Check desired size
            if (count <= MaximumStaticElementCount) {
                // Static buffer is sufficient...
                m_staticElementCount = count;
            } else {
                // We need the dynamic buffer...
                m_staticElementCount = MaximumStaticElementCount;
                m_dynamicBuffer = std::vector<T, Allocator>(count - MaximumStaticElementCount);
            }
        }

        //! Specialized constructor - constructs the container with `count` copies of elements with value `value`.
        /*!
         * \param [in] count The size of the container.
         * \param [in] value The value to initialize elements of the container with.
         */
        SmartVector(const size_t& count, const T& value)
            : m_staticElementCount(0)
        {
            // Check desired size
            if (count <= MaximumStaticElementCount) {
                // Static buffer is sufficient...
                m_staticElementCount = count;
                m_staticBuffer.fill(value);
            } else {
                // We need the dynamic buffer...
                m_staticElementCount = MaximumStaticElementCount;
                m_staticBuffer.fill(value);
                m_dynamicBuffer = std::vector<T, Allocator>(count - MaximumStaticElementCount, value);
            }
        }

        //! Specialized constructor - constructs the container with the contents of the initializer list `init`.
        /*! \param [in] init Initializer list to initialize the elements of the container with. */
        SmartVector(std::initializer_list<T> init)
        {
            // Check desired size
            if (init.size() <= MaximumStaticElementCount) {
                // Static buffer is sufficient...
                m_staticElementCount = init.size();
                std::copy(init.begin(), init.end(), m_staticBuffer.begin());
            } else {
                // We need the dynamic buffer...
                m_staticElementCount = MaximumStaticElementCount;
                m_dynamicBuffer.reserve(init.size() - MaximumStaticElementCount);
                std::copy_n(init.begin(), MaximumStaticElementCount, m_staticBuffer.begin());
                std::copy(init.begin() + MaximumStaticElementCount, init.end(), std::back_inserter(m_dynamicBuffer));
            }
        }

        //! Copy constructor
        SmartVector(const SmartVector& other)
            : m_staticElementCount(other.m_staticElementCount)
            , m_dynamicBuffer(other.m_dynamicBuffer)
        {
            for (size_t i = 0; i < other.m_staticElementCount; i++) // Copy only "filled" elements in static buffer (skip possibly uninitialized elements)
                m_staticBuffer[i] = other.m_staticBuffer[i];
        }

        //! Copy assignment operator
        SmartVector& operator=(const SmartVector& other)
        {
            if (this == &other)
                return *this;
            for (size_t i = 0; i < other.m_staticElementCount; i++) // Copy only "filled" elements in static buffer (skip possibly uninitialized elements)
                m_staticBuffer[i] = other.m_staticBuffer[i];
            m_staticElementCount = other.m_staticElementCount;
            m_dynamicBuffer = other.m_dynamicBuffer;
            return *this;
        }

        //! Move constructor
        SmartVector(SmartVector&& other)
            : m_staticElementCount(std::move(other.m_staticElementCount))
            , m_dynamicBuffer(std::move(other.m_dynamicBuffer))
        {
            for (size_t i = 0; i < m_staticElementCount; i++) // Move only "filled" elements in static buffer (skip possibly uninitialized elements)
                m_staticBuffer[i] = std::move(other.m_staticBuffer[i]);
        }

        //! Move assignment operator
        SmartVector& operator=(SmartVector&& other)
        {
            if (this == &other)
                return *this;
            for (size_t i = 0; i < other.m_staticElementCount; i++) // Move only "filled" elements in static buffer (skip possibly uninitialized elements)
                m_staticBuffer[i] = std::move(other.m_staticBuffer[i]);
            m_staticElementCount = std::move(other.m_staticElementCount);
            m_dynamicBuffer = std::move(other.m_dynamicBuffer);
            return *this;
        }

        // Members
        // -------
    protected:
        std::array<T, MaximumStaticElementCount> m_staticBuffer; //!< Buffer for **statically** allocated elements
        size_t m_staticElementCount; //!< Total count of elements in \ref m_staticBuffer
        std::vector<T, Allocator> m_dynamicBuffer; //!< Buffer for **dynamically** allocated elements

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const SmartVector& reference) const
        {
            // Compare static buffer
            if (m_staticElementCount != reference.m_staticElementCount)
                return false;
            for (size_t i = 0; i < m_staticElementCount; i++) // Compare only "filled" elements in static buffer (skip possibly uninitialized elements)
                if (m_staticBuffer[i] != reference.m_staticBuffer[i])
                    return false;

            // Compare dynamic buffer
            if (m_dynamicBuffer != reference.m_dynamicBuffer)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const SmartVector& reference) const { return !(*this == reference); }

        // Element access
        // --------------
    public:
        //! Returns a reference to the element at specified location, **with** bounds checking.
        /*!
         * \param [in] pos Position of the element to return
         */
        const T& at(const size_t& pos) const
        {
            // Check boundaries
            if (pos >= size()) {
                assert(false);
                throw std::out_of_range("SmartVector:: index out of range!");
            }

            // Pass back element
            return this->operator[](pos);
        }

        //! Returns a reference to the element at specified location, **with** bounds checking.
        /*!
         * \param [in] pos Position of the element to return
         */
        T& at(const size_t& pos) { return const_cast<T&>(static_cast<const SmartVector&>(*this).at(pos)); }

        //! Returns a reference to the element at specified location, **without** bounds checking.
        /*!
         * \param [in] pos Position of the element to return
         */
        const T& operator[](const size_t& pos) const
        {
            // Check, if element is in static or dynamic buffer
            if (pos < MaximumStaticElementCount)
                return m_staticBuffer[pos];
            else {
                return m_dynamicBuffer[pos - MaximumStaticElementCount];
            }
        }

        //! Returns a reference to the element at specified location, **without** bounds checking.
        /*!
         * \param [in] pos Position of the element to return
         */
        T& operator[](const size_t& pos) { return const_cast<T&>(static_cast<const SmartVector&>(*this)[pos]); }

        //! Returns a reference to the first element in the container.
        /*!
         * \warning Calling front on an empty container is undefined.
         */
        const T& front() const { return this->operator[](0); }

        //! Returns a reference to the first element in the container.
        /*!
         * \warning Calling front on an empty container is undefined.
         */
        T& front() { return this->operator[](0); }

        //! Returns a reference to the last element in the container.
        /*!
         * \warning Calling back on an empty container is undefined.
         */
        const T& back() const { return this->operator[](size() - 1); }

        //! Returns a reference to the last element in the container.
        /*!
         * \warning Calling back on an empty container is undefined.
         */
        T& back() { return this->operator[](size() - 1); }

        // Iterators
        // ---------
    public:
        //! Returns an iterator to the beginning
        Iterator begin() { return Iterator(*this, 0); }

        //! Returns an iterator to the end
        Iterator end() { return Iterator(*this, size()); }

        //! Returns a const iterator to the beginning
        ConstIterator cbegin() const { return ConstIterator(*this, 0); }

        //! Returns a const iterator to the end
        ConstIterator cend() const { return ConstIterator(*this, size()); }

        // Capacity
        // --------
    public:
        //! Checks if the container has no elements
        bool empty() const
        {
            // Compute from size
            if (size() == 0)
                return true;
            else
                return false;
        }

        //! Returns the number of elements in the static buffer
        size_t staticBuffer_size() const { return m_staticElementCount; }

        //! Returns the number of elements in the dynamic buffer
        size_t dynamicBuffer_size() const { return m_dynamicBuffer.size(); }

        //! Returns the number of elements in the container
        size_t size() const { return staticBuffer_size() + dynamicBuffer_size(); }

        //! Returns the maximum number of elements the static buffer is able to hold.
        size_t staticBuffer_max_size() const { return MaximumStaticElementCount; }

        //! Returns the maximum number of elements the dynamic buffer is able to hold due to system or library implementation limitations.
        size_t dynamicBuffer_max_size() const { return m_dynamicBuffer.max_size(); }

        //! Returns the maximum number of elements the container is able to hold due to system or library implementation limitations.
        size_t max_size() const { return staticBuffer_max_size() + dynamicBuffer_max_size(); }

        //! Increase the capacity of the vector to a value that's greater or equal to `new_cap`.
        /*!
         * \param [in] new_cap New capacity of the vector.
         */
        void reserve(const size_t& new_cap)
        {
            // Check, if new capacity exceeds static buffer size
            if (new_cap > MaximumStaticElementCount)
                m_dynamicBuffer.reserve(new_cap - MaximumStaticElementCount);
            // else: nothing to do
        }

        //! Returns the number of elements that the container has currently allocated space for.
        size_t capacity() const { return MaximumStaticElementCount + m_dynamicBuffer.capacity(); }

        //! Requests the removal of unused capacity.
        void shrink_to_fit() { m_dynamicBuffer.shrink_to_fit(); }

        // Modifiers
        // ---------
    public:
        //! Erases all elements from the container.
        void clear()
        {
            // Clear static buffer
            m_staticElementCount = 0;

            // Clear dynamic buffer
            m_dynamicBuffer.clear();
        }

        //! Assigns the given value to all elements in the container.
        /*!
         * \param [in] value The value to assign to the elements.
         */
        void fill(const T& value)
        {
            // Fill static buffer
            for (size_t i = 0; i < m_staticElementCount; i++)
                m_staticBuffer[i] = value;

            // Fill dynamic buffer
            for (size_t i = 0; i < m_dynamicBuffer.size(); i++)
                m_dynamicBuffer[i] = value;
        }

        //! Appends the given element to the end of the container.
        /*!
         * The new element is initialized as a copy of \p value.
         *
         * \param [in] value The value of the element to append.
         */
        void push_back(const T& value)
        {
            // Check, if static buffer is not full
            if (m_staticElementCount < MaximumStaticElementCount) {
                // Append to static buffer
                m_staticBuffer[m_staticElementCount] = value;
                m_staticElementCount++;
            } else
                m_dynamicBuffer.push_back(value);
        }

        //! Appends the given element to the end of the container.
        /*!
         * \p value is moved into the new element.
         *
         * \param [in] value The value of the element to append.
         */
        void push_back(T&& value)
        {
            // Check, if static buffer is not full
            if (m_staticElementCount < MaximumStaticElementCount) {
                // Append to static buffer
                m_staticBuffer[m_staticElementCount] = value;
                m_staticElementCount++;
            } else
                m_dynamicBuffer.push_back(std::forward<T>(value));
        }

        //! Appends a new element to the end of the container.
        /*! \param [in] args Arguments to forward to the constructor of the element */
        template <typename... Args>
        void emplace_back(Args&&... args)
        {
            // Check, if static buffer is not full
            if (m_staticElementCount < MaximumStaticElementCount) {
                // Append to static buffer
                m_staticBuffer[m_staticElementCount] = T(std::forward<Args>(args)...);
                m_staticElementCount++;
            } else
                m_dynamicBuffer.emplace_back(std::forward<Args>(args)...);
        }

        //! Removes the last element of the container.
        void pop_back()
        {
            // Check, if last element is in the dynamic buffer
            if (m_dynamicBuffer.size() > 0)
                m_dynamicBuffer.pop_back(); // Remove last element from dynamic buffer
            else if (m_staticElementCount > 0)
                m_staticElementCount--; // Remove last element from static buffer
        }

        //! Resizes the container to contain the specified count of elements.
        /*!
         * If the current size is less than \p count, additional default-inserted elements are appended.
         *
         * \param [in] count New size of the container.
         */
        void resize(const size_t& count) { resizeGeneralized(count, nullptr); }

        //! Resizes the container to contain the specified count of elements.
        /*!
         * If the current size is less than \p count, additional copies of `value` are appended.
         *
         * \param [in] count New size of the container.
         * \param [in] value The value to initialize the new elements with.
         */
        void resize(const size_t& count, const T& value) { resizeGeneralized(count, &value); }

        // Getters
        // -------
    public:
        //! Getter for \ref m_staticBuffer
        const std::array<T, MaximumStaticElementCount>& staticBuffer() const { return m_staticBuffer; }

        //! Getter for \ref m_staticElementCount
        const size_t& staticElementCount() const { return m_staticElementCount; }

        //! Getter for \ref m_dynamicBuffer;
        const std::vector<T, Allocator>& dynamicBuffer() const { return m_dynamicBuffer; }

        // Helpers
        // -------
    protected:
        //! Resizes the container to contain the specified count of elements.
        /*!
         * If the current size is less than \p count, additional elements are appended which are
         *  * copies of \p value if \p value is not `nullptr` or
         *  * default constructed if \p value is `nullptr`.
         *
         * \param [in] count New size of the container.
         * \param [in] value Pointer to the value to initialize the new elements with (use `nullptr` for default initialization of new elements).
         *
         * \remark This is a **generalized** function which is used internally to provide a consistent interface (compare `std::vector::resize()`).
         */
        void resizeGeneralized(const size_t& count, const T* const value = nullptr)
        {
            // Initialize helpers
            const size_t oldSize = size();

            // Check, if we need to add or remove elements
            if (count > oldSize) {
                // ...we need to add elements

                // Check, if static buffer is already full
                if (m_staticElementCount >= MaximumStaticElementCount) {
                    // ...static buffer is already full -> resize dynamic buffer only
                    if (value == nullptr)
                        m_dynamicBuffer.resize(count - MaximumStaticElementCount);
                    else
                        m_dynamicBuffer.resize(count - MaximumStaticElementCount, *value);
                } else {
                    // ...static buffer is not full (yet)

                    // Check, if static buffer size is sufficient
                    if (count <= MaximumStaticElementCount) {
                        // ...static buffer size is sufficient -> just append new elements to static buffer
                        for (size_t i = m_staticElementCount; i < count; i++) {
                            if (value == nullptr)
                                m_staticBuffer[i] = T(); // Default-constructed elements
                            else
                                m_staticBuffer[i] = *value; // Copies of the given element
                        }
                        m_staticElementCount = count;
                    } else {
                        // ...static buffer size is not sufficient -> fill static buffer and additionally add elements to the dynamic buffer
                        for (size_t i = m_staticElementCount; i < MaximumStaticElementCount; i++) {
                            if (value == nullptr)
                                m_staticBuffer[i] = T(); // Default-constructed elements
                            else
                                m_staticBuffer[i] = *value; // Copies of the given element
                        }
                        m_staticElementCount = MaximumStaticElementCount;
                        if (value == nullptr)
                            m_dynamicBuffer.resize(count - MaximumStaticElementCount);
                        else
                            m_dynamicBuffer.resize(count - MaximumStaticElementCount, *value);
                    }
                }
            } else if (count < oldSize) {
                // ...we need to remove elements

                // Check, if static buffer is sufficient for new count
                if (count <= MaximumStaticElementCount) {
                    // ...yes, static buffer is sufficient
                    m_staticElementCount = count;
                    m_dynamicBuffer.clear();
                } else {
                    // ...no, we still need the dynamic buffer
                    m_dynamicBuffer.resize(count - MaximumStaticElementCount);
                }
            }
            // else: nothing to do!
        }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };
} // namespace memory
} // namespace broccoli
