/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#ifdef HAVE_EIGEN3
#include <Eigen/StdVector> // Necessary for 128bit alginnment and STL containers (for vectorization)
#endif // HAVE_EIGEN3
#include "../parallel/ThreadSafeContainer.hpp"
#include <vector>

// broccoli libraries
// ...

namespace broccoli {
namespace memory {
    //! Generic template for **thread-safe** cicular buffers
    /*!
     * \ingroup broccoli_memory
     *
     * Allocates and manages memory of a cicular buffer with elements of the specified type \p T. Behaves like an infinite list of elements where
     * dynamic allocation at runtime is avoided. The data is protected by a mutex for thread-safety. Aside from getters/setters also copying
     * or assigning the whole circular buffer class is thread-safe.
     *
     * One can specify the allocator to be used for building the buffer by the optional template parameter \p BufferAllocator. If no allocator is
     * specified the standard allocator or Eigen-aligned-allocater (in case Eigen is available) is used.
     *
     * One can add new elements to the buffer by calling \ref push(). This adds the specified element(s) to the end of the list ("newest elements").
     * One can get the first ("oldest") element(s) of the buffer by calling \ref pop(). This will simultaneously remove the element(s) from the buffer.
     * Additionally on can get the last ("newest") element of the buffer by calling \ref popNewestAndClear(). This will clear all elements from the buffer.
     */
    template <class T,
#ifdef HAVE_EIGEN3
        class BufferAllocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class BufferAllocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    class CircularBuffer : public parallel::ThreadSafeContainer {
    public:
        //! Default constructor
        /*!
         * Initialization and allocation of buffer memory. The memory stays in the heap until the instance is destroyed.
         * \param [in] maximumElementCount Maximum count of elements the buffer should be able to hold.
         */
        CircularBuffer(const size_t& maximumElementCount)
            : ThreadSafeContainer()
            , m_maximumElementCount(maximumElementCount > 0 ? maximumElementCount : 1)
            , m_newElementIndex(0) // Start with the first position in the buffer
            , m_elementCount(0) // Initialize as "empty"/"free"
            , m_totalOverwrittenElements(0)
        {
            m_buffer.reserve(m_maximumElementCount);
        }

    protected:
        //! Copy constructor (internal)
        /*!
         * \warning **Not thread-safe** -> should only be called by the thread-safe wrapper!
         *
         * \param [in] original Reference to original object.
         */
        CircularBuffer(const CircularBuffer& original, const int& /* <- trick used for locking mutex */)
            : ThreadSafeContainer(original)
            , m_buffer(original.m_buffer)
            , m_maximumElementCount(original.m_maximumElementCount)
            , m_newElementIndex(original.m_newElementIndex)
            , m_elementCount(original.m_elementCount)
            , m_totalOverwrittenElements(original.m_totalOverwrittenElements)
        {
        }

    public:
        //! Copy constructor (wrapper) (thread-safe)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        CircularBuffer(const CircularBuffer& original)
            : CircularBuffer(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (thread-safe)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         *
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        CircularBuffer& operator=(const CircularBuffer& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            ThreadSafeContainer::operator=(reference);
            m_buffer = reference.m_buffer;
            m_maximumElementCount = reference.m_maximumElementCount;
            m_newElementIndex = reference.m_newElementIndex;
            m_elementCount = reference.m_elementCount;
            m_totalOverwrittenElements = reference.m_totalOverwrittenElements;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

    protected:
        //! Move constructor (internal)
        /*!
         * \warning **Not thread-safe** -> should only be called by the thread-safe wrapper!
         *
         * \param [in] other Reference to other object.
         */
        CircularBuffer(CircularBuffer&& other, const int& /* <- trick used for locking mutex */)
            : ThreadSafeContainer(std::move(other))
            , m_buffer(std::move(other.m_buffer))
            , m_maximumElementCount(std::move(other.m_maximumElementCount))
            , m_newElementIndex(std::move(other.m_newElementIndex))
            , m_elementCount(std::move(other.m_elementCount))
            , m_totalOverwrittenElements(std::move(other.m_totalOverwrittenElements))
        {
        }

    public:
        //! Move constructor (wrapper) (thread-safe)
        /*!
         * Locks the mutex of the other object for writing before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] other Reference to other object.
         */
        CircularBuffer(CircularBuffer&& other)
            : CircularBuffer(other, other.lockForWrite() /* <- lock mutex of original object first (for writing since we move the object) */)
        {
            other.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Move assignment operator (thread-safe)
        /*!
         * Uses own mutex and mutex of the other object to guarantee thread-safe moving of members.
         *
         * \param [in] other Reference to other object.
         * \return Pointer to this instance.
         */
        CircularBuffer& operator=(CircularBuffer&& other)
        {
            // Avoid self-assignment
            if (this == &other)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // Spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (other.tryLockForWrite() == true) // Try to lock reference for writing
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Move data
            ThreadSafeContainer::operator=(std::move(other));
            m_buffer = std::move(other.m_buffer);
            m_maximumElementCount = std::move(other.m_maximumElementCount);
            m_newElementIndex = std::move(other.m_newElementIndex);
            m_elementCount = std::move(other.m_elementCount);
            m_totalOverwrittenElements = std::move(other.m_totalOverwrittenElements);

            // Unlock other object and ourselves
            other.unlock();
            unlock();

            return *this;
        }

        //! Clear all elements from buffer (thread-safe)
        /*!
         * Marks all positions in the buffer as "free". The deprecated elements in the buffer are **not** overwritten but keep the old data!
         * \remark Very cheap operation.
         */
        void clear()
        {
            // Lock protected memory for write
            lockForWrite();

            // Mark every position of the buffer as "free"
            m_newElementIndex = 0;
            m_elementCount = 0;

            // Unlock protected memory
            unlock();
        }

        //! Clear all elements from buffer **and** reallocate memory (thread-safe)
        /*!
         * Marks all positions in the buffer as "free" and reallocates the complete buffer.
         *
         * \warning Slow since we have to recreate the whole buffer. May be used if elements are of complex type holding their own data containers.
         * In this case a reallocation would free unused memory "hidden" by the element containers (not located in the circular buffer).
         */
        void reAllocate()
        {
            // Lock protected memory for write
            lockForWrite();

            // Reallocate memory and mark every position as "free"
            m_buffer = std::vector<T, BufferAllocator>();
            m_buffer.reserve(m_maximumElementCount);
            m_newElementIndex = 0;
            m_elementCount = 0;

            // Unlock protected memory
            unlock();
        }

        //! Adds a single element to the buffer (thread-safe).
        /*!
         * Copies the data of the given element to the next free position in the buffer.
         * If there is no free position left it overwrites the "oldest" element (first element in list).
         * \param [in] newElement Reference to the new element to add to the buffer.
         * \return Count of overwritten elements in case the buffer was not big enough (if 0 -> buffer was big enough).
         */
        size_t push(const T& newElement)
        {
            // Lock protected memory for write
            lockForWrite();

            // Add single element to buffer
            size_t overwrittenElements = pushElementToBuffer(newElement);

            // Unlock protected memory
            unlock();

            // Return count of overwritten elements
            return overwrittenElements;
        }

        //! Adds a vector of elements to the buffer (thread-safe).
        /*!
         * Copies the data of the given elements to the next free positions in the buffer (preserves order).
         * If there is no free position left it overwrites the "oldest" elements (first elements in list).
         * \param [in] newElements Reference to the vector containing new elements to add to the buffer (adds the first "count" elements).
         * \param [in] count Count of elements to add from the given reference (use 0 (default) to add ALL).
         * \return Count of overwritten elements in case the buffer was not big enough (if 0 -> buffer was big enough).
         */
        size_t push(const std::vector<T, BufferAllocator>& newElements, const size_t& count = 0)
        {
            // Initialize helpers
            size_t overwrittenElements = 0; // How many elements have been overwritten?
            size_t numberOfElementsToPush = count; // How many elements should be added?
            if (numberOfElementsToPush == 0 || numberOfElementsToPush > newElements.size())
                numberOfElementsToPush = newElements.size();

            // Lock protected memory for write
            lockForWrite();

            // Add elements to buffer
            for (size_t i = 0; i < numberOfElementsToPush; i++)
                overwrittenElements += pushElementToBuffer(newElements[i]);

            // Unlock protected memory
            unlock();

            // Return count of overwritten elements
            return overwrittenElements;
        }

        //! Gets the "oldest" element (first in list) from the buffer (thread-safe).
        /*!
         * Copies the data of the "oldest" element (first in list) in the buffer to the given reference element and marks
         * the position in the buffer as "free". Skips, if there are no elements in the list.
         * \param [out] targetElement Reference to the target element in which the data of the "oldest" element should be copied.
         * \return Count of elements actually popped from the buffer (if 0 -> buffer was empty).
         */
        size_t pop(T& targetElement)
        {
            // Lock protected memory for write
            lockForWrite();

            // Pop single element from buffer
            size_t poppedElements = popElementFromBuffer(targetElement);

            // Unlock protected memory
            unlock();

            // Return count of popped elements
            return poppedElements;
        }

        //! Gets a vector of the "oldest" elements from the buffer (thread-safe).
        /*!
         * Copies the data of the "count" "oldest" elements (first in list) in buffer to the given reference element vector and marks
         * the positions in the buffer as "free". Skips, if there are no elements in the list.
         *
         * \par Case 1:
         * The size of the given target vector is too small to hold requested elements...
         *  - Case 1a: ...but the capacity is big enough (allocated memory is sufficient) -> target vector is resized without reallocation (**fast**)
         *  - Case 1b: ...and the capacity is not big enough (allocated memory is insufficient) -> complete target vector is recreated and new allocated (**slow**)
         *
         * \par Case 2:
         * The size of the given target vector is big enough to store all requested elements
         * -# The target vector is resized if the size is bigger as necessary (capacity stays untouched, no re-allocation!)
         * -# The requested elements are copied to the (front) of the target vector.
         *
         * \param [out] targetElements Reference to the vector of elements in which the data of the "oldest" elements should be copied.
         * \param [in] count Count of elements to pop from the buffer (use 0 to add **all**).
         * \return Count of elements actually popped from the buffer (if 0 -> buffer was empty).
         */
        size_t pop(std::vector<T, BufferAllocator>& targetElements, const size_t& count = 0)
        {
            // Initialize helpers
            size_t poppedElements = 0; // How many elements have been popped from the buffer?
            size_t numberOfElementsToPop = count; // How many elements should be popped?

            // Pre-allocate memory for target element buffer (if we know the count of elements to pop in advance)
            if (count > 0)
                targetElements.reserve(count);

            // Lock protected memory for write
            lockForWrite();

            // Compute how many pop() operations we should perform
            if (numberOfElementsToPop == 0 || numberOfElementsToPop > m_elementCount)
                numberOfElementsToPop = m_elementCount;

            // Resize target vectors size
            targetElements.resize(numberOfElementsToPop);
            // Case 1a: filling up with dummy elements (FAST)
            // Case 1b: reallocation (SLOW)
            // Case 2: removes extra elements (FAST)

            // Pop requested elements from buffer
            for (size_t i = 0; i < numberOfElementsToPop; i++)
                poppedElements += popElementFromBuffer(targetElements[i]);

            // Unlock protected memory
            unlock();

            // Return count of popped elements
            return poppedElements;
        }

        //! Gets the "newest" element (last in list) from the buffer (thread-safe).
        /*!
         * \warning Discards all other elements!
         *
         * Copies the data of the "newest" element (last in list) in the buffer to the given reference element and clears
         * the complete buffer (elements are discarded without beeing copied!). Skips, if there are no elements in the list.
         * \param [out] targetElement Reference to the target element in which the data of the "newest" element should be copied.
         * \return Count of elements popped from the buffer (if 0: buffer was empty, if 1: there was only one element, if >1: (count-1) elements got discarded).
         */
        size_t popNewestAndClear(T& targetElement)
        {
            // Lock protected memory for write
            lockForWrite();

            // All elements are cleared
            size_t poppedElements = m_elementCount;

            // Check, element count
            if (m_elementCount == 0) {
                // ...nothing to do
            } else if (m_elementCount == 1) {
                // ...only one element in buffer
                popElementFromBuffer(targetElement); // Pop single element from buffer
            } else {
                // ...more than one element in buffer
                m_elementCount = 1; // Clear all elements except for the last in list ("newest")
                popElementFromBuffer(targetElement); // Pop single element from buffer
            }

            // Unlock protected memory
            unlock();

            // Return count of popped elements
            return poppedElements;
        }

    private:
        // Start protected area
        std::vector<T, BufferAllocator> m_buffer; //!< Actual buffer holding the "payload".
        size_t m_maximumElementCount; //!< Maximum count of elements in buffer ( = exact size(buffer) = minimum capacity(buffer))
        size_t m_newElementIndex; //!< Index in buffer where new elements should be written to.
        size_t m_elementCount; //!< Count of currently stored elements in buffer.
        unsigned long long m_totalOverwrittenElements; //!< Total count of overwritten elements.
        // End protected area

        // Internal helpers
        // ----------------
        //! Adds a single element to the buffer (**not** thread-safe)
        /*!
         * Copies the data of the given element to the next free place in the buffer.
         * If there is no free position left it overwrites the "oldest element" (first element in list).
         * \param [in] newElement Reference to the new element to add to the buffer.
         * \return Count of overwritten elements in case the buffer was not big enough (if 0 -> buffer was big enough).
         */
        size_t pushElementToBuffer(const T& newElement)
        {
            // Copy new element to buffer (possibly overwrites oldest non-free element in buffer)
            if (m_buffer.size() > m_newElementIndex)
                m_buffer[m_newElementIndex] = newElement; // Calls copy assignment operator of element
            else
                m_buffer.push_back(newElement); // Calls copy constructor of element

            // Increment (circular) writing position in buffer
            m_newElementIndex++;
            if (m_newElementIndex == m_maximumElementCount) // Have we reached the end of the buffer?
                m_newElementIndex = 0; // ...yes -> jump back to front

            // Check, if buffer was already full and we have overwritten an element
            if (m_elementCount == m_maximumElementCount) {
                // ... yes -> we have overwritten an element
                // (element count stays the same as we have not ADDED but REPLACED an element)
                m_totalOverwrittenElements++;
                return 1;
            } else {
                // ... no -> we wrote to a free position
                // (element count increases as we really ADDED an element)
                m_elementCount++;
                return 0;
            }
        }

        //! Gets a single element from the buffer (**not** thread-safe)
        /*!
         * Copies the data of the "oldest" element (first in list) in buffer to the given reference element and marks
         * the position in the buffer as "free". Skips, if there are no elements in the list.
         * \param [out] targetElement Reference to the target element in which the data of the "oldest" element should be copied.
         * \return Count of elements popped from the buffer (if 0 -> buffer was empty).
         */
        size_t popElementFromBuffer(T& targetElement)
        {
            // Skip, if buffer is "empty"
            if (m_elementCount == 0)
                return 0;

            // Compute position of oldest element in buffer
            signed long long oldestElementIndex = ((signed long long)m_newElementIndex) - ((signed long long)m_elementCount);
            if (oldestElementIndex < 0) // Check, if we hit the front of the buffer...
                oldestElementIndex += m_maximumElementCount; // ...yes -> jump to end of the buffer

            // Copy element from buffer to target element
            targetElement = m_buffer[oldestElementIndex]; // Calls copy assignment operator of element

            // Decrement element count (marks the buffer position as "free")
            m_elementCount--;

            // Success
            return 1;
        }

        // Getters (thread-safe)
        // ---------------------
    public:
        //! Returns the maximum count of elements the buffer can hold. (**thread-safe**)
        inline size_t maximumElementCount() const { return getProtectedData(m_maximumElementCount); }

        //! Returns the current count of elements in the buffer. (**thread-safe**)
        inline size_t elementCount() const { return getProtectedData(m_elementCount); }

        //! Returns total number of overwritten elments (if 0 -> there was never an overflow (ideal case)). (**thread-safe**)
        inline unsigned long long totalOverwrittenElements() const { return getProtectedData(m_totalOverwrittenElements); }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };
} // namespace memory
} // namespace broccoli
