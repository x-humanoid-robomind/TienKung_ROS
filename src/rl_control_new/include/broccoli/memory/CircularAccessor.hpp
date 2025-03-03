/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include <algorithm>

namespace broccoli {
namespace memory {

    /*!
     * \brief Accesses the elements of a linear data structure in a circular way
     *
     * Example: Given a container of Type ContainerType, which satisfies the STL SequenceContainer requirements,
     * the sequential data of this container is mapped to a different element index representation with size() = N:
     * \code
     * Original container: [0]   [1]  ...   [T]    ...  [size()-1]
     * Circular accessor:  [1]   [0]  ..   [N-1]    ..   [3]   [2]
     * \endcode
     * The CircularAccessor class represents the sequential data in circular form:
     * \code
     *                                T=[N-1]..   <--------   There may be unused elements here if size() of the container >
     *                                .       .               valid elements (count()) in circular representation.
     *                               [0]    [count()-1]
     *                                .       .
     *                                   [.]
     * \endcode
     * T describes the tail element. It is defined at the construction of the circular accessor and describes the end of the circular representation. (The
     * container may have count() < size() valid elements). With the element accessor the elements can be accessed in a circular way, where index 0 always
     * represents the front element (which is left to the tail). With pushFront() a new element is inserted right to the old front element by overwriting the
     * tail element. The new tail is then again right of the front element.
     *
     * \attention When resizing the container, make sure the current tail element is kept. Otherwise, the behavior is undefined.
     *
     * \ingroup broccoli_memory
     * \tparam ContainerType A container type satisfying the STL SequenceContainer requirements
     */
    class CircularAccessor {
    public:
        template <typename ContainerType, typename AccessorType>
        class SpecificCircularAccessor {
        public:
            /*!
             * \brief Returns the linear-spaced index of a circular accessor index
             * \param index The index from front to tail (0 = front element, size()-1 = tail element)
             * \returns The corresponding index in the original linear view
             */
            std::size_t circularToLinearIndex(const std::size_t& index) const
            {
                return ((m_container.size() + m_accessor.tailElementIndex() - 1 - index) % m_container.size());
            }

            /*!
             * \brief Access the container elements (0 = front element, size()-1 = tail element) without bounds checking
             *
             * The number of valid elements is returned by count(). Accessing indices beyond the size of the container
             * wraps these indices to the tail elements in the container.
             *
             * \param index Zero-based index starting from the front element (0).
             * \return Element of the container at specified index
             */
            const typename ContainerType::value_type& operator[](typename ContainerType::size_type index) const
            {
                return m_container[circularToLinearIndex(index)];
            }

            //! \copydoc SpecificCircularAccessor::operator[]
            typename ContainerType::value_type& operator[](typename ContainerType::size_type index)
            {
                return const_cast<typename ContainerType::value_type&>(static_cast<const SpecificCircularAccessor&>(*this).operator[](index));
            }

            /*!
             * \brief Access the container elements (0 = front element, size()-1 = tail element) with bounds checking
             *
             * The number of valid elements is returned by count(). Accessing indices beyond the size of the container
             * wraps these indices to the tail elements in the container.
             *
             * \param index Zero-based index starting from the front element (0).
             * \return Element of the container at specified index
             */
            const typename ContainerType::value_type& at(typename ContainerType::size_type index) const
            {
                return m_container.at(circularToLinearIndex(index));
            }

            //! \copydoc SpecificCircularAccessor::at()
            typename ContainerType::value_type& at(typename ContainerType::size_type index)
            {
                return const_cast<typename ContainerType::value_type&>(static_cast<const SpecificCircularAccessor&>(*this).at(index));
            }

            /*!
             * \brief Add to the front of the container representation
             *
             * This overwrites the element at the tail of the circular representation when count() == size().
             * \param newValue The new sample added at the front
             */
            void pushFront(const typename ContainerType::value_type& newValue)
            {
                m_container[m_accessor.m_tailElementIndex] = newValue;
                updateElementCount(count() + 1);

                m_accessor.m_tailElementIndex++;
                if (m_accessor.m_tailElementIndex > m_container.size() - 1) {
                    m_accessor.m_tailElementIndex = 0;
                }
            }

            /*!
             * \brief Returns the number of valid elements in the circular representation
             */
            typename ContainerType::size_type count() const
            {
                return std::min(m_accessor.m_nrValidElements, m_container.size());
            }

        protected:
            /*!
             * \brief Constructs a specific circular accessor
             * \param accessor The accessor parent
             * \param container The container to access
             */
            SpecificCircularAccessor(AccessorType& accessor, ContainerType& container)
                : m_accessor(accessor)
                , m_container(container)
            {
            }

            /*!
             * \brief Set valid element counter to new value
             * \param newCount The new count
             */
            void updateElementCount(const typename ContainerType::size_type& newCount)
            {
                m_accessor.m_nrValidElements = std::min(m_container.size(), newCount);
            }

            friend class CircularAccessor;

        private:
            AccessorType& m_accessor;
            ContainerType& m_container;
        };

        /*!
         * \brief Constructs a circular accessor for an STL compatible container
         * \param tailElementIndex The index of the tail element (index in container units)
         * \param nrOfValidElements The number of elements left of tail to be considered valid.
         */
        explicit CircularAccessor(const std::size_t& tailElementIndex, const std::size_t& nrOfValidElements = 0)
            : m_tailElementIndex(tailElementIndex)
            , m_nrValidElements(nrOfValidElements)
        {
        }

        /*!
         * \brief Resets the accessor to a new tail index and number of valid elements.
         *
         * This does not change the actual data.
         *
         * \param tailElementIndex The new tail index (index in container units)
         * \param nrOfValidElements The number of elements left of tail to be considered valid.
         */
        void reset(const std::size_t& tailElementIndex, const std::size_t& nrOfValidElements = 0)
        {
            m_tailElementIndex = tailElementIndex;
            m_nrValidElements = nrOfValidElements;
        }

        /*!
         * \brief Returns specific accessor linked to passed container object
         *
         * All specific accessors share the same tail element and number of valid elements.
         */
        template <typename ContainerType>
        SpecificCircularAccessor<ContainerType, CircularAccessor> access(ContainerType& container)
        {
            return SpecificCircularAccessor<ContainerType, CircularAccessor>(*this, container);
        }

        /*!
         * \brief Returns specific accessor linked to passed container object
         *
         * All specific accessors share the same tail element and number of valid elements.
         */
        template <typename ContainerType>
        SpecificCircularAccessor<const ContainerType, const CircularAccessor> access(const ContainerType& container) const
        {
            return SpecificCircularAccessor<const ContainerType, const CircularAccessor>(*this, container);
        }

        /*!
         * \brief Returns the index of the tail element as original container index
         */
        const std::size_t& tailElementIndex() const
        {
            return m_tailElementIndex;
        }

    private:
        //! Index of the last element in the container (the tail element)
        std::size_t m_tailElementIndex;

        //! Number of valid elements
        std::size_t m_nrValidElements = 0;
    };

} // namespace memory
} // namespace broccoli
