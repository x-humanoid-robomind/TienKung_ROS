/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "MultiVector.hpp"
#include <vector>
#ifdef HAVE_EIGEN3
#include <Eigen/StdVector> // Necessary for 128bit alginment and STL containers (for vectorization)
#endif // HAVE_EIGEN3

namespace broccoli {
namespace memory {
    //! Abstraction of a multi-level grid data structure
    /*!
     * \ingroup broccoli_memory
     * This data structure contains an array of grids (alias "levels"). Each grid is a condensed representation of the
     * corresponding linked lower level grid (like an image in different resolutions). The count of linked lower level
     * cells of each cell depends on the grid dimension (\f$2^N\f$ lower level cells for each cell).
     *
     * The class implements convencience functions like automatic data propagation from the lowest to the highest level.
     *
     * The following methods for cell operations may be re-implemented by the user:
     *  * \ref cellReset() - \copybrief cellReset()
     *  * \ref cellPropagateUp() - \copybrief cellPropagateUp()
     *  * \ref cellPropagateDown() - \copybrief cellPropagateDown()
     *
     * \warning In contrast to "sparse" tree structures, this class works with "dense" grid data. This means that
     * each cell has 2^N linked lower level cells (except for the lowest level for which the grid cells do not have
     * any "child" cells).
     *
     * \tparam N The dimension of the grids (N=1: grid="line", N=2: grid="square", N=3: grid="cube", ...)
     * \tparam L The count of levels (alias "layers") (has to be greater than 0)
     * \tparam T The Type of each "cell" in the grid
     * \tparam Allocator The allocator to be used to create new grid cells
     */
    template <unsigned int N, unsigned int L, class T,
#ifdef HAVE_EIGEN3
        class Allocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class Allocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    class MultiLevelGrid {
    public:
        static_assert(N > 0, "Count of dimensions has to be greater than zero!");
        static_assert(L > 0, "Count of levels has to be greater than zero!");

        //! The dimension of the grids (N=1: grid="line", N=2: grid="square", N=3: grid="cube", ...)
        static constexpr unsigned int gridDimensions() { return N; }

        //! The count of levels (alias "layers")
        static constexpr unsigned int levelCount() { return L; }

        //! Count of linked lower level cells for each cell (for all layers **above** the lowest level)
        static constexpr unsigned int linkedLowerLevelCellCount() { return (1 << N); }

        // Type definitions
        using GridType = MultiVector<T, N, Allocator>; //!< Data type of a single grid level

        //! Default constructor - constructs an **empty** structure
        MultiLevelGrid()
        {
        }

        //! Specialized constructor - constructs a multi-level grid with the given first level size
        /*! \note See \ref resize() for details */
        MultiLevelGrid(const std::array<size_t, N>& firstLevelSize)
            : MultiLevelGrid()
        {
            resize(firstLevelSize);
        }

        //! Specialized constructor - constructs a multi-level grid with the given first level size
        /*!
         * \param [in] firstSize Size of first dimension of the first level
         * \param [in] remainingSizes List of sizes of remaining dimensions of the first level
         */
        template <typename... Targs>
        MultiLevelGrid(const size_t& firstSize, const Targs&... remainingSizes)
            : MultiLevelGrid()
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of sizes has to match dimension!");
            resize(firstSize, static_cast<size_t>(remainingSizes)...);
        }

        //! Virtual destructor
        virtual ~MultiLevelGrid()
        {
        }

        // Members
        // -------
    protected:
        std::array<GridType, L> m_levels; //!< N-dimensional grids at different levels

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const MultiLevelGrid& reference) const
        {
            // Compare grids
            if (m_levels != reference.m_levels)
                return false;

            // Otherwise -> equality
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const MultiLevelGrid& reference) const { return !(*this == reference); }

        // Element access
        // --------------
    public:
        //! Returns a reference to the cell at the specified location
        /*!
         * \param [in] level Level in which the desired cell is contained
         * \param [in] indices List of indices to specify the location of the desired cell (in the grid related to the specified level)
         */
        const T& operator()(const size_t& level, const std::array<size_t, N>& indices) const { return m_levels[level](indices); }

        //! \copydoc operator()(const size_t&, const std::array<size_t, N>&) const
        T& operator()(const size_t& level, const std::array<size_t, N>& indices) { return const_cast<T&>(static_cast<const MultiLevelGrid&>(*this)(level, indices)); }

        //! Returns a reference to the cell at the specified location
        /*!
         * \param [in] level Level in which the desired cell is contained
         * \param [in] firstIndex Index in first dimension of the related grid
         * \param [in] remainingIndices List of indices in remaining dimensions of the related grid
         */
        template <typename... Targs>
        const T& operator()(const size_t& level, const size_t& firstIndex, const Targs&... remainingIndices) const
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of indices has to match dimension!");
            return operator()(level, std::array<size_t, N>{ { firstIndex, static_cast<size_t>(remainingIndices)... } });
        }

        //! Returns a reference to the cell at the specified location
        /*!
         * \param [in] level Level in which the desired cell is contained
         * \param [in] firstIndex Index in first dimension of the related grid
         * \param [in] remainingIndices List of indices in remaining dimensions of the related grid
         */
        template <typename... Targs>
        T& operator()(const size_t& level, const size_t& firstIndex, const Targs&... remainingIndices) { return const_cast<T&>(static_cast<const MultiLevelGrid&>(*this)(level, firstIndex, remainingIndices...)); }

        //! Returns a reference to the cell at the specified location
        /*!
         * \param [in] level Level in which the desired cell is contained
         * \param [in] cellIndex Index of the cell in the **stacked** element buffer of the underlying multi-dimensional vector
         */
        const T& cell(const size_t& level, const size_t& cellIndex) const { return m_levels[level][cellIndex]; }

        //! \copydoc cell(const size_t&, const size_t&) const
        T& cell(const size_t& level, const size_t& cellIndex) { return const_cast<T&>(static_cast<const MultiLevelGrid&>(*this).cell(level, cellIndex)); }

        // Capacity
        // --------
    public:
        //! List of sizes for each dimension of the specified grid level
        const std::array<size_t, N>& size(const size_t& level) const { return m_levels[level].size(); }

        //! Count of cells (all dimensions) of the specified grid level
        size_t cellCount(const size_t& level) const { return m_levels[level].elementCount(); }

        //! Total count of cells (all dimensions of all grid levels)
        size_t totalCellCount() const
        {
            size_t totalCellCount = cellCount(0);
            for (size_t i = 1; i < L; i++)
                totalCellCount += cellCount(i);
            return totalCellCount;
        }

        // Modifiers
        // ---------
    public:
        //! Assigns the given value to all cells of the specified level
        /*!
         * \param [in] level The level containing the cells to modify
         * \param [in] value The value to assign to the cells
         */
        void fill(const size_t& level, const T& value) { m_levels[level].fill(value); }

        //! Assigns the given value to all cells of all levels
        /*! \param [in] value The value to assign to the cells */
        void fill(const T& value)
        {
            for (size_t i = 0; i < L; i++)
                m_levels[i].fill(value);
        }

        //! Resizes the multi-level grid
        /*!
         * \note Only the grid size of the first level has to be specified. The lower level sizes are derived automatically.
         *
         * \param [in] firstLevelSize List of sizes for all dimensions of the **first level grid**
         * \return `true` on success, `false` otherwise (allocation failed)
         */
        bool resize(const std::array<size_t, N>& firstLevelSize)
        {
            // Compute sizes for all levels
            std::array<std::array<size_t, N>, L> levelSizes;
            levelSizes[0] = firstLevelSize;
            for (size_t level = 1; level < L; level++)
                for (size_t dimension = 0; dimension < N; dimension++)
                    levelSizes[level][dimension] = levelSizes[level - 1][dimension] * 2;

            // (Try to) resize grids of all levels (start with lowest level which is the biggest one)
            for (int level = L - 1; level >= 0; level--)
                if (m_levels[level].resize(levelSizes[level]) == false)
                    return false;

            // Success
            return true;
        }

        //! Resizes the multi-level grid
        /*!
         * \param [in] firstSize Size of first dimension of first level
         * \param [in] remainingSizes List of sizes of remaining dimensions of the first level
         * \return `true` on success, `false` otherwise (allocation failed)
         */
        template <typename... Targs>
        bool resize(const size_t& firstSize, const Targs&... remainingSizes)
        {
            static_assert((sizeof...(Targs) + 1 == N), "Count of sizes has to match dimension!");
            return resize(std::array<size_t, N>{ { firstSize, static_cast<size_t>(remainingSizes)... } });
        }

        // Grid operations
        // ---------------
    public:
        //! Resets all cells of the **given level**
        virtual void reset(const size_t& level)
        {
            // Pass through all cells of the corresponding grid
            for (size_t i = 0; i < m_levels[level].elementCount(); i++)
                cellReset(m_levels[level][i], level);
        }

        //! Resets **all** cells (on all levels)
        virtual void reset()
        {
            // Pass through all levels
            for (size_t i = 0; i < L; i++)
                reset(i);
        }

        //! Computes the indices of linked lower level cells for a given upper level cell
        /*!
         * \param [in] upperLevelIndices The indices of the upper level cell in the upper level grid
         * \param [out] lowerLevelIndices The indices of linked lower level cells in the lower level grid
         */
        virtual void computeLinkedLowerLevelIndices(const std::array<size_t, N>& upperLevelIndices, std::array<std::array<size_t, N>, linkedLowerLevelCellCount()>& lowerLevelIndices) const
        {
            for (uint64_t i = 0; i < linkedLowerLevelCellCount(); i++)
                for (uint64_t j = 0; j < N; j++)
                    lowerLevelIndices[i][j] = upperLevelIndices[j] * 2 + ((i & (1 << j)) > 0);
        }

        //! Computes the indices of the linked upper level cell for a given lower level cell
        /*!
         * \param [in] lowerLevelIndices The indices of the lower level cell in the lower level grid
         * \param [out] upperLevelIndices The indices of the linked upper level cell in the upper level grid
         */
        virtual void computeLinkedUpperLevelIndices(const std::array<size_t, N>& lowerLevelIndices, std::array<size_t, N>& upperLevelIndices) const
        {
            for (size_t i = 0; i < N; i++)
                upperLevelIndices[i] = lowerLevelIndices[i] / 2;
        }

        //! Propagates the data of the given level to the upper level(s)
        /*!
         * \param [in] level The level to propagate up
         * \param [in] recursive If `true`, the data is propagated up to the highest level
         */
        virtual void propagateUp(const size_t& level, const bool& recursive)
        {
            // Check, if this is the highest level
            if (level == 0)
                return; // Yes -> no upper level -> done

            // Initialize helpers
            GridType& upperLevel = m_levels[level - 1];
            const size_t upperLevelElementCount = upperLevel.elementCount();
            const GridType& lowerLevel = m_levels[level];
            std::array<std::array<size_t, N>, linkedLowerLevelCellCount()> lowerLevelIndices;
            std::vector<T const*> lowerLevelCells;
            lowerLevelCells.reserve(linkedLowerLevelCellCount());

            // Iterate over all elements of the upper level
            for (size_t i = 0; i < upperLevelElementCount; i++) {
                // Get upper level cell
                T& upperLevelCell = upperLevel[i];
                const std::array<size_t, N> upperLevelIndices = upperLevel.indices(i);

                // Get lower level cells
                computeLinkedLowerLevelIndices(upperLevelIndices, lowerLevelIndices);
                lowerLevelCells.clear();
                for (size_t j = 0; j < linkedLowerLevelCellCount(); j++)
                    lowerLevelCells.push_back(&lowerLevel(lowerLevelIndices[j]));

                // Trigger propagation
                cellPropagateUp(upperLevelCell, lowerLevelCells, level - 1, upperLevelIndices);
            }

            // Trigger recursion
            if (recursive == true && level > 1)
                propagateUp(level - 1, recursive);
        }

        //! Propagates the data from the bottom level to the top level
        virtual void propagateBottomUp() { propagateUp(L - 1, true); }

        //! Propagates the data of the given level to the lower level(s)
        /*!
         * \param [in] level The level to propagate down
         * \param [in] recursive If `true`, the data is propagated down to the lowest level
         */
        virtual void propagateDown(const size_t& level, const bool& recursive)
        {
            // Check, if this is the lowest level
            if (level + 1 >= L)
                return; // Yes -> no lower level -> done

            // Initialize helpers
            const GridType& upperLevel = m_levels[level];
            const size_t upperLevelElementCount = upperLevel.elementCount();
            GridType& lowerLevel = m_levels[level + 1];
            std::array<std::array<size_t, N>, linkedLowerLevelCellCount()> lowerLevelIndices;
            std::vector<T*> lowerLevelCells;
            lowerLevelCells.reserve(linkedLowerLevelCellCount());

            // Iterate over all elements of the upper level
            for (size_t i = 0; i < upperLevelElementCount; i++) {
                // Get upper level cell
                const T& upperLevelCell = upperLevel[i];
                const std::array<size_t, N> upperLevelIndices = upperLevel.indices(i);

                // Get lower level cells
                computeLinkedLowerLevelIndices(upperLevelIndices, lowerLevelIndices);
                lowerLevelCells.clear();
                for (size_t j = 0; j < linkedLowerLevelCellCount(); j++)
                    lowerLevelCells.push_back(&lowerLevel(lowerLevelIndices[j]));

                // Trigger propagation
                cellPropagateDown(upperLevelCell, lowerLevelCells, level, upperLevelIndices);
            }

            // Trigger recursion
            if (recursive == true && level + 1 < L)
                propagateDown(level + 1, recursive);
        }

        //! Propagates the data from the top level to the bottom level
        virtual void propagateTopDown() { propagateDown(0, true); }

        // Cell operations
        // ---------------
    protected:
        //! Resets the given cell to its initial state
        /*!
         * \param [in] cell Reference to cell to be reset
         * \param [in] level The level in the multi-level grid the cell belongs to
         */
        virtual void cellReset(T& cell, const size_t& level)
        {
            // Default: do nothing (may be re-implemented by user)
            (void)cell;
            (void)level;
        }

        //! Propagates the data of the lower level cells to the data of the linked upper level cell
        /*!
         * \param [out] upperLevelCell Reference to linked upper level cell
         * \param [in] lowerLevelCells Pointers to linked lower level cells
         * \param [in] upperLevel The level the upper level cell belongs to
         * \param [in] upperLevelIndices The indices of the upper level cell in the upper level grid
         */
        virtual void cellPropagateUp(T& upperLevelCell, const std::vector<T const*>& lowerLevelCells, const size_t& upperLevel, const std::array<size_t, N>& upperLevelIndices)
        {
            // Default: do nothing (may be re-implemented by user)
            (void)upperLevelCell;
            (void)lowerLevelCells;
            (void)upperLevel;
            (void)upperLevelIndices;
        }

        //! Propagates the data of the upper level cell to the data of the linked lower level cells
        /*!
         * \param [in] upperLevelCell Reference to linked upper level cell
         * \param [out] lowerLevelCells Pointers to linked lower level cells
         * \param [in] upperLevel The level the upper level cell belongs to
         * \param [in] upperLevelIndices The indices of the upper level cell in the upper level grid
         */
        virtual void cellPropagateDown(const T& upperLevelCell, const std::vector<T*>& lowerLevelCells, const size_t& upperLevel, const std::array<size_t, N>& upperLevelIndices)
        {
            // Default: do nothing (may be re-implemented by user)
            (void)upperLevelCell;
            (void)lowerLevelCells;
            (void)upperLevel;
            (void)upperLevelIndices;
        }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };

    // Type definitions
    // ----------------
    //! Definition of (dense) **binary trees** (for convenience)
    /*!
     * \ingroup broccoli_memory
     * \tparam L The count of levels (alias "layers") (has to be greater than 0)
     * \tparam T The Type of each "cell" in the grid
     * \tparam Allocator The allocator to be used to create new grid cells
     *
     * For details see \ref MultiLevelGrid.
     */
    template <unsigned int L, class T,
#ifdef HAVE_EIGEN3
        class Allocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class Allocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    using DenseBinaryTree = MultiLevelGrid<1, L, T, Allocator>;

    //! Definition of (dense) **quadtrees** (for convenience)
    /*!
     * \ingroup broccoli_memory
     * \tparam L The count of levels (alias "layers") (has to be greater than 0)
     * \tparam T The Type of each "cell" in the grid
     * \tparam Allocator The allocator to be used to create new grid cells
     *
     * For details see \ref MultiLevelGrid.
     */
    template <unsigned int L, class T,
#ifdef HAVE_EIGEN3
        class Allocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class Allocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    using DenseQuadtree = MultiLevelGrid<2, L, T, Allocator>;

    //! Definition of (dense) **octrees** (for convenience)
    /*!
     * \ingroup broccoli_memory
     * \tparam L The count of levels (alias "layers") (has to be greater than 0)
     * \tparam T The Type of each "cell" in the grid
     * \tparam Allocator The allocator to be used to create new grid cells
     *
     * For details see \ref MultiLevelGrid.
     */
    template <unsigned int L, class T,
#ifdef HAVE_EIGEN3
        class Allocator = Eigen::aligned_allocator<T> // <-- with Eigen library: use Eigen aligned allocator per default
#else
        class Allocator = std::allocator<T> // <-- no Eigen library: use standard allocator per default
#endif // HAVE_EIGEN3
        >
    using DenseOctree = MultiLevelGrid<3, L, T, Allocator>;
} // namespace memory
} // namespace broccoli
