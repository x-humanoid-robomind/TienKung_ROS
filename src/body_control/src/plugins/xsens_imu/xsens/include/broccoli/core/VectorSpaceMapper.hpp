/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once

#include <Eigen/Dense>
#include <array>

namespace broccoli {
namespace core {
    /*!
     * \brief Converts vector representations between two spaces, defined by a binary selection matrix
     * \ingroup broccoli_core
     *
     * We define a vector-space \f$\mathcal{F}\f$ and a sub-space \f$\mathcal{S}\f$ with \f$ \mathcal{S} \subseteq \mathcal{F}\f$.
     * The sub-space vector \f$v \in \mathcal{S}\f$ can be described by \f$v = B \cdot u\f$, where
     * \f$B\f$ is a binary selection matrix.
     *
     * \tparam FullSpaceSize The dimension of \f$\mathcal{F}\f$
     * \tparam SubSpaceSize The dimension of \f$\mathcal{S}\f$
     */
    template <std::size_t FullSpaceSize, std::size_t SubSpaceSize>
    class VectorSpaceMapper {
    public:
        //! The full space size
        static constexpr std::size_t FullSpaceSizeConstant = FullSpaceSize;

        //! The size of the subspace
        static constexpr std::size_t SubSpaceSizeConstant = SubSpaceSize;

        //! Default constructor
        VectorSpaceMapper()
        {
            m_selectionMatrix.setZero();
            m_expansionMatrix.setZero();
        }

        //! A Constructor
        /*!
         * \param subSpaceIndices List of indices (starting from zero) to indicate which dimensions of \f$\mathcal{F}\f$ span the sub-space \f$\mathcal{S}\f$.
         * The order of the sub-space components is based on the order of the indices.
         */
        VectorSpaceMapper(const std::array<std::size_t, SubSpaceSize>& subSpaceIndices)
        {
            setSubSpace(subSpaceIndices);
        }

        //! (Re-)set the selection matrix based on the given index list.
        /*!
         * \param subSpaceIndices List of indices (starting from zero) to indicate which dimensions of \f$\mathcal{F}\f$ span the sub-space \f$\mathcal{S}\f$.
         * The order of the sub-space components is based on the order of the indices.
         */
        void setSubSpace(const std::array<std::size_t, SubSpaceSize>& subSpaceIndices)
        {
            m_selectionMatrix.setZero();

            std::size_t i = 0;
            for (auto index : subSpaceIndices) {
                assert(index >= 0 && index < FullSpaceSize && "Provided index is out of range!");

                m_selectionMatrix(i, index) = 1.0;
                i++;
            }

            m_expansionMatrix = m_selectionMatrix.transpose();
        }

        //! Returns the sub-space representation of the given argument
        /*!
         * \tparam Derived Derived Eigen type
         * \param fullSpace The full-space representation vector \f$u\f$
         * \return \f$B * u\f$
         */
        template <typename Derived>
        Eigen::Matrix<double, SubSpaceSize, 1> subSpaceOf(const Eigen::MatrixBase<Derived>& fullSpace) const
        {
            return selectionMatrix() * fullSpace;
        }

        //! Returns the full-space representation of the given argument
        /*!
         * \tparam Derived Derived Eigen type
         * \param subSpace The sub-space representation vector \f$v\f$
         * \return \f$B^T * v\f$
         */
        template <typename Derived>
        Eigen::Matrix<double, FullSpaceSize, 1> fullSpaceOf(const Eigen::MatrixBase<Derived>& subSpace) const
        {
            return expansionMatrix() * subSpace;
        }

        //! Returns the binary selection matrix \f$B\f$
        const Eigen::Matrix<double, SubSpaceSize, FullSpaceSize>& selectionMatrix() const
        {
            return m_selectionMatrix;
        }

        //! Returns the binary expansion matrix \f$B^T\f$
        Eigen::Matrix<double, FullSpaceSize, SubSpaceSize> expansionMatrix() const
        {
            return m_expansionMatrix;
        }

    protected:
        //! The binary selection matrix \f$B\f$ for the subspace \f$\mathcal{S}\f$
        Eigen::Matrix<double, SubSpaceSize, FullSpaceSize> m_selectionMatrix;

        //! The binary expansion matrix \f$B^T\f$ for the full space \f$\mathcal{F}\f$
        Eigen::Matrix<double, FullSpaceSize, SubSpaceSize> m_expansionMatrix;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace core
} // namespace broccoli
