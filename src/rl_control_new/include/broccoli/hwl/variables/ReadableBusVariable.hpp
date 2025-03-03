/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "BusVariableBase.hpp"

namespace broccoli {
namespace hwl {

    namespace internal {
        template <typename ContainerType_, template <typename> class ImplType_>
        struct BusVariableTypes<ReadableBusVariable<ContainerType_, ImplType_>> {
            using ImplType = ImplType_<ContainerType_>;
            using ContainerType = ContainerType_;
            using DataType = typename ContainerType_::DataType;
        };
    }

    /*!
     * \brief Interface to a readable bus variable
     * \ingroup broccoli_hwl_variables
     *
     * Readable bus variables can be move- and copy-constructed.
     * A copy of a bus variable uses a separate data area and linking to object
     * identifiers (via a registry) is lost.
     *
     * A variable can be hashed via std::hash. Its hash is identical to the hash of
     * a BusVariablePointerBase instance pointing to the same variable.
     *
     * \tparam ContainerType_ Container type for data storage
     * \tparam ImplType_ Implementation type for this variable
     */
    template <typename ContainerType_, template <typename> class ImplType_>
    class ReadableBusVariable : public BusVariableBase<ReadableBusVariable<ContainerType_, ImplType_>> {
    public:
        //! The container data type
        using ContainerType = ContainerType_;

        //! The implementation type
        using ImplType = typename internal::BusVariableTypes<ReadableBusVariable<ContainerType_, ImplType_>>::ImplType;

        //! The variable data type
        using DataType = typename internal::BusVariableTypes<ReadableBusVariable<ContainerType_, ImplType_>>::DataType;

        using BusVariableBase<ReadableBusVariable<ContainerType_, ImplType_>>::BusVariableBase;
        ReadableBusVariable() = default;
        ReadableBusVariable(const ReadableBusVariable&) = default;
        ReadableBusVariable(ReadableBusVariable&&) noexcept = default;

        ReadableBusVariable& operator=(const ReadableBusVariable&) = delete;
        ReadableBusVariable& operator=(ReadableBusVariable&&) = delete;
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <typename ContainerType_, template <typename> class ImplType_>
struct hash<broccoli::hwl::ReadableBusVariable<ContainerType_, ImplType_>> : hash<broccoli::hwl::BusVariableBase<broccoli::hwl::ReadableBusVariable<ContainerType_, ImplType_>>> {
};
} // namespace std
