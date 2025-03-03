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
        struct BusVariableTypes<WriteableBusVariable<ContainerType_, ImplType_>> : BusVariableTypes<ReadableBusVariable<ContainerType_, ImplType_>> {
        };
    }

    /*!
     * \brief Interface to a writeable bus variable
     * \ingroup broccoli_hwl_variables
     *
     * Writeable bus variables are assignable from the underlying
     * variable data type but not from other bus variable types directly.
     *
     * Variables can be move- and copy-constructed. A copy of a bus variable uses a
     * separate data area and linking to object identifiers (via a registry) is lost.
     *
     * A variable can be hashed via std::hash. Its hash is identical to the hash of
     * a BusVariablePointerBase instance pointing to the same variable.
     *
     * \tparam ContainerType_ Container type for data storage
     * \tparam ImplType_ Implementation type for this variable
     */
    template <typename ContainerType_, template <typename> class ImplType_>
    class WriteableBusVariable : public BusVariableBase<WriteableBusVariable<ContainerType_, ImplType_>> {
    public:
        //! The container data type
        using ContainerType = ContainerType_;

        //! The implementation type
        using ImplType = typename internal::BusVariableTypes<WriteableBusVariable<ContainerType_, ImplType_>>::ImplType;

        //! The variable data type
        using DataType = typename internal::BusVariableTypes<WriteableBusVariable<ContainerType_, ImplType_>>::DataType;

        using BusVariableBase<WriteableBusVariable<ContainerType_, ImplType_>>::BusVariableBase;
        WriteableBusVariable() = default;
        WriteableBusVariable(const WriteableBusVariable&) = default;
        WriteableBusVariable(WriteableBusVariable&&) noexcept = default;

        WriteableBusVariable& operator=(const WriteableBusVariable& other) = delete;
        WriteableBusVariable& operator=(WriteableBusVariable&& other) = delete;

        WriteableBusVariable& operator=(const DataType& data) noexcept(std::is_nothrow_assignable<ImplType, const DataType&>::value)
        {
            this->impl() = data;
            return *this;
        }

        WriteableBusVariable& operator=(DataType&& data) noexcept
        {
            this->impl() = std::move(data);
            return *this;
        }
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <typename ContainerType_, template <typename> class ImplType_>
struct hash<broccoli::hwl::WriteableBusVariable<ContainerType_, ImplType_>> : hash<broccoli::hwl::BusVariableBase<broccoli::hwl::WriteableBusVariable<ContainerType_, ImplType_>>> {
};
} // namespace std
