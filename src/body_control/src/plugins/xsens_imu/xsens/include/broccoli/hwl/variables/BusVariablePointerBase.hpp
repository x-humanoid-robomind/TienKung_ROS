/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../../core/platform/PlatformHelperFactory.hpp"
#include "../../core/utils.hpp"
#include "../../memory/ThreadSafeData.hpp"
#include "../ForwardDeclarations.hpp"
#include <cstring>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Implementation of a thread-safe pointer to a bus variable container
     * \ingroup broccoli_hwl_variables
     *
     * This is used on the BusDriver side to access bus variables via a common interface.
     * The pointer implementation uses a ContainerType_<void*> specialization to store the
     * actual pointer to a bus variable container. This special type must meet the following requirements:
     * 1. Its constructor must take a ContainerType_<T> reference to the **unlocked** container. This means it must not
     * change the container's data during construction.
     * 2. It must provide const/non-const accessors void* ptr() that return a pointer to the container's variable data
     *
     * A bus variable pointer can be hashed via std::hash.
     *
     * \tparam ContainerType_ The bus variable container type
     */
    template <template <typename> class ContainerType_>
    class BusVariablePointerBase : public memory::ThreadSafeData<ContainerType_<void*>, BROCCOLI_HWL_MUTEX_TYPE, BROCCOLI_HWL_READ_LOCK_TYPE, BROCCOLI_HWL_WRITE_LOCK_TYPE> {
    public:
        using Base = memory::ThreadSafeData<ContainerType_<void*>, BROCCOLI_HWL_MUTEX_TYPE, BROCCOLI_HWL_READ_LOCK_TYPE, BROCCOLI_HWL_WRITE_LOCK_TYPE>;

        /*!
         * \brief Construct from variable
         * \tparam Derived Derived bus variable type
         * \param variable The bus variable to point to
         * \param littleEndianInterface True if the interface provided by copyTo() and copyFrom() assumes little endianness of the target/source memory region
         */
        template <typename Derived>
        explicit BusVariablePointerBase(BusVariableBase<Derived>& variable, bool littleEndianInterface = true)
            : Base(variable.impl().mutex(), ContainerType_<void*>{ variable.unsafeContainerRef() })
            , m_objectSize(variable.size())
            , m_reverseByteOrder(mustReverseByteOrder(littleEndianInterface))
        {
        }

        BusVariablePointerBase(const BusVariablePointerBase& other)
            : Base(other.mutex(), other.unsafeRef())
            , m_objectSize(other.m_objectSize)
            , m_reverseByteOrder(other.m_reverseByteOrder)
        {
        }

        BusVariablePointerBase(BusVariablePointerBase&& other) noexcept
            : Base(other.mutex(), std::move(other.unsafeRef()))
            , m_objectSize(std::move(other.m_objectSize))
            , m_reverseByteOrder(std::move(other.m_reverseByteOrder))
        {
        }

        BusVariablePointerBase& operator=(BusVariablePointerBase&&) noexcept = delete;
        BusVariablePointerBase& operator=(const BusVariablePointerBase&) = delete;

        //! Returns the size of the variable pointed to (data only)
        const std::size_t& size() const noexcept { return m_objectSize; }

        /*!
         * \brief Copy variable content to memory region
         * \warning This method locks the bus variable. Be sure it is not locked already!
         *
         * \param target Pointer to the target memory region
         * \param targetSize Total size of the target memory region in bytes
         * \param targetOffset Target position offset in bytes (within the total memory region)
         * \return True on success
         */
        bool copyTo(void* const target, std::size_t targetSize, std::size_t targetOffset = 0) const noexcept
        {
            auto guard = this->lockConstWithGuard();
            return copyTo(guard, target, targetSize, targetOffset);
        }

        /*!
         * \brief Copy variable content to memory region when locked already
         *
         * \param lockGuard Locked guard for this bus variable pointer
         * \param target Pointer to the target memory region
         * \param targetSize Total size of the target memory region in bytes
         * \param targetOffset Target position offset in bytes (within the total memory region)
         * \return True on success
         */
        template <typename GuardType>
        bool copyTo(const GuardType& lockGuard, void* const target, std::size_t targetSize, std::size_t targetOffset = 0) const noexcept
        {
            assert(targetSize - targetOffset >= size() && "Size mismatch to destination!");
            copyFunctor()(static_cast<char* const>(target) + targetOffset, lockGuard->ptr(), std::min(size(), targetSize - targetOffset));
            return targetSize - targetOffset >= size();
        }

        /*!
         * \brief Copy memory region to variable location
         * \warning This method locks the bus variable. Be sure it is not locked already!
         *
         * \param source Pointer to the source memory region
         * \param sourceSize Total size of the source memory region in bytes
         * \param sourceOffset Source position offset in bytes (within the total memory region)
         * \return True on success
         */
        bool copyFrom(const void* const source, std::size_t sourceSize, std::size_t sourceOffset = 0) noexcept
        {
            auto guard = this->lockWithGuard();
            return copyFrom(guard, source, sourceSize, sourceOffset);
        }

        /*!
         * \brief Copy memory region to variable location when locked already
         *
         * \param lockGuard Locked guard for this bus variable pointer
         * \param source Pointer to the source memory region
         * \param sourceSize Total size of the source memory region in bytes
         * \param sourceOffset Source position offset in bytes (within the total memory region)
         * \return True on success
         */
        template <typename GuardType>
        bool copyFrom(GuardType& lockGuard, const void* const source, std::size_t sourceSize, std::size_t sourceOffset = 0) noexcept
        {
            assert(sourceSize - sourceOffset >= size() && "Size mismatch with origin!");
            copyFunctor()(lockGuard->ptr(), static_cast<const char* const>(source) + sourceOffset, std::min(size(), sourceSize - sourceOffset));
            return sourceSize - sourceOffset >= size();
        }

        bool operator==(const BusVariablePointerBase& rhs) const
        {
            return this->unsafeRef().ptr() == rhs.unsafeRef().ptr();
        }
        bool operator!=(const BusVariablePointerBase& rhs) const
        {
            return !(rhs == *this);
        }

    protected:
        /*!
         * \brief Determines if byte order must be reversed to be compatible with target endianness
         * \param targetIsLittleEndian Target/Source endianness is little endian?
         * \return True if reversing is necessary
         */
        static bool mustReverseByteOrder(bool targetIsLittleEndian) noexcept
        {
            static bool littleEndianPlatform = core::PlatformHelperFactory::create()->usesLittleEndian();
            return littleEndianPlatform ? !targetIsLittleEndian : targetIsLittleEndian;
        }

        //! Returns the callable used for copying data (compatible with the memcpy interface)
        auto copyFunctor() const noexcept
        {
            return m_reverseByteOrder ? core::reverse_memcpy : std::memcpy;
        }

        //! The object size (not necessarily minimal size) in bytes
        std::size_t m_objectSize;

        //! Reverse byte order when using copyTo() / copyFrom()?
        const bool m_reverseByteOrder;
    };

} // namespace hwl
} // namespace broccoli

namespace std {
template <template <typename> class ContainerType_>
struct hash<broccoli::hwl::BusVariablePointerBase<ContainerType_>> : std::hash<const void*> {
    std::size_t operator()(const broccoli::hwl::BusVariablePointerBase<ContainerType_>& pointer) const noexcept
    {
        // use pointer to shared data area
        return std::hash<const void*>()(pointer.unsafeRef().ptr());
    }
};
} // namespace std
