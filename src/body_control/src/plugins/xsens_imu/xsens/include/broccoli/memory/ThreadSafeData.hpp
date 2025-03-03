/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include <cassert>
#include <mutex>
#include <shared_mutex>
#include <type_traits>

namespace broccoli {
namespace memory {

    /*!
     * \brief Stores data guarded by mutual exclusion
     * \ingroup broccoli_memory
     *
     * A ThreadSafeData wrapper stores data, which is protected with locks. A reference to the mutex used to lock the data
     * area must be passed during construction. The wrapper automatically uses read-only locking on const objects.
     * Apart from standard access methods via explicit type cast and assignment, a scope-based guarded access
     * to the data area is provided, refer to lockWithGuard(), lockConstWithGuard().
     *
     * \warning This class provides no exception guarantee for the sake of performance. When an exception occurs, the program may be in an invalid state.
     *
     * \tparam DataType_ The type of the data to guard
     * \tparam MutexType_ The mutex type to use. Must meet the STL Mutex requirements.
     * \tparam DefaultReadLock_ Default lock type for reading. Must be constructable from MutexType_ and lock it on construction.
     * \tparam DefaultWriteLock_ Default lock type for writing. Must be constructable from MutexType_ and lock it on construction.
     */
    template <typename DataType_, typename MutexType_ = std::shared_timed_mutex, template <typename> class DefaultReadLock_ = std::shared_lock, template <typename> class DefaultWriteLock_ = std::unique_lock>
    class ThreadSafeData {
    public:
        //! Type of data to guard
        using DataType = DataType_;

        //! Mutex type used for exclusion
        using MutexType = MutexType_;

        //! Default read lock type
        template <typename T>
        using DefaultReadLock = DefaultReadLock_<T>;

        //! Default write lock type
        template <typename T>
        using DefaultWriteLock = DefaultWriteLock_<T>;

        /*!
         * \brief Construct
         * \param mutex Reference to mutex used for exclusion
         */
        explicit ThreadSafeData(MutexType& mutex) noexcept(std::is_nothrow_default_constructible<DataType>::value)
            : m_mutex(mutex)
            , m_data{}
        {
        }

        /*!
         * \brief Construct from data value
         * \param mutex Reference to mutex used for exclusion
         * \param data Initial data value
         */
        ThreadSafeData(MutexType& mutex, const DataType& data) noexcept(std::is_nothrow_copy_constructible<DataType>::value)
            : m_mutex(mutex)
            , m_data(data)
        {
        }

        /*!
         * \brief Move-Construct from data value
         * \param mutex Reference to mutex used for exclusion
         * \param data Initial data value
         */
        ThreadSafeData(MutexType& mutex, DataType&& data) noexcept(std::is_nothrow_move_constructible<DataType>::value)
            : m_mutex(mutex)
            , m_data(std::move(data))
        {
        }

        ThreadSafeData(ThreadSafeData&& other, const DefaultReadLock<MutexType>&) noexcept(std::is_nothrow_move_constructible<DataType>::value)
            : m_mutex(other.m_mutex)
            , m_data(std::move(other.m_data))
        {
        }

        ThreadSafeData(ThreadSafeData&& other) noexcept
            : ThreadSafeData(std::move(other), DefaultReadLock<MutexType>(other.m_mutex))
        {
        }

        ThreadSafeData(const ThreadSafeData&) = delete;
        ThreadSafeData& operator=(const ThreadSafeData& other)
        {
            DefaultWriteLock<MutexType> writeLock(m_mutex, std::defer_lock_t());
            DefaultReadLock<MutexType> readLock(other.m_mutex, std::defer_lock_t());

            if (&m_mutex != &other.m_mutex) {
                std::lock(writeLock, readLock); // avoids deadlock
            } else {
                writeLock.lock();
            }

            m_data = other.m_data;
            return *this;
        }

        ThreadSafeData& operator=(ThreadSafeData&& other) noexcept
        {
            DefaultWriteLock<MutexType> writeLock(m_mutex, std::defer_lock_t());
            DefaultReadLock<MutexType> readLock(other.m_mutex, std::defer_lock_t());

            if (&m_mutex != &other.m_mutex) {
                std::lock(writeLock, readLock); // avoids deadlock
            } else {
                writeLock.lock();
            }

            m_data = std::move(other.m_data);
            return *this;
        }

        ThreadSafeData& operator=(const DataType& data)
        {
            auto guard = this->lockWithGuard();
            *guard = data;
            return *this;
        }

        ThreadSafeData& operator=(DataType&& data) noexcept
        {
            auto guard = this->lockWithGuard();
            *guard = std::move(data);
            return *this;
        }

        explicit operator DataType() const
        {
            return copy();
        }

        //! Returns a copy of the data
        DataType copy() const
        {
            auto guard = this->lockWithGuard();
            DataType copy(*guard);
            return copy;
        }

        /*!
         * \brief A data guard providing an interface to a locked ThreadSafeData instance
         * \tparam DataType The data type to access
         * \tparam LockType The lock type used
         */
        template <typename DataType, typename LockType>
        class DataGuard {
        public:
            DataGuard(const DataGuard&) = delete;
            DataGuard& operator=(const DataGuard&) = delete;

            DataGuard(DataGuard&& other) noexcept
                : m_reference(other.m_reference)
                , m_lock(std::move(other.m_lock))
            {
            }

            DataGuard& operator=(DataGuard&& other) noexcept
            {
                m_reference = other.m_reference;
                m_lock = std::move(other.m_lock);
                return *this;
            }

            /*!
             * \brief The member of pointer operator allows direct access to DataType's members
             *
             * \warning The operator does not check the lock for validity. When using try lock, check the lock status via hasLock() before access.
             * \return Pointer to the data area
             */
            DataType* operator->() & noexcept
            {
                assert(hasLock() && "Accessed guarded storage without active lock (did you check try lock with hasLock()?)");
                return &m_reference;
            }

            /*!
             * \brief The member of pointer operator allows direct access to DataType's members
             *
             * \warning The operator does not check the lock for validity.
             * When using try lock, check the lock status via hasLock() before access.
             * \return Const pointer to the data area
             */
            const DataType* operator->() const& noexcept
            {
                assert(hasLock() && "Accessed guarded storage without active lock (did you check try lock with hasLock()?)");
                return &m_reference;
            }

            /*!
             * \brief The pointer indirection operator returns a reference to the guarded DataType instance
             *
             * \warning In contrast to get(), this method does not check the lock for validity.
             * When using try lock, check the lock status via hasLock() before access.
             * \return Reference to the data area
             */
            DataType& operator*() & noexcept
            {
                assert(hasLock() && "Accessed guarded storage without active lock (did you check try lock with hasLock()?)");
                return m_reference;
            }

            /*!
             * \brief The pointer indirection operator returns a reference to the guarded DataType instance
             *
             * \warning In contrast to get(), this method does not check the lock for validity.
             * When using try lock, check the lock status via hasLock() before access.
             * \return Const reference to the data area
             */
            const DataType& operator*() const& noexcept
            {
                assert(hasLock() && "Accessed guarded storage without active lock (did you check try lock with hasLock()?)");
                return m_reference;
            }

            /*!
             * \brief Access via reference to locked data area
             *
             * In contrast to the pointer indirection operator*(), this method checks the lock via hasLock().
             * If the lock is not valid, an std::logic_error is thrown.
             * \warning: Accessing the returned reference after destruction of the guard leads to undefined behavior.
             * \return Reference to the locked data area
             */
            DataType& get() &
            {
                checkLock();
                return m_reference;
            }

            /*!
             * \brief Access via reference to locked data area
             *
             * In contrast to the pointer indirection operator*(), this method checks the lock via hasLock().
             * If the lock is not valid, an std::logic_error is thrown.
             * \warning: Accessing the returned reference after destruction of the guard leads to undefined behavior.
             * \return Const reference to the locked data area
             */
            const DataType& get() const&
            {
                checkLock();
                return m_reference;
            }

            /*!
             * \brief Returns lock status
             *
             * This can be used to check if a lock is valid when
             * the guard has been created via try-lock operations.
             * Accessing
             * \return True if the lock is valid
             */
            bool hasLock() const noexcept
            {
                return m_lock.owns_lock();
            }

            //! \copydoc hasLock()
            explicit operator bool() const noexcept
            {
                return hasLock();
            }

        protected:
            /*!
             * \brief Construct from reference to data and lock
             * \param dataReference Reference to data
             * \param lock r-value ref to lock
             */
            DataGuard(DataType& dataReference, LockType&& lock) noexcept
                : m_reference(dataReference)
                , m_lock(std::move(lock))
            {
            }

            //! Only ThreadSafeData may create a DataGuard
            friend class ThreadSafeData;

            //! Throws std::logic_error if lock is invalid
            void checkLock() const
            {
                if (!m_lock) {
                    throw std::logic_error("Trying to access guarded storage before lock was acquired.");
                }
            }

        private:
            DataType& m_reference;
            LockType m_lock;
        };

        /*!
         * \brief Lock the ThreadSafeData and return a DataGuard for access
         *
         * The lock is owned for the lifetime of the returned guard object.
         * By default, const objects are locked via DefaultReadLock, non-const
         * objects are locked via DefaultWriteLock. It is possible to pass
         * arguments to the constructor of the underlying LockType.
         * Examples:
         * \code
         * // creates write lock, owned until myguard is destructed
         * auto myguard = threadSafeData.lockWithGuard();
         *
         * // Tries to acquire write lock (fails)
         * auto guard = threadSafeData.lockWithGuard(std::try_to_lock_t{});
         * // guard->hasLock() is false
         * \endcode
         *
         * \tparam LockType The STL compatible lock type
         * \tparam Args Types of lock constructor arguments
         * \param args Lock constructor arguments
         * \return A DataGuard
         */
        template <template <typename> class LockType = DefaultWriteLock, typename... Args>
        DataGuard<DataType, LockType<MutexType>> lockWithGuard(Args&&... args) noexcept
        {
            return DataGuard<DataType, LockType<MutexType>>(m_data, LockType<MutexType>(m_mutex, args...));
        }

        //! \copydoc lockWithGuard()
        template <typename... TArgs>
        DataGuard<const DataType, DefaultReadLock<MutexType>> lockWithGuard(TArgs&&... args) const noexcept
        {
            return lockConstWithGuard(args...);
        }

        /*!
         * \brief Lock the ThreadSafeData read-only and return a DataGuard for access
         *
         * This is the explicit read-only variant of lockWithGuard().
         *
         * \tparam LockType The STL compatible lock type
         * \tparam Args Types of lock constructor arguments
         * \param args Lock constructor arguments
         * \return A DataGuard
         */
        template <typename... TArgs>
        DataGuard<const DataType, DefaultReadLock<MutexType>> lockConstWithGuard(TArgs&&... args) const noexcept
        {
            return DataGuard<const DataType, DefaultReadLock<MutexType>>(m_data, DefaultReadLock<MutexType>(m_mutex, args...));
        }

        //! Returns an unsafe (unlocked) reference to the protected data
        DataType& unsafeRef() noexcept { return m_data; }

        //! Returns an unsafe (unlocked) const reference to the protected data
        const DataType& unsafeRef() const noexcept { return m_data; }

        //! Returns the mutex used to protect this data
        MutexType& mutex() const noexcept { return m_mutex; }

    private:
        MutexType& m_mutex;
        DataType m_data;
    };

} // namespace memory
} // namespace broccoli
