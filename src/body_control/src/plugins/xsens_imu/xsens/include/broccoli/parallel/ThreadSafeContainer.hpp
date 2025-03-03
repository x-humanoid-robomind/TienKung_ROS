/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <pthread.h>

namespace broccoli {
namespace parallel {
    //! Base class for thread-safe data containers
    /*!
     * The class provides helper functions for thread-safe access to class members. The user
     * can choose to use the provided "internal" mutex, or a custom "external" mutex.
     */
    class ThreadSafeContainer {
    public:
        //! Default constructor - initializes internal mutex
        ThreadSafeContainer()
            : m_mutex(PTHREAD_RWLOCK_INITIALIZER)
        {
        }

        //! Copy constructor
        /*! \param [in] original Reference to original object. */
        ThreadSafeContainer(const ThreadSafeContainer& original)
            : m_mutex(PTHREAD_RWLOCK_INITIALIZER) // m_mutex = DO NOT COPY FROM ORIGINAL (use own mutex)
        {
            (void)original;
        }

        //! Copy assignment operator
        /*!
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        ThreadSafeContainer& operator=(const ThreadSafeContainer& reference)
        {
            (void)reference;
            // m_mutex = DO NOT COPY FROM ORIGINAL (use own mutex)
            return *this;
        }

        //! Move constructor
        /*! \attention Does **not** move mutex (uses own mutex) */
        ThreadSafeContainer(ThreadSafeContainer&& other)
            : m_mutex(PTHREAD_RWLOCK_INITIALIZER) // m_mutex = DO NOT MOVE FROM ORIGINAL (use own mutex)
        {
            (void)other; // Unused
        }

        //! Move assignment operator
        /*! \attention Does **not** move mutex (uses own mutex) */
        ThreadSafeContainer& operator=(ThreadSafeContainer&& other)
        {
            (void)other; // Unused
            // m_mutex = DO NOT MOVE FROM ORIGINAL (use own mutex)
            return *this;
        }

        // Members
        // -------
    protected:
        mutable pthread_rwlock_t m_mutex; //!< "Internal" read-write mutex for thread-safety

        // Generic locking
        // ---------------
    protected:
        //! Lock external mutex for read-access
        /*!
         * \param [in] mutex Reference to external mutex to be locked
         * \return `true`, if the lock could be aquired, `false` otherwise
         */
        static inline bool lockForRead(pthread_rwlock_t& mutex) { return (pthread_rwlock_rdlock(&mutex) == 0); }

        //! Lock internal mutex for read-access
        /*! \return `true`, if the lock could be aquired, `false` otherwise */
        inline bool lockForRead() const { return lockForRead(m_mutex); }

        //! Try to lock external mutex for read-access
        /*!
         * \param [in] mutex Reference to external mutex to be locked
         * \return `true`, if the lock could be aquired, `false` otherwise
         */
        static inline bool tryLockForRead(pthread_rwlock_t& mutex) { return (pthread_rwlock_tryrdlock(&mutex) == 0); }

        //! Try to lock internal mutex for read-access
        /*! \return `true`, if the lock could be aquired, `false` otherwise */
        inline bool tryLockForRead() const { return tryLockForRead(m_mutex); }

        //! Lock external mutex for write-access
        /*!
         * \param [in] mutex Reference to external mutex to be locked
         * \return `true`, if the lock could be aquired, `false` otherwise
         */
        static inline bool lockForWrite(pthread_rwlock_t& mutex) { return (pthread_rwlock_wrlock(&mutex) == 0); }

        //! Lock internal mutex for write-access
        /*! \return `true`, if the lock could be aquired, `false` otherwise */
        inline bool lockForWrite() const { return lockForWrite(m_mutex); }

        //! Try to lock external mutex for write-access
        /*!
         * \param [in] mutex Reference to external mutex to be locked
         * \return `true`, if the lock could be aquired, `false` otherwise
         */
        static inline bool tryLockForWrite(pthread_rwlock_t& mutex) { return (pthread_rwlock_trywrlock(&mutex) == 0); }

        //! Try to lock internal mutex for write-access
        /*! \return `true`, if the lock could be aquired, `false` otherwise */
        inline bool tryLockForWrite() const { return tryLockForWrite(m_mutex); }

        //! Unlock external mutex
        /*!
         * \param [in] mutex Reference to external mutex to be unlocked
         * \return `true`, if the lock could be released, `false` otherwise
         */
        static inline bool unlock(pthread_rwlock_t& mutex) { return (pthread_rwlock_unlock(&mutex) == 0); }

        //! Unlock internal mutex
        /*! \return `true`, if the lock could be released, `false` otherwise */
        inline bool unlock() const { return unlock(m_mutex); }

        // Thread-safe member access
        // -------------------------
    protected:
        //! Setter template for protected data (with external mutex)
        /*!
         * Sets the value of a protected data member. Thread-safety is guaranteed by an external mutex.
         * \param [in] data Reference to data member
         * \param [in] newValue New value for data member
         * \param [in] mutex Mutex used for locking
         */
        template <typename T>
        static inline void setProtectedData(T& data, const T& newValue, pthread_rwlock_t& mutex)
        {
            lockForWrite(mutex);
            data = newValue;
            unlock(mutex);
        }

        //! Setter template for protected data (with internal mutex)
        /*!
         * Sets the value of a protected data member. Thread-safety is guaranteed by the internal mutex.
         * \param [in] data Reference to data member
         * \param [in] newValue New value for data member
         */
        template <typename T>
        inline void setProtectedData(T& data, const T& newValue) { setProtectedData<T>(data, newValue, m_mutex); }

        //! Getter template for protected data (with external mutex)
        /*!
         * Gets the value of a protected data member. Thread-safety is guaranteed by an external mutex.
         * \param [in] data Reference to data member
         * \param [in] mutex Mutex used for locking
         * \return Value of data member (copy)
         */
        template <typename T>
        static inline T getProtectedData(const T& data, pthread_rwlock_t& mutex)
        {
            lockForRead(mutex);
            T returnValue = data;
            unlock(mutex);
            return returnValue;
        }

        //! Getter template for protected data (with internal mutex)
        /*!
         * Gets the value of a protected data member. Thread-safety is guaranteed by the internal mutex.
         * \param [in] data Reference to data member
         * \return Value of data member (copy)
         */
        template <typename T>
        inline T getProtectedData(const T& data) const { return getProtectedData<T>(data, m_mutex); }

        //! Addition template for protected data (with external mutex)
        /*!
         * Adds the given value to a protected data member. Thread-safety is guaranteed by an external mutex.
         * \param [in] data Reference to data member
         * \param [in] value The value to add to the data member
         * \param [in] mutex Mutex used for locking
         */
        template <typename T>
        static inline void protectedDataAddition(T& data, const T& value, pthread_rwlock_t& mutex)
        {
            lockForWrite(mutex);
            data += value;
            unlock(mutex);
        }

        //! Addition template for protected data (with internal mutex)
        /*!
         * Adds the given value to a protected data member. Thread-safety is guaranteed by the internal mutex.
         * \param [in] data Reference to data member
         * \param [in] value The value to add to the data member
         */
        template <typename T>
        inline void protectedDataAddition(T& data, const T& value) { protectedDataAddition<T>(data, value, m_mutex); }

        //! Multiplication template for protected data (with external mutex)
        /*!
         * Multiplies a protected data member with the given value. Thread-safety is guaranteed by an external mutex.
         * \param [in] data Reference to data member
         * \param [in] value The value to multiply to the data member with
         * \param [in] mutex Mutex used for locking
         */
        template <typename T>
        static inline void protectedDataMultiplication(T& data, const T& value, pthread_rwlock_t& mutex)
        {
            lockForWrite(mutex);
            data *= value;
            unlock(mutex);
        }

        //! Multiplication template for protected data (with internal mutex)
        /*!
         * Multiplies a protected data member with the given value. Thread-safety is guaranteed by the internal mutex.
         * \param [in] data Reference to data member
         * \param [in] value The value to multiply to the data member with
         */
        template <typename T>
        inline void protectedDataMultiplication(T& data, const T& value) { protectedDataMultiplication<T>(data, value, m_mutex); }
    };
} // namespace parallel
} // namespace broccoli
