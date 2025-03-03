/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "LogFileBase.hpp"
#include "LogFileData.hpp"
#include <memory>

namespace broccoli {
namespace io {
    //! Template class for logfiles (arbitrary types of logdata)
    /*!
     * \ingroup broccoli_io_logging
     * The class derives from \ref LogFileBase and adds the missing functionality to process \ref LogFileData objects. The class is
     * implemented as template to allow arbitrary variants of \ref LogFileData (template parameter \p LogFileDataType). The parameter
     * \p LogFileDataType has to be derived from the abstract base class \ref LogFileData.
     *
     * LogFile supports file attributes. These key/value string pairs are encoded to the logfile via LogFileData::encodeHeaderToLogStream()
     * and LogFileData::encodeFooterToLogStream() of the LogFileDataType. Custom attributes can be added using setAttribute() or addAttributes().
     * There are several default attributes, which are always provided:
     * - "identifier": The log file identifier passed to its constructor
     * - "logStartTime": The log start time as string
     * - "logStopTime": The log stop time as string (only available in footer)
     */
    template <class LogFileDataType>
    class LogFile : public LogFileBase {
    public:
        // Specialized constructor (see base class for details)
        //! \copydoc LogFileBase::LogFileBase()
        /*! \param [in] bufferSize Size of circular buffer for storing \ref LogFileData objects. A buffer overflow is safe, but implies discarding not written \ref LogFileData objects. */
        LogFile(const std::string& identifier, const std::string& targetDirectory, const std::string& fileName, const std::string& fileExtension, const bool& addTimeStampToName, const bool& compressed, const bool& enableSkipping, const size_t& bufferSize, const size_t& maximumStreamSize = 4096 /* <-- 4kB per default */)
            : LogFileBase(identifier, targetDirectory, fileName, fileExtension, addTimeStampToName, compressed, enableSkipping, maximumStreamSize)
            , m_dataBuffer(bufferSize)
        {
            setAttribute("Identifier", identifier);
            m_headerData = std::make_unique<LogFileDataType>();
            m_footerData = std::make_unique<LogFileDataType>();
            m_currentData = std::make_unique<LogFileDataType>();
        }

        //! Default constructor (minimal set of parameters)
        /*!
         * \param [in] identifier Initializes: \copybrief identifier()
         * \param [in] bufferSize Size of circular buffer for storing \ref LogFileData objects. A buffer overflow is safe, but implies discarding not written \ref LogFileData objects.
         * \param [in] maximumStreamSize Estimated maximum count of bytes to write at once. Used to pre-allocate memory for better runtime performance.
         */
        LogFile(const std::string& identifier, const size_t& bufferSize, const size_t& maximumStreamSize = 4096 /* <-- 4kB per default */)
            : LogFile(identifier, "", "unknown", ".log", false, false, true, bufferSize, maximumStreamSize)
        {
        }

        //! Deleted copy constructor (prevent user to copy object - would be dangerous due to internal mutexes)
        LogFile(const LogFile& original) = delete;

        //! Deleted copy assignment operator (prevent user to copy object - would be dangerous due to internal mutexes)
        LogFile& operator=(const LogFile& reference) = delete;

        // Destructor (see base class for details)
        //! \copydoc ~LogFileBase()
        virtual ~LogFile()
        {
            // Check, if file is open
            if (isOpen() == true) {
                assert(false && "this should never happen");
            }

            // Clear buffers
            m_dataBuffer.clear();
        }

    protected:
        //! Move constructor (internal)
        /*!
         * \warning **Not thread-safe** -> should only be called by the thread-safe wrapper!
         *
         * \param [in] other Reference to other object.
         */
        LogFile(LogFile&& other, const int& /* <- trick used for locking mutex */)
            : LogFileBase(std::move(other))
            , m_dataBuffer(std::move(other.m_dataBuffer))
            , m_headerData(std::move(other.m_headerData))
            , m_footerData(std::move(other.m_footerData))
            , m_currentData(std::move(other.m_currentData))
        {
        }

    public:
        //! Move constructor (wrapper) (thread-safe)
        /*!
         * Locks the mutex of the other object for writing before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] other Reference to other object.
         */
        LogFile(LogFile&& other)
            : LogFile(other, other.lockForWrite() /* <- lock mutex of original object first (for writing since we move the object) */)
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
        LogFile& operator=(LogFile&& other)
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
            LogFileBase::operator=(std::move(other));
            m_dataBuffer = std::move(other.m_dataBuffer);
            m_headerData = std::move(other.m_headerData);
            m_footerData = std::move(other.m_footerData);
            m_currentData = std::move(other.m_currentData);

            // Unlock other object and ourselves
            other.unlock();
            unlock();

            return *this;
        }

        //! **Thread-safe** adding of the given logdata object to the logdata buffer
        /*!
         * Efficient copy to buffer (no new allocations!). May skip, depending on state of \ref active() and \ref countOfObjectsToSkip()
         *
         * \attention The original data is modified (updated timing information).
         *
         * \param [in] newData Logfile data object to add to the buffer.
         * \return `true`, in case the object was added to the buffer, `false` otherwise
         */
        bool addNewDataToBuffer(const LogFileDataType& newData)
        {
            // Initialize helpers
            const uint64_t currentCountOfObjectsToSkip = countOfObjectsToSkip();

            // Update count of received objects
            const uint64_t currentTotalReceivedObjectCount = incrementTotalReceivedObjectCount();

            // Check, if logfile is deactivated
            if (active() == false)
                return false; // ...inactive -> do not add to buffer

            // Check, if we should skip this object
            if (enableSkipping() == false || currentCountOfObjectsToSkip == 0 || currentTotalReceivedObjectCount % (currentCountOfObjectsToSkip + 1) == 1) {
                // ...no skipping -> add object to buffer

                // Set time in new data object to current time (remember this event)
                newData.m_timeAddedToBuffer = core::Time::currentTime();

                // Add new data element to the buffer
                m_dataBuffer.push(newData);

                // Object was added to the buffer
                return true;
            } else
                return false; // Object was skipped
        }

        // Forced clearing of log buffer (thread-safe) (see base class for details)
        virtual void clearLogBuffer()
        {
            // Clear all elements
            m_dataBuffer.clear();

            // Reset counter for received objects
            setTotalReceivedObjectCount(0);
        }

        // Thread-safe writing of the file header (see base class for details)
        virtual bool writeLogHeader(const core::Time& logStartTime)
        {
            // Get header object
            const LogFileDataType currentHeaderData = headerData();

            // Lock file-mutex for write
            lockForWrite(m_mutexFile);

            // Check, if file is open
            // ----------------------
            if (m_isOpen == false) {
                m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_WRITE_FAILED_NOTOPEN));
                unlock(m_mutexFile); // Unlock file-mutex
                return false;
            }

            // Write header
            // ------------
            setAttribute("LogStartTime", logStartTime.encodeToDateTimeString(true, "%Y-%m-%d %H:%M:%S", 9));
            currentHeaderData.encodeHeaderToLogStream(m_logStream, fileAttributes());

            // Write stream to file
            if (writeCurrentLogStreamToFile() == false) {
                // Writing header failed -> try to close file
                unlock(m_mutexFile); // Unlock file-mutex
                closeLogFile();
                return false;
            }

            // Unlock file-mutex
            unlock(m_mutexFile);

            // Success!
            return true;
        }

        // Write current contents of buffer to file (thread-safe) (see base class for details)
        virtual bool processLogBuffer()
        {
            // Lock file-mutex for write
            lockForWrite(m_mutexFile);

            // Check, if file is open
            // (If file is not open we can not write to it -> skip but keep data in buffer (in case we want to dump the data later))
            if (m_isOpen == true) {
                // Encode elements in buffer and add them to the stream
                while (m_dataBuffer.pop(*m_currentData) > 0) // Try to get next element (stops if there are no elements left)
                {
                    // Update counter of written elements
                    incrementTotalWrittenObjectCount();

                    // Encode elements
                    m_currentData->encodeDataToLogStream(m_logStream);
                }

                // Write stream to file
                if (writeCurrentLogStreamToFile() == false) {
                    // Writing header failed -> try to close file
                    unlock(m_mutexFile); // Unlock file-mutex
                    closeLogFile();
                    return false;
                }
            }

            // Unlock file-mutex
            unlock(m_mutexFile);

            // Success
            return true;
        }

        // Thread-safe writing of the file footer (see base class for details)
        virtual bool writeLogFooter(const core::Time& logStopTime)
        {
            // Get footer object
            const LogFileDataType currentFooterData = footerData();

            // Lock file-mutex for write
            lockForWrite(m_mutexFile);

            // Check, if file is open
            // ----------------------
            if (m_isOpen == false) {
                m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_WRITE_FAILED_NOTOPEN));
                unlock(m_mutexFile); // Unlock file-mutex
                return false;
            }

            // Write footer
            // ------------
            setAttribute("LogStopTime", logStopTime.encodeToDateTimeString(true, "%Y-%m-%d %H:%M:%S", 9));
            currentFooterData.encodeFooterToLogStream(m_logStream, fileAttributes());
            removeAttribute("LogStopTime");

            // Write stream to file
            if (writeCurrentLogStreamToFile() == false) {
                // Writing footer failed -> try to close file
                unlock(m_mutexFile); // Unlock file-mutex
                closeLogFile();
                return false;
            }

            // Unlock file-mutex
            unlock(m_mutexFile);

            // Success!
            return true;
        }

        // Count of objects currently in data buffer (see base class for details)
        virtual inline uint64_t countOfObjectsInBuffer() const { return m_dataBuffer.elementCount(); }

        // Members
        // -------
    protected:
        memory::CircularBuffer<LogFileDataType> m_dataBuffer; //!< Efficient **thread-safe** circular buffer for logfile data. Implements its own mutex.
        std::unique_ptr<LogFileDataType> m_headerData; //!< \copybrief headerData()
        std::unique_ptr<LogFileDataType> m_footerData; //!< \copybrief footerData()
    private:
        std::unique_ptr<LogFileDataType> m_currentData; //!< Internal buffer for data objects

        // Getters ("external" data)
        // -------
    public:
        //! Data object instance used for creating the file header \details **Thread-safe getter**
        inline LogFileDataType headerData() const { return getProtectedData(*m_headerData); }

        //! Data object instance used for creating the file footer \details **Thread-safe getter**
        inline LogFileDataType footerData() const { return getProtectedData(*m_footerData); }

        // Setters ("external" data)
        // -------
    public:
        //! **Thread-safe** setter for \ref headerData()
        void setHeaderData(const LogFileDataType& newData) { setProtectedData(*m_headerData, newData); }

        //! **Thread-safe** setter for \ref footerData()
        void setFooterData(const LogFileDataType& newData) { setProtectedData(*m_footerData, newData); }
    };
} // namespace io
} // namespace broccoli
