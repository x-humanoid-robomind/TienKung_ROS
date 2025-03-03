/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../memory/CircularBuffer.hpp"
#include "../../parallel/ThreadSafeContainer.hpp"
#include "../filesystem.hpp"
#include "LogFileData.hpp"
#include "LogFileSignal.hpp"
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#ifdef HAVE_ZLIB // is zlib library available?
#include <zlib.h>
#endif // HAVE_ZLIB

namespace broccoli {
namespace io {
    //! Abstract base class of logfile template
    /*!
     * \ingroup broccoli_io_logging
     * Necessary to allow management of multiple logfiles (with different \ref LogFileData types)
     * by one \ref Logger. Contains pure virtual functions
     *
     *  - \ref clearLogBuffer()
     *  - \ref writeLogHeader()
     *  - \ref processLogBuffer()
     *  - \ref writeLogFooter()
     *
     * which have to be defined by the derived class since they depend on the chosen type of \ref LogFileData. Those
     * methods are triggered by the \ref Logger and thus have to be known by the \ref Logger at compile time. In contrast
     * it is not neccessary to know the exact type of the logdata at this point.
     *
     * The class further handles all neccessary file and parameter members including corresponding **thread-safe** getters
     * and setters.
     *
     * \remark The class includes a signal buffer (\ref m_signalBuffer) which allows the \ref Logger to pass through signals
     * of the \ref LogFile to a supervising observer (i.e. the user application).
     */
    class LogFileBase : protected parallel::ThreadSafeContainer {
    public:
        //! Specialized constructor
        /*!
         * \param [in] identifier Initializes: \copybrief identifier()
         * \param [in] targetDirectory Initializes the **current** and **desired** target directory containing the logfile
         * \param [in] fileName Initializes the **current** and **desired** name of the logfile (without extension)
         * \param [in] fileExtension Initializes the **current** and **desired** file extension (including dot)
         * \param [in] addTimeStampToName Initializes the **current** and **desired** setting for adding a timestamp (system time) to the filename (after name, before extension)
         * \param [in] compressed Initializes the **current** and **desired** setting for compression of the file (if `true` the extension ".gz" is added to the filepath)
         * \param [in] enableSkipping Initializes: \copybrief enableSkipping()
         * \param [in] maximumStreamSize Estimated maximum count of bytes to write at once. Used to pre-allocate memory for better runtime performance.
         */
        LogFileBase(const std::string& identifier, const std::string& targetDirectory, const std::string& fileName, const std::string& fileExtension, const bool& addTimeStampToName, const bool& compressed, const bool& enableSkipping, const size_t& maximumStreamSize = 4096 /* <-- 4kB per default */)
            : ThreadSafeContainer()
            , m_identifier(identifier)
            , m_mutexFile(PTHREAD_RWLOCK_INITIALIZER)
            , m_desiredTargetDirectory(targetDirectory)
            , m_desiredFileName(fileName)
            , m_desiredFileExtension(fileExtension)
            , m_desiredAddTimeStampToName(addTimeStampToName)
            , m_desiredCompressed(compressed)
            , m_enableSkipping(enableSkipping)
            , m_targetDirectory(targetDirectory)
            , m_fileName(fileName)
            , m_fileExtension(fileExtension)
            , m_addTimeStampToName(addTimeStampToName)
            , m_compressed(compressed)
            , m_signalBuffer(1000)
        {
            // Pre-allocate memory (estimate memory requirements)
            m_logStream.reserve(maximumStreamSize);
        }

        //! Deleted copy constructor (prevent user to copy object - would be dangerous due to internal mutexes)
        LogFileBase(const LogFileBase& original) = delete;

        //! Deleted copy assignment operator (prevent user to copy object - would be dangerous due to internal mutexes)
        LogFileBase& operator=(const LogFileBase& reference) = delete;

        //! Destructor
        /*! Closes the file and clears buffers. */
        virtual ~LogFileBase()
        {
            // Check, if file is open
            if (isOpen() == true) {
                assert(false && "This should never happen");
            }

            // Clear buffers
            m_signalBuffer.clear();
        }

    protected:
        //! Move constructor (internal) (**not** thread-safe, should only be called by thread-safe wrapper (to be implemented by derived class))
        LogFileBase(LogFileBase&& other)
            : ThreadSafeContainer(std::move(other))
            , m_identifier(std::move(other.m_identifier))
            , m_mutexFile(PTHREAD_RWLOCK_INITIALIZER) // DO NOT MOVE MUTEX (USE OWN MUTEX)
            , m_desiredTargetDirectory(std::move(other.m_desiredTargetDirectory))
            , m_desiredFileName(std::move(other.m_desiredFileName))
            , m_desiredFileExtension(std::move(other.m_desiredFileExtension))
            , m_desiredAddTimeStampToName(std::move(other.m_desiredAddTimeStampToName))
            , m_desiredCompressed(std::move(other.m_desiredCompressed))
            , m_active(std::move(other.m_active))
            , m_enableSkipping(std::move(other.m_enableSkipping))
            , m_countOfObjectsToSkip(std::move(other.m_countOfObjectsToSkip))
            , m_enableFlush(std::move(other.m_enableFlush))
            , m_flushPacketSize(std::move(other.m_flushPacketSize))
            , m_fileAttributes(std::move(other.m_fileAttributes))
            , m_targetDirectory(std::move(other.m_targetDirectory))
            , m_fileName(std::move(other.m_fileName))
            , m_fileExtension(std::move(other.m_fileExtension))
            , m_addTimeStampToName(std::move(other.m_addTimeStampToName))
            , m_compressed(std::move(other.m_compressed))
            , m_totalReceivedObjectCount(std::move(other.m_totalReceivedObjectCount))
            , m_totalWrittenObjectCount(std::move(other.m_totalWrittenObjectCount))
            , m_logStream(std::move(other.m_logStream))
            , m_logStreamStartIndex(std::move(other.m_logStreamStartIndex))
            , m_uncompressedFile(other.m_uncompressedFile) // Copy file pointer
#ifdef HAVE_ZLIB
            , m_compressedFile(other.m_compressedFile) // Copy file pointer
#endif
            , m_isOpen(std::move(other.m_isOpen))
            , m_totalBytesWritten(std::move(other.m_totalBytesWritten))
            , m_lastFlushByteCount(std::move(other.m_lastFlushByteCount))
            , m_signalBuffer(std::move(other.m_signalBuffer))
        {
        }

        //! Move assignment operator (**not** thread-safe, should only be called by thread-safe wrapper (to be implemented by derived class))
        LogFileBase& operator=(LogFileBase&& other)
        {
            // Avoid self-assignment
            if (this == &other)
                return *this;

            // Move data
            ThreadSafeContainer::operator=(std::move(other));
            m_identifier = std::move(other.m_identifier);
            m_mutexFile = PTHREAD_RWLOCK_INITIALIZER; // DO NOT MOVE MUTEX  = USE OWN MUTEX;
            m_desiredTargetDirectory = std::move(other.m_desiredTargetDirectory);
            m_desiredFileName = std::move(other.m_desiredFileName);
            m_desiredFileExtension = std::move(other.m_desiredFileExtension);
            m_desiredAddTimeStampToName = std::move(other.m_desiredAddTimeStampToName);
            m_desiredCompressed = std::move(other.m_desiredCompressed);
            m_active = std::move(other.m_active);
            m_enableSkipping = std::move(other.m_enableSkipping);
            m_countOfObjectsToSkip = std::move(other.m_countOfObjectsToSkip);
            m_enableFlush = std::move(other.m_enableFlush);
            m_flushPacketSize = std::move(other.m_flushPacketSize);
            m_fileAttributes = std::move(other.m_fileAttributes);
            m_targetDirectory = std::move(other.m_targetDirectory);
            m_fileName = std::move(other.m_fileName);
            m_fileExtension = std::move(other.m_fileExtension);
            m_addTimeStampToName = std::move(other.m_addTimeStampToName);
            m_compressed = std::move(other.m_compressed);
            m_totalReceivedObjectCount = std::move(other.m_totalReceivedObjectCount);
            m_totalWrittenObjectCount = std::move(other.m_totalWrittenObjectCount);
            m_logStream = std::move(other.m_logStream);
            m_logStreamStartIndex = std::move(other.m_logStreamStartIndex);
            m_uncompressedFile = other.m_uncompressedFile; // Copy file pointer
#ifdef HAVE_ZLIB
            m_compressedFile = other.m_compressedFile; // Copy file pointer
#endif
            m_isOpen = std::move(other.m_isOpen);
            m_totalBytesWritten = std::move(other.m_totalBytesWritten);
            m_lastFlushByteCount = std::move(other.m_lastFlushByteCount);
            m_signalBuffer = std::move(other.m_signalBuffer);

            return *this;
        }

        // Main functionality (triggered by Logger)
        // ------------------
    public:
        //! **Thread-safe** opening of new log file to write into (triggered by supervising \ref Logger)
        /*!
         * \remark In case of an error a corrresponding \ref LogFileSignal is added to the buffer (\ref m_signalBuffer).
         *
         * \return `true` on success, `false` otherwise
         */
        virtual bool openLogFile()
        {
            // Lock file-mutex for write
            lockForWrite(m_mutexFile);

            // Check, if file has not been opened already
            // ------------------------------------------
            if (m_isOpen == true) {
                m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_OPEN_FAILED_ALREADY_OPEN));
                unlock(m_mutexFile); // Unlock file-mutex
                return false;
            }

            // Update file information
            // -----------------------
            // Update all within one mutex-lock to avoid inconsistent data
            lockForWrite(); // Lock parameter-mutex for write
            m_targetDirectory = m_desiredTargetDirectory;
            m_fileName = m_desiredFileName;
            m_fileExtension = m_desiredFileExtension;
            m_addTimeStampToName = m_desiredAddTimeStampToName;
            m_compressed = m_desiredCompressed;
            std::string currentTargetDirectory = m_targetDirectory;
            std::string currentFileName = m_fileName;
            std::string currentFileExtension = m_fileExtension;
            bool currentAddTimeStampToName = m_addTimeStampToName;
            bool currentCompressed = m_compressed;
            unlock(); // Unlock parameter-mutex

            // Compute new total filepath
            // --------------------------
            std::stringstream filePathStream;

            // Add target directory
            if (currentTargetDirectory.length() > 0)
                filePathStream << currentTargetDirectory << "/";

            // Add file name
            filePathStream << currentFileName;

            // Optionally add time stamp
            if (currentAddTimeStampToName == true)
                filePathStream << "-" << core::Time::currentTime().encodeToDateTimeString(true, "%Y-%m-%d_%H-%M-%S", 0);

            // Add extension
            filePathStream << currentFileExtension;

            // Add extension of compression file format
            if (currentCompressed == true)
                filePathStream << ".gz";

            // Convert to string
            std::string filePath = filePathStream.str();

            // Check, if target directory exists and try to create it if not
            // -------------------------------------------------------------
            if (currentTargetDirectory.length() > 0) {
                // Try to create folder if it does not exist already
                if (filesystem::directoryExists(currentTargetDirectory) == false) {
                    (void)filesystem::createDirectory(currentTargetDirectory); // May fail, if another thread/process creates the directory in the meantime -> return value is not representative

                    // Check, if directory exists now
                    if (filesystem::directoryExists(currentTargetDirectory) == false) {
                        // We could not create the directory so we also cannot open the log file -> abort
                        m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_OPEN_FAILED_FOLDER_CREATION_FAILED));
                        unlock(m_mutexFile); // Unlock file-mutex
                        return false;
                    }
                }
            }

            // Open file
            // ---------
            // Check, if we should use file compression
            if (currentCompressed == false) {
                // Try to open uncompressed file
                try {
                    m_uncompressedFile = fopen(filePath.c_str(), "w");
                    if (m_uncompressedFile != nullptr)
                        m_isOpen = true;
                } catch (...) {
                    m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_OPEN_FAILED_LOWLEVEL));
                    unlock(m_mutexFile); // Unlock file-mutex
                    return false;
                }
            } else {
                // Try to open compressed file
#ifdef HAVE_ZLIB
                try {
                    m_compressedFile = gzopen(filePath.c_str(), "w");
                    if (m_compressedFile != nullptr)
                        m_isOpen = true;
                } catch (...) {
                    m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_OPEN_FAILED_LOWLEVEL));
                    unlock(m_mutexFile); // Unlock file-mutex
                    return false;
                }
#else
                // Not compiled with zlib support
                m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_ZLIB_NOT_FOUND));
                unlock(m_mutexFile); // Unlock file-mutex
                return false;
#endif
            }

            // Reset byte counters
            m_totalBytesWritten = 0;
            m_lastFlushByteCount = 0;

            // Unlock file-mutex
            unlock(m_mutexFile);

            // Reset counter for written objects
            setTotalWrittenObjectCount(0);

            // Success!
            return true;
        }

        //! **Thread-safe** closing of logfile (triggered by supervising \ref Logger)
        /*! \return `true` if file was open, `false` if it was not open */
        virtual bool closeLogFile()
        {
            // Get compression state
            bool currentCompressed = compressed();

            // Lock file-mutex for write
            lockForWrite(m_mutexFile);

            // Remember current state
            bool wasFileOpen = m_isOpen;

            // Close file (only if it was open)
            // ----------
            if (wasFileOpen == true) {
                try {
                    if (currentCompressed == false) {
                        fclose(m_uncompressedFile);
                    }
#ifdef HAVE_ZLIB
                    else {
                        gzclose(m_compressedFile);
                    }
#endif

                } catch (...) {
                    // ...do nothing in case of a lowlevel error
                }
            }

            // Update state
            m_isOpen = false;

            // Unlock file-mutex
            unlock(m_mutexFile);

            // Return previous state
            return wasFileOpen;
        }

        // Abstract implementations
        // ------------------------
        //! **Thread-safe** forced clearing of logdata buffer
        /*! Clears the complete logdata buffer. \warning All data in the buffer is discarded! */
        virtual void clearLogBuffer() = 0;

        //! **Thread-safe** writing of the file header
        /*!
         * \remark In case of an error a corrresponding \ref LogFileSignal is added to the buffer (\ref m_signalBuffer).
         *
         * \param [in] logStartTime The time when the log was started.
         * \return `true` on success, `false` otherwise
         */
        virtual bool writeLogHeader(const core::Time& logStartTime) = 0;

        //! **Thread-safe** writing of current contents of the logdata buffer to the logfile
        /*!
         * \remark In case of an error a corrresponding \ref LogFileSignal is added to the buffer (\ref m_signalBuffer).
         *
         * \return `true` on success, `false` otherwise
         */
        virtual bool processLogBuffer() = 0;

        //! **Thread-safe** writing of the file footer
        /*!
         * \remark In case of an error a corrresponding \ref LogFileSignal is added to the buffer (\ref m_signalBuffer).
         *
         * \param [in] logStopTime The time when the log was stopped.
         * \return `true` on success, `false` otherwise
         */
        virtual bool writeLogFooter(const core::Time& logStopTime) = 0;

    protected:
        //! Write current content of log stream to file (**not** thread-safe, file-mutex has to be locked by caller!)
        /*!
         * \warning **Not** thread-safe, file-mutex (\ref m_mutexFile) has to be locked by caller!
         *
         * \remark In case of an error a corrresponding \ref LogFileSignal is added to the buffer (\ref m_signalBuffer).
         *
         * \return `true` on success, `false` otherwise
         */
        bool writeCurrentLogStreamToFile()
        {
            // Initialize helpers
            static const int gzprintfBufferSize = 8191; // gzprintf can only take a maximum of 8191 bytes (2^13 - 1 (null-terminated))
            const int bytesToWrite = m_logStream.size() - m_logStreamStartIndex; // Count of (uncompressed) bytes to write
            int bytesWritten = 0; // Count of written (uncompressed) bytes

            // Only proceed, if there is really something to write
            if (bytesToWrite == 0)
                return true; // Nothing to do -> handle as success

            // Get compression state
            const bool currentCompressed = compressed();

            // Get flush settings
            const bool currentEnableFlush = enableFlush();
            const uint64_t currentFlushPacketSize = flushPacketSize();

            // Step 1: write data
            // ------------------
            try {
                if (currentCompressed == false) {
                    // (Try to) write bytes to file
                    bytesWritten = fprintf(m_uncompressedFile, "%.*s", bytesToWrite, &m_logStream[m_logStreamStartIndex]);

                    // Check, how many bytes have been written
                    if (bytesWritten < 0) {
                        // An error occured...
                        bytesWritten = 0; // Nothing has been written
                        m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_WRITE_FAILED_DATABUFFER));
                    }
                    // else:
                    // Option A: bytesWritten < bytesToWrite -> keep data in buffer and try to write remaining data next time
                    // Option B: bytesWritten = bytesToWrite -> everything was fine!
                    // Option C: bytesWritten > bytesToWrite -> should NEVER happen
                }
#ifdef HAVE_ZLIB
                else {
                    // Stream splitting
                    // ----------------
                    // gzprintf can only take a maximum of 8191 bytes (2^13 - 1 (null-terminated))
                    // -> stream has to be split into parts, if it is greater

                    // Compute in how many parts we have to split
                    int partCount = bytesToWrite / gzprintfBufferSize;
                    if (bytesToWrite % gzprintfBufferSize > 0)
                        partCount += 1;

                    // Write all parts
                    for (int part = 0; part < partCount; part++) {
                        // Initialize helpers
                        int currentPartBytesToWrite = gzprintfBufferSize; // Count of (uncompressed) bytes to write of current part
                        if (part == partCount - 1)
                            currentPartBytesToWrite = (bytesToWrite - part * gzprintfBufferSize); // Last part -> write only remaining bytes

                        // (Try to) write bytes to file
                        int currentPartBytesWritten = gzprintf(m_compressedFile, "%.*s", currentPartBytesToWrite, &m_logStream[m_logStreamStartIndex + part * gzprintfBufferSize]);

                        // Check, how many bytes have been written
                        if (currentPartBytesWritten < 0) {
                            // An error occured...
                            currentPartBytesWritten = 0;
                            m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_WRITE_FAILED_DATABUFFER));
                            break; // Exit loop (try later again)
                        } else if (currentPartBytesWritten != currentPartBytesToWrite) {
                            bytesWritten += currentPartBytesWritten; // Update written bytes
                            break; // We could not write everything -> skip remaining parts
                        } else
                            bytesWritten += currentPartBytesWritten; // Update written bytes
                    }
                }
#endif
            } catch (...) {
                m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_WRITE_FAILED_LOWLEVEL));
                return false;
            }

            // Update counter of totally written bytes (used for flush control)
            m_totalBytesWritten += bytesWritten;

            // Remove written data from buffer
            if (bytesWritten > 0) {
                // Check, if everything has been written
                if (bytesWritten >= bytesToWrite) {
                    m_logStream.clear(); // Clear data but keep capacity -> reusing already allocated memory!
                    m_logStreamStartIndex = 0; // Re-start right from the beginning
                } else {
                    // ...not all of the data has been written -> shift beginning
                    m_logStreamStartIndex += bytesWritten;
                }
            }

            // Step 2: flush data
            // ------------------
            if (currentEnableFlush == true && m_totalBytesWritten >= m_lastFlushByteCount + currentFlushPacketSize) {
                try {
                    // Flush
                    if (currentCompressed == false)
                        fflush(m_uncompressedFile);
#ifdef HAVE_ZLIB
                    else
                        gzflush(m_compressedFile, Z_SYNC_FLUSH);
#endif

                    // Remember event
                    m_lastFlushByteCount = m_totalBytesWritten;
                } catch (...) {
                    m_signalBuffer.push(LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_WRITE_FAILED_FLUSH));
                    return false;
                }
            }

            // Success
            return true;
        }

        // Constants
        // ---------
    private:
        std::string m_identifier; //!< \copybrief identifier()

        // Protected data
        // --------------
    protected:
        mutable pthread_rwlock_t m_mutexFile; //!< Read-write mutex for thread-safety of file access

    private:
        // Start protected area of m_mutex (external-visible parameters)
        std::string m_desiredTargetDirectory; //!< \copybrief desiredTargetDirectory()
        std::string m_desiredFileName; //!< \copybrief desiredFileName()
        std::string m_desiredFileExtension; //!< \copybrief desiredFileExtension()
        bool m_desiredAddTimeStampToName; //!< \copybrief desiredAddTimeStampToName()
        bool m_desiredCompressed; //!< \copybrief desiredCompressed()
        bool m_active = true; //!< \copybrief active()
        bool m_enableSkipping; //!< \copybrief enableSkipping()
        uint64_t m_countOfObjectsToSkip = 0; //!< \copybrief countOfObjectsToSkip()
        bool m_enableFlush = false; //!< \copybrief enableFlush()
        uint64_t m_flushPacketSize = 4096; //!< \copybrief flushPacketSize()
        std::unordered_map<std::string, std::string> m_fileAttributes; //!< \copybrief fileAttributes()
        // End protected area of m_mutex (external-visible parameters)

    private:
        // Start protected area of m_mutex (internal-only parameters)
        std::string m_targetDirectory; //!< \copybrief targetDirectory()
        std::string m_fileName; //!< \copybrief fileName()
        std::string m_fileExtension; //!< \copybrief fileExtension()
        bool m_addTimeStampToName; //!< \copybrief addTimeStampToName()
        bool m_compressed; //!< \copybrief compressed()
        uint64_t m_totalReceivedObjectCount = 0; //!< \copybrief totalReceivedObjectCount()
        uint64_t m_totalWrittenObjectCount = 0; //!< \copybrief totalWrittenObjectCount()
        // End protected area of m_mutex (internal-only parameters)

    protected:
        // Start protected area of m_mutexFile ("internal-only")
        LogFileData::LogStream m_logStream; //!< Pre-allocated data stream (for reusing already allocated memory)
        size_t m_logStreamStartIndex = 0; //!< Index of first byte to be written from \ref m_logStream (all previous bytes have already been written)
        FILE* m_uncompressedFile = nullptr; //!< File object for writing (uncompressed)
#ifdef HAVE_ZLIB
        gzFile m_compressedFile = nullptr; //!< File object for writing (compressed)
#endif
        bool m_isOpen = false; //!< \copybrief isOpen()
        uint64_t m_totalBytesWritten = 0; //!< Count of total bytes written (uncompressed)
        uint64_t m_lastFlushByteCount = 0; //!< Count of total bytes written at last flush event (used for flush control)
        // End protected area of m_mutexFile ("internal-only")

        // Buffers
        // -------
    public:
        memory::CircularBuffer<LogFileSignal> m_signalBuffer; //!< Buffer for signals of the logfile to a supervising \ref Logger (**thread-safe** on its own)

        // Getters ("external" data)
        // -------
    public:
        //! Unique identifier for this log file \details **Thread-safe getter**
        inline std::string identifier() const { return m_identifier; }
        //! The **desired** target directory containing the logfile (will be used on next \ref openLogFile() call) \details **Thread-safe getter**
        inline std::string desiredTargetDirectory() const { return getProtectedData(m_desiredTargetDirectory); }
        //! The **desired** name of the logfile (without extension) (will be used on next \ref openLogFile() call) \details **Thread-safe getter**
        inline std::string desiredFileName() const { return getProtectedData(m_desiredFileName); }
        //! The **desired** file extension (including dot) (will be used on next \ref openLogFile() call) \details **Thread-safe getter**
        inline std::string desiredFileExtension() const { return getProtectedData(m_desiredFileExtension); }
        //! The **desired** setting for adding a timestamp (system time) to the filename (after name, before extension) (will be used on next \ref openLogFile() call) \details **Thread-safe getter**
        inline bool desiredAddTimeStampToName() const { return getProtectedData(m_desiredAddTimeStampToName); }
        //! The **desired** setting for compression of the file (if `true` the extension ".gz" is added to the filepath) (will be used on next \ref openLogFile() call) \details **Thread-safe getter**
        inline bool desiredCompressed() const { return getProtectedData(m_desiredCompressed); }
        //! Flag to (de-)activate the logdata buffer. If set to `false`, incoming logdata-objects are not added to the buffer (discards objects immediately before adding to the buffer) \details **Thread-safe getter**
        inline bool active() const { return getProtectedData(m_active); }
        //! Flag to (de-)activate skipping of objects (count of objects to skip is determined by \ref countOfObjectsToSkip()) \details **Thread-safe getter**
        inline bool enableSkipping() const { return getProtectedData(m_enableSkipping); }
        //! Count of objects to skip (imagine as "write only every i-th dataset") (discards objects immediately before adding to the buffer) \details **Thread-safe getter**
        inline uint64_t countOfObjectsToSkip() const { return getProtectedData(m_countOfObjectsToSkip); }
        //! Flag to (de-)activate data flush after a certain amount of bytes in the buffer
        /*! \warning Explicit flushing forces the system to empty the buffer and write the data to the file system. Frequent flushing can reduce the system performance significantly! */
        inline bool enableFlush() const { return getProtectedData(m_enableFlush); }
        //! Count of bytes in the write buffer after which an explicit flush is triggered (if flushing is enabled)
        /*! \warning Avoid to call flush too often, since this reduces the compression level in the compressed case (use at least 4096 bytes). Also flushing reduced performance when writing over the network (e.g. through NFS). */
        inline uint64_t flushPacketSize() const { return getProtectedData(m_flushPacketSize); }
        //! Count of objects currently in data buffer \details **Thread-safe getter**
        virtual inline uint64_t countOfObjectsInBuffer() const = 0;
        //! The file attributes (key/value pairs of strings), which are encoded to the header-section of the logfile. \details **Thread-safe getter**
        inline std::unordered_map<std::string, std::string> fileAttributes() const { return getProtectedData(m_fileAttributes); }

        // Getters ("internal" data)
        // -------
    public:
        //! The **current** target directory containing the logfile \details **Thread-safe getter**
        inline std::string targetDirectory() const { return getProtectedData(m_targetDirectory); }
        //! The **current** name of the logfile (without extension) \details **Thread-safe getter**
        inline std::string fileName() const { return getProtectedData(m_fileName); }
        //! The **current** file extension (including dot) \details **Thread-safe getter**
        inline std::string fileExtension() const { return getProtectedData(m_fileExtension); }
        //! The **current** setting for adding a timestamp (system time) to the filename (after name, before extension) \details **Thread-safe getter**
        inline bool addTimeStampToName() const { return getProtectedData(m_addTimeStampToName); }
        //! The **current** setting for compression of the file (if `true` the extension ".gz" is added to the filepath) \details **Thread-safe getter**
        inline bool compressed() const { return getProtectedData(m_compressed); }
        //! Total count of **received** objects (some of them may have been skipped/discarded). Gets resetted with \ref clearLogBuffer() \details **Thread-safe getter**
        inline uint64_t totalReceivedObjectCount() const { return getProtectedData(m_totalReceivedObjectCount); }
        //! Total count of **written** objects. Gets resetted with \ref openLogFile() \details **Thread-safe getter**
        inline uint64_t totalWrittenObjectCount() const { return getProtectedData(m_totalWrittenObjectCount); }
        //! Flag to indicate if the the file is **currently** open \details **Thread-safe getter**
        inline bool isOpen() const
        {
            lockForRead(m_mutexFile); // Lock protected memory for read
            bool returnValue = m_isOpen; // Store data to temporary variable
            unlock(m_mutexFile); // Unlock protected memory
            return returnValue; // Return temporary variable
        }

        // Setters ("external" data)
        // -------
    public:
        //! **Thread-safe setter for:** \copybrief desiredTargetDirectory()
        inline void setDesiredTargetDirectory(const std::string& newValue) { setProtectedData(m_desiredTargetDirectory, newValue); }
        //! **Thread-safe setter for:** \copybrief desiredFileName()
        inline void setDesiredFileName(const std::string& newValue) { setProtectedData(m_desiredFileName, newValue); }
        //! **Thread-safe setter for:** \copybrief desiredFileExtension()
        inline void setDesiredFileExtension(const std::string& newValue) { setProtectedData(m_desiredFileExtension, newValue); }
        //! **Thread-safe setter for:** \copybrief desiredAddTimeStampToName()
        inline void setDesiredAddTimeStampToName(const bool& newValue) { setProtectedData(m_desiredAddTimeStampToName, newValue); }
        //! **Thread-safe setter for:** \copybrief desiredCompressed()
        inline void setDesiredCompressed(const bool& newValue) { setProtectedData(m_desiredCompressed, newValue); }
        //! **Thread-safe setter for:** \copybrief active()
        inline void setActive(const bool& newValue) { setProtectedData(m_active, newValue); }
        //! **Thread-safe setter for:** \copybrief enableSkipping()
        inline void setEnableSkipping(const bool& newValue) { setProtectedData(m_enableSkipping, newValue); }
        //! **Thread-safe setter for:** \copybrief countOfObjectsToSkip()
        inline void setCountOfObjectsToSkip(const uint64_t& newValue) { setProtectedData(m_countOfObjectsToSkip, newValue); }
        //! **Thread-safe setter for:** \copybrief enableFlush()
        inline void setEnableFlush(const bool& newValue) { setProtectedData(m_enableFlush, newValue); }
        //! **Thread-safe setter for:** \copybrief flushPacketSize()
        inline void setFlushPacketSize(const uint64_t& newValue) { setProtectedData(m_flushPacketSize, newValue); }

        /*!
         * \brief Sets a file attribute value.
         *
         * File attributes are encoded to the header of a logfile.
         * \details Thread-Safe
         * \param key The string key of the attribute
         * \param value The string value of the attribute
         */
        void setAttribute(const std::string& key, const std::string& value)
        {
            lockForWrite();
            m_fileAttributes[key] = value;
            unlock();
        }

        /*!
         * \brief Remove a file attribute
         * \details Thread-Safe
         * \param key The string key of the attribute to remove
         */
        void removeAttribute(const std::string& key)
        {
            lockForWrite();
            m_fileAttributes.erase(key);
            unlock();
        }

        /*!
         * \brief Add the provided attributes to the file attributes.
         *
         * Already existing attributes are **NOT** overwritten.
         * \details Thread-Safe
         * \param attributes The attribute map with the file attributes to add
         */
        void addAttributes(const std::unordered_map<std::string, std::string>& attributes)
        {
            lockForWrite();
            m_fileAttributes.insert(attributes.begin(), attributes.end());
            unlock();
        }

        /*!
         * \brief Clears all file attributes
         * \details Thread-Safe
         */
        void clearAttributes()
        {
            lockForWrite();
            m_fileAttributes.clear();
            unlock();
        }

        // Setters ("internal" data)
        // -------
    protected:
        // "Internal" data
        //! **Thread-safe setter for:** \copybrief targetDirectory()
        inline void setTargetDirectory(const std::string& newValue) { setProtectedData(m_targetDirectory, newValue); }
        //! **Thread-safe setter for:** \copybrief fileName()
        inline void setFileName(const std::string& newValue) { setProtectedData(m_fileName, newValue); }
        //! **Thread-safe setter for:** \copybrief fileExtension()
        inline void setFileExtension(const std::string& newValue) { setProtectedData(m_fileExtension, newValue); }
        //! **Thread-safe setter for:** \copybrief addTimeStampToName()
        inline void setAddTimeStampToName(const bool& newValue) { setProtectedData(m_addTimeStampToName, newValue); }
        //! **Thread-safe setter for:** \copybrief compressed()
        inline void setCompressed(const bool& newValue) { setProtectedData(m_compressed, newValue); }
        //! **Thread-safe setter for:** \copybrief totalReceivedObjectCount()
        inline void setTotalReceivedObjectCount(const uint64_t& newValue) { setProtectedData(m_totalReceivedObjectCount, newValue); }
        //! **Thread-safe setter for:** \copybrief totalWrittenObjectCount()
        inline void setTotalWrittenObjectCount(const uint64_t& newValue) { setProtectedData(m_totalWrittenObjectCount, newValue); }

        // Helpers
        // -------
        //! **Thread-safe:** Increments \copybrief totalReceivedObjectCount() by the specified value.
        /*!
         * \param [in] value The value to add to the counter
         * \return The **new** value of the counter (after incrementing it)
         */
        inline uint64_t incrementTotalReceivedObjectCount(const uint64_t& value = 1)
        {
            lockForWrite(); // Lock protected memory for write
            m_totalReceivedObjectCount += value;
            const uint64_t currentValue = m_totalReceivedObjectCount;
            unlock(); // Unlock protected memory
            return currentValue;
        }

        //! **Thread-safe:** Increments \copybrief totalWrittenObjectCount() by the specified value.
        /*!
         * \param [in] value The value to add to the counter
         * \return The **new** value of the counter (after incrementing it)
         */
        inline uint64_t incrementTotalWrittenObjectCount(const uint64_t& value = 1)
        {
            lockForWrite(); // Lock protected memory for write
            m_totalWrittenObjectCount += value;
            const uint64_t currentValue = m_totalWrittenObjectCount;
            unlock(); // Unlock protected memory
            return currentValue;
        }
    };
} // namespace io
} // namespace broccoli
