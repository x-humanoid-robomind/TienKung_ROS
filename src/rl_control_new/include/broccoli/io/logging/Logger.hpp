/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include "../../memory/CircularBuffer.hpp"
#include "../../parallel/BackgroundWorker.hpp"
#include "LogFile.hpp"
#include "LoggerAction.hpp"
#include "LoggerSignal.hpp"
#include <vector>

namespace broccoli {
namespace io {
    //! Supervisor module for handling multiple logfiles
    /*!
     * \ingroup broccoli_io_logging
     *
     * The files to supervise have to be attached to an internal list (pointers to logfiles to keep track of).
     * \warning One has to ensure, that the logfiles are accessable by the logger at any time! Thus the logger
     * must be deinitialized with \ref deInitialize() **before** the logfiles get destroyed (or change their
     * pointer address)!
     *
     * \remark Optionally runs in a background thread for efficiency.
     *
     * Folder Structure
     * ----------------
     * \code
     * <FirstLevelDirectory>/<SecondLevelDirectory>/<LogFileName><LogFileExtension>
     * \endcode
     *
     * \par First Level Directory:
     *
     * - specified for the \ref Logger (use \ref setFirstLevelDirectory() to change)
     * - applies to **every** \ref LogFile in the list of tracked files (changed through \ref LogFile::setTargetDirectory(`FirstLevelDirectory/SecondLevelDirectory`))
     * - may be a complete absolute or relative (to executable) path
     *
     * \par Second Level Directory:
     *
     * - specified with triggering a \ref LoggerAction::Type::LOG_START or \ref LoggerAction::Type::LOG_DUMP action as parameter \ref LoggerAction::m_text
     * - applies to **every** \ref LogFile in the list of tracked files (changed through \ref LogFile::setTargetDirectory(`FirstLevelDirectory/SecondLevelDirectory`))
     *
     * \par Log File Name and Extension:
     *
     * - specified for each \ref LogFile independently (stored in the \ref LogFile itself)
     * - use \ref LogFile::setFileName() and \ref LogFile::setFileExtension() to change
     *
     * \attention The \ref Logger automatically creates the **last** folder in `FirstLevelDirectory`, if it does not exist. All higher levels have to exist already!
     * Furthermore the `SecondLevelDirectory` has to be a **single folder** which will also be created automatically. Complex folder hierarchies are **not supported**
     * for `SecondLevelDirectory`!
     */
    class Logger final : public parallel::BackgroundWorker {
    public:
        //! Constructor (runs in parent thread)
        /*!
         * \param [in] threadName Initializes: \copybrief broccoli::parallel::BackgroundWorker::name()
         * \param [in] multiThreaded Initializes: \copybrief broccoli::parallel::BackgroundWorker::multiThreaded()
         * \param [in] threadPriority Initializes: \copybrief broccoli::parallel::BackgroundWorker::threadPriority()
         * \param [in] firstLevelDirectory Initializes: \copybrief firstLevelDirectory()
         */
        Logger(const std::string& threadName, const bool& multiThreaded, const int& threadPriority, const std::string& firstLevelDirectory = "")
            : BackgroundWorker(threadName, multiThreaded, threadPriority, 0.001 /* <-- max. 1kHz */, false) // Setup background thread for low CPU usage (data is buffered -> logging is NOT time-critical)
            , m_firstLevelDirectory(firstLevelDirectory)
            , m_signalBuffer(1000)
            , m_actionBuffer(1000)
        {
        }

    private:
        //! Copy constructor (internal) (**not** thread-safe -> should only be called by the thread-safe wrapper)
        /*! \param [in] original Reference to original object. */
        Logger(const Logger& original, const int& /* <- trick used for locking mutex */)
            : BackgroundWorker(original)
            , m_trackedFiles(original.m_trackedFiles)
            , m_firstLevelDirectory(original.m_firstLevelDirectory)
            , m_signalBuffer(original.m_signalBuffer)
            , m_actionBuffer(original.m_actionBuffer)
        {
        }

    public:
        //! Copy constructor (wrapper) (**thread-safe**)
        /*!
         * Locks the mutex of the original object for reading before calling the internal (protected) constructor. Unlocks original object afterwards.
         * \param [in] original Reference to original object.
         */
        Logger(const Logger& original)
            : Logger(original, original.lockForRead() /* <- lock mutex of original object first (for reading since also using "const") */)
        {
            original.unlock(); // Unlock mutex of original object after internal (protected) constructor has been called
        }

        //! Copy assignment operator (**thread-safe**)
        /*!
         * Uses own mutex and mutex of the reference object to guarantee thread-safe copying of members.
         * \param [in] reference Reference to reference object.
         * \return Pointer to this instance.
         */
        Logger& operator=(const Logger& reference)
        {
            // Avoid self-assignment
            if (this == &reference)
                return *this;

            // Try to lock ourselves and reference (while avoiding deadlocks)
            while (true) { // spinning
                lockForWrite(); // Lock ourselves for writing (blocking)
                if (reference.tryLockForRead() == true) // Try to lock reference for reading
                    break; // ...success -> stop spinning
                else
                    unlock(); // ...fail -> unlock ourselves to allow other threads to access us and thus resolve possible deadlocks
            }

            // Copy data
            BackgroundWorker::operator=(reference);
            m_trackedFiles = reference.m_trackedFiles;
            m_firstLevelDirectory = reference.m_firstLevelDirectory;
            m_signalBuffer = reference.m_signalBuffer;
            m_actionBuffer = reference.m_actionBuffer;

            // Unlock reference object and ourselves
            reference.unlock();
            unlock();

            return *this;
        }

        //! Destructor
        ~Logger()
        {
            // Join background thread (if not already done)
            join(core::Time(10));
        }

        //! Initialize the \ref Logger (runs in parent thread)
        /*!
         * Starts background thread.
         * \return `true` on success, `false` on failure
         */
        bool initialize() { return start(); }

        //! De-initialize the \ref Logger (runs in parent thread)
        /*!
         * Waits until background thread has finished its work. Forces the files to close (if they have not already).
         *
         * \warning This has to be called **before** the logfiles get destroyed (or change their pointer address)!
         *
         * \par Notes on \p timeout
         * Log data is very important especially in case of errors. Thus one should set the timeout to a high value
         * to store as much data as possible. However one should stop after some reasonable time (when it is unlikely
         * that data is really written but it is more likely that something crashed within the logging process).
         *
         * \param [in] timeout Timeout for waiting until background thread finished its work.
         * \return `true` if background worker was shutdown properly, `false` otherwise
         */
        bool deInitialize(const core::Time& timeout = core::Time(10))
        {
            // Join background thread and remember result
            bool threadJoinSuccessful = join(timeout);

            // Close log files, if they are still open
            closeFiles(core::Time::currentTime());

            // Pass back result
            return threadJoinSuccessful;
        }

        //! Add file to list of tracked files (**thread-safe**)
        /*!
         * Checks, if the identifier already exists in the list of tracked files to avoid adding the same file twice.
         * \param [in] newFile Pointer to the file to be tracked.
         * \return `true`, if the file was added, `false` otherwise.
         */
        bool addFile(LogFileBase* const newFile)
        {
            // Check, if pointer is valid
            if (newFile == nullptr)
                return false;

            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Pass through list and search for identifier
            for (size_t i = 0; i < currentTrackedFiles.size(); i++)
                if (currentTrackedFiles[i]->identifier() == newFile->identifier())
                    return false;

            // Everything seems to be ok -> add file to list
            currentTrackedFiles.push_back(newFile);

            // Update list
            setTrackedFiles(currentTrackedFiles);

            // File was added -> success
            return true;
        }

        //! Get pointer of tracked file by identifier (**thread-safe**)
        /*!
         * Searches in the list of tracked files for the specified identifier.
         * \param [in] identifier Identifier of the desired tracked file
         * \return Pointer to the tracked file, or `nullptr` in case the identifier does not exist
         */
        LogFileBase* getFile(const std::string& identifier)
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Pass through list and search for identifier
            for (size_t i = 0; i < currentTrackedFiles.size(); i++)
                if (currentTrackedFiles[i]->identifier() == identifier)
                    return currentTrackedFiles[i];

            // Element could not be found -> error
            return nullptr;
        }

        //! Remove file of list of tracked filed (**thread-safe**)
        /*!
         * Searches for the specified file (by identifier) and removes it from the list of tracked files.
         * \param [in] identifier Indentifier of file to remove
         * \return `true`, if the file was found and removed from the list, `false` otherwise
         */
        bool removeFile(const std::string& identifier)
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Pass through list and search for identifier
            for (size_t i = 0; i < currentTrackedFiles.size(); i++)
                if (currentTrackedFiles[i]->identifier() == identifier) {
                    // Remove this item
                    currentTrackedFiles.erase(currentTrackedFiles.begin() + i);

                    // Update list
                    setTrackedFiles(currentTrackedFiles);

                    // We found the element -> success
                    return true;
                }

            // Element could not be found -> error
            return false;
        }

    private:
        // Hide base class functionality (use initialize() and deInitialize() instead!)
        using BackgroundWorker::join;
        using BackgroundWorker::start;
        using BackgroundWorker::stop;

    private:
        //! Main execution function (one cylce of execution loop) (runs in background thread)
        /*! Processes the action buffer and triggers processing of buffers for each tracked file */
        void execute()
        {
            // Setup helpers
            std::string currentFirstLevelDirectory = firstLevelDirectory();
            std::string secondLevelDirectory = "";
            std::string targetDirectory; // Concantenation of first and second level directory (complete path)

            // Process actions
            // --------------
            LoggerAction currentAction;
            // Process all events in the order of their occurence
            while (m_actionBuffer.pop(currentAction) == 1 && stopTriggered() == false) {
                // Create output and target directory, if necessary
                if (currentAction.m_type == LoggerAction::Type::LOG_DUMP || currentAction.m_type == LoggerAction::Type::LOG_START) {
                    // Create first level directory, if it does not exist already
                    if (currentFirstLevelDirectory.size() > 0) {
                        if (filesystem::directoryExists(currentFirstLevelDirectory) == false) {
                            (void)filesystem::createDirectory(currentFirstLevelDirectory); // Try to create directory (may fail, if another process creates it at the same time)

                            // Check, if the directory exists now
                            if (filesystem::directoryExists(currentFirstLevelDirectory) == false) {
                                // Cannot create directory -> skipping
                                m_signalBuffer.push(LoggerSignal("logger", LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_OPEN_FAILED_FOLDER_CREATION_FAILED)));
                                return;
                            }
                        }
                    }

                    // Get second-level directory from action
                    secondLevelDirectory = currentAction.m_text;

                    // Compute and create final directory
                    if (secondLevelDirectory.size() > 0) {
                        // Case A: target directory specified -> concatenate both directories
                        std::stringstream targetDirectoryStream;
                        if (currentFirstLevelDirectory.size() > 0)
                            targetDirectoryStream << currentFirstLevelDirectory << "/";
                        targetDirectoryStream << secondLevelDirectory;
                        targetDirectory = targetDirectoryStream.str();

                        // Create second level directory, if it does not exist already
                        if (filesystem::directoryExists(targetDirectory) == false) {
                            (void)filesystem::createDirectory(targetDirectory); // Try to create directory (may fail, if another process creates it at the same time)

                            // Check, if the second-level directory exists now
                            if (filesystem::directoryExists(targetDirectory) == false) {
                                // Cannot create final directory -> skipping
                                m_signalBuffer.push(LoggerSignal("logger", LogFileSignal(LogFileSignal::Type::ERROR, LogFileSignal::Message::FILE_OPEN_FAILED_FOLDER_CREATION_FAILED)));
                                return;
                            }
                        }
                    } else {
                        // Case B: no second-level directory specified -> final directory is first-level directory
                        targetDirectory = currentFirstLevelDirectory;

                        // No need to create target directory (does already exist)
                    }
                }

                // Process event
                if (currentAction.m_type == LoggerAction::Type::LOG_DUMP) {
                    // Case: DUMP log
                    // --------------
                    // Emergency case: open files (if they are not open already) and write the current buffer into it
                    openFiles(targetDirectory, currentAction.m_time); // Try to open files (fails, if they are already open)
                    processBuffers(); // Write complete data buffer to files
                    closeFiles(currentAction.m_time); // Close files to ensure that files are readable after program crash
                } else if (currentAction.m_type == LoggerAction::Type::LOG_START) {
                    // Case: START log
                    // ---------------
                    clearBuffers(); // Discard all old data up to now (fresh start)
                    openFiles(targetDirectory, currentAction.m_time); // Try to open files (fails, if they are already open)
                } else if (currentAction.m_type == LoggerAction::Type::LOG_STOP) {
                    // Case: STOP log
                    // --------------
                    closeFiles(currentAction.m_time); // Close all files properly
                }
            }

            // Writing data
            // ------------
            // Trigger processing the buffers of all tracked logfiles
            processBuffers();

            // Gather signals from logfiles
            // ----------------------------
            // Trigger gathering signals from all tracked logfiles
            collectSignals();
        }

        //! Open all tracked files (thread-safe) (runs in background thread)
        /*!
         * \param [in] targetDirectory Final directory for log files to write into (*first-level-directory* + *second-level-directory*)
         * \param [in] logStartTime The time when the log was started.
         */
        void openFiles(const std::string& targetDirectory, const core::Time& logStartTime)
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Open all files
            for (size_t i = 0; i < currentTrackedFiles.size(); i++) {
                // Update target directory for this file
                currentTrackedFiles[i]->setDesiredTargetDirectory(targetDirectory);

                // Open file
                currentTrackedFiles[i]->openLogFile();

                // Write header
                currentTrackedFiles[i]->writeLogHeader(logStartTime);
            }
        }

        //! Close all tracked files (thread-safe) (runs in parent **or** background thread)
        /*!
         * \param [in] logStopTime The time when the log was stopped.
         */
        void closeFiles(const core::Time& logStopTime)
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Close all files
            for (size_t i = 0; i < currentTrackedFiles.size(); i++) {
                // Check, if file is open
                if (currentTrackedFiles[i]->isOpen() == true) {
                    // Write footer
                    currentTrackedFiles[i]->writeLogFooter(logStopTime);

                    // Close file
                    currentTrackedFiles[i]->closeLogFile();
                }
            }
        }

        //! Clear data buffers of all tracked files (thread-safe) (runs in background thread)
        void clearBuffers()
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Clear buffers of all files
            for (size_t i = 0; i < currentTrackedFiles.size(); i++)
                currentTrackedFiles[i]->clearLogBuffer();
        }

        //! Process data buffers of all tracked files (thread-safe) (runs in background thread)
        void processBuffers()
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Process buffers of all files
            for (size_t i = 0; i < currentTrackedFiles.size(); i++)
                currentTrackedFiles[i]->processLogBuffer();
        }

        //! Collect signals of tracked logfiles and store them in own signal buffer (thread-safe) (runs in background thread)
        void collectSignals()
        {
            // Get current list of tracked files
            std::vector<LogFileBase*> currentTrackedFiles = trackedFiles();

            // Create helper signal (on stack -> FAST)
            LogFileSignal currentLogFileSignal = LogFileSignal(static_cast<LogFileSignal::Type>(0), static_cast<LogFileSignal::Message>(0));

            // Pass through all tracked files
            for (size_t i = 0; i < currentTrackedFiles.size(); i++) {
                // Get all signals of the buffer
                // Process all events in the order of their occurence
                while (currentTrackedFiles[i]->m_signalBuffer.pop(currentLogFileSignal) == 1 && stopTriggered() == false)
                    m_signalBuffer.push(LoggerSignal(currentTrackedFiles[i]->identifier(), currentLogFileSignal));
            }
        }

        // Start protected area of m_mutex
        std::vector<LogFileBase*> m_trackedFiles; //!< \copybrief trackedFiles()
        std::string m_firstLevelDirectory; //!< \copybrief firstLevelDirectory()
        // End protected area of m_mutex

    public:
        memory::CircularBuffer<LoggerSignal> m_signalBuffer; //!< Buffer for signals of the logger to a supervising observer (thread-safe by its own)
        memory::CircularBuffer<LoggerAction> m_actionBuffer; //!< Buffer for actions to perform by the logger (thread-safe by its own)

        // Getters (thread-safe)
        // ---------------------
    public:
        //! List of pointers to tracked log files \details **Thread-safe getter**
        inline std::vector<LogFileBase*> trackedFiles() const { return BackgroundWorker::getProtectedData(m_trackedFiles); }
        //! `FirstLevelDirectory` to store the logfiles into (absolute path or relative to executable path) \details **Thread-safe getter**
        inline std::string firstLevelDirectory() const { return BackgroundWorker::getProtectedData(m_firstLevelDirectory); }

        // Setters (thread-safe)
        // ---------------------
    private:
        // "Internal"
        //! **Thread-safe setter for:** \copybrief trackedFiles() \warning The list or the contained pointers **must not change** until \ref deInitialize() has been triggered!
        inline void setTrackedFiles(const std::vector<LogFileBase*>& newValue) { BackgroundWorker::setProtectedData(m_trackedFiles, newValue); }

    public:
        // "External"
        //! **Thread-safe setter for:** \copybrief firstLevelDirectory()
        inline void setFirstLevelDirectory(const std::string& newValue) { BackgroundWorker::setProtectedData(m_firstLevelDirectory, newValue); }
    };
} // namespace io
} // namespace broccoli
