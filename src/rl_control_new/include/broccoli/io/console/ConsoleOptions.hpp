/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "ConsoleANSICodes.hpp"
#include "ConsoleInterfaceOptions.hpp"
#include <string>
#include <vector>

namespace broccoli {
namespace io {
    //! Container for options of the console
    /*! \ingroup broccoli_io_console */
    class ConsoleOptions {
    public:
        //! Specification of default interface types (adapted from Python's logging levels)
        enum class DefaultInterfaceType : size_t {
            PLAIN = 0, //!< Unformatted console-only output which may be used for progress bars etc - typically not logged to file.
            DEBUG, //!< Detailed information, typically of interest only when diagnosing problems.
            INFO, //!< Confirmation that things are working as expected.
            WARNING, //!< An indication that something unexpected happened, or indicative of some problem in the near future (e.g. ‘disk space low’). The software is still working as expected.
            ERROR, //!< Due to a more serious problem, the software has not been able to perform some function.
            CRITICAL, //!< A serious error, indicating that the program itself may be unable to continue running.
            DEFAULTINTERFACETYPE_COUNT //!< Total count of elements
        };

        //! Count of default interfaces
        static constexpr size_t defaultInterfaceCount() { return static_cast<size_t>(DefaultInterfaceType::DEFAULTINTERFACETYPE_COUNT); }

        //! Default constructor
        ConsoleOptions()
        {
            setupDefaultInterfaces();
        }

        //! Specialized constructor
        /*!
         * \param [in] threadName Initializes \ref m_threadName - \copybrief m_threadName
         * \param [in] threadPriority Initializes \ref m_threadPriority - \copybrief m_threadPriority
         * \param [in] bufferSize Initializes \ref m_bufferSize - \copybrief m_bufferSize
         * \param [in] writeLogFile Initializes \ref m_writeLogFile - \copybrief m_writeLogFile
         * \param [in] logFileName Initializes \ref m_logFileFileName - \copybrief m_logFileFileName
         */
        ConsoleOptions(const std::string& threadName, const int& threadPriority, const size_t& bufferSize, const bool& writeLogFile, const std::string& logFileName)
            : m_threadName(threadName)
            , m_threadPriority(threadPriority)
            , m_bufferSize(bufferSize)
            , m_writeLogFile(writeLogFile)
            , m_logFileFileName(logFileName)
        {
            setupDefaultInterfaces();
        }

        // Members
        // -------
    public:
        // Background thread
        std::string m_threadName = "Console"; //!< Initializes: \copybrief broccoli::parallel::BackgroundWorker::name()
        int m_threadPriority = 0; //!< Initializes: \copybrief broccoli::parallel::BackgroundWorker::threadPriority()

        // Message buffer
        size_t m_bufferSize = 10000; //!< Specifies the size of the message buffer

        // Log file
        bool m_writeLogFile = false; //!< If `true`, the console output is also written to a logfile
        std::string m_logFileIdentifier = "console"; //!< Initializes: \copybrief broccoli::io::LogFileBase::identifier()
        std::string m_logFileTargetDirectory = ""; //!< Initializes: \copybrief broccoli::io::LogFileBase::targetDirectory()
        std::string m_logFileFileName = "console"; //!< Initializes: \copybrief broccoli::io::LogFileBase::fileName()
        std::string m_logFileFileExtension = ".log"; //!< Initializes: \copybrief broccoli::io::LogFileBase::fileExtension()
        bool m_logFileAddTimeStampToName = true; //!< Initializes: \copybrief broccoli::io::LogFileBase::addTimeStampToName()
        bool m_logFileCompressed = true; //!< Initializes: \copybrief broccoli::io::LogFileBase::compressed()

    private:
        // Interface options
        std::vector<ConsoleInterfaceOptions> m_interfaceOptions; //!< \copybrief interfaceOptions()

        // Setters and Getters
        // -------------------
    public:
        //! Returns total count of interfaces
        size_t interfaceCount() const { return m_interfaceOptions.size(); }

        //! Returns reference to the options of a specific interface (by index)
        const ConsoleInterfaceOptions& interfaceOptions(const size_t& index) const { return m_interfaceOptions[index]; }

        //! \copydoc interfaceOptions(const size_t&) const
        ConsoleInterfaceOptions& interfaceOptions(const size_t& index) { return m_interfaceOptions[index]; }

        //! Returns reference to the options of a specific interface (by default type)
        const ConsoleInterfaceOptions& interfaceOptions(const DefaultInterfaceType& type) const { return m_interfaceOptions[static_cast<size_t>(type)]; }

        //! \copydoc interfaceOptions(const DefaultInterfaceType&) const
        ConsoleInterfaceOptions& interfaceOptions(const DefaultInterfaceType& type) { return m_interfaceOptions[static_cast<size_t>(type)]; }

        //! Returns reference to the options of the "plain" interface
        const ConsoleInterfaceOptions& plainOptions() const { return interfaceOptions(DefaultInterfaceType::PLAIN); }

        //! \copydoc plainOptions() const
        ConsoleInterfaceOptions& plainOptions() { return interfaceOptions(DefaultInterfaceType::PLAIN); }

        //! Returns reference to the options of the "debug" interface
        const ConsoleInterfaceOptions& debugOptions() const { return interfaceOptions(DefaultInterfaceType::DEBUG); }

        //! \copydoc debugOptions() const
        ConsoleInterfaceOptions& debugOptions() { return interfaceOptions(DefaultInterfaceType::DEBUG); }

        //! Returns reference to the options of the "info" interface
        const ConsoleInterfaceOptions& infoOptions() const { return interfaceOptions(DefaultInterfaceType::INFO); }

        //! \copydoc infoOptions() const
        ConsoleInterfaceOptions& infoOptions() { return interfaceOptions(DefaultInterfaceType::INFO); }

        //! Returns reference to the options of the "warning" interface
        const ConsoleInterfaceOptions& warningOptions() const { return interfaceOptions(DefaultInterfaceType::WARNING); }

        //! \copydoc warningOptions() const
        ConsoleInterfaceOptions& warningOptions() { return interfaceOptions(DefaultInterfaceType::WARNING); }

        //! Returns reference to the options of the "error" interface
        const ConsoleInterfaceOptions& errorOptions() const { return interfaceOptions(DefaultInterfaceType::ERROR); }

        //! \copydoc errorOptions() const
        ConsoleInterfaceOptions& errorOptions() { return interfaceOptions(DefaultInterfaceType::ERROR); }

        //! Returns reference to the options of the "critical" interface
        const ConsoleInterfaceOptions& criticalOptions() const { return interfaceOptions(DefaultInterfaceType::CRITICAL); }

        //! \copydoc criticalOptions() const
        ConsoleInterfaceOptions& criticalOptions() { return interfaceOptions(DefaultInterfaceType::CRITICAL); }

        //! Adds the given interface to the list of interfaces
        /*!
         * \param [in] options The options of the interface to add to the list
         * \return The index of the new interface in the list of interfaces
         */
        size_t addInterface(const ConsoleInterfaceOptions& options)
        {
            const size_t index = m_interfaceOptions.size();
            m_interfaceOptions.push_back(options);
            return index;
        }

        //! Sets the logging level for console output
        /*! \param [in] lowestInterfaceIndex The lowest interface index used for output (interfaces with lower index are ignored) */
        void setLoggingLevelConsole(const size_t& lowestInterfaceIndex)
        {
            for (size_t i = 0; i < m_interfaceOptions.size(); i++)
                m_interfaceOptions[i].m_writeToConsole = (i >= lowestInterfaceIndex);
        }

        //! Sets the logging level for console output
        /*! \param [in] lowestInterfaceType The "lowest" interface type used for output (lower level interfaces are ignored) */
        void setLoggingLevelConsole(const DefaultInterfaceType& lowestInterfaceType) { setLoggingLevelConsole(static_cast<size_t>(lowestInterfaceType)); }

        //! Sets the logging level for log file output
        /*! \param [in] lowestInterfaceIndex The lowest interface index used for output (interfaces with lower index are ignored) */
        void setLoggingLevelLogFile(const size_t& lowestInterfaceIndex)
        {
            for (size_t i = 0; i < m_interfaceOptions.size(); i++)
                m_interfaceOptions[i].m_writeToLogFile = (i >= lowestInterfaceIndex);
        }

        //! Sets the logging level for log file output
        /*! \param [in] lowestInterfaceType The "lowest" interface type used for output (lower level interfaces are ignored) */
        void setLoggingLevelLogFile(const DefaultInterfaceType& lowestInterfaceType) { setLoggingLevelLogFile(static_cast<size_t>(lowestInterfaceType)); }

        // Helpers
        // -------
    private:
        //! Creates options for default interfaces which are always available
        /*! \warning Discards any custom interfaces which may have been added already */
        void setupDefaultInterfaces()
        {
            m_interfaceOptions.clear();
            m_interfaceOptions.reserve(defaultInterfaceCount());

            // Plain
            ConsoleInterfaceOptions plainInterface;
            addInterface(plainInterface);

            // Debug
            ConsoleInterfaceOptions debugInterface;
            debugInterface.m_prefixConsole = ConsoleANSICodes::ForeGround::blue();
            debugInterface.m_suffixConsole = ConsoleANSICodes::reset();
            debugInterface.m_prefixLogFile = "Debug: ";
            addInterface(debugInterface);

            // Info
            ConsoleInterfaceOptions infoInterface;
            infoInterface.m_prefixConsole = ConsoleANSICodes::ForeGround::green();
            infoInterface.m_suffixConsole = ConsoleANSICodes::reset();
            infoInterface.m_prefixLogFile = "Info: ";
            addInterface(infoInterface);

            // Warning
            ConsoleInterfaceOptions warningInterface;
            warningInterface.m_prefixConsole = ConsoleANSICodes::ForeGround::brightYellow();
            warningInterface.m_suffixConsole = ConsoleANSICodes::reset();
            warningInterface.m_prefixLogFile = "Warning: ";
            addInterface(warningInterface);

            // Error
            ConsoleInterfaceOptions errorInterface;
            errorInterface.m_prefixConsole = ConsoleANSICodes::ForeGround::brightRed();
            errorInterface.m_suffixConsole = ConsoleANSICodes::reset();
            errorInterface.m_prefixLogFile = "Error: ";
            addInterface(errorInterface);

            // Critical
            ConsoleInterfaceOptions criticalnterface;
            criticalnterface.m_prefixConsole = ConsoleANSICodes::BackGround::red() + ConsoleANSICodes::ForeGround::brightWhite();
            criticalnterface.m_suffixConsole = ConsoleANSICodes::reset();
            criticalnterface.m_prefixLogFile = "Critical: ";
            addInterface(criticalnterface);

            // Set logging levels
            setLoggingLevelConsole(DefaultInterfaceType::PLAIN);
            setLoggingLevelLogFile(DefaultInterfaceType::INFO);
        }
    };
} // namespace io
} // namespace broccoli
