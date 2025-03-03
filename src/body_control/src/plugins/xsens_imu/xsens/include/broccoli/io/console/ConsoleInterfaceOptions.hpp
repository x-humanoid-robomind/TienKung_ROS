/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <string>

namespace broccoli {
namespace io {
    //! Options of console interfaces
    class ConsoleInterfaceOptions {
    public:
        //! Default constructor
        ConsoleInterfaceOptions() = default;

        //! Specialized constructor
        /*!
         * \param [in] writeToConsole Initializes \ref m_writeToConsole - \copybrief m_writeToConsole
         * \param [in] prefixConsole Initializes \ref m_prefixConsole - \copybrief m_prefixConsole
         * \param [in] suffixConsole Initializes \ref m_suffixConsole - \copybrief m_suffixConsole
         * \param [in] writeToLogFile Initializes \ref m_writeToLogFile - \copybrief m_writeToLogFile
         * \param [in] prefixLogFile Initializes \ref m_prefixLogFile - \copybrief m_prefixLogFile
         * \param [in] suffixLogFile Initializes \ref m_suffixLogFile - \copybrief m_suffixLogFile
         */
        ConsoleInterfaceOptions(const bool& writeToConsole, const std::string& prefixConsole, const std::string& suffixConsole, const bool& writeToLogFile, const std::string& prefixLogFile, const std::string& suffixLogFile)
            : m_writeToConsole(writeToConsole)
            , m_prefixConsole(prefixConsole)
            , m_suffixConsole(suffixConsole)
            , m_writeToLogFile(writeToLogFile)
            , m_prefixLogFile(prefixLogFile)
            , m_suffixLogFile(suffixLogFile)
        {
        }

        // Console output
        bool m_writeToConsole = true; //!< Flag to enable output of this interface to the console
        std::string m_prefixConsole = ""; //!< Prefix used for writing to console (per-line basis)
        std::string m_suffixConsole = ""; //!< Suffix used for writing to console (per-line basis)
        // File output
        bool m_writeToLogFile = true; //!< Flag to enable output of this interface to the log file
        std::string m_prefixLogFile = ""; //!< Prefix used for writing to log file (per-line basis)
        std::string m_suffixLogFile = ""; //!< Suffix used for writing to log file (per-line basis)
    };
} // namespace io
} // namespace broccoli
