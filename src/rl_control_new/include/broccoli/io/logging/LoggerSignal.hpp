/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "LogFileSignal.hpp"
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \brief Data container for signals of a logger to a supervising observer
     * \ingroup broccoli_io_logging
     */
    class LoggerSignal {
    public:
        //! Specialized constructor
        /*!
         * \param [in] logFileIdentifier Initializes \ref m_logFileIdentifier - \copybrief m_logFileIdentifier
         * \param [in] logFileSignal Initializes \ref m_logFileSignal - \copybrief m_logFileSignal
         */
        LoggerSignal(const std::string& logFileIdentifier, const LogFileSignal& logFileSignal)
            : m_logFileIdentifier(logFileIdentifier)
            , m_logFileSignal(logFileSignal)
        {
        }

        //! Default constructor
        LoggerSignal()
            : LoggerSignal("", LogFileSignal())
        {
        }

        // Members
        std::string m_logFileIdentifier; //!< Identifier of the related logfile (information, warning, error)
        LogFileSignal m_logFileSignal; //!< Signal of the corresponding logfile
    };
} // namespace io
} // namespace broccoli
