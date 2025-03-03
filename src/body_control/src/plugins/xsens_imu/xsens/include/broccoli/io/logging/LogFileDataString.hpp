/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../encoding.hpp"
#include "LogFileData.hpp"

namespace broccoli {
namespace io {
    /*!
     * \brief Implements a simple string-based LogFileData container
     * \ingroup broccoli_io_logging
     *
     * Implementation of a \ref LogFileData container for simple data to log (which can be represented as string).
     * This implementation does not write any header or footer by default.
     */
    class LogFileDataString : public LogFileData {
    public:
        //! Default constructor
        /*!
         * \param [in] data Sets \ref m_data - \copybrief m_data
         */
        LogFileDataString(const std::string& data = "")
            : m_data(data)
        {
        }

        //! Destructor
        virtual ~LogFileDataString()
        {
        }

        // Members
        std::string m_data; //!< Contains the string to write

        void encodeDataToLogStream(LogStream& stream) const override
        {
            // Simple string encoding
            encoding::encode(stream, m_data);
        }
    };
} // namespace io
} // namespace broccoli
