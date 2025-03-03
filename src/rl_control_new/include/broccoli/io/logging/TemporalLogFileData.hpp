/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../../core/Time.hpp"
#include "ColumnBasedLogFileData.hpp"

namespace broccoli {
namespace io {
    /*!
     * \brief Specialization of broccoli::io::ColumnBasedLogFileData for temporal log-file data
     * \ingroup broccoli_io_logging
     *
     * This class extends ColumnBasedLogFileData by an additional time field, which is automatically
     * added as first column.
     */
    class TemporalLogFileData : public ColumnBasedLogFileData {
    public:
        /*!
         * \brief Set the current time used for logging
         *
         * \param time Logged time in seconds.
         */
        void setLogTime(const double& time)
        {
            m_time = time;
        }

        /*!
         * \brief Set the current time used for logging
         *
         * \param time Logged time
         */
        void setLogTime(const core::Time& time)
        {
            m_time = time;
        }

        //! Returns the time used for logging
        const core::Time& logTime() const
        {
            return m_time;
        }

    protected:
        /*!
         * \brief Implement this method to encode the temporal log data to the LogStream.
         *
         * Use the public methods of ColumnBasedLogStream to add objects for logging.
         * See ColumnBasedLogFileData for more details.
         * The time information is automatically added as first column.
         * \param stream The target stream
         */
        virtual void encodeTimeDataToLogStream(ColumnBasedLogStream& stream) const = 0;

        void encodeColumnDataToLogStream(ColumnBasedLogStream& stream) const override
        {
            stream.addData(m_time.encodeToDurationString(), "time");
            encodeTimeDataToLogStream(stream);
        }

        //! The time data used for the log file
        core::Time m_time;

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };

} // namespace io
} // namespace broccoli
