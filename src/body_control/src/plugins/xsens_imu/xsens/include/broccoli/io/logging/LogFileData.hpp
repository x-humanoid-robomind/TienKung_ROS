/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include "../encoding.hpp"
#include <string>
#include <unordered_map>
#ifdef HAVE_EIGEN3
#include <Eigen/StdVector>
#endif // HAVE_EIGEN3

namespace broccoli {
namespace io {
    /*!
     * \brief Abstract interface for log data containers
     * \ingroup broccoli_io_logging
     *
     * Pure abstract class defining an interface for log-data containers. Encodes its own data
     * members for writing to a log file (text based, **not** binary). This is done by \ref encodeDataToLogStream()
     * which has to be implemented by the derived class.
     *
     * Furthermore one can overwrite \ref encodeHeaderToLogStream() and \ref encodeFooterToLogStream() to
     * specify how the file header and footer should look like.
     *
     * The methods
     * - \ref encodeHeaderToLogStream()
     * - \ref encodeDataToLogStream()
     * - \ref encodeFooterToLogStream()
     *
     * are handled by the corresponding \ref LogFile which in turn is supervised by a \ref Logger. This way the
     * \ref Logger can execute the encoding efficiently in a background thread.
     *
     * \attention
     * It is important to provide a default constructor which does **not** require any parameters. This is necessary
     * since the corresponding \ref Logger will create "dummy" \p LogFileData objects just to be able to encode
     * the file header and footer. Because the \ref Logger does not know the type of the \ref LogFileData at compile
     * time a "trivial" default constructor has to be provided.
     */
    class LogFileData {
    public:
        // Type definitions
        typedef encoding::CharacterStream LogStream; //!< Specifies the used type for storing log stream data
        typedef encoding::CharacterStreamSize LogStreamSize; //!< Specifies the used type for describing the size of a log stream

        //! Default constructor
        /*!
         * \attention Derived classes must also provide a default constructor which does not require any parameters!
         * (see detailed class description of \ref LogFileData for details)
         */
        LogFileData()
            : m_timeAddedToBuffer(-1)
        {
        }

        //! Destructor
        virtual ~LogFileData()
        {
        }

        // Members
        mutable core::Time m_timeAddedToBuffer; //!< Indicates when (system time) the data has been added to the buffer of the corresponding \ref LogFile. For \f$ <0 \f$: data has not been added to the buffer yet

        //! Encode file header to log stream
        /*!
         * The header is written **before** member data is written to the stream. The header is written only **once**.
         * \param [in,out] stream The logging stream to which the header should be appended to.
         * \param [in] fileAttributes Attributes, which should be added to the header (key/value strings)
         */
        virtual void encodeHeaderToLogStream(LogStream& stream, const std::unordered_map<std::string, std::string>& fileAttributes) const
        {
            (void)stream;
            (void)fileAttributes;
        }

        //! Encode member data to log stream
        /*!
         * Encodes member data and appends it to the specified stream.
         * \param [in,out] stream The logging stream to which the member data should be appended to.
         * \return Count of appended characters (elements added to stream).
         */
        virtual void encodeDataToLogStream(LogStream& stream) const = 0;

        //! Encode file footer to log stream
        /*!
         * The footer is written **after** member data is written to the stream. The footer is written only **once**.
         * \param [in,out] stream The logging stream to which the footer should be appended to.
         * \param [in] fileAttributes Attributes, which should be added to the footer (key/value strings)
         * \return Count of appended characters (elements added to stream).
         */
        virtual void encodeFooterToLogStream(LogStream& stream, const std::unordered_map<std::string, std::string>& fileAttributes) const
        {
            (void)stream;
            (void)fileAttributes;
        }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };
} // namespace io
} // namespace broccoli
