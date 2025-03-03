/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "ColumnBasedLogStream.hpp"
#include "LogFileData.hpp"

namespace broccoli {
namespace io {
    /*!
     * \brief Specialization of broccoli::io::LogFileData for column-based log files
     * \ingroup broccoli_io_logging
     * This LogFileData specialization encodes data to a column-based, whitespace separated, plain text file format.
     * The file is formatted in the following way. Variables are written in curly brackets.
     * \par
     * \code
     * # {fileAttribute-key0}: {fileAttribute-value0}
     * # {fileAttribute-key1}: {fileAttribute-value1}
     * # ...
     * # {columnHeaderNames()[0]}:0 {columnHeaderNames()[1]}:1 {columnHeaderNames()[2]}:2 ...
     * {value0} {value1} {value2}
     * ...
     * \endcode
     *
     * Example code for a class derived from ColumnBasedLogFileData:
     * \par
     * \code
     * class LogMe : public broccoli::io::ColumnBasedLogFileData {
     * protected:
     *
     *    void encodeColumnDataToLogStream(ColumnBasedLogStream& stream) const override
     *    {
     *        // Defines the data to log and the corresponding header names
     *
     *        // There are data addition methods for arithmetic types...
     *        stream.addData(3.0, "aFloatValue"); // Results in column header "aFloatValue"
     *
     *        // ... for matrices ...
     *        Eigen::Matrix2d matrix;
     *        stream.addData(matrix, "myMatrix"); // Results in column header "matrix(0,0) matrix(1,0) matrix(0,1) matrix(1,1)"
     *
     *        // ... for vectors ...
     *        Eigen::Vector3d vector;
     *        stream.addData(vector, "v"); // Results in column header "v(0) v(1) v(2)"
     *
     *        // ... for arrays ...
     *        std::array<double, 2> array;
     *        stream.addData(array, "a"); // Results in column header "a[0] a[1]"
     *
     *        // ... and for arrays of vectors/matrices.
     *        std::array<Eigen::Vector2d, 2> vectorArray;
     *        stream.addData(vectorArray, "va"); // Results in column header "va[0](0) va[0](1) va[1](0) va[1](1)"
     *    }
     * };
     * \endcode
     */
    class ColumnBasedLogFileData : public LogFileData {
    public:
        void encodeHeaderToLogStream(LogStream& stream, const std::unordered_map<std::string, std::string>& fileAttributes) const override
        {
            LogFileData::encodeHeaderToLogStream(stream, fileAttributes);

            for (const auto& attribute : fileAttributes) {
                encoding::encode(stream, "# " + attribute.first + ": " + attribute.second + "\n");
            }

            encodeColumnHeaderToLogStream(stream);
        }

        void encodeDataToLogStream(LogStream& stream) const override
        {
            ColumnBasedLogStream columnStream(stream);
            encodeColumnDataToLogStream(columnStream);
            columnStream.addEndOfLine();
        }

    protected:
        /*!
         * \brief Implement this method to encode the column-based log data to the LogStream.
         *
         * Use the public methods of ColumnBasedLogStream to add objects for logging.
         * \param stream The target stream
         */
        virtual void encodeColumnDataToLogStream(ColumnBasedLogStream& stream) const = 0;

        /*!
         * \brief Encodes the header line with the column names to the given stream
         * \param [in, out] stream The target stream
         */
        virtual void encodeColumnHeaderToLogStream(LogStream& stream) const
        {
            ColumnBasedLogStream columnStream(stream, true);
            encodeColumnDataToLogStream(columnStream);
            if (columnStream.m_columnNames.size() == 0)
                return;

            encoding::encode(stream, "# ");
            for (uint64_t i = 0; i < columnStream.m_columnNames.size(); i++) {
                encoding::encode(stream, columnStream.m_columnNames[i]);
                encoding::encode(stream, ':');
                encoding::encode(stream, (i + 1));
                if (i < columnStream.m_columnNames.size() - 1)
                    encoding::encode(stream, ' ');
            }
            columnStream.addEndOfLine();
        }

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };
} // namespace io
} // namespace broccoli
