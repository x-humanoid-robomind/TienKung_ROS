/* 
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "LogFileData.hpp"
#include <array>
#include <string>
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif

namespace broccoli {
namespace io {

    /*!
     * \brief A LogStream for column-based logging
     * \ingroup broccoli_io_logging
     * Aggregates the column header names and the actual LogStream reference
     */
    class ColumnBasedLogStream {
    public:
        /*!
         * \brief A constructor
         * \param stream The raw target stream
         * \param generateColumnNames true when this stream is used solely for header name generation.
         */
        ColumnBasedLogStream(LogFileData::LogStream& stream, const bool& generateColumnNames = false)
            : m_logStream(stream)
            , m_generateColumnNames(generateColumnNames)
        {
        }

        /*!
         * \brief Add log data to the LogStream
         *
         * If the object is an array or Eigen vector, the object's internals
         * are automatically expanded to separate columns.
         *
         * \param data Data object to log.
         * \param name The column name for the object (parentheses are added automatically for complex objects).
         * An empty name omits generation of a header column - data will be logged anyway
         */
        template <typename T, typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
        void addData(const T& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            encodeDataItem((int64_t)data);
        }

        //! \copydoc ColumnBasedLogStream::addData()
        void addData(const uint64_t& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            encodeDataItem(data);
        }

        //! \copydoc ColumnBasedLogStream::addData()
        template <typename T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
        void addData(const T& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            encodeDataItem((double)data);
        }

        //! \copydoc ColumnBasedLogStream::addData()
        void addData(const std::string& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            encodeDataItem(data);
        }

        //! \copydoc ColumnBasedLogStream::addData()
        template <typename T, std::size_t size>
        void addData(const std::array<T, size>& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            for (const auto& item : data) {
                addData(item, name);
            }
        }

#ifdef HAVE_EIGEN3
        //! \copydoc ColumnBasedLogStream::addData()
        template <typename Derived>
        void addData(const Eigen::MatrixBase<Derived>& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            for (int i = 0; i < data.rows(); i++) {
                for (int j = 0; j < data.cols(); j++) {
                    addData(data(i, j), name);
                }
            }
        }

        /*!
         * \copydoc ColumnBasedLogStream::addData()
         *
         * \remark The coefficients of the quaternion are written in the following order: w, x, y, z
         */
        void addData(const Eigen::Quaterniond& data, const std::string& name)
        {
            if (m_generateColumnNames) {
                addColumns(name, data);
                return;
            }

            // Output coefficients
            addData(data.w(), name);
            addData(data.x(), name);
            addData(data.y(), name);
            addData(data.z(), name);
        }
#endif // HAVE_EIGEN3

        //! Add an end of line character to the log-file
        void addEndOfLine()
        {
            broccoli::io::encoding::encode(m_logStream, '\n');
        }

        /*!
         * \brief Sets a prefix for all column header names generated
         * \param prefix A string to use as prefix for all column header names
         */
        void setColumnNamePrefix(const std::string& prefix)
        {
            m_prefix = prefix;
        }

        /*!
         * \brief Appends a string to the column header name prefix (see \ref setColumnNamePrefix())
         * \param prefix A string to append to the current prefix
         */
        void appendColumnNamePrefix(const std::string& prefix)
        {
            m_prefix.append(prefix);
        }

        //! Reference to the raw LogStream
        std::reference_wrapper<LogFileData::LogStream> m_logStream;

        //! Storage for the generated column names
        std::vector<std::string> m_columnNames;

    protected:
        /*!
         * \brief Encodes a data item to the raw LogStream
         * \tparam T The data item type
         * \param data The data object
         */
        template <typename T>
        void encodeDataItem(const T& data)
        {
            if (!m_logStream.get().empty() && m_logStream.get().back() != '\n') {
                broccoli::io::encoding::encode(m_logStream, ' ');
            }

            broccoli::io::encoding::encode(m_logStream, data);
        }

        /*!
         * \brief Add header columns with given name for the given object
         *
         * If the object is a complex type, multiple header columns are
         * automatically added.
         * \param name Name of the variable located in this column
         * \param object Object these column(s) are intended for.
         */
        template <typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
        void addColumns(const std::string& name, const T& object)
        {
            if (name.empty()) {
                return;
            }

            m_columnNames.push_back(m_prefix + name);
            (void)object;
        }

        //! \copydoc ColumnBasedLogStream::addColumns()
        void addColumns(const std::string& name, const std::string& object)
        {
            if (name.empty()) {
                return;
            }

            m_columnNames.push_back(m_prefix + name);
            (void)object;
        }

#ifdef HAVE_EIGEN3
        //! \copydoc ColumnBasedLogStream::addColumns()
        template <typename Derived>
        void addColumns(const std::string& name, const Eigen::MatrixBase<Derived>& object)
        {
            if (name.empty()) {
                return;
            }

            if (object.rows() == 1 || object.cols() == 1) {
                addVectorColumns(name, object.size());
            } else {
                addMatrixColumns(name, object.rows(), object.cols());
            }
        }

        /*!
         * \copydoc ColumnBasedLogStream::addColumns()
         *
         * \remark The coefficients of the quaternion are written in the following order: w, x, y, z
         */
        void addColumns(const std::string& name, const Eigen::Quaterniond& object)
        {
            if (name.empty()) {
                return;
            }

            m_columnNames.push_back(m_prefix + name + ".w()");
            m_columnNames.push_back(m_prefix + name + ".x()");
            m_columnNames.push_back(m_prefix + name + ".y()");
            m_columnNames.push_back(m_prefix + name + ".z()");
            (void)object;
        }
#endif // HAVE_EIGEN3

        /*!
         * \brief Add header columns with given name for a matrix object
         *
         * Calling this method results in the following header column entries: (for all rows, columns) "name(row,column)"
         * \param name Name of the variable located in this column
         * \param rows Number of matrix rows
         * \param cols Number of matrix columns
         */
        void addMatrixColumns(const std::string& name, const std::size_t& rows, const std::size_t& cols)
        {
            std::stringstream stream;
            for (size_t i = 0; i < rows; i++) {
                for (size_t j = 0; j < cols; j++) {
                    stream.str("");
                    stream << name << "(" << i << "," << j << ")";
                    m_columnNames.push_back(m_prefix + stream.str());
                }
            }
        }

        /*!
         * \brief Add header columns with given name for a vector object
         *
         * Calling this method results in the following header column entries: (for i in \p size) "name(i)"
         * \param name Name of the variable located in this column
         * \param size The size of the vector
         */
        void addVectorColumns(const std::string& name, const std::size_t& size)
        {
            std::stringstream stream;
            for (size_t i = 0; i < size; i++) {
                stream.str("");
                stream << name << "(" << i << ")";
                m_columnNames.push_back(m_prefix + stream.str());
            }
        }

        //! \copydoc ColumnBasedLogStream::addColumns()
        template <typename T, std::size_t size>
        void addColumns(const std::string& name, const std::array<T, size>& object)
        {
            if (name.empty()) {
                return;
            }

            std::stringstream newName;
            for (size_t i = 0; i < size; i++) {
                newName.str("");
                newName << name << "[" << i << "]";
                addColumns(newName.str(), object[i]);
            }
        }

    private:
        //! \copybrief generateColumnNames()
        const bool m_generateColumnNames;

        //! Prefix string for all column names
        std::string m_prefix = "";

    public:
        // Getters
        // -------
        //! Indicates if the column names should be generated in this run rather than logging data
        bool generateColumnNames() const { return m_generateColumnNames; }
    };

} // namespace io
} // namespace broccoli
