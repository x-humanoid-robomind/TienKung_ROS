/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../memory/SmartVector.hpp"
#include <array>
#include <assert.h>
#include <limits>
#include <sstream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif // HAVE_EIGEN3

namespace broccoli {
namespace io {
    namespace encoding {
        /*!
         * \addtogroup broccoli_io_encoding
         * \{
         */

        // Type definitions
        typedef std::vector<uint8_t> CharacterStream; //!< Specifies the used type for storing character stream data
        typedef uint64_t CharacterStreamSize; //!< Specifies the used type for describing the size of a character stream

        // =======================
        // --- BASIC DATATYPES ---
        // =======================
        // Encoding of basic datatypes
        // ---------------------------
        //! Appends specified character to a given stream
        /*!
         * \param [in,out] stream The stream to which the character should be appended to.
         * \param [in] character The character to append.
         * \return Count of appended characters (elements in stream).
         */
        static inline CharacterStreamSize encode(CharacterStream& stream, const char& character)
        {
            // Push single character to end of stream
            stream.push_back(character);

            // Pass back length of added data
            return 1;
        }

        //! Encodes specified numeric value and appends it to a given stream
        /*!
         * \note With the parameter \p format, a regular C-format specifier has to be provided. This has to start with a '%' character.
         * However, this method allows to further customize the output (as extension to the standard). For this, one of the following
         * characters may precede the starting '%' character:
         *   * '^' - for scientific notation this ensures that there are no leading zeroes in the exponent
         *
         * \param [in,out] stream The stream to which the value should be appended to.
         * \param [in] value The value to encode.
         * \param [in] format C-format specifier.
         * \return Count of appended characters (elements in stream) or 0 in case of an error.
         */
        template <typename NumericType, typename std::enable_if<std::is_arithmetic<NumericType>::value, int>::type = 0>
        static inline CharacterStreamSize encodeNumeric(CharacterStream& stream, const NumericType& value, const std::string& format)
        {
            // Initialize helpers
            const size_t oldSize = stream.size(); // Remember old size of stream
            static constexpr size_t maxLengthNewData = 24; // Define maximum length of new data (without null-termination)
            stream.resize(oldSize + maxLengthNewData + 1, 0); // Pre-allocate buffer (+1 since '\0' (null-termination) is always written to the end of the new data)
            int realLengthNewData = 0; // Stores length of "real" new data

            // Encode according to format specifier
            if (format.size() < 2) {
                // Invalid format specifier
                assert(false);
            } else if (format[0] == (char)'%') {
                // "Regular" encoding
                realLengthNewData = snprintf(reinterpret_cast<char*>(stream.data()) + oldSize, stream.size() - oldSize, format.c_str(), value);
                if (realLengthNewData < 0 || realLengthNewData > (int)maxLengthNewData) {
                    // An encoding error occured
                    realLengthNewData = 0;
                    assert(false);
                }
            } else if (format[0] == (char)'^' && format[1] == (char)'%' && format.size() > 2) {
                // Avoid leading zeros in the exponent of scientific notations
                realLengthNewData = snprintf(reinterpret_cast<char*>(stream.data()) + oldSize, stream.size() - oldSize, format.c_str() + 1, value);
                if (realLengthNewData < 0 || realLengthNewData > (int)maxLengthNewData) {
                    // An encoding error occured
                    realLengthNewData = 0;
                    assert(false);
                } else {
                    // Regular encoding done -> patch exponent
                    if (realLengthNewData > 2) {
                        for (size_t i = oldSize + realLengthNewData - 2; i > oldSize; i--) {
                            if (stream[i] == (char)'e' || stream[i] == (char)'E') {
                                // Find start index of exponent (without sign)
                                size_t exponentStartIndex = i + 1;
                                if (stream[exponentStartIndex] == (char)'+' || stream[exponentStartIndex] == (char)'-')
                                    exponentStartIndex++;

                                // Exponent found -> re-encode exponent
                                const std::string exponentString(stream.begin() + exponentStartIndex, stream.begin() + oldSize + realLengthNewData);
                                const auto exponentInteger = std::stoull(exponentString);
                                realLengthNewData = realLengthNewData - exponentString.size() + snprintf(reinterpret_cast<char*>(stream.data()) + exponentStartIndex, stream.size() - exponentStartIndex, "%llu", exponentInteger);

                                // Stop search for exponent
                                break;
                            }
                        }
                    }
                }
            } else {
                // Invalid format specifier
                assert(false);
            }

            // Shrink stream to its actual size (remove placeholders and null-termination)
            stream.resize(oldSize + realLengthNewData);

            // Pass back length of added data
            return realLengthNewData;
        }

        //! Encodes specified unsigned integer value and appends it to a given stream
        /*! \copydetails encodeNumeric() */
        template <typename UnsignedType, typename std::enable_if<std::is_integral<UnsignedType>::value && !std::is_signed<UnsignedType>::value, int>::type = 0>
        static inline CharacterStreamSize encode(CharacterStream& stream, const UnsignedType& value, const std::string& format = "%llu")
        {
            return encodeNumeric<uint64_t>(stream, (uint64_t)value, format);
        }

        //! Encodes specified signed integer value and appends it to a given stream
        /*! \copydetails encodeNumeric() */
        template <typename SignedType, typename std::enable_if<std::is_integral<SignedType>::value && std::is_signed<SignedType>::value, int>::type = 0>
        static inline CharacterStreamSize encode(CharacterStream& stream, const SignedType& value, const std::string& format = "%+lld")
        {
            return encodeNumeric<int64_t>(stream, (int64_t)value, format);
        }

        //! Encodes specified single-precision floating point value and appends it to a given stream
        /*! \copydetails encodeNumeric() */
        static inline CharacterStreamSize encode(CharacterStream& stream, const float& value, const std::string& format = "%+.7e")
        {
            return encodeNumeric<float>(stream, value, format);
        }

        //! Encodes specified double-precision floating point value and appends it to a given stream
        /*! \copydetails encodeNumeric() */
        static inline CharacterStreamSize encode(CharacterStream& stream, const double& value, const std::string& format = "%+.16e")
        {
            return encodeNumeric<double>(stream, value, format);
        }

        //! Appends specified string to a given stream
        /*!
         * \warning Does **not** add the null-termination character to the stream!
         *
         * \param [in,out] stream The stream to which the string should be appended to.
         * \param [in] string The string to append.
         * \return Count of appended characters (elements in stream).
         */
        static inline CharacterStreamSize encode(CharacterStream& stream, const std::string& string)
        {
            // Directly copy string data to stream (FAST, if stream capacity is already big enough -> no memory allocation necessary)
            std::copy(string.begin(), string.end(), std::back_inserter(stream));

            // Pass back length of added data
            return string.size();
        }

        // Decoding of basic datatypes
        // ---------------------------
        //! Decodes a single character from the given stream
        /*!
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the encoded value in the stream (=index of the **first element**).
         * \param [out] value Reference to returned value. The result of the decoding will be stored in this parameter.
         * \return Size of encoded value in case of success or 0 on failure
         */
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, char& value)
        {
            // Check, if stream is big enough
            if (stream.size() < index + 1) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Extract character from stream
            value = stream[index];

            // Return size of encoded value in stream
            return 1;
        }

        //! Interface for decoding unsigned integer values using the standard library
        /*!
         * \warning Throws an exeption if the conversion failed!
         *
         * \param [in] string The string containing the encoded value
         * \return The decoded value
         */
        template <typename UnsignedType, typename std::enable_if<std::is_integral<UnsignedType>::value && !std::is_signed<UnsignedType>::value, int>::type = 0>
        static inline UnsignedType decodeNumeric(const std::string& string)
        {
            const uint64_t value = std::stoull(string);
            if (value < std::numeric_limits<UnsignedType>::lowest() || value > std::numeric_limits<UnsignedType>::max())
                throw std::out_of_range("overflow in decoding of unsigned integer value");
            else
                return value;
        }

        //! Interface for decoding signed integer values using the standard library
        /*!
         * \warning Throws an exeption if the conversion failed!
         *
         * \param [in] string The string containing the encoded value
         * \return The decoded value
         */
        template <typename SignedType, typename std::enable_if<std::is_integral<SignedType>::value && std::is_signed<SignedType>::value, int>::type = 0>
        static inline SignedType decodeNumeric(const std::string& string)
        {
            const int64_t value = std::stoll(string);
            if (value < std::numeric_limits<SignedType>::lowest() || value > std::numeric_limits<SignedType>::max())
                throw std::out_of_range("overflow in decoding of signed integer value");
            else
                return value;
        }

        //! Interface for decoding floating point values using the standard library
        /*!
         * \warning Throws an exeption if the conversion failed!
         *
         * \param [in] string The string containing the encoded value
         * \return The decoded value
         */
        template <typename FloatingPointType, typename std::enable_if<std::is_floating_point<FloatingPointType>::value, int>::type = 0>
        static inline FloatingPointType decodeNumeric(const std::string& string)
        {
            const double value = std::stod(string);
            if (value < std::numeric_limits<FloatingPointType>::lowest() || value > std::numeric_limits<FloatingPointType>::max())
                throw std::out_of_range("overflow in decoding of floating point value");
            else
                return value;
        }

        //! Decodes a numeric value from the given stream
        /*!
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the encoded value in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded value in the stream (count of characters).
         * \param [out] value Reference to returned value. The result of the decoding will be stored in this parameter.
         * \return Size of encoded value in case of success or 0 on failure or overflow
         */
        template <typename NumericType, typename std::enable_if<std::is_arithmetic<NumericType>::value, int>::type = 0>
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, NumericType& value)
        {
            // Check if length is valid
            if (length == 0) {
                assert(false);
                return 0; // Error -> invalid length!
            }

            // Check, if stream is big enough
            if (stream.size() < index + length) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Decode (catch low level exceptions)
            try {
                value = decodeNumeric<NumericType>(std::string(stream.begin() + index, stream.begin() + (index + length)));
            } catch (...) {
                // Error -> the conversion failed (either the format is invalid or the converted value would fall out of range)
                assert(false);
                return 0;
            }

            // Return size of encoded value in stream
            return length;
        }

        //! Decodes a string from the given stream
        /*!
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the encoded value in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded value in the stream (count of characters).
         * \param [out] value Reference to returned value. The result of the decoding will be stored in this parameter.
         * \return Size of encoded value in case of success or 0 on failure
         */
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, std::string& value)
        {
            // Check if length is valid
            if (length == 0) {
                assert(false);
                return 0; // Error -> invalid length!
            }

            // Check, if stream is big enough
            if (stream.size() < index + length) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Extract string from stream
            value = std::string(stream.begin() + index, stream.begin() + (index + length));

            // Return size of encoded value in stream
            return length;
        }

        // =========================
        // --- COMPLEX DATATYPES ---
        // =========================
        // Encoding of complex datatypes
        // -----------------------------
#ifdef HAVE_EIGEN3
        //! Appends specified matrix to a given stream
        /*!
         * Columns are separated by a ',' character. Rows are separated by a ';' character.
         *
         * \tparam Derived The derived type, e.g. a matrix type, or an expression, etc.
         *
         * \param [in,out] stream The stream to which the matrix should be appended to.
         * \param [in] matrix The matrix to encode.
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Count of appended characters (elements in stream).
         */
        template <typename Derived>
        static inline CharacterStreamSize encode(CharacterStream& stream, const Eigen::MatrixBase<Derived>& matrix, const std::string& format = "")
        {
            CharacterStreamSize addedElements = 0;
            for (int i = 0; i < matrix.rows(); i++) {
                for (int j = 0; j < matrix.cols(); j++) {
                    if (j == 0) {
                        if (i > 0)
                            addedElements += encode(stream, (char)';');
                    } else {
                        addedElements += encode(stream, (char)',');
                    }
                    if (format == "")
                        addedElements += encode(stream, matrix(i, j));
                    else
                        addedElements += encode(stream, matrix(i, j), format);
                }
            }
            return addedElements;
        }

        //! Appends specified quaternion to a given stream
        /*!
         * Coefficients are separated by a ',' character and written in the order: w, x, y, z.
         *
         * \tparam Derived The derived type.
         *
         * \param [in,out] stream The stream to which the quaternion should be appended to.
         * \param [in] quaternion The quaternion to encode.
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Count of appended characters (elements in stream).
         */
        template <typename Derived>
        static inline CharacterStreamSize encode(CharacterStream& stream, const Eigen::QuaternionBase<Derived>& quaternion, const std::string& format = "")
        {
            CharacterStreamSize addedElements = 0;
            if (format == "") {
                addedElements += encode(stream, quaternion.w());
                addedElements += encode(stream, (char)',');
                addedElements += encode(stream, quaternion.x());
                addedElements += encode(stream, (char)',');
                addedElements += encode(stream, quaternion.y());
                addedElements += encode(stream, (char)',');
                addedElements += encode(stream, quaternion.z());
            } else {
                addedElements += encode(stream, quaternion.w(), format);
                addedElements += encode(stream, (char)',');
                addedElements += encode(stream, quaternion.x(), format);
                addedElements += encode(stream, (char)',');
                addedElements += encode(stream, quaternion.y(), format);
                addedElements += encode(stream, (char)',');
                addedElements += encode(stream, quaternion.z(), format);
            }
            return addedElements;
        }
#endif // HAVE_EIGEN3

        //! Helper function to encode array-like objects
        /*!
         * The array-like object has to specify
         *   * size()     ...function to return count of elements and
         *   * operator[] ...access to the i-th element in the array.
         *
         * The elements are separated by a ',' character.
         *
         * \tparam ArrayLikeType Type of the array-like object
         *
         * \param [in,out] stream The stream to which the array-like object should be appended to.
         * \param [in] arrayLike The array-like object to encode.
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Count of appended characters (elements in stream).
         */
        template <typename ArrayLikeType>
        static inline CharacterStreamSize encodeArrayLike(CharacterStream& stream, const ArrayLikeType& arrayLike, const std::string& format = "")
        {
            CharacterStreamSize addedElements = 0;
            for (size_t i = 0; i < arrayLike.size(); i++) {
                if (i > 0)
                    addedElements += encode(stream, (char)',');
                if (format == "")
                    addedElements += encode(stream, arrayLike[i]);
                else
                    addedElements += encode(stream, arrayLike[i], format);
            }
            return addedElements;
        }

        //! Appends specified array to a given stream
        /*!
         * The elements are separated by a ',' character.
         *
         * \tparam T Type of elements in array
         * \tparam N Count of elements in array
         *
         * \param [in,out] stream The stream to which the array should be appended to.
         * \param [in] array The array to encode.
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Count of appended characters (elements in stream).
         */
        template <typename T, size_t N>
        static inline CharacterStreamSize encode(CharacterStream& stream, const std::array<T, N>& array, const std::string& format = "")
        {
            return encodeArrayLike<std::array<T, N>>(stream, array, format);
        }

        //! Appends specified vector to a given stream
        /*!
         * The elements are separated by a ',' character.
         *
         * \tparam T Type of elements in vector
         * \tparam Allocator Allocator used by vector
         *
         * \param [in,out] stream The stream to which the vector should be appended to.
         * \param [in] vector The vector to encode.
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Count of appended characters (elements in stream).
         */
        template <typename T, typename Allocator = std::allocator<T>>
        static inline CharacterStreamSize encode(CharacterStream& stream, const std::vector<T, Allocator>& vector, const std::string& format = "")
        {
            return encodeArrayLike<std::vector<T, Allocator>>(stream, vector, format);
        }

        //! Appends specified broccoli::memory::SmartVector to a given stream
        /*!
         * The elements are separated by a ',' character.
         *
         * \tparam T Element type
         * \tparam MaximumStaticElementCount Count of elements of the underlying `std::array`
         * \tparam Allocator The allocator to be used for the underlying `std::vector`
         *
         * \param [in,out] stream The stream to which the SmartVector should be appended to.
         * \param [in] vector The SmartVector to encode.
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Count of appended characters (elements in stream).
         */
        template <typename T, size_t MaximumStaticElementCount = 0, typename Allocator = std::allocator<T>>
        static inline CharacterStreamSize encode(CharacterStream& stream, const memory::SmartVector<T, MaximumStaticElementCount, Allocator>& vector, const std::string& format = "")
        {
            return encodeArrayLike<memory::SmartVector<T, MaximumStaticElementCount, Allocator>>(stream, vector, format);
        }

        // Decoding of complex datatypes
        // -----------------------------
#ifdef HAVE_EIGEN3
        //! Decodes a matrix from the given stream
        /*!
         * Columns have to be separated by a ',' character. Rows have to be separated by a ';' character.
         *
         * \tparam Derived The derived type, e.g. a matrix type, or an expression, etc.
         *
         * \param [in] stream The stream from which the matrix should be extracted.
         * \param [in] index The starting index of the encoded matrix in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded matrix in the stream (count of characters).
         * \param [out] matrix Reference to returned matrix. The result of the decoding will be stored in this parameter.
         * \return Size of encoded matrix in case of success or 0 on failure
         */
        template <typename Derived>
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, Eigen::MatrixBase<Derived>& matrix)
        {
            // Check if length is valid
            if (length == 0) {
                assert(false);
                return 0; // Error -> invalid length!
            }

            // Check, if stream is big enough
            if (stream.size() < index + length) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Search for row delimiters
            std::vector<CharacterStreamSize> rowDelimiterIndices;
            rowDelimiterIndices.reserve(length);
            for (CharacterStreamSize i = index; i < index + length; i++)
                if (stream[i] == ';')
                    rowDelimiterIndices.push_back(i);
            const CharacterStreamSize rowCount = rowDelimiterIndices.size() + 1;

            // Search for column delimiters
            std::vector<std::vector<CharacterStreamSize>> columnDelimiterIndices;
            columnDelimiterIndices.resize(rowCount);
            CharacterStreamSize columnCount = 0;
            for (size_t i = 0; i < rowCount; i++) {
                // Define start- and end-index of this row in the stream
                CharacterStreamSize startIndex = index;
                if (i > 0)
                    startIndex = rowDelimiterIndices[i - 1] + 1;
                CharacterStreamSize endIndex = index + length - 1;
                if (i < rowCount - 1)
                    endIndex = rowDelimiterIndices[i] - 1;
                const CharacterStreamSize rowLength = endIndex - startIndex + 1;

                // Search for column delimiters in this row
                columnDelimiterIndices[i].reserve(rowLength);
                for (CharacterStreamSize j = startIndex; j < startIndex + rowLength; j++)
                    if (stream[j] == ',')
                        columnDelimiterIndices[i].push_back(j);
                if (i == 0)
                    columnCount = columnDelimiterIndices[i].size() + 1;
                else if (columnDelimiterIndices[i].size() + 1 != columnCount) {
                    assert(false);
                    return 0; // Error -> invalid column count in this row!
                }
            }

            // Resize matrix
            if ((matrix.RowsAtCompileTime != Eigen::Dynamic && matrix.rows() != (int)rowCount) || (matrix.ColsAtCompileTime != Eigen::Dynamic && matrix.cols() != (int)columnCount)) {
                assert(false);
                return 0; // Error -> the dimensions of the provided matrix do not match the decoded data!
            }
            matrix.derived().resize(rowCount, columnCount);

            // Decode elements
            for (size_t i = 0; i < rowCount; i++) {
                for (size_t j = 0; j < columnCount; j++) {
                    // Define start- and end-index of this element in the stream
                    CharacterStreamSize startIndex = index;
                    if (j > 0)
                        startIndex = columnDelimiterIndices[i][j - 1] + 1;
                    else if (i > 0)
                        startIndex = rowDelimiterIndices[i - 1] + 1;
                    CharacterStreamSize endIndex = index + length - 1;
                    if (j < columnCount - 1)
                        endIndex = columnDelimiterIndices[i][j] - 1;
                    else if (i < rowCount - 1)
                        endIndex = rowDelimiterIndices[i] - 1;
                    const CharacterStreamSize elementLength = endIndex - startIndex + 1;

                    // Decode element
                    if (decode(stream, startIndex, elementLength, matrix(i, j)) == 0) {
                        assert(false);
                        return 0; // Error -> decoding element failed!
                    }
                }
            }

            // Return size of encoded array in stream
            return length;
        }

        //! Decodes a quaternion from the given stream
        /*!
         * The elements have to be separated by a ',' character. The coefficients must have the order: w, x, y, z.
         *
         * \tparam Derived The derived type.
         *
         * \param [in] stream The stream from which the quaternion should be extracted.
         * \param [in] index The starting index of the encoded quaternion in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded quaternion in the stream (count of characters).
         * \param [out] quaternion Reference to returned quaternion. The result of the decoding will be stored in this parameter.
         * \return Size of encoded quaternion in case of success or 0 on failure
         */
        template <typename Derived>
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, Eigen::QuaternionBase<Derived>& quaternion)
        {
            // Check if length is valid
            if (length == 0) {
                assert(false);
                return 0; // Error -> invalid length!
            }

            // Check, if stream is big enough
            if (stream.size() < index + length) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Search for delimiters
            std::vector<CharacterStreamSize> delimiterIndices;
            delimiterIndices.reserve(3);
            for (CharacterStreamSize i = index; i < index + length; i++)
                if (stream[i] == ',')
                    delimiterIndices.push_back(i);
            if (delimiterIndices.size() != 3) {
                assert(false);
                return 0; // Error -> invalid count of delimiters found!
            }

            // Decode elements
            for (size_t i = 0; i < 4; i++) {
                // Define start- and end-index of this element in the stream
                CharacterStreamSize startIndex = index;
                if (i > 0)
                    startIndex = delimiterIndices[i - 1] + 1;
                CharacterStreamSize endIndex = index + length - 1;
                if (i < 4 - 1)
                    endIndex = delimiterIndices[i] - 1;

                // Decode element
                auto* element = &quaternion.w();
                if (i == 1)
                    element = &quaternion.x();
                else if (i == 2)
                    element = &quaternion.y();
                else if (i == 3)
                    element = &quaternion.z();
                if (decode(stream, startIndex, endIndex - startIndex + 1, *element) == 0) {
                    assert(false);
                    return 0; // Error -> decoding element failed!
                }
            }

            // Return size of encoded array in stream
            return length;
        }
#endif // HAVE_EIGEN3

        //! Decodes an array from the given stream
        /*!
         * The elements have to be separated by a ',' character.
         *
         * \tparam T Type of elements in array
         * \tparam N Count of elements in array
         *
         * \param [in] stream The stream from which the array should be extracted.
         * \param [in] index The starting index of the encoded array in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded array in the stream (count of characters).
         * \param [out] array Reference to returned array. The result of the decoding will be stored in this parameter.
         * \return Size of encoded array in case of success or 0 on failure
         */
        template <typename T, size_t N>
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, std::array<T, N>& array)
        {
            // Check if length is valid
            if (length == 0) {
                assert(false);
                return 0; // Error -> invalid length!
            }

            // Check, if stream is big enough
            if (stream.size() < index + length) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Search for delimiters
            std::vector<CharacterStreamSize> delimiterIndices;
            delimiterIndices.reserve(N - 1);
            for (CharacterStreamSize i = index; i < index + length; i++)
                if (stream[i] == ',')
                    delimiterIndices.push_back(i);
            if (delimiterIndices.size() != N - 1) {
                assert(false);
                return 0; // Error -> invalid count of delimiters found!
            }

            // Decode elements
            for (size_t i = 0; i < N; i++) {
                // Define start- and end-index of this element in the stream
                CharacterStreamSize startIndex = index;
                if (i > 0)
                    startIndex = delimiterIndices[i - 1] + 1;
                CharacterStreamSize endIndex = index + length - 1;
                if (i < N - 1)
                    endIndex = delimiterIndices[i] - 1;

                // Decode element
                if (decode(stream, startIndex, endIndex - startIndex + 1, array[i]) == 0) {
                    assert(false);
                    return 0; // Error -> decoding element failed!
                }
            }

            // Return size of encoded array in stream
            return length;
        }

        //! Helper function to decode vector-like objects
        /*!
         * The vector-like object has to specify
         *   * resize()   ...possiblity to resize the container such that it has the specified count of elements and
         *   * operator[] ...access to the i-th element in the vector.
         *
         * \tparam VectorLikeType Type of vector-like object
         *
         * \param [in] stream The stream from which the vector-like should be extracted.
         * \param [in] index The starting index of the encoded vector-like in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded vector-like in the stream (count of characters).
         * \param [out] vectorLike Reference to returned vector-like. The result of the decoding will be stored in this parameter.
         * \return Size of encoded vector-like in case of success or 0 on failure
         */
        template <typename VectorLikeType>
        static inline CharacterStreamSize decodeVectorLike(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, VectorLikeType& vectorLike)
        {
            // Check if length is valid
            if (length == 0) {
                assert(false);
                return 0; // Error -> invalid length!
            }

            // Check, if stream is big enough
            if (stream.size() < index + length) {
                assert(false);
                return 0; // Error -> stream is not big enough
            }

            // Search for delimiters
            std::vector<CharacterStreamSize> delimiterIndices;
            delimiterIndices.reserve(length);
            for (CharacterStreamSize i = index; i < index + length; i++)
                if (stream[i] == ',')
                    delimiterIndices.push_back(i);
            const CharacterStreamSize elementCount = delimiterIndices.size() + 1;

            // Decode elements
            vectorLike.resize(elementCount);
            for (size_t i = 0; i < elementCount; i++) {
                // Define start- and end-index of this element in the stream
                CharacterStreamSize startIndex = index;
                if (i > 0)
                    startIndex = delimiterIndices[i - 1] + 1;
                CharacterStreamSize endIndex = index + length - 1;
                if (i < elementCount - 1)
                    endIndex = delimiterIndices[i] - 1;
                const CharacterStreamSize elementLength = endIndex - startIndex + 1;

                // Decode element
                if (decode(stream, startIndex, elementLength, vectorLike[i]) == 0) {
                    assert(false);
                    return 0; // Error -> decoding element failed!
                }
            }

            // Return size of encoded array in stream
            return length;
        }

        //! Decodes a vector from the given stream
        /*!
         * The elements have to be separated by a ',' character.
         *
         * \tparam T Type of elements in vector
         * \tparam Allocator Allocator used by vector
         *
         * \param [in] stream The stream from which the vector should be extracted.
         * \param [in] index The starting index of the encoded vector in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded vector in the stream (count of characters).
         * \param [out] vector Reference to returned vector. The result of the decoding will be stored in this parameter.
         * \return Size of encoded vector in case of success or 0 on failure
         */
        template <typename T, typename Allocator = std::allocator<T>>
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, std::vector<T, Allocator>& vector)
        {
            return decodeVectorLike<std::vector<T, Allocator>>(stream, index, length, vector);
        }

        //! Decodes a broccoli::memory::SmartVector from the given stream
        /*!
         * The elements have to be separated by a ',' character.
         *
         * \tparam T Element type
         * \tparam MaximumStaticElementCount Count of elements of the underlying `std::array`
         * \tparam Allocator The allocator to be used for the underlying `std::vector`
         *
         * \param [in] stream The stream from which the SmartVector should be extracted.
         * \param [in] index The starting index of the encoded SmartVector in the stream (=index of the **first element**).
         * \param [in] length The length of the encoded SmartVector in the stream (count of characters).
         * \param [out] vector Reference to returned SmartVector. The result of the decoding will be stored in this parameter.
         * \return Size of encoded SmartVector in case of success or 0 on failure
         */
        template <typename T, size_t MaximumStaticElementCount = 0, typename Allocator = std::allocator<T>>
        static inline CharacterStreamSize decode(const CharacterStream& stream, const CharacterStreamSize& index, const CharacterStreamSize& length, memory::SmartVector<T, MaximumStaticElementCount, Allocator>& vector)
        {
            return decodeVectorLike<memory::SmartVector<T, MaximumStaticElementCount, Allocator>>(stream, index, length, vector);
        }

        // Interface for std::string
        // -------------------------
        //! Encodes the given data object to a std::string
        /*!
         * \tparam T Type of data to encode
         *
         * \param [in] data The data to encode
         * \param [in] format C-format specifier (see \ref encodeNumeric() for details).
         * \return Data encoded as a string.
         */
        template <typename T>
        static inline std::string encodeToString(const T& data, const std::string& format = "")
        {
            CharacterStream stream;
            if (format == "")
                encode(stream, data);
            else
                encode(stream, data, format);
            return std::string(stream.begin(), stream.end());
        }

        //! Decodes the given data object from a std::string
        /*!
         * \tparam T Type of data to decode
         *
         * \param [out] data The decoded data object (result gets stored in this parameter)
         * \param [in] string The encoded data object as string
         * \return `true` on success, `false` otherwise
         */
        template <typename T>
        static inline bool decodeFromString(T& data, const std::string& string)
        {
            CharacterStream stream(string.begin(), string.end());
            if (decode(stream, 0, stream.size(), data) == 0)
                return false; // Decoding failed!
            else
                return true; // Success!
        }

        // Base64 encoding and decoding
        // ----------------------------
        //! Encodes the given binary stream to a text stream using base64 encoding
        /*!
         * \param [in] binaryStream Arbitary binary stream as input
         * \param [out] textStream Text stream encoded as base64 as output
         * \param [in] withPadding If `true`, padding '=' characters are appended, otherwise not (according to base64 definition)
         */
        static inline void encodeToBase64(const CharacterStream& binaryStream, std::string& textStream, const bool& withPadding = true)
        {
            // Initialize helpers
            static const char* binaryToText = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            const size_t binaryStreamSize = binaryStream.size();

            // Determine count of blocks
            size_t blockCount = binaryStreamSize / 3;
            const size_t remainder = binaryStreamSize % 3;
            if (remainder > 0)
                blockCount++;

            // Pre-allocate output buffer
            textStream.clear();
            if (withPadding == true || remainder == 0)
                textStream.resize(blockCount * 4);
            else if (remainder == 1)
                textStream.resize(blockCount * 4 - 2);
            else
                textStream.resize(blockCount * 4 - 1);

            // Encode blocks
            for (size_t i = 0; i < blockCount; i++) {
                const size_t b0Idx = i * 3;
                size_t txIdx = i * 4;

                // Encode first binary byte
                const uint8_t& b0 = binaryStream[b0Idx];
                textStream[txIdx++] = binaryToText[(b0 & 0xfc) >> 2];

                // Encode second binary byte
                const size_t b1Idx = b0Idx + 1;
                if (b1Idx < binaryStreamSize) {
                    const uint8_t& b1 = binaryStream[b1Idx];
                    textStream[txIdx++] = binaryToText[((b0 & 0x03) << 4) + ((b1 & 0xf0) >> 4)];

                    // Encode third binary byte
                    const size_t b2Idx = b1Idx + 1;
                    if (b2Idx < binaryStreamSize) {
                        const uint8_t& b2 = binaryStream[b2Idx];
                        textStream[txIdx++] = binaryToText[((b1 & 0x0f) << 2) + ((b2 & 0xc0) >> 6)];
                        textStream[txIdx] = binaryToText[b2 & 0x3f];
                    } else {
                        // Only first two binary bytes are available
                        textStream[txIdx++] = binaryToText[(b1 & 0x0f) << 2];
                        if (withPadding == true)
                            textStream[txIdx] = (char)'=';
                    }
                } else {
                    // Only the first binary byte is available
                    textStream[txIdx++] = binaryToText[(b0 & 0x03) << 4];
                    if (withPadding == true) {
                        textStream[txIdx++] = (char)'=';
                        textStream[txIdx] = (char)'=';
                    }
                }
            }
        }

        //! Encodes the given binary stream to a text stream using base64 encoding
        /*!
         * \param [in] binaryStream Arbitary binary stream as input
         * \param [in] withPadding If `true`, padding '=' characters are appended, otherwise not (according to base64 definition)
         * \return Text stream encoded as base64 as output
         */
        static inline std::string encodeToBase64(const CharacterStream& binaryStream, const bool& withPadding = true)
        {
            std::string returnValue;
            encodeToBase64(binaryStream, returnValue, withPadding);
            return returnValue;
        }

        //! Decodes the given text stream to a binary stream using base64 decoding
        /*!
         * \param [out] textStream Text stream encoded as base64 as input
         * \param [out] binaryStream Decoded binary stream as output
         * \return `true` on success, `false` otherwise
         */
        static inline bool decodeFromBase64(const std::string& textStream, CharacterStream& binaryStream)
        {
            // Initialize helpers
            static const uint8_t textToBinary[256] = {
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [0-9]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [10-19]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [20-29]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [30-39]
                255, 255, 255, 62 /*'+'*/, 255, 255, 255, 63 /*'/'*/, 52 /*'0'*/, 53 /*'1'*/, // [40-49]
                54 /*'2'*/, 55 /*'3'*/, 56 /*'4'*/, 57 /*'5'*/, 58 /*'6'*/, 59 /*'7'*/, 60 /*'8'*/, 61 /*'9'*/, 255, 255, // [50-59]
                255, 255, 255, 255, 255, 0 /*'A'*/, 1 /*'B'*/, 2 /*'C'*/, 3 /*'D'*/, 4 /*'E'*/, // [60-69]
                5 /*'F'*/, 6 /*'G'*/, 7 /*'H'*/, 8 /*'I'*/, 9 /*'J'*/, 10 /*'K'*/, 11 /*'L'*/, 12 /*'M'*/, 13 /*'N'*/, 14 /*'O'*/, // [70-79]
                15 /*'P'*/, 16 /*'Q'*/, 17 /*'R'*/, 18 /*'S'*/, 19 /*'T'*/, 20 /*'U'*/, 21 /*'V'*/, 22 /*'W'*/, 23 /*'X'*/, 24 /*'Y'*/, // [80-89]
                25 /*'Z'*/, 255, 255, 255, 255, 255, 255, 26 /*'a'*/, 27 /*'b'*/, 28 /*'c'*/, // [90-99]
                29 /*'d'*/, 30 /*'e'*/, 31 /*'f'*/, 32 /*'g'*/, 33 /*'h'*/, 34 /*'i'*/, 35 /*'j'*/, 36 /*'k'*/, 37 /*'l'*/, 38 /*'m'*/, // [100-109]
                39 /*'n'*/, 40 /*'o'*/, 41 /*'p'*/, 42 /*'q'*/, 43 /*'r'*/, 44 /*'s'*/, 45 /*'t'*/, 46 /*'u'*/, 47 /*'v'*/, 48 /*'w'*/, // [110-119]
                49 /*'x'*/, 50 /*'y'*/, 51 /*'z'*/, 255, 255, 255, 255, 255, 255, 255, // [120-129]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [130-139]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [140-149]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [150-159]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [160-169]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [170-179]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [180-189]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [190-199]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [200-209]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [210-219]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [220-229]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [230-239]
                255, 255, 255, 255, 255, 255, 255, 255, 255, 255, // [240-249]
                255, 255, 255, 255, 255, 255 // [250-255]
            };
            const size_t textStreamSize = textStream.size();

            // Determine count of blocks
            size_t blockCount = textStreamSize / 4;
            const size_t remainder = textStreamSize % 4;
            if (remainder > 0)
                blockCount++;

            // Pre-allocate output buffer
            binaryStream.clear();
            if (remainder == 0)
                binaryStream.resize(blockCount * 3);
            else if (remainder == 1) {
                assert(false);
                return false;
            } else if (remainder == 2)
                binaryStream.resize(blockCount * 3 - 2);
            else
                binaryStream.resize(blockCount * 3 - 1);

            // Decode blocks
            for (size_t i = 0; i < blockCount; i++) {
                const size_t t0Idx = i * 4;
                const size_t t1Idx = t0Idx + 1;
                size_t bxIdx = i * 3;

                // Decode first two text bytes
                const uint8_t t0 = textToBinary[(uint8_t)textStream[t0Idx]];
                const uint8_t t1 = textToBinary[(uint8_t)textStream[t1Idx]];
                if (t0 > 63 || t1 > 63) {
                    assert(false);
                    return false;
                }
                binaryStream[bxIdx++] = (t0 << 2) + ((t1 & 0x30) >> 4);

                // Decode third text byte
                const size_t t2Idx = t1Idx + 1;
                if (t2Idx < textStreamSize) {
                    if (i + 1 == blockCount /* last block */ && textStream[t2Idx] == (char)'=') {
                        // We are done here
                        binaryStream.resize(binaryStream.size() - 2);
                        break;
                    } else {
                        const uint8_t t2 = textToBinary[(uint8_t)textStream[t2Idx]];
                        if (t2 > 63) {
                            assert(false);
                            return false;
                        }
                        binaryStream[bxIdx++] = ((t1 & 0x0f) << 4) + ((t2 & 0x3c) >> 2);

                        // Decode fourth text byte
                        const size_t t3Idx = t2Idx + 1;
                        if (t3Idx < textStreamSize) {
                            if (i + 1 == blockCount /* last block */ && textStream[t3Idx] == (char)'=') {
                                // We are done here
                                binaryStream.resize(binaryStream.size() - 1);
                                break;
                            } else {
                                const uint8_t t3 = textToBinary[(uint8_t)textStream[t3Idx]];
                                if (t3 > 63) {
                                    assert(false);
                                    return false;
                                }
                                binaryStream[bxIdx] = ((t2 & 0x03) << 6) + t3;
                            }
                        } else
                            break; //Only the first three text bytes are available -> done!
                    }
                } else
                    break; //Only the first two text bytes are available -> done!
            }

            // Success
            return true;
        }

        //! \}
    } // namespace encoding
} // namespace io
} // namespace broccoli
