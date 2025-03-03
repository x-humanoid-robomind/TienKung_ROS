/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../encoding.hpp"
#include "../serialization/serialization.hpp"
#include "PLYFormat.hpp"
#include "PLYResult.hpp"
#include "PLYScalar.hpp"
#include <assert.h>
#include <cstring>
#include <stdint.h>
#include <vector>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Abstraction layer for property values of the Polygon File Format (PLY)
    /*!
     * Handles storage of various property values (scalars or list of scalars with different data types). Internally uses a
     * buffer to/from which values are written/read using memcpy.
     *
     * \warning It is the task of the user to keep track of the data types. This means that if a float has been written to the buffer the
     * user may only read a float value from the buffer. Otherwise the data is parsed in a wrong way!
     */
    class PLYPropertyValue {
    public:
        //! Constructor
        PLYPropertyValue()
        {
        }

        //! Comparison operator: **equality**
        bool operator==(const PLYPropertyValue& reference) const
        {
            // Compare members
            if (m_data != reference.m_data)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        bool operator!=(const PLYPropertyValue& reference) const { return !(*this == reference); }

        // Members
    protected:
        std::vector<uint8_t> m_data; //!< Internal data buffer (contains scalar or list of scalars)

    public:
        // Setters
        // -------
        //! Writes the given scalar to the buffer
        template <typename T>
        void setScalar(const T& scalar)
        {
            static const uint64_t byteCount = sizeof(T);
            m_data.resize(byteCount);
            memcpy((void*)&m_data[0], (void*)&scalar, byteCount);
        }

        //! Writes the given list of scalars to the buffer
        template <typename T>
        void setList(const std::vector<T>& list)
        {
            const uint64_t byteCount = sizeof(T) * list.size();
            m_data.resize(byteCount);
            if (list.size() > 0)
                memcpy((void*)&m_data[0], (void*)&list[0], byteCount);
        }

        // Getters
        // -------
        //! Returns scalar from the buffer
        /*!
         * \tparam ScalarType Datatype of the scalar in the buffer
         *
         * \param [out] scalar The scalar is written into this parameter
         * \return `true` on success, `false` in case of an error (scalar type does not match buffer size)
         */
        template <typename ScalarType>
        bool scalar(ScalarType& scalar) const
        {
            static const uint64_t byteCount = sizeof(ScalarType);
            if (byteCount == m_data.size()) {
                memcpy((void*)&scalar, (void*)&m_data[0], byteCount);
                return true;
            } else {
                // Size of data buffer does not fit scalar type!
                assert(false);
                return false;
            }
        }

        //! Returns scalar (converted to specified data type) from the buffer
        /*!
         * \tparam ScalarType Datatype of the scalar in the buffer
         * \tparam OutputType Datatype of the converted output scalar
         *
         * \param [out] convertedScalar The converted scalar is written into this parameter
         * \return `true` on success, `false` in case of an error (scalar type does not match buffer size)
         */
        template <typename ScalarType, typename OutputType>
        bool scalarConverted(OutputType& convertedScalar) const
        {
            ScalarType originalScalar = 0;
            const bool returnValue = scalar(originalScalar);
            convertedScalar = originalScalar;
            return returnValue;
        }

        //! Returns scalar (converted to specified data type) from the buffer
        /*!
         * \tparam OutputType Datatype of the converted output scalar
         *
         * \param [in] scalarType Datatype of the scalar in the buffer
         * \param [out] convertedScalar The converted scalar is written into this parameter
         * \return `true` on success, `false` in case of an error (scalar type does not match buffer size)
         */
        template <typename OutputType>
        bool scalarConverted(const PLYScalar::Type& scalarType, OutputType& convertedScalar) const
        {
            if (scalarType == PLYScalar::Type::CHAR)
                return scalarConverted<CHAR_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::UCHAR)
                return scalarConverted<UCHAR_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::SHORT)
                return scalarConverted<SHORT_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::USHORT)
                return scalarConverted<USHORT_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::INT)
                return scalarConverted<INT_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::UINT)
                return scalarConverted<UINT_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::FLOAT)
                return scalarConverted<FLOAT_t, OutputType>(convertedScalar);
            else if (scalarType == PLYScalar::Type::DOUBLE)
                return scalarConverted<DOUBLE_t, OutputType>(convertedScalar);
            else {
                assert(false);
                return false;
            }
        }

        //! Returns list of scalars from the buffer
        /*!
         * \param [out] list List of scalars is written into this parameter
         * \return `true` on success, `false` in case of an error (scalar type does not match buffer size)
         */
        template <typename T>
        bool list(std::vector<T>& list) const
        {
            if (m_data.size() % sizeof(T) == 0) {
                list.resize(m_data.size() / sizeof(T));
                if (list.size() > 0)
                    memcpy((void*)&list[0], (void*)&m_data[0], m_data.size());
                return true;
            } else {
                // Size of data buffer does not fit scalar type!
                assert(false);
                return false;
            }
        }

        //! Returns list of scalars (converted to specified data type) from the buffer
        /*!
         * \tparam ScalarType Datatype of the scalars in the buffer
         * \tparam OutputType Datatype of the converted output scalars
         *
         * \param [out] convertedList List of converted scalars is written into this parameter
         * \return `true` on success, `false` in case of an error (scalar type does not match buffer size)
         */
        template <typename ScalarType, typename OutputType>
        bool listConverted(std::vector<OutputType>& convertedList) const
        {
            std::vector<ScalarType> originalList;
            const bool returnValue = list(originalList);
            convertedList.resize(originalList.size());
            for (size_t i = 0; i < originalList.size(); i++)
                convertedList[i] = originalList[i];
            return returnValue;
        }

        //! Returns list of scalars (converted to specified data type) from the buffer
        /*!
         * \tparam OutputType Datatype of the converted output scalars
         *
         * \param [in] scalarType Datatype of the scalars in the buffer
         * \param [out] convertedList List of converted scalars is written into this parameter
         * \return `true` on success, `false` in case of an error (scalar type does not match buffer size)
         */
        template <typename OutputType>
        bool listConverted(const PLYScalar::Type& scalarType, std::vector<OutputType>& convertedList) const
        {
            if (scalarType == PLYScalar::Type::CHAR)
                return listConverted<CHAR_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::UCHAR)
                return listConverted<UCHAR_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::SHORT)
                return listConverted<SHORT_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::USHORT)
                return listConverted<USHORT_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::INT)
                return listConverted<INT_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::UINT)
                return listConverted<UINT_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::FLOAT)
                return listConverted<FLOAT_t, OutputType>(convertedList);
            else if (scalarType == PLYScalar::Type::DOUBLE)
                return listConverted<DOUBLE_t, OutputType>(convertedList);
            else {
                assert(false);
                return false;
            }
        }

        // Encoding
        // --------
        //! Encodes scalar property to the given stream
        /*!
         * \param [out] stream Stream to append encoded property to
         * \param [in] format The format to be used
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of added elements to stream or 0 in case of an error
         */
        template <typename ValueType>
        encoding::CharacterStreamSize encodeScalar(encoding::CharacterStream& stream, const PLYFormat::Type& format, PLYResult* const result = nullptr) const
        {
            encoding::CharacterStreamSize addedElements = 0;

            // Get scalar from buffer
            ValueType scalarToEncode;
            if (scalar(scalarToEncode) == false) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                assert(false);
                return 0;
            }

            // Encode scalar
            if (format == PLYFormat::Type::ASCII)
                addedElements = encoding::encode(stream, scalarToEncode);
            else if (format == PLYFormat::Type::BINARY_LITTLE_ENDIAN)
                addedElements = serialization::serialize(stream, serialization::Endianness::LITTLE, scalarToEncode);
            else if (format == PLYFormat::Type::BINARY_BIG_ENDIAN)
                addedElements = serialization::serialize(stream, serialization::Endianness::BIG, scalarToEncode);
            else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_FORMAT;
                assert(false);
                return 0;
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return addedElements;
        }

        //! Encodes list property to the given stream
        /*!
         * \param [out] stream Stream to append encoded property to
         * \param [in] format The format to be used
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of added elements to stream or 0 in case of an error
         */
        template <typename CounterType, typename ValueType>
        encoding::CharacterStreamSize encodeList(encoding::CharacterStream& stream, const PLYFormat::Type& format, PLYResult* const result = nullptr) const
        {
            encoding::CharacterStreamSize addedElements = 0;

            // Get list from buffer
            std::vector<ValueType> listToEncode;
            if (list(listToEncode) == false) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                assert(false);
                return 0;
            }
            const CounterType listSize = listToEncode.size();

            // Encode list
            if (format == PLYFormat::Type::ASCII) {
                addedElements += encoding::encode(stream, listSize);
                for (size_t i = 0; i < listToEncode.size(); i++) {
                    addedElements += encoding::encode(stream, (char)' ');
                    addedElements += encoding::encode(stream, listToEncode[i]);
                }
            } else if (format == PLYFormat::Type::BINARY_LITTLE_ENDIAN || format == PLYFormat::Type::BINARY_BIG_ENDIAN) {
                // Detect endianness
                serialization::Endianness endianness = serialization::Endianness::LITTLE;
                if (format == PLYFormat::Type::BINARY_BIG_ENDIAN)
                    endianness = serialization::Endianness::BIG;

                // Encode counter and values
                addedElements += serialization::serialize(stream, endianness, listSize);
                for (size_t i = 0; i < listToEncode.size(); i++)
                    addedElements += serialization::serialize(stream, endianness, listToEncode[i]);
            } else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_FORMAT;
                assert(false);
                return 0;
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return addedElements;
        }

        // Decoding
        // --------
        //! Decodes scalar property from the given stream
        /*!
         * \param [in] stream Stream from which the property should be extracted
         * \param [in] index Starting index in the given stream
         * \param [in] format The used format
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of decoded elements in the stream or 0 in case of an error
         */
        template <typename ValueType>
        encoding::CharacterStreamSize decodeScalar(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& index, const PLYFormat::Type& format, PLYResult* const result = nullptr)
        {
            encoding::CharacterStreamSize decodedElements = 0;

            // Decode scalar
            ValueType decodedScalar;
            if (format == PLYFormat::Type::ASCII) {
                // Check stream size
                if (index >= stream.size()) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_SCALAR;
                    return 0;
                }

                // Find starting index
                encoding::CharacterStreamSize startingIndex = index;
                while ((stream[startingIndex] == (uint8_t)' ' || stream[startingIndex] == (uint8_t)'\n') && startingIndex < stream.size() - 1)
                    startingIndex++;
                decodedElements += startingIndex - index;

                // Compute length of scalar in stream
                encoding::CharacterStreamSize length = stream.size() - startingIndex;
                for (encoding::CharacterStreamSize i = startingIndex; i < stream.size(); i++) {
                    if (stream[i] == (uint8_t)' ' || stream[i] == (uint8_t)'\n') {
                        length = i - startingIndex;
                        break;
                    }
                }
                if (length == 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_SCALAR;
                    return 0;
                }

                // Decode scalar (with known length)
                auto decodedElementsOnlyValue = encoding::decode(stream, startingIndex, length, decodedScalar);
                if (decodedElementsOnlyValue == 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_SCALAR;
                    return 0;
                }
                decodedElements += decodedElementsOnlyValue;
            } else if (format == PLYFormat::Type::BINARY_LITTLE_ENDIAN || format == PLYFormat::Type::BINARY_BIG_ENDIAN) {
                serialization::Endianness endianness = serialization::Endianness::LITTLE;
                if (format == PLYFormat::Type::BINARY_BIG_ENDIAN)
                    endianness = serialization::Endianness::BIG;
                decodedElements = serialization::deSerialize(stream, index, endianness, decodedScalar);
                if (decodedElements == 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_SCALAR;
                    return 0;
                }
            } else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_FORMAT;
                assert(false);
                return 0;
            }

            // Copy scalar to buffer
            setScalar(decodedScalar);

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return decodedElements;
        }

        //! Decodes list property from the given stream
        /*!
         * \param [in] stream Stream from which the property should be extracted
         * \param [in] index Starting index in the given stream
         * \param [in] format The used format
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of decoded elements in the stream or 0 in case of an error
         */
        template <typename CounterType, typename ValueType>
        encoding::CharacterStreamSize decodeList(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& index, const PLYFormat::Type& format, PLYResult* const result = nullptr)
        {
            encoding::CharacterStreamSize decodedElements = 0;

            // Decode list
            CounterType counter = 0;
            std::vector<ValueType> decodedList;
            if (format == PLYFormat::Type::ASCII) {
                // Check stream size
                if (index >= stream.size()) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_LIST;
                    return 0;
                }

                // Find starting index
                encoding::CharacterStreamSize counterStartingIndex = index;
                while ((stream[counterStartingIndex] == (uint8_t)' ' || stream[counterStartingIndex] == (uint8_t)'\n') && counterStartingIndex < stream.size() - 1)
                    counterStartingIndex++;
                decodedElements += counterStartingIndex - index;

                // Compute length of counter in stream
                encoding::CharacterStreamSize counterLength = stream.size() - counterStartingIndex;
                for (encoding::CharacterStreamSize i = counterStartingIndex; i < stream.size(); i++) {
                    if (stream[i] == (uint8_t)' ' || stream[i] == (uint8_t)'\n') {
                        counterLength = i - counterStartingIndex;
                        break;
                    }
                }
                if (counterLength == 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_LIST;
                    return 0;
                }

                // Decode counter (with known length)
                auto decodedElementsCounter = encoding::decode(stream, counterStartingIndex, counterLength, counter);
                if (decodedElementsCounter == 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_LIST;
                    return 0;
                }
                decodedElements += decodedElementsCounter;
                if (counter < 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_LIST;
                    return 0;
                }

                // Decode values
                decodedList.resize(counter);
                encoding::CharacterStreamSize currentIndex = counterStartingIndex + counterLength;
                for (size_t i = 0; i < (size_t)counter; i++) {
                    // Find starting index
                    encoding::CharacterStreamSize valueStartingIndex = currentIndex;
                    while ((stream[valueStartingIndex] == (uint8_t)' ' || stream[valueStartingIndex] == (uint8_t)'\n') && valueStartingIndex < stream.size() - 1)
                        valueStartingIndex++;
                    decodedElements += valueStartingIndex - currentIndex;

                    // Compute length of value in stream
                    encoding::CharacterStreamSize valueLength = stream.size() - valueStartingIndex;
                    for (encoding::CharacterStreamSize i = valueStartingIndex; i < stream.size(); i++) {
                        if (stream[i] == (uint8_t)' ' || stream[i] == (uint8_t)'\n') {
                            valueLength = i - valueStartingIndex;
                            break;
                        }
                    }
                    if (valueLength == 0) {
                        assert(false);
                        if (result != nullptr)
                            *result = PLYResult::ERROR_DECODE_LIST;
                        return 0;
                    }

                    // Decode value (with known length)
                    auto decodedElementsValue = encoding::decode(stream, valueStartingIndex, valueLength, decodedList[i]);
                    decodedElements += decodedElementsValue;
                    if (decodedElementsValue == 0) {
                        assert(false);
                        if (result != nullptr)
                            *result = PLYResult::ERROR_DECODE_LIST;
                        return 0;
                    }

                    // Update starting index
                    currentIndex = valueStartingIndex + valueLength;
                }
            } else if (format == PLYFormat::Type::BINARY_LITTLE_ENDIAN || format == PLYFormat::Type::BINARY_BIG_ENDIAN) {
                // Detect endianness
                serialization::Endianness endianness = serialization::Endianness::LITTLE;
                if (format == PLYFormat::Type::BINARY_BIG_ENDIAN)
                    endianness = serialization::Endianness::BIG;

                // Decode counter
                auto decodedElementsCounter = serialization::deSerialize(stream, index, endianness, counter);
                decodedElements += decodedElementsCounter;
                if (decodedElementsCounter == 0) {
                    assert(false);
                    if (result != nullptr)
                        *result = PLYResult::ERROR_DECODE_LIST;
                    return 0;
                }

                // Decode elements
                decodedList.resize(counter);
                for (size_t i = 0; i < (size_t)counter; i++) {
                    auto decodedElementsValue = serialization::deSerialize(stream, index + sizeof(CounterType) + i * sizeof(ValueType), endianness, decodedList[i]);
                    decodedElements += decodedElementsValue;
                    if (decodedElementsValue == 0) {
                        assert(false);
                        if (result != nullptr)
                            *result = PLYResult::ERROR_DECODE_LIST;
                        return 0;
                    }
                }
            } else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_FORMAT;
                assert(false);
                return 0;
            }

            // Copy list to buffer
            setList(decodedList);

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return decodedElements;
        }

        // Helpers
        // -------
        //! Returns count of currently stored bytes in the internal buffer
        size_t bytesInBuffer() const { return m_data.size(); }
    };

    //! \}
} // namespace io
} // namespace broccoli
