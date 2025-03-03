/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/string.hpp"
#include "../encoding.hpp"
#include "PLYFormat.hpp"
#include "PLYPropertyType.hpp"
#include "PLYPropertyValue.hpp"
#include "PLYResult.hpp"
#include <stdint.h>
#include <string>
#include <vector>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Abstraction layer for element types of the Polygon File Format (PLY)
    class PLYElementType {
    public:
        //! Constructor
        PLYElementType()
            : m_name("")
            , m_elementCount(0)
        {
        }

        //! Comparison operator: **equality**
        bool operator==(const PLYElementType& reference) const
        {
            // Compare members
            if (m_name != reference.m_name)
                return false;
            if (m_elementCount != reference.m_elementCount)
                return false;
            if (m_propertyTypes != reference.m_propertyTypes)
                return false;
            if (m_data != reference.m_data)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        bool operator!=(const PLYElementType& reference) const { return !(*this == reference); }

        // Members
        std::string m_name; //!< Name of the element type as string
        uint64_t m_elementCount; //!< Count of instances of **this** element type in the buffer
        std::vector<PLYPropertyType> m_propertyTypes; //!< Definition of related property types belonging to **this** element type
        std::vector<std::vector<PLYPropertyValue>> m_data; //!< Data container ("buffer") for related property values as multi-dimensional vector [element-index][property-index]

        // Generic
        // -------
        //! Checks, if the element type buffer \ref m_data matches the element type specification
        /*!
         * \param [out] result Pointer to flag in which the result should be stored
         * \return `true`, if the element type buffer matches the element type specification, `false` otherwise.
         */
        bool bufferMatchesSpecification(PLYResult* const result = nullptr) const
        {
            // Check, if buffer size matches element count
            if (m_data.size() != m_elementCount) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_BUFFER_ELEMENTCOUNT;
                return false;
            }

            // Pass through elements
            for (size_t i = 0; i < m_data.size(); i++) {
                // Check, if property count matches specification
                if (m_data[i].size() != m_propertyTypes.size()) {
                    if (result != nullptr)
                        *result = PLYResult::ERROR_INVALID_BUFFER_PROPERTYCOUNT;
                    return false;
                }

                // Pass through properties
                for (size_t j = 0; j < m_data[i].size(); j++) {
                    const size_t bytesInBuffer = m_data[i][j].bytesInBuffer();
                    const size_t bytesPerValue = PLYScalar::byteCount(m_propertyTypes[j].m_valueType);
                    if (bytesPerValue == 0) {
                        if (result != nullptr)
                            *result = PLYResult::ERROR_INVALID_SCALARTYPE;
                        return false;
                    }
                    if (bytesInBuffer % bytesPerValue != 0) {
                        if (result != nullptr)
                            *result = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                        return false;
                    }
                    if (m_propertyTypes[j].m_isListProperty == false) {
                        if (bytesInBuffer != bytesPerValue) {
                            if (result != nullptr)
                                *result = PLYResult::ERROR_INVALID_BUFFER_PROPERTYVALUE;
                            return false;
                        }
                    }
                }
            }

            // Otherwise -> valid
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return true;
        }

        // De- and encoding of header
        // --------------------------
        //! Encodes the header of this element type to the given stream
        /*!
         * \param [out] stream Stream to append encoded header to
         */
        void encodeHeader(encoding::CharacterStream& stream) const
        {
            // Encode element type header
            encoding::encode(stream, "element " + m_name + " ");
            encoding::encode(stream, (uint64_t)m_elementCount);
            encoding::encode(stream, (char)'\n');

            // Encode property type headers
            for (size_t i = 0; i < m_propertyTypes.size(); i++) {
                m_propertyTypes[i].encodeHeader(stream);
                encoding::encode(stream, (char)'\n');
            }
        }

        //! Decodes given header lines to obtain the element type specification
        /*!
         * \param [in] headerLines Lines in header related to this element type
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool decodeHeader(const std::vector<std::string>& headerLines, PLYResult* const result = nullptr)
        {
            // Reset buffers
            m_propertyTypes.clear();
            m_propertyTypes.reserve(headerLines.size() - 1);

            // Iterate through lines
            bool foundElementTypeHeader = false; // Flag indicating, if the header of this element type was found
            for (size_t i = 0; i < headerLines.size(); i++) {
                // Parse element type name and element count
                if (core::stringStartsWith(headerLines[i], "element ")) {
                    foundElementTypeHeader = true;
                    auto parts = core::stringSplit(headerLines[i], ' ', true);
                    if (parts.size() != 3) {
                        // Could not decode element count
                        if (result != nullptr)
                            *result = PLYResult::ERROR_INVALID_HEADER_ELEMENTTYPE;
                        assert(false);
                        return false;
                    }
                    m_name = parts[1];
                    const encoding::CharacterStream elementCountStream(parts[2].begin(), parts[2].end());
                    if (encoding::decode(elementCountStream, 0, elementCountStream.size(), m_elementCount) == 0) {
                        // Could not decode element count
                        if (result != nullptr)
                            *result = PLYResult::ERROR_INVALID_HEADER_ELEMENTTYPE;
                        assert(false);
                        return false;
                    }
                }
                // Parse properties
                else if (core::stringStartsWith(headerLines[i], "property ")) {
                    PLYPropertyType newPropertyType;
                    if (newPropertyType.decodeHeader(headerLines[i], result) == false) {
                        assert(false);
                        return false;
                    }
                    m_propertyTypes.push_back(newPropertyType);
                }
            }

            // Check, if at least element header was found
            if (foundElementTypeHeader == false) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_HEADER_ELEMENTTYPE;
                assert(false);
                return false;
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return true;
        }

        // De- and encoding of buffer
        // --------------------------
        //! Encodes the buffer of this element type to the given stream (ascii or binary)
        /*!
         * \param [out] stream Stream to append encoded buffer to
         * \param [in] format The format to be used
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of added elements to stream or 0 in case of an error
         */
        encoding::CharacterStreamSize encodeBuffer(encoding::CharacterStream& stream, const PLYFormat::Type& format, PLYResult* const result = nullptr) const
        {
            encoding::CharacterStreamSize addedCharacters = 0;

            // Check, if buffer is valid
            if (bufferMatchesSpecification(result) == false) {
                assert(false);
                return 0;
            }

            // Iterate through elements
            for (size_t i = 0; i < m_elementCount; i++) {
                // Iterate through properties
                for (size_t j = 0; j < m_propertyTypes.size(); j++) {
                    encoding::CharacterStreamSize addedPropertyCharacters = encodePropertyBuffer(stream, format, m_propertyTypes[j], m_data[i][j], result);
                    addedCharacters += addedPropertyCharacters;
                    if (addedPropertyCharacters == 0) {
                        assert(false);
                        return 0;
                    }

                    // Whitespace after each property buffer in ascii-mode
                    if (j < m_propertyTypes.size() - 1 && format == PLYFormat::Type::ASCII)
                        addedCharacters += encoding::encode(stream, ' ');
                }

                // Line-break for each element buffer in ascii-mode
                if (i < m_elementCount - 1 && format == PLYFormat::Type::ASCII)
                    addedCharacters += encoding::encode(stream, '\n');
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return addedCharacters;
        }

        //! Decodes the buffer of this element type from the given stream (ascii or binary)
        /*!
         * \param [in] stream Stream to decode the buffer from
         * \param [in] index Starting index in the given stream
         * \param [in] format The used format
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of decoded elements in the stream or 0 in case of an error
         */
        encoding::CharacterStreamSize decodeBuffer(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& index, const PLYFormat::Type& format, PLYResult* const result = nullptr)
        {
            // Prepare data buffer
            m_data.resize(m_elementCount);
            for (size_t i = 0; i < m_data.size(); i++)
                m_data[i].resize(m_propertyTypes.size());

            // Iterate through elements
            encoding::CharacterStreamSize currentIndex = index;
            for (size_t i = 0; i < m_elementCount; i++) {
                // Iterate through properties
                for (size_t j = 0; j < m_propertyTypes.size(); j++) {
                    // Decode property
                    auto decodedElementsProperty = decodePropertyBuffer(stream, currentIndex, format, m_propertyTypes[j], m_data[i][j], result);
                    if (decodedElementsProperty == 0) {
                        assert(false);
                        return 0;
                    }

                    // Jump to next property
                    currentIndex += decodedElementsProperty;
                }
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return currentIndex - index;
        }

        // Helpers
        // -------
        //! Helper function to get the index in \ref m_propertyTypes by a given property type name
        /*!
         * \param [in] propertyTypeName Name of the property type we would like to get the index of
         * \return Index in \ref m_propertyTypes or -1 if property type could not be found
         */
        int64_t getPropertyTypeIndexByName(const std::string& propertyTypeName) const
        {
            // Run through all types and compare the names
            for (size_t i = 0; i < m_propertyTypes.size(); i++)
                if (m_propertyTypes[i].m_name == propertyTypeName)
                    return i;

            // Name not found
            return -1;
        }

    protected:
        //! \copydoc encodePropertyBuffer()
        /*! \remark The datatypes of the counter and values are explicitly given by the template parameters. */
        template <typename CounterType, typename ValueType>
        encoding::CharacterStreamSize encodePropertyBufferTypedCounterTypedValue(encoding::CharacterStream& stream, const PLYFormat::Type& format, const PLYPropertyType& propertyType, const PLYPropertyValue& propertyValue, PLYResult* const result = nullptr) const
        {
            if (propertyType.m_isListProperty == false)
                return propertyValue.encodeScalar<ValueType>(stream, format, result);
            else
                return propertyValue.encodeList<CounterType, ValueType>(stream, format, result);
        }

        //! \copydoc encodePropertyBuffer()
        /*! \remark The datatype of the counter is explicitly given by the template parameter. */
        template <typename CounterType>
        encoding::CharacterStreamSize encodePropertyBufferTypedCounter(encoding::CharacterStream& stream, const PLYFormat::Type& format, const PLYPropertyType& propertyType, const PLYPropertyValue& propertyValue, PLYResult* const result = nullptr) const
        {
            // Check datatype of counter
            if (propertyType.m_valueType == PLYScalar::Type::CHAR)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, CHAR_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::UCHAR)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, UCHAR_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::SHORT)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, SHORT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::USHORT)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, USHORT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::INT)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, INT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::UINT)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, UINT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::FLOAT)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, FLOAT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::DOUBLE)
                return encodePropertyBufferTypedCounterTypedValue<CounterType, DOUBLE_t>(stream, format, propertyType, propertyValue, result);
            else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_SCALARTYPE;
                assert(false);
                return 0;
            }
        }

        //! Encodes the buffer data of the given property to the given stream
        /*!
         * \param [out] stream Stream to append encoded property buffer to
         * \param [in] format The `.ply` format to be used
         * \param [in] propertyType Property type specification
         * \param [in] propertyValue Buffered data of the property
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of added elements to stream or 0 in case of an error
         */
        encoding::CharacterStreamSize encodePropertyBuffer(encoding::CharacterStream& stream, const PLYFormat::Type& format, const PLYPropertyType& propertyType, const PLYPropertyValue& propertyValue, PLYResult* const result = nullptr) const
        {
            // Check datatype of counter
            if (propertyType.m_counterType == PLYScalar::Type::CHAR)
                return encodePropertyBufferTypedCounter<CHAR_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::UCHAR)
                return encodePropertyBufferTypedCounter<UCHAR_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::SHORT)
                return encodePropertyBufferTypedCounter<SHORT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::USHORT)
                return encodePropertyBufferTypedCounter<USHORT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::INT)
                return encodePropertyBufferTypedCounter<INT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::UINT)
                return encodePropertyBufferTypedCounter<UINT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::FLOAT)
                return encodePropertyBufferTypedCounter<FLOAT_t>(stream, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::DOUBLE)
                return encodePropertyBufferTypedCounter<DOUBLE_t>(stream, format, propertyType, propertyValue, result);
            else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_SCALARTYPE;
                assert(false);
                return 0;
            }
        }

        //! \copydoc decodePropertyBuffer()
        /*! \remark The datatypes of the counter and values are explicitly given by the template parameters. */
        template <typename CounterType, typename ValueType>
        encoding::CharacterStreamSize decodePropertyBufferTypedCounterTypedValue(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& index, const PLYFormat::Type& format, const PLYPropertyType& propertyType, PLYPropertyValue& propertyValue, PLYResult* const result = nullptr)
        {
            if (propertyType.m_isListProperty == false)
                return propertyValue.decodeScalar<ValueType>(stream, index, format, result);
            else
                return propertyValue.decodeList<CounterType, ValueType>(stream, index, format, result);
        }

        //! \copydoc decodePropertyBuffer()
        /*! \remark The datatype of the counter is explicitly given by the template parameter. */
        template <typename CounterType>
        encoding::CharacterStreamSize decodePropertyBufferTypedCounter(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& index, const PLYFormat::Type& format, const PLYPropertyType& propertyType, PLYPropertyValue& propertyValue, PLYResult* const result = nullptr)
        {
            // Check datatype of counter
            if (propertyType.m_valueType == PLYScalar::Type::CHAR)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, CHAR_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::UCHAR)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, UCHAR_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::SHORT)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, SHORT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::USHORT)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, USHORT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::INT)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, INT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::UINT)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, UINT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::FLOAT)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, FLOAT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_valueType == PLYScalar::Type::DOUBLE)
                return decodePropertyBufferTypedCounterTypedValue<CounterType, DOUBLE_t>(stream, index, format, propertyType, propertyValue, result);
            else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_SCALARTYPE;
                assert(false);
                return 0;
            }
        }

        //! Decodes the buffer data of the given property from the given stream
        /*!
         * \param [in] stream Stream to decode the buffer from
         * \param [in] index Starting index in the given stream
         * \param [in] format The used format
         * \param [in] propertyType Property type specification
         * \param [out] propertyValue Reference to property buffer
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return Count of decoded elements in the stream or 0 in case of an error
         */
        encoding::CharacterStreamSize decodePropertyBuffer(const encoding::CharacterStream& stream, const encoding::CharacterStreamSize& index, const PLYFormat::Type& format, const PLYPropertyType& propertyType, PLYPropertyValue& propertyValue, PLYResult* const result = nullptr)
        {
            // Check datatype of counter
            if (propertyType.m_counterType == PLYScalar::Type::CHAR)
                return decodePropertyBufferTypedCounter<CHAR_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::UCHAR)
                return decodePropertyBufferTypedCounter<UCHAR_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::SHORT)
                return decodePropertyBufferTypedCounter<SHORT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::USHORT)
                return decodePropertyBufferTypedCounter<USHORT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::INT)
                return decodePropertyBufferTypedCounter<INT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::UINT)
                return decodePropertyBufferTypedCounter<UINT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::FLOAT)
                return decodePropertyBufferTypedCounter<FLOAT_t>(stream, index, format, propertyType, propertyValue, result);
            else if (propertyType.m_counterType == PLYScalar::Type::DOUBLE)
                return decodePropertyBufferTypedCounter<DOUBLE_t>(stream, index, format, propertyType, propertyValue, result);
            else {
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_SCALARTYPE;
                assert(false);
                return 0;
            }
        }
    };
    //! \}
} // namespace io
} // namespace broccoli
