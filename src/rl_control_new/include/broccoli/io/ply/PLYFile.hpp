/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../core/string.hpp"
#include "../encoding.hpp"
#include "../filesystem.hpp"
#include "PLYElementType.hpp"
#include "PLYFormat.hpp"
#include "PLYResult.hpp"
#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Container for data stored in the Polygon File Format (PLY)
    class PLYFile {
        // Construction
        // ------------
    public:
        //! Default constructor
        PLYFile() = default;

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        bool operator==(const PLYFile& reference) const
        {
            // Compare members
            if (m_elementTypes != reference.m_elementTypes)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        bool operator!=(const PLYFile& reference) const { return !(*this == reference); }

        // Members
        // -------
    public:
        std::vector<PLYElementType> m_elementTypes; //!< List of element types (contains complete data structure)

        // Input and output
        // ----------------
    public:
        //! Encodes the PLY data structure to the given stream (ascii or binary)
        /*!
         * \param [out] stream Stream to append encoded PLY structure to
         * \param [in] format The format to be used
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool encode(encoding::CharacterStream& stream, const PLYFormat::Type& format, PLYResult* const result = nullptr) const
        {
            // Pre-allocate memory
            stream.reserve(stream.size() + streamSizeUpperBound(format));

            // Encode generic header
            encoding::encode(stream, "ply\nformat ");
            encoding::encode(stream, PLYFormat::toString(format));
            encoding::encode(stream, " 1.0\n");

            // Encode header of element types
            for (size_t i = 0; i < m_elementTypes.size(); i++)
                m_elementTypes[i].encodeHeader(stream);

            // Close header
            encoding::encode(stream, "end_header\n");

            // Encode buffer for each element type
            for (size_t i = 0; i < m_elementTypes.size(); i++) {
                if (m_elementTypes[i].encodeBuffer(stream, format, result) == false) {
                    assert(false);
                    return false;
                }

                // Line-break for each element type buffer in ascii-mode
                if (i < m_elementTypes.size() - 1 && format == PLYFormat::Type::ASCII)
                    encoding::encode(stream, '\n');
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return true;
        }

        //! Writes the buffered data to a `.ply` file (ascii or binary)
        /*!
        * \param [in] filePath Destination file path
        * \param [in] format The format to be used
         * \param [in] compression Specifies compression options
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
        * \return `true` on success, `false` otherwise
        */
        bool writeFile(const std::string& filePath, const PLYFormat::Type& format, const compression::CompressionOptions& compression, PLYResult* const result = nullptr) const
        {
            // Initialize stream
            encoding::CharacterStream stream;

            // Encode data to buffer
            if (encode(stream, format, result) == false)
                return false;

            // Write file
            if (filesystem::writeFile(filePath, stream, compression) == false) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_FILE_OPEN;
                assert(false);
                return false;
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return true;
        }

        //! Writes the buffered data to a `.ply` file (ascii or binary) (automatically detects compression options)
        /*!
        * \param [in] filePath Destination file path
        * \param [in] format The format to be used
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
        * \return `true` on success, `false` otherwise
        */
        bool writeFile(const std::string& filePath, const PLYFormat::Type& format, PLYResult* const result = nullptr) const { return writeFile(filePath, format, filesystem::filePathOrNameToCompressionOptions(filePath), result); }

        //! Decodes the PLY data structure from the given stream
        /*!
         * \param [in] stream Stream containing the complete PLY structure (ascii or binary)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool decode(const encoding::CharacterStream& stream, PLYResult* const result = nullptr)
        {
            // Clear buffer
            m_elementTypes.clear();

            // Extract header
            // --------------
            size_t headerLength = 0;
            static const encoding::CharacterStream endHeaderTag{ { 'e', 'n', 'd', '_', 'h', 'e', 'a', 'd', 'e', 'r', '\n' } };
            for (size_t i = 0; (int)i < ((int)stream.size() - (int)endHeaderTag.size()) + 1; i++) {
                bool endHeaderTagFound = true;
                for (size_t j = 0; j < endHeaderTag.size(); j++) {
                    if (stream[i + j] != endHeaderTag[j]) {
                        endHeaderTagFound = false;
                        break;
                    }
                }
                if (endHeaderTagFound == true) {
                    headerLength = i + endHeaderTag.size();
                    break;
                }
            }
            if (headerLength == 0 || headerLength > stream.size()) {
                // Invalid header
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_HEADER_END;
                assert(false);
                return false;
            }

            // Decode header to ascii string
            const std::string fileHeader(stream.begin(), stream.begin() + headerLength);

            // Check file type
            if (core::stringStartsWith(fileHeader, "ply") == false) {
                // Invalid header
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_HEADER_PLY;
                assert(false);
                return false;
            }

            // Parse header lines
            // ------------------
            PLYFormat::Type fileFormat = PLYFormat::Type::UNKNOWN; // Parsed file format
            std::string fileFormatVersion = "";
            bool elementTypeActive = false; // Indicates, if the element type specification is active
            int64_t elementTypeStartLineIndex = -1; // Index of starting line of active element specification
            auto headerLines = core::stringSplit(fileHeader, '\n', true);
            m_elementTypes.reserve(headerLines.size());
            for (size_t i = 0; i < headerLines.size(); i++) {
                // Skip first line and comments
                if (core::stringStartsWith(headerLines[i], "ply") || core::stringStartsWith(headerLines[i], "comment")) {
                    // Do nothing...
                } else {
                    // Parse format
                    if (core::stringStartsWith(headerLines[i], "format ")) {
                        auto parts = core::stringSplit(headerLines[i], ' ', true);
                        if (parts.size() >= 2) {
                            fileFormat = PLYFormat::fromString(parts[1]);
                            if (parts.size() >= 3)
                                fileFormatVersion = parts[2];
                        }
                    }
                    // Parse element
                    else if (core::stringStartsWith(headerLines[i], "element ") || core::stringStartsWith(headerLines[i], "end_header")) {
                        // Stop previous element type
                        if (elementTypeActive == true) {
                            PLYElementType newElementType;
                            const std::vector<std::string> elementHeaderLines(headerLines.begin() + elementTypeStartLineIndex, headerLines.begin() + i);
                            if (newElementType.decodeHeader(elementHeaderLines, result) == false) {
                                assert(false);
                                return false;
                            }
                            m_elementTypes.push_back(newElementType);
                            elementTypeActive = false;
                        }

                        // Start new element type
                        if (core::stringStartsWith(headerLines[i], "element ")) {
                            elementTypeActive = true;
                            elementTypeStartLineIndex = i;
                        } else
                            break; // Otherwise -> end of header
                    }
                }
            }

            // Check format
            if (fileFormat == PLYFormat::Type::UNKNOWN) {
                // Could not identify ply format
                if (result != nullptr)
                    *result = PLYResult::ERROR_INVALID_HEADER_FORMAT;
                assert(false);
                return false;
            }

            // Decode buffer
            // -------------
            // Decode buffer data for all element types
            encoding::CharacterStreamSize currentBufferIndex = headerLength;
            for (size_t i = 0; i < m_elementTypes.size(); i++) {
                // Decode buffer
                encoding::CharacterStreamSize decodedElements = m_elementTypes[i].decodeBuffer(stream, currentBufferIndex, fileFormat, result);
                if (decodedElements == 0) {
                    assert(false);
                    return false;
                }

                // Switch to start of next property type
                currentBufferIndex += decodedElements;
            }

            // Success
            if (result != nullptr)
                *result = PLYResult::SUCCESS;
            return true;
        }

        //! Reads and parses the given `.ply` file
        /*!
         * \param [in] filePath Path to the `.ply` file (file may be in ascii or binary format)
         * \param [in] compression Specifies compression options
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool readFile(const std::string& filePath, const compression::DecompressionOptions& compression, PLYResult* const result = nullptr)
        {
            // Abort, if file does not exist
            if (filesystem::fileExists(filePath) == false) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_FILE_NOEXIST;
                assert(false);
                return false;
            }

            // Read file
            encoding::CharacterStream stream;
            if (filesystem::readFile(filePath, stream, compression) == false) {
                if (result != nullptr)
                    *result = PLYResult::ERROR_FILE_OPEN;
                assert(false);
                return false;
            }

            // Decode buffer
            return decode(stream, result);
        }

        //! Reads and parses the given `.ply` file (automatically detects compression options)
        /*!
         * \param [in] filePath Path to the `.ply` file (file may be in ascii or binary format)
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool readFile(const std::string& filePath, PLYResult* const result = nullptr) { return readFile(filePath, filesystem::filePathOrNameToCompressionOptions(filePath), result); }

        // Helpers
        // -------
    public:
        //! Helper function to get the index in \ref m_elementTypes by a given element type name
        /*!
         * \param [in] elementTypeName Name of the element type we would like to get the index of
         * \return Index in \ref m_elementTypes or -1 if element type could not be found
         */
        int64_t getElementTypeIndexByName(const std::string& elementTypeName) const
        {
            // Run through all types and compare the names
            for (size_t i = 0; i < m_elementTypes.size(); i++)
                if (m_elementTypes[i].m_name == elementTypeName)
                    return i;

            // Name not found
            return -1;
        }

    protected:
        //! Computes occupied stream size for encoded scalar type
        encoding::CharacterStreamSize streamSizeScalarType(const PLYScalar::Type& scalarType) const
        {
            if (scalarType == PLYScalar::Type::INT)
                return 3; // "int"
            else if (scalarType == PLYScalar::Type::CHAR || scalarType == PLYScalar::Type::UINT)
                return 4; // "char", "uint"
            else if (scalarType == PLYScalar::Type::UCHAR || scalarType == PLYScalar::Type::SHORT || scalarType == PLYScalar::Type::FLOAT)
                return 5; // "uchar", "short", "float"
            else
                return 6; // "ushort", "double"
        }

        //! Computes upper bound for occupied stream size for encoded scalar value (ascii format only)
        encoding::CharacterStreamSize streamSizeUpperBoundScalarValueAscii(const PLYScalar::Type& scalarType) const
        {
            if (scalarType == PLYScalar::Type::UCHAR)
                return 3; // "255"
            else if (scalarType == PLYScalar::Type::CHAR)
                return 4; // "-127"
            else if (scalarType == PLYScalar::Type::USHORT)
                return 5; // "65535"
            else if (scalarType == PLYScalar::Type::SHORT)
                return 6; // "-32767"
            else if (scalarType == PLYScalar::Type::UINT)
                return 10; // "4294967295"
            else if (scalarType == PLYScalar::Type::INT)
                return 11; // "-2147483647"
            else if (scalarType == PLYScalar::Type::FLOAT)
                return 14; // "-1.2300000e+00"
            else
                return 23; // "+2.3399999999999999e+00"
        }

        //! Computes (upper bound for) occupied stream size for encoded file (used to pre-allocate memory)
        /*!
         * \note For ascii format an upper bound is computed. For the binary formats the returned value is **exact**.
         *
         * \param [in] format The format to use for encoding
         * \return Upper bound for (ascii format) or exact (binary format) stream size
         */
        encoding::CharacterStreamSize streamSizeUpperBound(const PLYFormat::Type& format) const
        {
            encoding::CharacterStreamSize returnValue = 0;

            // Estimate size of header
            // -----------------------
            returnValue += 4; // "ply\n"
            returnValue += 7; // "format "
            if (format == PLYFormat::Type::ASCII)
                returnValue += 5; // "ascii"
            else if (format == PLYFormat::Type::BINARY_LITTLE_ENDIAN)
                returnValue += 20; // "binary_little_endian"
            else
                returnValue += 17; // "binary_big_endian"
            returnValue += 5; // " 1.0\n"
            for (size_t i = 0; i < m_elementTypes.size(); i++) {
                // Element specification
                returnValue += 8; // "element "
                returnValue += m_elementTypes[i].m_name.size();
                returnValue += 1; // " "
                if (m_elementTypes[i].m_elementCount < 10)
                    returnValue += 1; // element count (one digit)
                else
                    returnValue += (int)ceil(log10(m_elementTypes[i].m_elementCount)); // element count (multiple digits)
                returnValue += 1; // "\n"

                // Property specification
                for (size_t j = 0; j < m_elementTypes[i].m_propertyTypes.size(); j++) {
                    returnValue += 9; // "property "
                    if (m_elementTypes[i].m_propertyTypes[j].m_isListProperty == true) {
                        returnValue += 5; // "list "
                        returnValue += streamSizeScalarType(m_elementTypes[i].m_propertyTypes[j].m_counterType);
                        returnValue += 1; // " "
                    }
                    returnValue += streamSizeScalarType(m_elementTypes[i].m_propertyTypes[j].m_valueType);
                    returnValue += 1; // " "
                    returnValue += m_elementTypes[i].m_propertyTypes[j].m_name.size();
                    returnValue += 1; // "\n"
                }
            }
            returnValue += 11; // "end_header\n"

            // Estimate size of buffer
            // -----------------------
            for (size_t i = 0; i < m_elementTypes.size(); i++) { // i = element type index
                if (m_elementTypes[i].m_elementCount == m_elementTypes[i].m_data.size()) {
                    for (size_t j = 0; j < m_elementTypes[i].m_data.size(); j++) { // j = element index
                        if (m_elementTypes[i].m_propertyTypes.size() == m_elementTypes[i].m_data[j].size()) {
                            for (size_t k = 0; k < m_elementTypes[i].m_data[j].size(); k++) { // k = property index
                                // Grab helpers
                                const PLYPropertyType& propertyType = m_elementTypes[i].m_propertyTypes[k];
                                const PLYPropertyValue& propertyValue = m_elementTypes[i].m_data[j][k];
                                const auto bytesInBuffer = propertyValue.bytesInBuffer();
                                const auto bytesPerValue = PLYScalar::byteCount(propertyType.m_valueType);
                                size_t listElements = 0;
                                if (propertyType.m_isListProperty == true && bytesPerValue > 0)
                                    listElements = bytesInBuffer / bytesPerValue;
                                else
                                    listElements = 1;

                                // Space for counter
                                if (propertyType.m_isListProperty == true) {
                                    if (format == PLYFormat::Type::ASCII) {
                                        returnValue += streamSizeUpperBoundScalarValueAscii(propertyType.m_counterType);
                                        returnValue += 1; // " " or "\n"
                                    } else // Binary
                                        returnValue += PLYScalar::byteCount(propertyType.m_counterType);
                                }

                                // Space for value(s)
                                if (format == PLYFormat::Type::ASCII) {
                                    returnValue += streamSizeUpperBoundScalarValueAscii(propertyType.m_valueType) * listElements;
                                    returnValue += 1; // " " or "\n"
                                } else // Binary
                                    returnValue += PLYScalar::byteCount(propertyType.m_valueType) * listElements;
                            }
                        }
                    }
                }
            }

            // Pass back estimated size
            return returnValue;
        }
    };
    //! \}
} // namespace io
} // namespace broccoli
