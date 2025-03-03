/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../filesystem.hpp"
#include "PNMFileData.hpp"

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_pnm
     * \{
     */

    //! Container for data stored in the Portable Anymap (PNM) format
    /*!
     * This represents the family of
     *   * Portable Bitmap (PBM) (.pbm, P1, P4),
     *   * Portable Graymap (PGM) (.pgm, P2, P5), and
     *   * Portable Pixmap (PPM) (.ppm, P3, P6)
     * file formats (both, ASCII and binary versions).
     *
     * Restrictions
     * ------------
     * The official specifications
     *   * for Portable Bitmap (PBM): http://netpbm.sourceforge.net/doc/pbm.html
     *   * for Portable Graymap (PGM): http://netpbm.sourceforge.net/doc/pgm.html
     *   * for Portable Pixmap (PPM): http://netpbm.sourceforge.net/doc/ppm.html
     * allow storing more than one image in a file (binary version only). This is not supported by this abstraction layer.
     *
     * Coordinate system
     * -----------------
     * See \ref PNMFileData for details on the used coordinate system.
     */
    class PNMFile : public PNMFileData {
        // Construction
        // ------------
    public:
        //! Default constructor
        explicit PNMFile(const Type& type = Type::UNKNOWN)
            : PNMFileData(type)
        {
        }

        //! Typed constructor (with size)
        PNMFile(const Type& type, const size_t& rows, const size_t& columns)
            : PNMFileData(type, rows, columns)
        {
        }

        // Operators
        // ---------
    public:
        //! Comparison operator: **equality**
        inline bool operator==(const PNMFile& reference) const
        {
            // Compare base class
            if (PNMFileData::operator==(reference) == false)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        inline bool operator!=(const PNMFile& reference) const { return !(*this == reference); }

        // Input and output
        // ----------------
    public:
        //! Encodes the PNM data structure to the given stream (ascii or binary)
        /*!
         * \param [out] stream Stream to append encoded PNM structure to
         * \param [in] encoding The encoding to be used
         * \return `true` on success, `false` otherwise
         */
        bool encode(encoding::CharacterStream& stream, const PNMFileEncoding::Type& encoding) const
        {
            // Abort, if encoding or type is invalid
            if (encoding == PNMFileEncoding::Type::UNKNOWN || encoding == PNMFileEncoding::Type::TYPE_COUNT //
                || m_type == PNMFileData::Type::UNKNOWN || m_type == PNMFileData::Type::TYPE_COUNT) {
                assert(false);
                return false;
            }

            // Pre-allocate memory
            stream.reserve(stream.size() + streamSizeUpperBound(encoding));

            // Encode magic number
            if (encoding == PNMFileEncoding::Type::ASCII) {
                if (m_type == PNMFileData::Type::PBM)
                    encoding::encode(stream, "P1\n");
                else if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PGM_16BIT)
                    encoding::encode(stream, "P2\n");
                else
                    encoding::encode(stream, "P3\n");
            } else {
                if (m_type == PNMFileData::Type::PBM)
                    encoding::encode(stream, "P4\n");
                else if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PGM_16BIT)
                    encoding::encode(stream, "P5\n");
                else
                    encoding::encode(stream, "P6\n");
            }

            // Encode rows and columns
            encoding::encode(stream, width());
            encoding::encode(stream, (char)' ');
            encoding::encode(stream, height());
            encoding::encode(stream, (char)'\n');

            // Encode max. sample value
            if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PPM_8BIT)
                encoding::encode(stream, "255\n");
            else if (m_type == PNMFileData::Type::PGM_16BIT || m_type == PNMFileData::Type::PPM_16BIT)
                encoding::encode(stream, "65535\n");

            // Encode buffer
            encodeBuffer(stream, encoding);

            // Success
            return true;
        }

        //! Writes the buffered data to a PNM file (ascii or binary)
        /*!
        * \param [in] filePath Destination file path
        * \param [in] encoding The encoding to be used
         * \param [in] compression Specifies compression options
        * \return `true` on success, `false` otherwise
        */
        bool writeFile(const std::string& filePath, const PNMFileEncoding::Type& encoding, const compression::CompressionOptions& compression) const
        {
            encoding::CharacterStream stream;
            if (encode(stream, encoding) == false)
                return false;
            return filesystem::writeFile(filePath, stream, compression);
        }

        //! Writes the buffered data to a PNM file (ascii or binary) (automatically detects compression options)
        /*!
        * \param [in] filePath Destination file path
        * \param [in] encoding The encoding to be used
        * \return `true` on success, `false` otherwise
        */
        bool writeFile(const std::string& filePath, const PNMFileEncoding::Type& encoding) const { return writeFile(filePath, encoding, filesystem::filePathOrNameToCompressionOptions(filePath)); }

        //! Decodes the PNM data structure from the given stream
        /*!
         * \param [in] stream Stream containing the complete PNM structure (ascii or binary)
         * \return `true` on success, `false` otherwise
         */
        bool decode(const encoding::CharacterStream& stream)
        {
            // Clear data structure
            clear();

            // Setup helpers
            encoding::CharacterStreamSize index = 0; // Current position in stream

            // Extract header
            // --------------
            // Check minimum header length
            if (stream.size() < index + 7)
                return false;

            // Extract magic number
            if (stream[index] != (char)'P' || stream[index + 1] < (char)'1' || stream[index + 1] > (char)'6')
                return false;
            const uint8_t magicNumber = stream[index + 1] - (char)'0';
            index += 2;

            // Detect encoding
            const PNMFileEncoding::Type encoding = (magicNumber < 4) ? PNMFileEncoding::Type::ASCII : PNMFileEncoding::Type::BINARY;

            // Extract width
            uint64_t width = 0;
            index = extractNextUnsignedInteger(stream, index, width);
            if (index >= stream.size())
                return false;
            index += 1;

            // Extract height
            uint64_t height = 0;
            index = extractNextUnsignedInteger(stream, index, height);
            if (index >= stream.size())
                return false;
            index += 1;

            // Extract data type
            if (magicNumber == 1 || magicNumber == 4) {
                setType(Type::PBM);
                m_dataPBM.resize(height, width);
            } else {
                // Extract bit-depth
                uint64_t maximumSampleValue = 0;
                index = extractNextUnsignedInteger(stream, index, maximumSampleValue);
                if (index >= stream.size())
                    return false;
                index += 1;
                if (maximumSampleValue < 256) {
                    // 8-bit
                    if (magicNumber == 2 || magicNumber == 5) {
                        setType(Type::PGM_8BIT);
                        m_dataPGM8.resize(height, width);
                    } else {
                        setType(Type::PPM_8BIT);
                        m_dataPPM8.resize(height, width);
                    }
                } else {
                    // 16-bit
                    if (magicNumber == 2 || magicNumber == 5) {
                        setType(Type::PGM_16BIT);
                        m_dataPGM16.resize(height, width);
                    } else {
                        setType(Type::PPM_16BIT);
                        m_dataPPM16.resize(height, width);
                    }
                }
            }

            // Detect start of buffer
            bool inComment = false;
            for (size_t i = index; i < stream.size(); i++) {
                if (inComment == true) {
                    if (stream[i] == (char)'\r' || stream[i] == (char)'\n')
                        inComment = false;
                } else {
                    if (stream[i] == (char)'#') {
                        inComment = true;
                    } else {
                        if (isspace(stream[i])) {
                            index = i + 1;
                            break;
                        } else
                            return false; // Invalid character
                    }
                }
            }

            // Decode buffer
            // -------------
            return decodeBuffer(stream, index, encoding);
        }

        //! Reads and parses the given PNM file
        /*!
         * \param [in] filePath Path to the PNM file (file may be in ascii or binary format)
         * \param [in] compression Specifies compression options
         * \return `true` on success, `false` otherwise
         */
        bool readFile(const std::string& filePath, const compression::DecompressionOptions& compression)
        {
            encoding::CharacterStream stream;
            if (filesystem::readFile(filePath, stream, compression) == false)
                return false;
            return decode(stream);
        }

        //! Reads and parses the given PNM file (automatically detects compression options)
        /*!
         * \param [in] filePath Path to the PNM file (file may be in ascii or binary format)
         * \return `true` on success, `false` otherwise
         */
        bool readFile(const std::string& filePath) { return readFile(filePath, filesystem::filePathOrNameToCompressionOptions(filePath)); }

        // Helpers
        // -------
    public:
        //! Computes (upper bound for) occupied stream size for encoded file (used to pre-allocate memory)
        /*!
         * \note For ascii format an upper bound is computed. For the binary format the returned value is **exact**.
         *
         * \param [in] encoding The encoding to be used
         * \return Upper bound for (ascii) or exact (binary) stream size
         */
        encoding::CharacterStreamSize streamSizeUpperBound(const PNMFileEncoding::Type& encoding) const
        {
            encoding::CharacterStreamSize returnValue = 0;

            // Estimate size of header
            // -----------------------
            returnValue += 3; // "PX\n"
            const std::array<size_t, 2>& currentSize = size();
            const size_t& rows = currentSize[0];
            const size_t& columns = currentSize[1];
            returnValue += getCountOfDigits(rows) + 1 + getCountOfDigits(columns) + 1; // "RESX RESY\n"
            if (m_type == PNMFileData::Type::PGM_8BIT || m_type == PNMFileData::Type::PPM_8BIT)
                returnValue += 4; // "255\n"
            if (m_type == PNMFileData::Type::PGM_16BIT || m_type == PNMFileData::Type::PPM_16BIT)
                returnValue += 6; // "65535\n"

            // Estimate size of buffer
            // -----------------------
            returnValue += streamSizeUpperBoundBuffer(encoding);

            // Pass back estimated size
            return returnValue;
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
