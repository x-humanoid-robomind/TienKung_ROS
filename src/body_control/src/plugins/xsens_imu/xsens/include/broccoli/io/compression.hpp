/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <assert.h>
#include <stdexcept>
#include <stdint.h>
#include <vector>
#ifdef HAVE_ZLIB // is zlib library available?
#include <zlib.h>
#endif // HAVE_ZLIB

namespace broccoli {
namespace io {
    namespace compression {
        /*!
         * \addtogroup broccoli_io_compression
         * \{
         */

        // Type definitions
        typedef std::vector<uint8_t> DataStream; //!< Specifies the used type for data streams

        //! Specification of possible compression methods
        enum class Method : uint8_t {
            RAW = 0, //!< Raw deflate algorithm without any header or trailer
            ZLIB, //!< Deflate algorithm in zlib format with header and trailer
            GZIP, //!< Deflate algorithm in gzip format with header and trailer
            METHOD_COUNT //!< Total count of elements
        };

        //! Specifies options for compression
        class CompressionOptions {
            // Construction
            // ------------
        public:
            //! Specialized constructor
            /*!
             * \param [in] compressed Initializes \ref m_compressed - \copybrief m_compressed
             * \param [in] method Initializes \ref m_method - \copybrief m_method
             * \param [in] level Initializes \ref m_level - \copybrief m_level
             */
            CompressionOptions(const bool& compressed, const Method& method = Method::RAW, const int& level = -1)
            {
                setCompressed(compressed);
                setMethod(method);
                setLevel(level);
            }

            //! Option set for **no** compression
            static inline CompressionOptions noCompression() { return CompressionOptions(false); }

            //! Option set for compression using **RAW** method
            /*! \param [in] level Initializes \ref m_level - \copybrief m_level */
            static inline CompressionOptions raw(const int& level = -1) { return CompressionOptions(true, Method::RAW, level); }

            //! Option set for compression using **ZLIB** method
            /*! \param [in] level Initializes \ref m_level - \copybrief m_level */
            static inline CompressionOptions zlib(const int& level = -1) { return CompressionOptions(true, Method::ZLIB, level); }

            //! Option set for compression using **GZIP** method
            /*! \param [in] level Initializes \ref m_level - \copybrief m_level */
            static inline CompressionOptions gzip(const int& level = -1) { return CompressionOptions(true, Method::GZIP, level); }

            // Operators
            // ---------
        public:
            //! Comparison operator: **equality**
            bool operator==(const CompressionOptions& reference) const
            {
                if (m_compressed != reference.m_compressed)
                    return false;
                if (m_method != reference.m_method)
                    return false;
                if (m_level != reference.m_level)
                    return false;
                return true;
            }

            //! Comparison operator: **inequality**
            inline bool operator!=(const CompressionOptions& reference) const { return !(*this == reference); }

            // Members
            // -------
        protected:
            bool m_compressed; //!< \copybrief compressed()
            Method m_method; //!< \copybrief method()
            int m_level; //!< \copybrief level()

            // Getters
            // -------
        public:
            //! Specifies, if compression is enabled or not
            inline const bool& compressed() const { return m_compressed; }

            //! The used compression method (only relevant, if compression is enabled)
            inline const Method& method() const { return m_method; }

            //! The compression level to be used (0=no compression, 1=best speed, 9=best compression, -1=use default) (only relevant, if compression is enabled)
            inline const int& level() const { return m_level; }

            // Setters
            // -------
        public:
            //! Setter for \ref compressed()
            inline void setCompressed(const bool& newValue) { m_compressed = newValue; }

            //! Setter for \ref method()
            inline void setMethod(const Method& newValue)
            {
                assert(static_cast<uint8_t>(newValue) < static_cast<uint8_t>(Method::METHOD_COUNT));
                m_method = newValue;
            }

            //! Setter for \ref level()
            inline void setLevel(const int& level)
            {
                if (level < -1) {
                    assert(false);
                    m_level = -1;
                } else if (level > 9) {
                    assert(false);
                    m_level = 9;
                } else
                    m_level = level;
            }
        };

        //! Specifies options for decompression
        class DecompressionOptions {
            // Construction
            // ------------
        public:
            //! Constructor from related compression options
            DecompressionOptions(const CompressionOptions& compressionOptions)
                : m_compressed(compressionOptions.compressed())
                , m_raw((compressionOptions.method() == Method::RAW) ? true : false)
            {
            }

            // Operators
            // ---------
        public:
            //! Comparison operator: **equality**
            bool operator==(const DecompressionOptions& reference) const
            {
                if (m_compressed != reference.m_compressed)
                    return false;
                if (m_raw != reference.m_raw)
                    return false;
                return true;
            }

            //! Comparison operator: **inequality**
            inline bool operator!=(const DecompressionOptions& reference) const { return !(*this == reference); }

            // Members
            // -------
        protected:
            bool m_compressed = false; //!< \copybrief compressed()
            bool m_raw = true; //!< \copybrief raw()

            // Getters
            // -------
        public:
            //! Specifies, if data was compressed or not
            inline const bool& compressed() const { return m_compressed; }

            //! If `true`, the data was compressed with the method \ref Method::RAW (no header or trailer). If `false`, the header format (zlib or gzip) is automatically detected.
            inline const bool& raw() const { return m_raw; }
        };

        //! Compresses the given data stream
        /*!
         * \attention For best performance, the output buffer should be pre-allocated with an upper bound of its size (use resize() and not reserve())!
         * If the output buffer is empty (zero size), the upper bound is computed by zlib.
         *
         * \param [in] inputStart Pointer to first byte in input buffer (uncompressed data stream)
         * \param [in] inputLength The length of the input buffer (in bytes)
         * \param [out] output The compressed output stream
         * \param [in] options The used options for compression
         * \return `true` on success, `false` otherwise
         */
        static inline bool compress(const uint8_t* inputStart, const size_t& inputLength, DataStream& output, const CompressionOptions& options)
        {
            // Skip, if data buffer is empty
            if (inputLength == 0) {
                output.clear();
                return true;
            }

            // For no compression: just copy buffer
            if (options.compressed() == false) {
                output.assign(inputStart, inputStart + inputLength);
                return true;
            }

#ifdef HAVE_ZLIB
            // Select appropriate count of window bits
            int windowBits = 0;
            if (options.method() == Method::RAW)
                windowBits = -MAX_WBITS;
            else if (options.method() == Method::ZLIB)
                windowBits = MAX_WBITS;
            else if (options.method() == Method::GZIP)
                windowBits = MAX_WBITS | 16;
            else
                throw std::runtime_error("broccoli::io::compression: unknown method!");

            // Initialize compression
            z_stream stream;
            stream.next_in = (Bytef*)inputStart;
            stream.avail_in = inputLength;
            stream.zalloc = Z_NULL;
            stream.zfree = Z_NULL;
            stream.opaque = Z_NULL;
            if (deflateInit2(&stream, options.level(), Z_DEFLATED, windowBits, 9 /* <-- fastest but consumes more memory */, Z_DEFAULT_STRATEGY) != Z_OK) {
                // Something went wrong
                deflateEnd(&stream); // Free all memory
                return false;
            }

            // Compute upper bound for size of compressed data stream
            const size_t outputUpperBound = deflateBound(&stream, inputLength);

            // Check, if space in output buffer is already allocated
            if (output.size() == 0)
                output.resize(outputUpperBound); // No space allocated yet -> use upper bound
            stream.next_out = (Bytef*)&output[0];
            stream.avail_out = output.size();

            // Compress stream
            while (true) {
                // Compress as much data as possible
                const int deflateResult = deflate(&stream, Z_FINISH);
                if (deflateResult == Z_OK || deflateResult == Z_BUF_ERROR) {
                    // Not finished yet -> output buffer too small -> increase size -> repeat
                    const size_t addedBytesToOutput = output.size(); // Double the size of the output buffer
                    output.resize(output.size() + addedBytesToOutput);
                    stream.next_out = (Bytef*)&output[stream.total_out];
                    stream.avail_out += addedBytesToOutput;
                } else if (deflateResult == Z_STREAM_END) {
                    // Complete stream has been compressed -> finished
                    break;
                } else {
                    // Something went wrong
                    deflateEnd(&stream); // Free all memory
                    return false;
                }
            }

            // Resize output buffer to fit actual size
            output.resize(stream.total_out);

            // Free all memory
            if (deflateEnd(&stream) != Z_OK)
                return false;

            // Finished with success!
            return true;
#else
            throw std::runtime_error("broccoli::io::compression: zlib is unavailable!");
#endif
        }

        //! Compresses the given data stream
        /*!
         * \attention For best performance, the output buffer should be pre-allocated with an upper bound of its size (use resize() and not reserve())!
         * If the output buffer is empty (zero size), the upper bound is computed by zlib.
         *
         * \param [in] input The uncompressed input stream
         * \param [out] output The compressed output stream
         * \param [in] options The used options for compression
         * \return `true` on success, `false` otherwise
         */
        static inline bool compress(const DataStream& input, DataStream& output, const CompressionOptions& options) { return compress(input.data(), input.size(), output, options); }

        //! Decompresses the given data stream
        /*!
         * \attention For best performance, the output buffer should be pre-allocated with an upper bound of its size (use resize() and not reserve())!
         * If the output buffer is empty (zero size), the upper bound is estimated internally.
         *
         * \param [in] inputStart Pointer to first byte in input buffer (compressed data stream)
         * \param [in] inputLength The length of the input buffer (in bytes) (may hold more than the actual compressed data - trailing bytes are ignored)
         * \param [out] output The uncompressed output stream
         * \param [in] options The used options for decompression
         * \return `true` on success, `false` otherwise
         */
        static inline bool decompress(const uint8_t* inputStart, const size_t& inputLength, DataStream& output, const DecompressionOptions& options)
        {
            // Skip, if data buffer is empty
            if (inputLength == 0) {
                output.clear();
                return true;
            }

            // For no compression: just copy buffer
            if (options.compressed() == false) {
                output.assign(inputStart, inputStart + inputLength);
                return true;
            }

#ifdef HAVE_ZLIB
            // Select appropriate count of window bits
            int windowBits = 0;
            if (options.raw() == true)
                windowBits = -MAX_WBITS;
            else
                windowBits = MAX_WBITS | 32;

            // Initialize decompression
            z_stream stream;
            stream.next_in = (Bytef*)inputStart;
            stream.avail_in = inputLength;
            stream.zalloc = Z_NULL;
            stream.zfree = Z_NULL;
            stream.opaque = Z_NULL;
            if (inflateInit2(&stream, windowBits) != Z_OK) {
                // Something went wrong
                inflateEnd(&stream); // Free all memory
                return false;
            }

            // Compute upper bound for size of uncompressed data stream
            const size_t outputUpperBound = 5 * inputLength;

            // Check, if space in output buffer is already allocated
            if (output.size() == 0)
                output.resize(outputUpperBound); // No space allocated yet -> use upper bound
            stream.next_out = (Bytef*)&output[0];
            stream.avail_out = output.size();

            // Decompress stream
            while (true) {
                // Decompress as much data as possible
                const int inflateResult = inflate(&stream, Z_FINISH);
                if (inflateResult == Z_OK || inflateResult == Z_BUF_ERROR) {
                    if (stream.avail_out == 0) {
                        // Not finished yet -> output buffer too small -> increase size -> repeat
                        const size_t addedBytesToOutput = output.size(); // Double the size of the output buffer
                        output.resize(output.size() + addedBytesToOutput);
                        stream.next_out = (Bytef*)&output[stream.total_out];
                        stream.avail_out += addedBytesToOutput;
                    } else {
                        // Something went wrong
                        inflateEnd(&stream); // Free all memory
                        return false;
                    }
                } else if (inflateResult == Z_STREAM_END) {
                    // Complete stream has been decompressed -> finished
                    break;
                } else {
                    // Something went wrong
                    inflateEnd(&stream); // Free all memory
                    return false;
                }
            }

            // Resize output buffer to fit actual size
            output.resize(stream.total_out);

            // Free all memory
            if (inflateEnd(&stream) != Z_OK)
                return false;

            // Finished with success!
            return true;
#else
            throw std::runtime_error("broccoli::io::compression: zlib is unavailable!");
#endif
        }

        //! Decompresses the given data stream
        /*!
         * \attention For best performance, the output buffer should be pre-allocated with an upper bound of its size (use resize() and not reserve())!
         * If the output buffer is empty (zero size), the upper bound is estimated internally.
         *
         * \param [in] input The compressed input stream (may hold more than the actual compressed data - trailing bytes are ignored)
         * \param [out] output The uncompressed output stream
         * \param [in] options The used options for decompression
         * \return `true` on success, `false` otherwise
         */
        static inline bool decompress(const DataStream& input, DataStream& output, const DecompressionOptions& options) { return decompress(input.data(), input.size(), output, options); }

        //! \}
    } // namespace compression
} // namespace io
} // namespace broccoli
