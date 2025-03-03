/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include "../../core/platform/PlatformHelperFactory.hpp"
#include "../../memory/MultiLevelGrid.hpp"
#include "../../memory/MultiVector.hpp"
#include "../../memory/SmartVector.hpp"
#include "../compression.hpp"
#include "Endianness.hpp"
#include <array>
#include <assert.h>
#include <cstring> // For using memcpy (float and double conversion only)
#include <stdint.h>
#include <string>
#include <vector>
#ifdef HAVE_EIGEN3
#include <Eigen/Dense>
#endif // HAVE_EIGEN3

namespace broccoli {
namespace io {
    namespace serialization {
        /*!
         * \addtogroup broccoli_io_serialization
         * \{
         */

        // Type definitions
        typedef std::vector<uint8_t> BinaryStream; //!< Specifies the used type for storing binary stream data
        typedef uint32_t BinaryStreamSize; //!< Specifies the used type for describing the size of a binary stream (will be packed into messages in case of data hierarchies)

        // ===============
        // --- GENERAL ---
        // ===============
        //! Append binary stream to another binary stream
        /*!
         * \param [in,out] stream Stream which should be extended by "streamToAppend".
         * \param [in] streamToAppend Stream which is appended to "stream". "streamToAppend" is **not** changed (true copy).
         * \return Count of elements added to binary stream.
         */
        static inline BinaryStreamSize append(BinaryStream& stream, const BinaryStream& streamToAppend)
        {
            // Allocate memory first for maximum efficiency (does nothing if capacity of stream is already big enough)
            stream.reserve(stream.size() + streamToAppend.size());

            // Copy data into new allocated memory
            stream.insert(stream.end(), streamToAppend.begin(), streamToAppend.end());

            // Return count of added bytes
            return streamToAppend.size();
        }

        // =======================
        // --- BASIC DATATYPES ---
        // =======================
        // Serialization of basic datatypes
        // --------------------------------
        //! Encodes an unsigned integer to its binary represenation
        /*!
         * \param [in,out] stream Reference to stream we want to extend by the value.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] value Value to serialize.
         * \return Size of the binary represenation of the value (count of bytes added to stream).
         */
        template <typename UnsignedType, typename std::enable_if<std::is_integral<UnsignedType>::value && !std::is_signed<UnsignedType>::value, int>::type = 0>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const UnsignedType& value)
        {
            // Allocate additional memory (does nothing if capacity of stream is already big enough)
            const BinaryStreamSize startIndex = stream.size(); // Index of first new byte in stream
            stream.resize(stream.size() + sizeof(UnsignedType));

            // Add value to stream
            for (size_t i = 0; i < sizeof(UnsignedType); i++) {
                if (endianness == Endianness::LITTLE)
                    stream[startIndex + i] = (uint8_t)((value >> (8 * i)) & 0xFF);
                else
                    stream[startIndex + sizeof(UnsignedType) - 1 - i] = (uint8_t)((value >> (8 * i)) & 0xFF);
            }

            // Return count of added bytes
            return sizeof(UnsignedType);
        }

        //! Encodes a signed integer to its binary represenation
        /*!
         * \remark Bit-shift of negative integers is undefined, thus we have to handle the sign bit on our own!
         *
         * \param [in,out] stream Reference to stream we want to extend by the value.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] value Value to serialize.
         * \return Size of the binary represenation of the value (count of bytes added to stream).
         */
        template <typename SignedType, typename std::enable_if<std::is_integral<SignedType>::value && std::is_signed<SignedType>::value, int>::type = 0>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const SignedType& value)
        {
            // Get unsigned version of integral type
            using UnsignedType = typename std::make_unsigned<SignedType>::type;

            // Check sign
            if (value < 0) {
                // ...negative value
                BinaryStreamSize mostSignificantByte = stream.size(); // Get byte which should contain the sign bit
                if (endianness == Endianness::LITTLE)
                    mostSignificantByte += sizeof(SignedType) - 1;
                (void)serialize(stream, endianness, static_cast<UnsignedType>(-value)); // Serialize unsigned value
                stream[mostSignificantByte] = stream[mostSignificantByte] | 0x80; // Add sign-bit (0b10000000)
            } else {
                // ...positive value
                (void)serialize(stream, endianness, static_cast<UnsignedType>(value));
            }

            // Return count of added bytes
            return sizeof(SignedType);
        }

        //! Encodes a floating point number to its binary represenation
        /*!
         * \param [in,out] stream Reference to stream we want to extend by the value.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] value Value to serialize.
         * \return Size of the binary represenation of the value (count of bytes added to stream).
         */
        template <typename FloatingPointType, typename std::enable_if<std::is_floating_point<FloatingPointType>::value, int>::type = 0>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const FloatingPointType& value)
        {
            // Initialize helpers
            const auto platform = core::PlatformHelperFactory::create();
            const bool platformUsesLittleEndian = platform->usesLittleEndian();

            // Add space to stream
            const BinaryStreamSize startIndex = stream.size(); // Starting index of first new byte in stream
            stream.resize(stream.size() + sizeof(FloatingPointType));

            // Check endianness
            if (platformUsesLittleEndian == true && endianness == Endianness::LITTLE) {
                // Platform uses desired endianness -> direct copy
                memcpy((void*)&stream[startIndex], (void*)&value, sizeof(FloatingPointType));
            } else {
                // Platform does not use desired endianness -> reversed copy
                std::array<uint8_t, sizeof(FloatingPointType)> buffer;
                memcpy((void*)&buffer[0], (void*)&value, sizeof(FloatingPointType));
                for (size_t i = 0; i < buffer.size(); i++)
                    stream[startIndex + i] = buffer[buffer.size() - 1 - i];
            }

            // Pass back added bytes
            return sizeof(FloatingPointType);
        }

        //! Encodes a \ref broccoli::core::Time instance to its binary representation
        /*!
         * Serializes seconds and nanoseconds as two consecutive values.
         * \param [in,out] stream Reference to stream we want to extend by the value.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] value Value to serialize.
         * \return Size of the binary represenation of the value (count of bytes added to stream).
         */
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const core::Time& value)
        {
            // Allocate memory for maximum performance (does nothing if capacity of stream is already big enough)
            stream.reserve(stream.size() + sizeof(value.m_seconds) + sizeof(value.m_nanoSeconds));

            // Serialize seconds and nanoseconds
            serialize(stream, endianness, value.m_seconds);
            serialize(stream, endianness, value.m_nanoSeconds);

            // Return count of added bytes
            return sizeof(value.m_seconds) + sizeof(value.m_nanoSeconds);
        }

        //! Encodes a string to its binary representation
        /*!
         * Adds leading size specification (size of string). Although usually strings are null-terminated it is helpful to know the size in advance (especially during deserialization).
         * The string is serialized without trailing null-terminator.
         * \param [in,out] stream Reference to stream we want to extend by the value.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] value String to serialize.
         * \return Size of the binary represenation of the value (incl. header) (count of bytes added to stream).
         */
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const std::string& value)
        {
            // Allocate memory for maximum performance (does nothing if capacity of stream is already big enough)
            stream.reserve(stream.size() + sizeof(BinaryStreamSize) + value.size());

            // Encode size of string as header (this allows easier deserialization afterwards if size is known in advance)
            (void)serialize(stream, endianness, (BinaryStreamSize)value.size());

            // Encode string as "payload" and add to the end of the stream
            std::copy(value.begin(), value.end(), std::back_inserter(stream));

            // Return count of added bytes
            return sizeof(BinaryStreamSize) + value.size();
        }

        // Deserialization of basic datatypes
        // ----------------------------------
        //! Decodes the binary representation of an unsigned integer
        /*!
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the value in the stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] value Reference to returned value. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the value in case of success or 0 on failure
         */
        template <typename UnsignedType, typename std::enable_if<std::is_integral<UnsignedType>::value && !std::is_signed<UnsignedType>::value, int>::type = 0>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, UnsignedType& value)
        {
            // Check if stream is big enough
            if (stream.size() < index + sizeof(UnsignedType)) {
                assert(false);
                return 0;
            }

            // Extract value
            value = 0;
            for (size_t i = 0; i < sizeof(UnsignedType); i++) {
                if (endianness == Endianness::LITTLE)
                    value += (((UnsignedType)stream[index + i]) << (8 * i));
                else
                    value += (((UnsignedType)stream[index + sizeof(UnsignedType) - 1 - i]) << (8 * i));
            }

            // Return size of binary representation within stream
            return sizeof(UnsignedType);
        }

        //! Decodes the binary representation of a signed integer
        /*!
         * \remark Bit-shift of negative integers is undefined, thus we have to handle the sign bit on our own!
         *
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the value in the stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] value Reference to returned value. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the value in case of success or 0 on failure
         */
        template <typename SignedType, typename std::enable_if<std::is_integral<SignedType>::value && std::is_signed<SignedType>::value, int>::type = 0>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, SignedType& value)
        {
            // Get unsigned version of integral type
            using UnsignedType = typename std::make_unsigned<SignedType>::type;

            // Get unsigned representation
            UnsignedType unsignedValue = 0;
            if (deSerialize(stream, index, endianness, unsignedValue) == 0) {
                // Fails if stream is not big enough for value
                assert(false);
                return 0;
            }

            // Check, if sign bit is set
            if ((unsignedValue >> (8 * sizeof(SignedType) - 1)) == 0) {
                // ...positive value
                value = (SignedType)unsignedValue;
            } else {
                // ...negative value
                value = -(SignedType)(unsignedValue & (~(((UnsignedType)1) << (8 * sizeof(SignedType) - 1)))); // First remove sign bit, then convert to signed value, then negate
            }

            // Return size of binary representation within stream
            return sizeof(SignedType);
        }

        //! Decodes the binary representation of a floating point number
        /*!
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the value in the stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] value Reference to returned value. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the value in case of success or 0 on failure
         */
        template <typename FloatingPointType, typename std::enable_if<std::is_floating_point<FloatingPointType>::value, int>::type = 0>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, FloatingPointType& value)
        {
            // Initialize helpers
            const auto platform = core::PlatformHelperFactory::create();
            const bool platformUsesLittleEndian = platform->usesLittleEndian();

            // Check if stream is big enough
            if (stream.size() < index + sizeof(FloatingPointType)) {
                assert(false);
                return 0;
            }

            // Check endianness
            if (platformUsesLittleEndian == true && endianness == Endianness::LITTLE) {
                // Platform uses desired endianness -> direct copy
                memcpy((void*)&value, (void*)&stream[index], sizeof(FloatingPointType));
            } else {
                // Platform does not use desired endianness -> reversed copy
                std::array<uint8_t, sizeof(FloatingPointType)> buffer;
                for (size_t i = 0; i < buffer.size(); i++)
                    buffer[buffer.size() - 1 - i] = stream[index + i];
                memcpy((void*)&value, (void*)&buffer[0], sizeof(FloatingPointType));
            }

            // Return size of binary representation within stream
            return sizeof(FloatingPointType);
        }

        //! Decodes the binary representation of a \ref broccoli::core::Time instance
        /*!
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the value in the stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] value Reference to returned value. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the value in case of success or 0 on failure
         */
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, core::Time& value)
        {
            BinaryStreamSize targetSize = sizeof(value.m_seconds) + sizeof(value.m_nanoSeconds);

            // Check if stream is big enough
            if (stream.size() < index + targetSize) {
                assert(false);
                return 0;
            }

            // Deserialize seconds (stream size is checked in advance -> success is guaranteed)
            deSerialize(stream, index, endianness, value.m_seconds);

            // Deserialize nanoseconds (stream size is checked in advance -> success is guaranteed)
            deSerialize(stream, index + sizeof(value.m_seconds), endianness, value.m_nanoSeconds);

            // Return size of binary representation within stream
            return targetSize;
        }

        //! Decodes the binary representation of a string
        /*!
         * \remark Reads and checks the size from the header of the stream.
         *
         * \param [in] stream The stream from which the value should be extracted.
         * \param [in] index The starting index of the value in the stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] value Reference to returned value. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the value (header + string) in case of success or 0 on failure
         */
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, std::string& value)
        {
            // Extract header
            BinaryStreamSize stringLength = 0;
            if (deSerialize(stream, index, endianness, stringLength) == 0) {
                // Fails if stream is not big enough for header
                assert(false);
                return 0;
            }

            // Check, if stream is long enough for header and "payload" (=string)
            BinaryStreamSize targetSize = sizeof(BinaryStreamSize) + stringLength;
            if (stream.size() < index + targetSize) {
                assert(false);
                return 0;
            }

            // Extract string from stream
            value = std::string(stream.begin() + index + sizeof(BinaryStreamSize), stream.begin() + index + targetSize);

            // Return size of binary representation within stream
            return targetSize;
        }

        // =========================
        // --- COMPLEX DATATYPES ---
        // =========================
        //! Abstract base class for serializable data containers
        /*!
         * This class is intended to be used as base class for data exchange containers.
         *
         * One has to implement the functions:
         * - \ref serializePayload()
         * - \ref deSerializePayload()
         *
         * The functions \ref serializePayload() and \ref deSerializePayload() can be used by a background thread for efficient
         * multi-threaded (de-)serialization of the payload, where "payload" describes the actual data, i.e. the container members.
         * Note that \ref serializePayload() and \ref deSerializePayload() are indirectly triggered by calling \ref serialize()
         * and \ref deSerialize(). This automatically handles a **leading header** which contains the payload size. The header is
         * necessary to determine the end of a serialized data object in a continous data stream.
         *
         * Note that an automatic compression of the serialized payload is optional. This may be used for large data objects which
         * should be transferred over a network or saved to disk in order to save space. Compression can be enabled by overriding
         * \ref payloadCompressionOptions().
         *
         * \par Byte Mapping:
         * Size | Content
         * ---- | -------
         * \ref serialization::BinaryStreamSize | Payload size in bytes (=header)
         * ... | Actual payload, i.e. member data
         */
        class SerializableData {
            // Construction
            // ------------
        public:
            //! Constructor
            SerializableData()
            {
            }

            //! Destructor
            virtual ~SerializableData()
            {
            }

            // Serialization (main object)
            // -------------
        public:
            //! Serialization of complete data container (header + payload)
            /*!
             * Serializes header and payload and appends both to the given stream.
             * \param [in,out] stream Stream to be extended by the header and payload.
             * \param [in] endianness Defines the used byte-order.
             * \return Size of the binary represenation of header and payload (count of bytes added to stream).
             */
            virtual BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness) const
            {
                // Remember starting index of header (to overwrite it later on)
                const BinaryStreamSize headerStartIndex = stream.size();

                // Add dummy header (containing wrong information about size) (used as placeholder)
                const BinaryStreamSize headerSize = serialization::serialize(stream, endianness, (BinaryStreamSize)0);

                // Remember starting index of payload
                const BinaryStreamSize payloadStartIndex = headerStartIndex + headerSize;

                // Add actual payload and remember its size
                BinaryStreamSize reportedPayloadSize = serializePayload(stream, endianness);
                BinaryStreamSize actualPayloadSize = stream.size() - payloadStartIndex;
                if (reportedPayloadSize != actualPayloadSize) {
                    // Reported and actual payload sizes differ!
                    assert(false);
                    return 0;
                }

                // Optional: compression of payload data
                const compression::CompressionOptions compressionOptions = payloadCompressionOptions();
                if (compressionOptions.compressed() == true) {
                    // Compress payload
                    BinaryStream compressedPayload;
                    if (compression::compress(&stream[payloadStartIndex], actualPayloadSize, compressedPayload, compressionOptions) == false) {
                        // Compression failed
                        assert(false);
                        return 0;
                    }

                    // Replace uncompressed payload data with compressed payload data
                    stream.resize(payloadStartIndex); // Remove uncompressed payload data
                    reportedPayloadSize = append(stream, compressedPayload); // Add compressed payload data
                    actualPayloadSize = stream.size() - payloadStartIndex;
                    if (reportedPayloadSize != actualPayloadSize) {
                        // Reported and actual payload sizes differ!
                        assert(false);
                        return 0;
                    }
                }

                // Create auxillary binary stream only for header and write real size of payload to it
                BinaryStream headerStream;
                serialization::serialize(headerStream, endianness, actualPayloadSize);

                // Overwrite dummy header with correct information for payload size
                for (BinaryStreamSize i = 0; i < headerSize; i++)
                    stream[headerStartIndex + i] = headerStream[i];

                // Return count of added bytes
                return headerSize + actualPayloadSize;
            }

            //! Deserialization of complete data container (header + payload)
            /*!
             * Deserializes header and payload from a given binary stream.
             * \param [in] stream The stream from which the data should be extracted.
             * \param [in] index The starting index of the data in the stream.
             * \param [in] endianness Defines the used byte-order.
             * \return Size of binary representation (header + payload) in case of success or 0 on failure
             */
            virtual BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness)
            {
                // Get starting index of header
                const BinaryStreamSize& headerStartIndex = index;

                // Extract payload size
                BinaryStreamSize payloadSize = 0;
                const BinaryStreamSize headerSize = serialization::deSerialize(stream, headerStartIndex, endianness, payloadSize);
                if (headerSize == 0) {
                    // Fails, if stream is not big enough for header
                    assert(false);
                    return 0;
                }

                // Get starting index of payload
                const BinaryStreamSize payloadStartIndex = headerStartIndex + headerSize;

                // Check, if stream is big enough for payload
                if (stream.size() < payloadStartIndex + payloadSize) {
                    assert(false);
                    return 0;
                }

                // Compute target size of this object
                const BinaryStreamSize targetSize = headerSize + payloadSize;

                // Check, if compression is enabled
                const compression::DecompressionOptions decompressionOptions = payloadCompressionOptions();
                if (decompressionOptions.compressed() == true) {
                    // Decompress payload
                    BinaryStream uncompressedPayload;
                    if (compression::decompress(&stream[payloadStartIndex], payloadSize, uncompressedPayload, decompressionOptions) == false) {
                        assert(false);
                        return 0;
                    }

                    // Deserialize uncompressed payload
                    if (deSerializePayload(uncompressedPayload, 0, uncompressedPayload.size(), endianness) != uncompressedPayload.size()) {
                        assert(false);
                        return 0;
                    }
                } else {
                    // Directly extract payload (throw error, if reported size does not match size in header)
                    if (deSerializePayload(stream, payloadStartIndex, payloadSize, endianness) != payloadSize) {
                        assert(false);
                        return 0;
                    }
                }

                // Return size of binary representation (header + payload) within input stream
                return targetSize;
            }

            // Serialization (payload)
            // -------------
        protected:
            //! Serialization of payload
            /*!
             * Encodes all members (=payload) and appends them to a given binary stream.
             * \param [in,out] stream Stream to be extended by the payload.
             * \param [in] endianness Defines the used byte-order.
             * \return Size of the binary represenation of payload (count of bytes added to stream).
             */
            virtual BinaryStreamSize serializePayload(BinaryStream& stream, const Endianness& endianness) const = 0;

            //! Deserialization of payload
            /*!
             * Decodes given binary stream to get member data (payload)
             * \param [in] stream The stream from which the payload should be extracted.
             * \param [in] index The starting index of the payload in the stream.
             * \param [in] payloadSize Exact size of the payload data (important for decoding complex data containers of varying size - may be ignored for simple data containers).
             * \param [in] endianness Defines the used byte-order.
             * \return Size of binary representation of the payload in case of success or 0 on failure
             */
            virtual BinaryStreamSize deSerializePayload(const BinaryStream& stream, const BinaryStreamSize& index, const BinaryStreamSize& payloadSize, const Endianness& endianness) = 0;

            // Compression
            // -----------
        protected:
            //! Specifies the compression options for the payload
            virtual compression::CompressionOptions payloadCompressionOptions() const { return compression::CompressionOptions::noCompression(); }

#ifdef HAVE_EIGEN3
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
        };

        // Serialization of complex datatypes
        // ----------------------------------
#ifdef HAVE_EIGEN3
        //! Encodes a matrix to its binary representation
        /*!
         * Serializes all elements of the matrix in row-major order.
         * Additionally adds a header (2 x BinaryStreamSize) which specifies how many rows and columns are contained.
         *
         * \tparam Derived The derived type, e.g. a matrix type, or an expression, etc.
         *
         * \param [in,out] stream Reference to stream we want to extend by the matrix.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] matrix Matrix to serialize.
         * \return Size of the binary represenation of the matrix (count of bytes added to stream).
         */
        template <typename Derived>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const Eigen::MatrixBase<Derived>& matrix)
        {
            // Allocate additional memory (does nothing if capacity of stream is already big enough)
            stream.reserve(stream.size() + 2 * sizeof(BinaryStreamSize) + matrix.rows() * matrix.cols() * sizeof(typename Eigen::MatrixBase<Derived>::Scalar));

            // Encode count of rows and columns in header
            BinaryStreamSize totalSize = 0;
            totalSize += serialize(stream, endianness, (BinaryStreamSize)matrix.rows());
            totalSize += serialize(stream, endianness, (BinaryStreamSize)matrix.cols());

            // Iterate through all elements (row-major) and serialize them one after the other
            for (int i = 0; i < matrix.rows(); i++)
                for (int j = 0; j < matrix.cols(); j++)
                    totalSize += serialize(stream, endianness, matrix(i, j));

            // Return count of added bytes
            return totalSize;
        }

        //! Encodes a quaternion to its binary representation
        /*!
         * Serializes all coefficients of the quaternion in the order w, x, y, z.
         *
         * \tparam Derived The derived type.
         *
         * \param [in,out] stream Reference to stream we want to extend by the quaternion.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] quaternion Quaternion to serialize.
         * \return Size of the binary represenation of the quaternion (count of bytes added to stream).
         */
        template <typename Derived>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const Eigen::QuaternionBase<Derived>& quaternion)
        {
            // Allocate additional memory (does nothing if capacity of stream is already big enough)
            stream.reserve(stream.size() + 4 * sizeof(typename Eigen::QuaternionBase<Derived>::Scalar));

            // Encode coefficients
            BinaryStreamSize totalSize = 0;
            totalSize += serialize(stream, endianness, quaternion.w());
            totalSize += serialize(stream, endianness, quaternion.x());
            totalSize += serialize(stream, endianness, quaternion.y());
            totalSize += serialize(stream, endianness, quaternion.z());

            // Return count of added bytes
            return totalSize;
        }
#endif // HAVE_EIGEN3

        //! Encodes an object derived from SerializableData to its binary representation
        /*!
         * \param [in,out] stream Reference to stream we want to extend by the object.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] object Object to serialize.
         * \return Size of the binary represenation of the object (count of bytes added to stream).
         */
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const SerializableData& object)
        {
            // Call serialization function of object
            return object.serialize(stream, endianness);
        }

        //! Encodes an array to its binary representation
        /*!
         * Serializes all elements of the array preserving the array order.
         *
         * \tparam T Element type
         * \tparam N Count of elements
         *
         * \param [in,out] stream Reference to stream we want to extend by the array.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] array Array to serialize.
         * \return Size of the binary represenation of the array (count of bytes added to stream).
         */
        template <typename T, size_t N>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const std::array<T, N>& array)
        {
            // Note: we can not properly allocate memory in the stream, since the element type is not known (may contain dynamically allocated members)

            // Iterate through all elements and serialize them one after the other
            BinaryStreamSize totalSize = 0;
            for (size_t i = 0; i < N; i++)
                totalSize += serialize(stream, endianness, array[i]);

            // Return count of added bytes
            return totalSize;
        }

        //! Helper function to encode a vector-like object to its binary representation
        /*!
         * Serializes all elements of the vector preserving the order.
         * Additionally adds a header (1 x BinaryStreamSize) which specifies how many elements are contained.
         * The vector-like object has to specify
         *   * size()     ...function to return count of elements and
         *   * operator[] ...access to the i-th element in the vector.
         *
         * \tparam VectorLikeType Type of the vector-like object
         *
         * \param [in,out] stream Reference to stream we want to extend by the vector-like.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] vectorLike Vector-like object to serialize.
         * \return Size of the binary represenation of the vector-like object (count of bytes added to stream).
         */
        template <typename VectorLikeType>
        static inline BinaryStreamSize serializeVectorLike(BinaryStream& stream, const Endianness& endianness, const VectorLikeType& vectorLike)
        {
            // Note: we can not properly allocate memory in the stream, since the element type is not known (may contain dynamically allocated members)

            // Encode count of elements as header
            BinaryStreamSize totalSize = serialize(stream, endianness, (BinaryStreamSize)vectorLike.size());

            // Iterate through all elements and serialize them one after the other
            for (size_t i = 0; i < vectorLike.size(); i++)
                totalSize += serialize(stream, endianness, vectorLike[i]);

            // Return count of added bytes
            return totalSize;
        }

        //! Encodes a vector to its binary representation
        /*!
         * Serializes all elements of the vector preserving the order.
         *
         * \tparam T Type of elements in vector
         * \tparam Allocator Allocator used by vector
         *
         * \param [in,out] stream Reference to stream we want to extend by the vector.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] vector Vector to serialize.
         * \return Size of the binary represenation of the vector (count of bytes added to stream).
         */
        template <typename T, typename Allocator = std::allocator<T>>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const std::vector<T, Allocator>& vector)
        {
            return serializeVectorLike<std::vector<T, Allocator>>(stream, endianness, vector);
        }

        //! Encodes a broccoli::memory::SmartVector to its binary representation
        /*!
         * Serializes all elements of the SmartVector preserving the order.
         *
         * \tparam T Element type
         * \tparam MaximumStaticElementCount Count of elements of the underlying `std::array`
         * \tparam Allocator The allocator to be used for the underlying `std::vector`
         *
         * \param [in,out] stream Reference to stream we want to extend by the SmartVector.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] vector SmartVector to serialize.
         * \return Size of the binary represenation of the SmartVector (count of bytes added to stream).
         */
        template <typename T, size_t MaximumStaticElementCount = 0, typename Allocator = std::allocator<T>>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const memory::SmartVector<T, MaximumStaticElementCount, Allocator>& vector)
        {
            return serializeVectorLike<memory::SmartVector<T, MaximumStaticElementCount, Allocator>>(stream, endianness, vector);
        }

        //! Encodes a broccoli::memory::MultiVector to its binary representation
        /*!
         * Serializes all elements in all dimensions of the MultiVector preserving the size and order.
         *
         * \tparam T Type of each element
         * \tparam N Count of dimensions
         * \tparam Allocator Allocator used to create new elements
         *
         * \param [in,out] stream Reference to stream we want to extend by the MultiVector.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] vector MultiVector to serialize.
         * \return Size of the binary represenation of the MultiVector (count of bytes added to stream).
         */
        template <typename T, unsigned int N, class Allocator = std::allocator<T>>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const memory::MultiVector<T, N, Allocator>& vector)
        {
            // Note: we can not properly allocate memory in the stream, since the element type is not known (contains dynamically allocated members)
            BinaryStreamSize totalBytes = 0;

            // Get element count
            const size_t elementCount = vector.elementCount();

            // Encode size
            totalBytes += serialize(stream, endianness, vector.size());

            // Iterate through all elements and serialize them one after the other
            for (size_t i = 0; i < elementCount; i++)
                totalBytes += serialize(stream, endianness, vector[i]);

            // Return count of added bytes
            return totalBytes;
        }

        //! Encodes a broccoli::memory::MultiLevelGrid to its binary representation
        /*!
         * Serializes all elements in all dimensions and levels of the MultiLevelGrid preserving the size and order.
         *
         * \tparam N The dimension of the grids (N=1: grid="line", N=2: grid="square", N=3: grid="cube", ...)
         * \tparam L The count of levels (alias "layers") (has to be greater than 0)
         * \tparam T The Type of each "cell" in the grid
         * \tparam Allocator The allocator to be used to create new grid cells
         *
         * \param [in,out] stream Reference to stream we want to extend by the MultiLevelGrid.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] grid MultiLevelGrid to serialize.
         * \return Size of the binary represenation of the MultiLevelGrid (count of bytes added to stream).
         */
        template <unsigned int N, unsigned int L, class T, class Allocator = std::allocator<T>>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const memory::MultiLevelGrid<N, L, T, Allocator>& grid)
        {
            // Note: we can not properly allocate memory in the stream, since the element type is not known (contains dynamically allocated members)
            BinaryStreamSize totalBytes = 0;

            // Encode levels
            for (unsigned int l = 0; l < L; l++) {
                // Get cell count
                const size_t cellCount = grid.cellCount(l);

                // Encode (first) size
                if (l == 0)
                    totalBytes += serialize(stream, endianness, grid.size(l));

                // Iterate through all cells and serialize them one after the other
                for (size_t i = 0; i < cellCount; i++)
                    totalBytes += serialize(stream, endianness, grid.cell(l, i));
            }

            // Return count of added bytes
            return totalBytes;
        }

        // Deserialization of complex datatypes
        // ------------------------------------
#ifdef HAVE_EIGEN3
        //! Decodes the binary representation of a matrix
        /*!
         * \tparam Derived The derived type, e.g. a matrix type, or an expression, etc.
         *
         * \param [in] stream The binary stream from which the matrix should be extracted.
         * \param [in] index The starting index of the matrix in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] matrix Reference to returned matrix. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the matrix in case of success or 0 on failure
         */
        template <typename Derived>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, Eigen::MatrixBase<Derived>& matrix)
        {
            // Extract row and column count
            BinaryStreamSize rowCount = 0;
            if (deSerialize(stream, index, endianness, rowCount) == 0) {
                assert(false);
                return 0;
            }
            BinaryStreamSize columnCount = 0;
            if (deSerialize(stream, index + sizeof(BinaryStreamSize), endianness, columnCount) == 0) {
                assert(false);
                return 0;
            }

            // Resize matrix
            if ((matrix.RowsAtCompileTime != Eigen::Dynamic && matrix.rows() != (int)rowCount) || (matrix.ColsAtCompileTime != Eigen::Dynamic && matrix.cols() != (int)columnCount)) {
                assert(false);
                return 0; // Error -> the dimensions of the provided matrix do not match the deserialized data!
            }
            matrix.derived().resize(rowCount, columnCount);

            // Initialize helpers
            BinaryStreamSize totalSize = 2 * sizeof(BinaryStreamSize); // Count of deserialized bytes in total
            BinaryStreamSize currentIndex = index + 2 * sizeof(BinaryStreamSize); // Index of next element to deserialize

            // Deserialize elements
            for (size_t i = 0; i < rowCount; i++) {
                for (size_t j = 0; j < columnCount; j++) {
                    const BinaryStreamSize elementSize = deSerialize(stream, currentIndex, endianness, matrix(i, j));
                    if (elementSize == 0) {
                        // Element could not be deserialized
                        assert(false);
                        return 0;
                    } else {
                        totalSize += elementSize;
                        currentIndex += elementSize;
                    }
                }
            }

            // Return size of binary representation within stream
            return totalSize;
        }

        //! Decodes the binary representation of a quaternion
        /*!
         * The coefficients must have the order: w, x, y, z.
         *
         * \tparam Derived The derived type.
         *
         * \param [in] stream The binary stream from which the quaternion should be extracted.
         * \param [in] index The starting index of the quaternion in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] quaternion Reference to returned quaternion. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the quaternion in case of success or 0 on failure
         */
        template <typename Derived>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, Eigen::QuaternionBase<Derived>& quaternion)
        {
            // Initialize helpers
            BinaryStreamSize totalSize = 0; // Count of deserialized bytes in total
            BinaryStreamSize currentIndex = index; // Index of next coefficient to deserialize

            // Deserialize coefficients
            for (size_t i = 0; i < 4; i++) {
                // Deserialize coefficient
                auto* coefficient = &quaternion.w();
                if (i == 1)
                    coefficient = &quaternion.x();
                else if (i == 2)
                    coefficient = &quaternion.y();
                else if (i == 3)
                    coefficient = &quaternion.z();
                const BinaryStreamSize coefficientSize = deSerialize(stream, currentIndex, endianness, *coefficient);
                if (coefficientSize == 0) {
                    // Coefficient could not be deserialized
                    assert(false);
                    return 0;
                } else {
                    totalSize += coefficientSize;
                    currentIndex += coefficientSize;
                }
            }

            // Return size of binary representation within stream
            return totalSize;
        }
#endif // HAVE_EIGEN3

        //! Decodes the binary representation of an object derived from SerializableData
        /*!
         * \param [in] stream The binary stream from which the object should be extracted.
         * \param [in] index The starting index of the object in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] object Reference to returned object. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the object in case of success or 0 on failure
         */
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, SerializableData& object)
        {
            // Call deserialization function of object
            return object.deSerialize(stream, index, endianness);
        }

        //! Decodes the binary representation of an array
        /*!
         * \tparam T Element type
         * \tparam N Count of elements
         *
         * \param [in] stream The binary stream from which the array should be extracted.
         * \param [in] index The starting index of the array in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] array Reference to returned array. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the array in case of success or 0 on failure
         */
        template <typename T, size_t N>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, std::array<T, N>& array)
        {
            // Iterate through all elements and deserialize one after the other
            BinaryStreamSize totalSize = 0;
            BinaryStreamSize currentIndex = index; // Starting index of next element in stream
            for (size_t i = 0; i < N; i++) {
                const BinaryStreamSize elementSize = deSerialize(stream, currentIndex, endianness, array[i]);
                if (elementSize == 0) {
                    // Element could not be deserialized
                    assert(false);
                    return 0;
                } else {
                    totalSize += elementSize;
                    currentIndex += elementSize;
                }
            }

            // Return size of binary representation within stream
            return totalSize;
        }

        //! Helper function to decode the binary representation of a vector-like object
        /*!
         * Automatically determines the element count from the header
         *
         * The vector-like object has to specify
         *   * resize()   ...possiblity to resize the container such that it has the specified count of elements and
         *   * operator[] ...access to the i-th element in the vector.
         *
         * \tparam VectorLikeType Type of vector-like object
         *
         * \param [in] stream The binary stream from which the vector-like should be extracted.
         * \param [in] index The starting index of the vector-like in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] vectorLike Reference to returned vector-like. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the vector-like in case of success or 0 on failure
         */
        template <typename VectorLikeType>
        static inline BinaryStreamSize deSerializeVectorLike(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, VectorLikeType& vectorLike)
        {
            // Extract element count
            BinaryStreamSize elementCount = 0;
            if (deSerialize(stream, index, endianness, elementCount) == 0) {
                assert(false);
                return 0;
            }

            // Initialize helpers
            BinaryStreamSize totalSize = sizeof(BinaryStreamSize); // Count of deserialized bytes in total
            BinaryStreamSize currentIndex = index + sizeof(BinaryStreamSize); // Index of next element to deserialize

            // Deserialize elements
            vectorLike.resize(elementCount);
            for (size_t i = 0; i < elementCount; i++) {
                const BinaryStreamSize elementSize = deSerialize(stream, currentIndex, endianness, vectorLike[i]);
                if (elementSize == 0) {
                    // Element could not be deserialized
                    assert(false);
                    return 0;
                } else {
                    totalSize += elementSize;
                    currentIndex += elementSize;
                }
            }

            // Return size of binary representation within stream
            return totalSize;
        }

        //! Decodes the binary representation of a vector
        /*!
         * \tparam T Element type
         * \tparam Allocator The allocator to be used
         *
         * \param [in] stream The binary stream from which the vector should be extracted.
         * \param [in] index The starting index of the vector in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] vector Reference to returned vector. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the vector in case of success or 0 on failure
         */
        template <typename T, typename Allocator = std::allocator<T>>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, std::vector<T, Allocator>& vector)
        {
            return deSerializeVectorLike<std::vector<T, Allocator>>(stream, index, endianness, vector);
        }

        //! Decodes the binary representation of a SmartVector
        /*!
         * \tparam T Element type
         * \tparam MaximumStaticElementCount Count of elements of the underlying `std::array`
         * \tparam Allocator The allocator to be used for the underlying `std::vector`
         *
         * \param [in] stream The binary stream from which the SmartVector should be extracted.
         * \param [in] index The starting index of the SmartVector in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] vector Reference to returned SmartVector. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the SmartVector in case of success or 0 on failure
         */
        template <typename T, size_t MaximumStaticElementCount = 0, typename Allocator = std::allocator<T>>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, memory::SmartVector<T, MaximumStaticElementCount, Allocator>& vector)
        {
            return deSerializeVectorLike<memory::SmartVector<T, MaximumStaticElementCount, Allocator>>(stream, index, endianness, vector);
        }

        //! Decodes the binary representation of a MultiVector
        /*!
         * \tparam T Type of each element
         * \tparam N Count of dimensions
         * \tparam Allocator Allocator used to create new elements
         *
         * \param [in] stream The binary stream from which the MultiVector should be extracted.
         * \param [in] index The starting index of the MultiVector in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] vector Reference to returned MultiVector. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the MultiVector in case of success or 0 on failure
         */
        template <typename T, unsigned int N, class Allocator = std::allocator<T>>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, memory::MultiVector<T, N, Allocator>& vector)
        {
            // Initialize helpers
            BinaryStreamSize totalBytes = 0;
            BinaryStreamSize currentIndex = index; // Index of next element to deserialize

            // Extract size
            std::array<size_t, N> size{};
            const BinaryStreamSize sizeBytes = deSerialize(stream, currentIndex, endianness, size);
            if (sizeBytes == 0 || vector.resize(size) == false) {
                assert(false);
                return 0;
            }
            totalBytes += sizeBytes;
            currentIndex += sizeBytes;
            const size_t elementCount = vector.elementCount();

            // Deserialize elements
            for (size_t i = 0; i < elementCount; i++) {
                const BinaryStreamSize elementBytes = deSerialize(stream, currentIndex, endianness, vector[i]);
                if (elementBytes == 0) {
                    // Element could not be deserialized
                    assert(false);
                    return 0;
                } else {
                    totalBytes += elementBytes;
                    currentIndex += elementBytes;
                }
            }

            // Return size of binary representation within stream
            return totalBytes;
        }

        //! Decodes the binary representation of a MultiLevelGrid
        /*!
         * \tparam N The dimension of the grids (N=1: grid="line", N=2: grid="square", N=3: grid="cube", ...)
         * \tparam L The count of levels (alias "layers") (has to be greater than 0)
         * \tparam T The Type of each "cell" in the grid
         * \tparam Allocator The allocator to be used to create new grid cells
         *
         * \param [in] stream The binary stream from which the MultiLevelGrid should be extracted.
         * \param [in] index The starting index of the MultiLevelGrid in the binary stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [out] grid Reference to returned MultiLevelGrid. The result of the deserialization will be stored in this parameter.
         * \return Size of binary representation of the MultiLevelGrid in case of success or 0 on failure
         */
        template <unsigned int N, unsigned int L, class T, class Allocator = std::allocator<T>>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, memory::MultiLevelGrid<N, L, T, Allocator>& grid)
        {
            // Initialize helpers
            BinaryStreamSize totalBytes = 0;
            BinaryStreamSize currentIndex = index; // Index of next element to deserialize

            // Iterate over levels
            for (unsigned int l = 0; l < L; l++) {
                // Extract (first) size
                if (l == 0) {
                    std::array<size_t, N> size{};
                    const BinaryStreamSize sizeBytes = deSerialize(stream, currentIndex, endianness, size);
                    if (sizeBytes == 0 || grid.resize(size) == false) {
                        assert(false);
                        return 0;
                    }
                    totalBytes += sizeBytes;
                    currentIndex += sizeBytes;
                }
                const size_t cellCount = grid.cellCount(l);

                // Deserialize cells
                for (size_t i = 0; i < cellCount; i++) {
                    const BinaryStreamSize cellBytes = deSerialize(stream, currentIndex, endianness, grid.cell(l, i));
                    if (cellBytes == 0) {
                        // Cell could not be deserialized
                        assert(false);
                        return 0;
                    } else {
                        totalBytes += cellBytes;
                        currentIndex += cellBytes;
                    }
                }
            }

            // Return size of binary representation within stream
            return totalBytes;
        }

        // Serialization of tuples
        // -----------------------
        //! Dummy method to terminate recursive call of corresponding variadic template
        /*! \return 0 */
        static inline BinaryStreamSize serialize(BinaryStream&, const Endianness&) { return 0; }

        //! Encodes the given tuple of values to its binary representation
        /*!
         * \param [in,out] stream Reference to stream we want to extend.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] firstValue First value to serialize
         * \param [in] secondValue Second value to serialize
         * \param [in] remainingValues Tuple of remaining values to serialize
         * \return Size of the binary represenation of the tuple (count of bytes added to stream).
         */
        template <typename firstType, typename secondType, typename... remainingTypes>
        static inline BinaryStreamSize serialize(BinaryStream& stream, const Endianness& endianness, const firstType& firstValue, const secondType& secondValue, const remainingTypes&... remainingValues)
        {
            return serialize(stream, endianness, firstValue) + serialize(stream, endianness, secondValue) + serialize(stream, endianness, remainingValues...);
        }

        // Deserialization of tuples
        // -------------------------
        //! Dummy method to terminate recursive call of corresponding variadic template
        /*! \return 0 */
        static inline BinaryStreamSize deSerialize(const BinaryStream&, const BinaryStreamSize&, const Endianness&) { return 0; }

        //! Decodes the binary representation of the given tuple of values
        /*!
         * \param [in] stream The stream from which the tuple should be extracted.
         * \param [in] index The starting index of the tuple in the stream.
         * \param [in] endianness Defines the used byte-order.
         * \param [in] firstValue Reference to first value. The result of the deserialization will be stored in this parameter.
         * \param [in] secondValue Reference to second value. The result of the deserialization will be stored in this parameter.
         * \param [in] remainingValues Reference to remaining values. The result of the deserialization will be stored in this parameters.
         * \return Size of binary representation of the tuple in case of success or 0 on failure
         */
        template <typename firstType, typename secondType, typename... remainingTypes>
        static inline BinaryStreamSize deSerialize(const BinaryStream& stream, const BinaryStreamSize& index, const Endianness& endianness, firstType& firstValue, secondType& secondValue, remainingTypes&... remainingValues)
        {
            // Initialize helpers
            io::serialization::BinaryStreamSize currentIndex = index;

            // Extract values
            currentIndex += deSerialize(stream, currentIndex, endianness, firstValue);
            currentIndex += deSerialize(stream, currentIndex, endianness, secondValue);
            currentIndex += deSerialize(stream, currentIndex, endianness, remainingValues...);

            // Pass back decoded stream elements
            return currentIndex - index;
        }
        //! \}
    } // namespace serialization
} // namespace io
} // namespace broccoli
