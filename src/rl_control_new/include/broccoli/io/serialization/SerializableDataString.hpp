/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "serialization.hpp"

namespace broccoli {
namespace io {
    //! Data container for simple string data
    /*!
     * \ingroup broccoli_io_serialization
     *
     * Contains a single `std::string`. May be used as reference implementation.
     *
     * \par Byte Mapping:
     * Size | Content
     * ---- | -------
     * \ref serialization::BinaryStreamSize | Payload size in bytes (=header)
     * ... | \ref m_string
     */
    class SerializableDataString : public serialization::SerializableData {
    public:
        //! Constructor
        /*!
         * \param string Sets \ref m_string - \copybrief m_string
         */
        SerializableDataString(const std::string& string = "")
            : m_string(string)
        {
        }

        //! Destructor
        virtual ~SerializableDataString()
        {
        }

        // Members
        std::string m_string; //!< Simple string representing the member data

        //! Comparison operator
        /*!
         * \param [in] reference Reference to other object to compare to
         * \return `true`, if members match, `false` otherwise
         */
        inline bool operator==(const SerializableDataString& reference) const
        {
            return m_string == reference.m_string;
        }

    protected:
        // Serialization of payload (see base class for details)
        virtual serialization::BinaryStreamSize serializePayload(serialization::BinaryStream& stream, const serialization::Endianness& endianness) const
        {
            return serialization::serialize(stream, endianness, m_string);
        }

        //! Deserialization of payload
        /*!
         * \attention The parameter \p payloadSize is **not used** for \ref SerializableDataString!
         *
         * \copydoc serialization::SerializableData::deSerializePayload()
         */
        virtual serialization::BinaryStreamSize deSerializePayload(const serialization::BinaryStream& stream, const serialization::BinaryStreamSize& index, const serialization::BinaryStreamSize& payloadSize, const serialization::Endianness& endianness)
        {
            (void)payloadSize; // Not needed
            return serialization::deSerialize(stream, index, endianness, m_string);
        }
    };
} // namespace io
} // namespace broccoli
