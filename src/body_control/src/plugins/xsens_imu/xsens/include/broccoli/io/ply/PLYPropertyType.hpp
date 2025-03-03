/*
 * This file is part of broccoli.
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/string.hpp"
#include "../encoding.hpp"
#include "PLYResult.hpp"
#include "PLYScalar.hpp"
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \addtogroup broccoli_io_ply
     * \{
     */

    //! Abstraction layer for property types of the Polygon File Format (PLY)
    class PLYPropertyType {
    public:
        //! Constructor
        /*!
         * \param [in] isListProperty Initializes \ref m_isListProperty - \copybrief m_isListProperty
         */
        PLYPropertyType(const bool& isListProperty = false)
            : m_name("")
            , m_isListProperty(isListProperty)
            , m_counterType(PLYScalar::Type::UCHAR)
            , m_valueType(PLYScalar::Type::UCHAR)
        {
        }

        //! Comparison operator: **equality**
        bool operator==(const PLYPropertyType& reference) const
        {
            // Compare members
            if (m_name != reference.m_name)
                return false;
            if (m_isListProperty != reference.m_isListProperty)
                return false;
            if (m_counterType != reference.m_counterType)
                return false;
            if (m_valueType != reference.m_valueType)
                return false;

            // All members are equal -> equal
            return true;
        }

        //! Comparison operator: **inequality**
        bool operator!=(const PLYPropertyType& reference) const { return !(*this == reference); }

        // Members
        std::string m_name; //!< Name of the property type
        bool m_isListProperty; //!< If `true` this property is a `list property`, otherwise it is a `scalar property`
        PLYScalar::Type m_counterType; //!< Datatype of the element counter (**only** for list properties)
        PLYScalar::Type m_valueType; //!< Datatype of the value (scalar property) or list elements (list property)

        // De- and encoding of header
        // --------------------------
        //! Encodes the header of this property type to the given stream
        /*!
         * \param [out] stream Stream to append encoded header to
         */
        void encodeHeader(encoding::CharacterStream& stream) const
        {
            encoding::encode(stream, "property ");
            if (m_isListProperty == true)
                encoding::encode(stream, "list " + PLYScalar::typeToString(m_counterType) + " ");
            encoding::encode(stream, PLYScalar::typeToString(m_valueType) + " " + m_name);
        }

        //! Decodes given header line to obtain the property type specification
        /*!
         * \param [in] headerLine Line in header related to this property type
         * \param [out] result Pointer to flag in which the result of the algorithm should be stored
         * \return `true` on success, `false` otherwise
         */
        bool decodeHeader(const std::string& headerLine, PLYResult* const result = nullptr)
        {
            // Initialize helpers
            bool success = false; // Flag indicating, if the header was decoded successfully
            auto parts = core::stringSplit(headerLine, ' ', true); // Header line splitted into parts

            // Check, minimum part count and starting tag
            if (parts.size() >= 3 && parts[0] == "property") {
                if (parts[1] == "list") {
                    m_isListProperty = true;
                    if (parts.size() >= 5) {
                        m_counterType = PLYScalar::typeFromString(parts[2]);
                        m_valueType = PLYScalar::typeFromString(parts[3]);
                        m_name = parts[4];
                        if (m_counterType != PLYScalar::Type::UNKNOWN && m_valueType != PLYScalar::Type::UNKNOWN)
                            success = true;
                        // else: success = false
                    } // else: success = false
                } else {
                    m_isListProperty = false;
                    m_valueType = PLYScalar::typeFromString(parts[1]);
                    m_name = parts[2];
                    if (m_valueType != PLYScalar::Type::UNKNOWN)
                        success = true;
                    // else: success = false
                }
            } // else: success = false

            // Return result of decoding
            if (result != nullptr) {
                if (success)
                    *result = PLYResult::SUCCESS;
                else
                    *result = PLYResult::ERROR_INVALID_HEADER_PROPERTYTYPE;
            }
            assert(success);
            return success;
        }
    };

    //! \}
} // namespace io
} // namespace broccoli
