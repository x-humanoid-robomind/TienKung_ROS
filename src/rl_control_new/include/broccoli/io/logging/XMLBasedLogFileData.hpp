/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "LogFileData.hpp"
#include <string>

namespace broccoli {
namespace io {
    /*!
     * \brief Specialization of broccoli::io::LogFileData for XML-based log files
     * \ingroup broccoli_io_logging
     * This LogFileData specialization creates a default header and footer accoding to the XML standard. Note that
     * the header and footer can be customized by providing the attributes
     *
     *   * Key: "XMLVersion" - Default value: see `defaultXMLVersion()`
     *   * Key: "XMLEncoding" - Default value: see `defaultXMLEncoding()`
     *   * Key: "XMLRootElementTag" - Default value: see `defaultXMLRootElementTag()`
     *
     * to the corresponding log file (see \ref LogFileBase::setAttribute()).
     */
    class XMLBasedLogFileData : public LogFileData {
    public:
        //! Default constructor
        XMLBasedLogFileData()
        {
        }

        // Encode file header to log stream (see base class for details)
        void encodeHeaderToLogStream(LogStream& stream, const std::unordered_map<std::string, std::string>& fileAttributes) const
        {
            // Try to obtain special file attributes
            const auto XMLVersionKeyValue = fileAttributes.find("XMLVersion");
            const auto XMLEncodingKeyValue = fileAttributes.find("XMLEncoding");
            const auto XMLRootElementTagKeyValue = fileAttributes.find("XMLRootElementTag");

            // Encode XML specification for this file
            encoding::encode(stream, "<?xml version=\"");
            if (XMLVersionKeyValue != fileAttributes.end())
                encoding::encode(stream, XMLVersionKeyValue->second);
            else
                encoding::encode(stream, defaultXMLVersion());
            encoding::encode(stream, "\" encoding=\"");
            if (XMLEncodingKeyValue != fileAttributes.end())
                encoding::encode(stream, XMLEncodingKeyValue->second);
            else
                encoding::encode(stream, defaultXMLEncoding());
            encoding::encode(stream, "\"?>\n");

            // Start root element (there must be only **one** root element!)
            encoding::encode(stream, '<');
            if (XMLRootElementTagKeyValue != fileAttributes.end())
                encoding::encode(stream, XMLRootElementTagKeyValue->second);
            else
                encoding::encode(stream, defaultXMLRootElementTag());

            // Encode (remaining) file attributes as attributes of XML root element
            for (const auto& attributeKeyValue : fileAttributes) {
                if (attributeKeyValue.first != "XMLVersion" && attributeKeyValue.first != "XMLEncoding" && attributeKeyValue.first != "XMLRootElementTag") {
                    encoding::encode(stream, (char)' ');
                    encoding::encode(stream, attributeKeyValue.first); // Encode key
                    encoding::encode(stream, "=\"");
                    encoding::encode(stream, attributeKeyValue.second); // Encode value
                    encoding::encode(stream, (char)'\"');
                }
            }

            // End start of root element
            encoding::encode(stream, ">\n");
        }

        // Encode file footer to log stream (see base class for details)
        void encodeFooterToLogStream(LogStream& stream, const std::unordered_map<std::string, std::string>& fileAttributes) const
        {
            // Try to obtain special file attributes
            const auto XMLRootElementTagKeyValue = fileAttributes.find("XMLRootElementTag");

            // End root element (there must be only **one** root element!)
            encoding::encode(stream, "</");
            if (XMLRootElementTagKeyValue != fileAttributes.end())
                encoding::encode(stream, XMLRootElementTagKeyValue->second);
            else
                encoding::encode(stream, defaultXMLRootElementTag());
            encoding::encode(stream, ">\n");
        }

    protected:
        // Default values
        // --------------
        static inline std::string defaultXMLVersion() { return "1.0"; } //!< Default value for XML version as string (used in header)
        static inline std::string defaultXMLEncoding() { return "UTF-8"; } //!< Default value for XML encoding as string (used in header)
        static inline std::string defaultXMLRootElementTag() { return "Data"; } //!< Default value for XML root element tag as string (used in header and footer)

#ifdef HAVE_EIGEN3
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
#endif // HAVE_EIGEN3
    };
} // namespace io
} // namespace broccoli
