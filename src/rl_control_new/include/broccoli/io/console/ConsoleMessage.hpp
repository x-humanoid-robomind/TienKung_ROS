/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include "../../core/string.hpp"
#include "../encoding.hpp"
#include <string>

namespace broccoli {
namespace io {
    //! Container for messages handled by the console
    /*! \ingroup broccoli_io_console */
    class ConsoleMessage {
    public:
        //! Default constructor
        ConsoleMessage()
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] interfaceIndex Initializes \ref m_interfaceIndex - \copybrief m_interfaceIndex
         * \param [in] text Initializes \ref m_text - \copybrief m_text
         */
        ConsoleMessage(const size_t& interfaceIndex, const std::string& text)
            : m_interfaceIndex(interfaceIndex)
            , m_text(text)
        {
        }

        // Members
        // -------
        size_t m_interfaceIndex = 0; //!< Index of interface this message is linked to
        std::string m_text = ""; //!< Text to output (**without** prefix and suffix)

        // Helpers
        // -------
        //! Creates a console message from the given data
        /*!
         * \param [in] interfaceIndex Initializes \ref m_interfaceIndex - \copybrief m_interfaceIndex
         * \param [in] data The data to parse
         */
        template <typename T>
        static inline ConsoleMessage createFrom(const size_t& interfaceIndex, const T& data) { return ConsoleMessage(interfaceIndex, encoding::encodeToString(data)); }

        //! \copydoc ConsoleMessage::createFrom(const size_t&, const T&)
        static inline ConsoleMessage createFrom(const size_t& interfaceIndex, const char* data) { return ConsoleMessage(interfaceIndex, std::string(data)); }

        //! Create string for console/logfile output
        /*!
         * Automatically manages appending prefixes and suffixes on a per-line basis.
         *
         * \param [in] currentMessage The current message to write
         * \param [in] currentMessagePrefix The prefix of the interface linked to the current message
         * \param [in] currentMessageSuffix The suffix of the interface linked to the current message
         * \param [in,out] previousMessage The previously written message
         * \param [in] previousMessageSuffix The suffix of the interface linked to the previous message
         * \param [in,out] previousLineComplete Flag, indicating, if the previous line ended with a line break
         * \return Output string to write to the console or log file
         */
        static inline std::string createOutputString(const ConsoleMessage& currentMessage, const std::string& currentMessagePrefix, const std::string& currentMessageSuffix, ConsoleMessage& previousMessage, const std::string& previousMessageSuffix, bool& previousLineComplete)
        {
            // Split current message into lines
            const auto currentMessageLines = core::stringSplit(currentMessage.m_text, '\n', false);

            // Initialize return value
            std::string outputString = "";
            size_t estimatedOutputStringLength = previousMessageSuffix.length() + 1; // Probably complete previous message with suffix + line break
            for (size_t i = 0; i < currentMessageLines.size(); i++)
                estimatedOutputStringLength += currentMessagePrefix.length() + currentMessageLines[i].length() + currentMessageSuffix.length() + 1; // Prefix + text + suffix + line break
            outputString.reserve(estimatedOutputStringLength);

            // Complete previous message if necessary
            if (previousLineComplete == false && currentMessage.m_interfaceIndex != previousMessage.m_interfaceIndex) {
                outputString += previousMessageSuffix;
                outputString += '\n';
                previousLineComplete = true;
            }

            // Add lines of current message
            for (size_t i = 0; i < currentMessageLines.size(); i++) {
                // Prefix and text
                if (currentMessageLines[i].length() > 0) {
                    if (previousLineComplete == true)
                        outputString += currentMessagePrefix;
                    outputString += currentMessageLines[i];
                    previousLineComplete = false;
                }

                // Suffix
                if (i + 1 < currentMessageLines.size()) {
                    if (previousLineComplete == false)
                        outputString += currentMessageSuffix;
                    outputString += '\n';
                    previousLineComplete = true;
                }
            }

            // Update previous message
            previousMessage = currentMessage;

            // Pass back output string to write
            return outputString;
        }
    };
} // namespace io
} // namespace broccoli
