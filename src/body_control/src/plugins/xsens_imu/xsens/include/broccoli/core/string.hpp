/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

#include <stdint.h>
#include <string>
#include <vector>

namespace broccoli {
namespace core {
    /*!
     * \addtogroup broccoli_core_string
     * \{
     */

    //! Splits the given string into a vector of multiple strings
    /*!
     * \param [in] inputString The input string to be splitted
     * \param [in] delimiter Delimiting character used for splitting
     * \param [in] skipEmpty If `true`, empty strings are not added to the list of strings
     * \return Vector of strings representing the splitted parts
     */
    static inline std::vector<std::string> stringSplit(const std::string& inputString, const char& delimiter, const bool& skipEmpty = false)
    {
        // Initialize helpers
        std::vector<std::string> returnValue;
        if (skipEmpty == false)
            returnValue.reserve(inputString.size() + 1);
        else
            returnValue.reserve(inputString.size() / 2 + 1);

        // Run through input string
        size_t startIndex = 0; // Index of first character of current part in input string
        for (size_t i = 0; i < inputString.size(); i++) {
            // Check, if current character is the delimiter
            if (inputString[i] == delimiter) {
                // Check if delimiter is at beginning of string
                if (i == 0) {
                    // Add empty string
                    if (skipEmpty == false)
                        returnValue.push_back("");
                } else {
                    // Determine end-index
                    const size_t endIndex = i - 1; // Index of last character of current part in input string

                    // Check, if part is empty
                    if (endIndex < startIndex) {
                        if (skipEmpty == false)
                            returnValue.push_back("");
                    } else
                        returnValue.push_back(inputString.substr(startIndex, endIndex - startIndex + 1));
                }

                // Set start-index of next part
                startIndex = i + 1;
            }
        }

        // Add "last" part
        if (startIndex < inputString.size())
            returnValue.push_back(inputString.substr(startIndex, inputString.size() - startIndex));
        else if (skipEmpty == false)
            returnValue.push_back("");

        // Pass back list of parts
        return returnValue;
    }

    //! Checks, if the given string starts with the specified other string
    /*!
     * \param [in] baseString The basic string to investigate
     * \param [in] startString The characters to be searched for at the beginning of the base string.
     * \return `true` if \p baseString starts with \p startString, `false` otherwise
     */
    static inline bool stringStartsWith(const std::string& baseString, const std::string& startString)
    {
        return baseString.rfind(startString, 0) == 0;
    }

    //! Checks, if the given string ends with the specified other string
    /*!
     * \param [in] baseString The basic string to investigate
     * \param [in] endString The characters to be searched for at the end of the base string.
     * \return `true` if \p baseString ends with \p endString, `false` otherwise
     */
    static inline bool stringEndsWith(const std::string& baseString, const std::string& endString)
    {
        // Check dimensions
        if (baseString.size() < endString.size())
            return false;
        else
            return baseString.compare(baseString.size() - endString.size(), endString.size(), endString) == 0;
    }

    //! \}
} // namespace core
} // namespace broccoli
