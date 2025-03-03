/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include <string>
#include <utility>

namespace broccoli {
namespace core {
    /*!
     * \brief Objects with a name
     * \ingroup broccoli_core
     */
    class Nameable {
    public:
        //! Default constructor (empty name)
        Nameable() = default;

        /*!
         * \brief Construct with certain name
         * \param name Name of the object
         */
        explicit Nameable(const std::string& name)
            : m_name(name)
        {
        }

        /*!
         * \brief Construct with certain name
         * \param name Name of the object
         */
        explicit Nameable(std::string&& name)
            : m_name(std::move(name))
        {
        }

        //! Returns the name of this Nameable
        const std::string& name() const noexcept { return m_name; }

        /*!
         * \brief Sets the name of this Nameable
         * \param name The name of this object
         */
        void setName(const std::string& name) noexcept { m_name = name; }

        /*!
         * \brief Prefixes a string with the object's name
         * \param str The string to prefix with the name
         * \param delimiter The delimiting char between name and the string
         * \return A string consisting of object name(), delimiter, and str.
         */
        std::string prefixName(const std::string& str, const std::string& delimiter = " ") const noexcept
        {
            return name().empty() ? str : name() + delimiter + str;
        }

    private:
        std::string m_name;
    };

} // namespace core
} // namespace broccoli
