/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

#include "../../core/Time.hpp"
#include <assert.h>
#include <stdint.h>

namespace broccoli {
namespace io {
    /*!
     * \brief Data container for actions to perform by the logger
     * \ingroup broccoli_io_logging
     */
    class LoggerAction {
    public:
        //! Specification of action types
        enum class Type : uint8_t {
            LOG_START = 0, //!< Start logging (\ref m_text contains `SecondLevelDirectory` and must be a **single** folder!)
            LOG_STOP, //!< Stop logging
            LOG_DUMP, //!< Dump log (=start log, write data objects buffered so far and immediately stop log) (\ref m_text contains `SecondLevelDirectory` and must be a **single** folder!)
            TYPE_COUNT //!< Total count of elements
        };

        //! Returns the string representation of the given action type
        static std::string typeString(const Type& type)
        {
            // Check type
            switch (type) {
            case Type::LOG_START:
                return "LOG_START";
            case Type::LOG_STOP:
                return "LOG_STOP";
            case Type::LOG_DUMP:
                return "LOG_DUMP";
            default: {
                assert(false);
                return "UNKNOWN";
            }
            }
        }

        //! Returns the string representation of the type of **this** action
        std::string typeString() const { return typeString(m_type); }

        //! Specialized constructor
        /*!
         * \param [in] type Initializes \ref m_type - \copybrief m_type
         * \param [in] text Initializes \ref m_text - \copybrief m_text
         * \param [in] time Initializes \ref m_time - \copybrief m_time
         */
        LoggerAction(const Type& type, const std::string& text, const core::Time& time = core::Time::currentTime())
            : m_type(type)
            , m_text(text)
            , m_time(time)
        {
        }

        //! Specialized constructor
        /*!
         * \param [in] type Initializes \ref m_type - \copybrief m_type
         * \param [in] time Initializes \ref m_time - \copybrief m_time
         */
        LoggerAction(const Type& type, const core::Time& time = core::Time::currentTime())
            : LoggerAction(type, "", time)
        {
        }

        //! Default constructor
        LoggerAction()
            : LoggerAction(Type::LOG_START)
        {
        }

        // Members
        Type m_type; //!< Type of the action
        std::string m_text; //!< Custom action text (may have different meanings)
        core::Time m_time; //!< System time of occurance
    };
} // namespace io
} // namespace broccoli
