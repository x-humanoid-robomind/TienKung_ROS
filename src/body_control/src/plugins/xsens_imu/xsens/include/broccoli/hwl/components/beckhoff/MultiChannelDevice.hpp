/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include <cassert>
#include <cinttypes>
#include <string>

namespace broccoli {
namespace hwl {

    /*!
     * \brief A base class for multi-channel Beckhoff devices.
     *
     * Contains code to determine the variable names of multi-channel devices
     * \tparam NrOfChannels_ The number of channels of the device
     * \tparam IndexType_ The channel index type
     */
    template <uint8_t NrOfChannels_, typename IndexType_ = uint8_t>
    class MultiChannelDevice {
    public:
        //! The index data type
        using IndexType = IndexType_;

        //! Returns the number of channels
        static constexpr uint8_t nrOfChannels() { return NrOfChannels_; }

        /*!
         * \brief Generates a full channel variable name
         * \param channelIndex One-based channel index
         * \param subVariableName Name of the channel-subvariable
         * \return Channel-variable name
         */
        static std::string channelVariableName(IndexType channelIndex, const std::string& subVariableName)
        {
            return channelName(channelIndex) + std::string(".") + subVariableName;
        }

        /*!
         * \brief Generates a channel-name string from the channel number
         * \param channelIndex One-based channel index
         * \return Channel-name string
         */
        static std::string channelName(IndexType channelIndex)
        {
            return std::string("Channel ") + std::to_string(channelIndex);
        }

    protected:
        /*!
         * \brief Asserts channel index is valid
         * \param index One-based channel-index
         * \return True if index is valid
         */
        static bool assertChannelIndex(IndexType index)
        {
            assert(isValidChannelIndex(index) && "Invalid channel number");
            return isValidChannelIndex(index);
        }

        /*!
         * \brief Validator for channel index
         * \param index One-based channel-index
         * \return True if index is valid
         */
        static bool isValidChannelIndex(IndexType index)
        {
            return index > 0 && index <= nrOfChannels();
        }
    };

} // namespace hwl
} // namespace broccoli
