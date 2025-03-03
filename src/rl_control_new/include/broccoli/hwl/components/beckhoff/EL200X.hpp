/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "../../BusDevice.hpp"
#include "../../bus_types/EtherCAT.hpp"
#include "../../variables.hpp"
#include "MultiChannelDevice.hpp"

namespace broccoli {
namespace hwl {

    template <std::size_t, typename>
    class EL200X;

    /*!
     * \brief Implements an EtherCAT BusDevice for the Beckhoff EL200X digital output clamp with X channels
     * \ingroup broccoli_hwl_components
     * \tparam NrOfChannels_ Number of channels
     */
    template <std::size_t NrOfChannels_>
    class EL200X<NrOfChannels_, EtherCAT> : public BusDevice<EtherCAT>, public MultiChannelDevice<NrOfChannels_, uint8_t> {
    public:
        using BusType = EtherCAT;
        using Base = MultiChannelDevice<NrOfChannels_, uint8_t>;
        using Base::nrOfChannels;
        using typename Base::IndexType;
        using BusDevice<EtherCAT>::BusDevice;

        /*!
         * \brief Set the digital output value of a channel
         * \param index One-based index of the channel
         * \param enabled Desired output channel state
         */
        void setOutput(IndexType index, bool enabled)
        {
            if (!this->assertChannelIndex(index))
                return;

            m_out[index - 1] = enabled;
        }

        /*!
         * \brief Toggle an output state
         * \param index One-based index of the channel to toggle
         */
        void toggleOutput(IndexType index)
        {
            if (!this->assertChannelIndex(index))
                return;

            m_out[index - 1] = !m_out[index - 1];
        }

        void processDevice() override
        {
        }

        void onStateChange() override
        {
        }

        /*!
         * \brief Register the bus variable of this device
         * \param registry The BusVariableRegistry to register to
         */
        template <typename Derived>
        void linkVariables(BusVariableRegistryBase<Derived>& registry)
        {
            for (typename Base::IndexType i = 0; i < nrOfChannels(); i++) {
                registry.registerVariable(m_out[i], typename EtherCAT::ObjectIdentifierType{ this->channelVariableName(i + 1, "Output") });
            }
        }

    private:
        //! Bus variables for the output channels
        std::array<OutputBusVariable<bool>, nrOfChannels()> m_out;
    };

    /*!
     * \brief 2-Channel EL200X device
     * \ingroup broccoli_hwl_components
     */
    template <typename BusType>
    using EL1002 = EL200X<2, BusType>;

    /*!
     * \brief 4-Channel EL200X device
     * \ingroup broccoli_hwl_components
     */
    template <typename BusType>
    using EL2004 = EL200X<4, BusType>;

    /*!
     * \brief 8-Channel EL200X device
     * \ingroup broccoli_hwl_components
     */
    template <typename BusType>
    using EL2008 = EL200X<8, BusType>;
} // namespace hwl
} // namespace broccoli
