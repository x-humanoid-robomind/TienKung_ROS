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
    class EL101X;

    /*!
     * \brief Implements an EtherCAT BusDevice for the Beckhoff EL101X digital input clamp with X channels
     * \ingroup broccoli_hwl_components
     * \tparam NrOfChannels_ Number of channels
     */
    template <std::size_t NrOfChannels_>
    class EL101X<NrOfChannels_, EtherCAT> : public BusDevice<EtherCAT>, public MultiChannelDevice<NrOfChannels_, uint8_t> {
    public:
        using BusType = EtherCAT;
        using Base = MultiChannelDevice<NrOfChannels_, uint8_t>;
        using Base::nrOfChannels;
        using typename Base::IndexType;
        using BusDevice<EtherCAT>::BusDevice;

        /*!
         * \brief Return the state of an input channel
         *
         * \warning Does return false when an invalid input channel is specified
         * \param index One-based index of the channel
         */
        bool input(IndexType index)
        {
            if (!this->assertChannelIndex(index))
                return false;

            return m_in[index - 1];
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
            for (IndexType i = 0; i < nrOfChannels(); i++) {
                registry.registerVariable(m_in[i], typename EtherCAT::ObjectIdentifierType{ this->channelVariableName(i + 1, "Input") });
            }
        }

    private:
        //! Bus variables for the input channels
        std::array<InputBusVariable<bool>, nrOfChannels()> m_in;
    };

    /*!
     * \brief 2-Channel EL101X device
     * \ingroup broccoli_hwl_components
     */
    template <typename BusType>
    using EL1012 = EL101X<2, BusType>;

    /*!
     * \brief 4-Channel EL101X device
     * \ingroup broccoli_hwl_components
     */
    template <typename BusType>
    using EL1014 = EL101X<4, BusType>;

    /*!
     * \brief 8-Channel EL101X device
     * \ingroup broccoli_hwl_components
     */
    template <typename BusType>
    using EL1018 = EL101X<8, BusType>;
} // namespace hwl
} // namespace broccoli
