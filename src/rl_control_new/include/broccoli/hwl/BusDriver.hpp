/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "BusDevice.hpp"
#include "BusDriverControl.hpp"
#include <vector>

namespace broccoli {
namespace hwl {

    namespace internal {
        //! Traits object for BusDriver objects
        template <typename BusType>
        struct BusDriverTraits {
            using DeviceContainerType = std::vector<std::reference_wrapper<BusDevice<BusType>>>;
        };
    }

    /*!
     * \brief Defines the abstract interface for a bus driver
     *
     * A bus driver implements the interface between a certain bus technology and the
     * bus variables of multiple devices attached to this driver.
     * \ingroup broccoli_hwl
     * \tparam BusType The bus description type
     */
    template <typename BusType>
    class BusDriver : public BusDriverControl<BusType> {
    public:
        using DeviceContainerType = typename internal::BusDriverTraits<BusType>::DeviceContainerType;
        using BusDriverControl<BusType>::stateChanged;

        virtual ~BusDriver() = default;

        /*!
         * \brief Executes both bus driver tasks and tasks of attached devices
         *
         * This method must be called cyclically from the user. The synchronization
         * and specific call order depends on the specific implementation of the BusDriver.
         */
        virtual void process()
        {
            processBus();
            processDevices();
            m_previousState = this->state();
        }

        typename BusType::StateType previousState() const noexcept override
        {
            return m_previousState;
        }

        //! Returns an iterable container of all registered devices
        virtual const DeviceContainerType& devices() const = 0;

    protected:
        /*!
         * \brief Process bus driver tasks here
         *
         * This is cyclically called via process().
         */
        virtual void processBus() = 0;

        //! Processes all devices
        virtual void processDevices()
        {
            for (auto& device : devices()) {
                if (this->stateChanged()) {
                    device.get().onStateChange();
                }

                device.get().processDevice();
            }
        }

        //! Returns an iterable container of all registered devices
        virtual DeviceContainerType& devices() = 0;

    private:
        typename BusType::StateType m_previousState;
    };

} // namespace hwl
} // namespace broccoli
