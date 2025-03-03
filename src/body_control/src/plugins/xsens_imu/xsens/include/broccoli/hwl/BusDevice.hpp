/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../core/Nameable.hpp"
#include "BusDriverControl.hpp"
#include "ForwardDeclarations.hpp"

namespace broccoli {
namespace hwl {

    /*!
     * \brief Defines the abstract interface for a bus device
     * \ingroup broccoli_hwl
     *
     * A BusDevice implements logic to abstract a physical or logical device on a real-time communication bus.
     * The implementation refers to a certain bus description type (technology).
     * \tparam BusType The bus description type, e.g., broccoli::hwl::EtherCAT
     */
    template <typename BusType>
    class BusDevice : public core::Nameable {
    public:
        /*!
         * \brief Construct for specified bus control instance
         * \param bus Reference to the bus to allow the device reading and setting the bus state
         * and to transfer asynchronous messages
         */
        explicit BusDevice(BusDriverControl<BusType>& bus)
            : m_bus(bus)
        {
        }
        virtual ~BusDevice() = default;

    protected:
        /*!
         * \brief Process logic of this device
         *
         * This method is called cyclically from the BusDriver this device is attached to.
         * The duration between cyclic calls may be retrieved via BusDriverControl::cycleTimeInUs().
         * Its implementation must not block the program flow and should be implemented with
         * real-time constraints in mind.
         *
         * The calling thread is the same as the calling thread of the associated BusDriver::process() method.
         */
        virtual void processDevice() = 0;

        /*!
         * \brief Event handler called from BusDriver when the state of the bus changes.
         *
         * This is called before process() of a cycle. onStateChange() and process() are run in the same thread.
         */
        virtual void onStateChange() = 0;

        //! Returns a reference to the BusDriverControl instance of the bus
        BusDriverControl<BusType>& bus() noexcept { return m_bus.get(); }

        //! Returns a const reference to the BusDriverControl instance of the bus
        const BusDriverControl<BusType>& bus() const noexcept { return m_bus.get(); }

        //! Only BusDrivers may call private interface
        template <typename T>
        friend class BusDriver;

    private:
        std::reference_wrapper<BusDriverControl<BusType>> m_bus;
    };

} // namespace hwl
} // namespace broccoli
