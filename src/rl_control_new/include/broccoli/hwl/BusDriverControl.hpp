/* 
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "variables.hpp"

namespace broccoli {
namespace hwl {

    /*!
     * \brief Abstract interface for the control of a BusDriver
     * \ingroup broccoli_hwl
     * \tparam BusType The bus description type
     */
    template <typename BusType>
    class BusDriverControl {
    public:
        virtual ~BusDriverControl() = default;

        /*!
         * \brief Returns the minimum duration between two process() calls on a BusDriver
         * \return Duration in microseconds
         */
        virtual std::size_t cycleTimeInUs() const noexcept = 0;

        /*!
         * \brief Request a certain newState for a bus (non-blocking)
         * \param newState The desired bus state
         */
        virtual void requestState(const typename BusType::StateType& newState) = 0;

        //! Returns the current state of the bus
        virtual typename BusType::StateType state() const noexcept = 0;

        //! Returns true if the bus state changed since the last cycle
        virtual bool stateChanged() const noexcept
        {
            return this->state() != previousState();
        }

        //! Returns the state of the bus in the last cycle
        virtual typename BusType::StateType previousState() const noexcept = 0;

        /*!
         * \brief Triggers transfer of an asynchronous bus variable
         *
         * Triggering the transfer fails when a transfer is already in progress.
         * In this case, false is returned by this method.
         * \tparam Derived The derived bus variable type
         * \param variable The async bus variable to trigger
         * \return True when successful
         */
        template <typename Derived>
        bool transfer(const BusVariableBase<Derived>& variable)
        {
            return delegateTransfer(std::hash<Derived>{}(variable));
        }

    protected:
        /*!
         * \brief Called from BusDevice to trigger a transfer for an asynchronous bus variable
         * \param hash Hash obtained via std::hash<VariableType> or std::hash<PointerToVariableType>
         * \return True when successful
         */
        virtual bool delegateTransfer(std::size_t hash) = 0;
    };

} // namespace hwl
} // namespace broccoli
