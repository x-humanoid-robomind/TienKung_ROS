/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "Faultable.hpp"
#include <functional>
#include <vector>

namespace broccoli {
namespace hwl {
    /*!
     * \brief A group of Faultable objects
     * \ingroup broccoli_hwl_components
     */
    class FaultableGroup : public Faultable {
    public:
        //! Default constructor, empty group
        FaultableGroup() = default;

        /*!
         * \brief Construct from initializer list. The group stores references to the passed objects
         * \param initList Initializer list of Faultable objects
         */
        FaultableGroup(std::initializer_list<std::reference_wrapper<Faultable>> initList)
            : m_groupList(initList)
        {
        }

        /*!
         * \brief Add a Faultable object's reference to the group
         * \param toAdd Reference to the object.
         */
        void add(Faultable& toAdd) { m_groupList.emplace_back(toAdd); }

        /*!
         * \brief Is any of the group's devices in fault?
         * \return True on fault
         */
        bool onFault() const noexcept override
        {
            for (const auto& ref : m_groupList) {
                if (ref.get().onFault()) {
                    return true;
                }
            }
            return false;
        }

        /*!
         * \brief Reset fault state of all group devices
         * This resets any fault flags and enables continued operation after a fault.
         */
        void resetFault() noexcept override
        {
            for (auto& ref : m_groupList) {
                if (ref.get().onFault()) {
                    ref.get().resetFault();
                }
            }
        }

    protected:
        //! A list of references of Faultable objects
        std::vector<std::reference_wrapper<Faultable>> m_groupList;
    };

} // namespace hwl
} // namespace broccoli
