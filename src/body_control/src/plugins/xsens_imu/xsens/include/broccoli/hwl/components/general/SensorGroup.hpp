/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include "Sensor.hpp"
#include <functional>
#include <vector>

namespace broccoli {
namespace hwl {
    /*!
     * \brief A group of Sensor objects
     * \ingroup broccoli_hwl_components
     */
    class SensorGroup : public Sensor {
    public:
        //! Default constructor, empty group
        SensorGroup() = default;

        /*!
         * \brief Construct from initializer list. The group stores references to the passed objects
         * \param initList Initializer list of Sensor objects
         */
        SensorGroup(std::initializer_list<std::reference_wrapper<Sensor>> initList)
            : m_groupList(initList)
        {
        }

        /*!
         * \brief Add a Sensor object's reference to the group
         * \param toAdd Reference to the object.
         */
        void add(Sensor& toAdd) { m_groupList.emplace_back(toAdd); }

        //! Returns true if all Sensors in the group are ready
        bool ready() const noexcept override
        {
            for (const auto& ref : m_groupList) {
                if (!ref.get().ready()) {
                    return false;
                }
            }
            return true;
        }

        /*!
         * \brief Zero all sensors
         * \return True if the operation was successful for all sensors in the group
         */
        bool zeroState() noexcept override
        {
            bool success = true;
            for (auto& ref : m_groupList) {
                if (!ref.get().zeroState())
                    success = false;
            }
            return success;
        }

    protected:
        //! A list of references of Sensor objects
        std::vector<std::reference_wrapper<Sensor>> m_groupList;
    };

} // namespace hwl
} // namespace broccoli
