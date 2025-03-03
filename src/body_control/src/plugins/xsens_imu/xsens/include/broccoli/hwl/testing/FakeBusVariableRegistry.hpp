/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "../variables.hpp"
#include "../variables/pointers.hpp"
#include "broccoli/hwl/BusVariableRegistryBase.hpp"
#include "gtest/gtest.h"

#include <unordered_map>

namespace broccoli {
namespace hwl {

    /*!
     * \brief Namespace containing test suites and mocks
     * \ingroup broccoli_hwl
     */
    namespace testing {

        /*!
         * \brief Fake bus variable registry for testing purposes
         * \tparam BusType_ Type of bus
         */
        template <typename BusType_>
        class FakeBusVariableRegistry : public BusVariableRegistryBase<FakeBusVariableRegistry<BusType_>> {
        public:
            using BusType = BusType_;

            virtual ~FakeBusVariableRegistry() = default;

            //! Test if registry is valid
            virtual void testIfValid() const
            {
                EXPECT_TRUE(!m_syncInputVariables.empty() || !m_syncOutputVariables.empty() || !m_asyncInputVariables.empty() || !m_asyncOutputVariables.empty());
            }

            /*!
             * \brief Register the following synchronously-updated output BusVariable
             * \param variable The variable to register
             * \param id The object identifier to register to
             * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
             */
            template <typename T>
            void registerVariable(OutputBusVariable<T>& variable,
                const typename BusType::OutputObjectIdentifierType& id, bool isLittleEndian = true)
            {
                m_syncOutputVariables.emplace(std::make_pair(id, BusVariablePointer(variable, isLittleEndian)));
            }

            /*!
             * \brief Register the following synchronously-updated input BusVariable
             * \param variable The variable to register
             * \param id The object identifier to register to
             * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
             */
            template <typename T>
            void registerVariable(InputBusVariable<T>& variable,
                const typename BusType::InputObjectIdentifierType& id, bool isLittleEndian = true)
            {
                m_syncInputVariables.emplace(std::make_pair(id, BusVariablePointer(variable, isLittleEndian)));
            }

            /*!
            * \brief Register the following asynchronously-updated input BusVariable
            * \param variable The variable to register
            * \param id The object identifier to register to
            * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
            */
            template <typename T>
            void registerVariable(AsyncInputBusVariable<T>& variable,
                const typename BusType::AsyncInputObjectIdentifierType& id, bool isLittleEndian = true)
            {
                m_asyncInputVariables.emplace(std::make_pair(id, AsyncBusVariablePointer(variable, isLittleEndian)));
            }

            /*!
             * \brief Register the following asynchronously-updated output BusVariable
             * \param variable The variable to register
             * \param id The object identifier to register to
             * \param isLittleEndian True if linked data (data the bus variable links to) should be interpreted little endian
             */
            template <typename T>
            void registerVariable(AsyncOutputBusVariable<T>& variable,
                const typename BusType::AsyncOutputObjectIdentifierType& id, bool isLittleEndian = true)
            {
                m_asyncOutputVariables.emplace(std::make_pair(id, AsyncBusVariablePointer(variable, isLittleEndian)));
            }

            /*!
             * \brief Register the following emergency object
             * \param variable The variable to register
             */
            void registerVariable(AsyncInputBusVariable<typename BusType::EmergencyObject>& variable,
                const typename BusType::EmergencyObjectIdentifier&)
            {
                m_emergencyObjects.emplace_back(AsyncBusVariablePointer(variable));
            }

            /*!
             * \brief Check if a synchronous variable is registered
             * \param id Object identifier
             * \return True if a synchronous output variable is registered to the specified identifier
             */
            bool syncOutputVariableRegistered(const typename BusType::OutputObjectIdentifierType& id) const
            {
                return m_syncOutputVariables.find(id) != m_syncOutputVariables.end();
            }

            /*!
             * \brief Check if a synchronous variable is registered
             * \param id Object identifier
             * \return True if a synchronous input variable is registered to the specified identifier
             */
            bool syncInputVariableRegistered(const typename BusType::InputObjectIdentifierType& id) const
            {
                return m_syncInputVariables.find(id) != m_syncInputVariables.end();
            }

            /*!
             * \brief Check if an asynchronous variable is registered
             * \param id Object identifier
             * \return True if a synchronous input variable is registered to the specified identifier
             */
            bool asyncInputVariableRegistered(const typename BusType::AsyncInputObjectIdentifierType& id) const
            {
                return m_asyncInputVariables.find(id) != m_asyncInputVariables.end();
            }

            /*!
             * \brief Check if an asynchronous variable is registered
             * \param id Object identifier
             * \return True if a synchronous output variable is registered to the specified identifier
             */
            bool asyncOutputVariableRegistered(const typename BusType::AsyncOutputObjectIdentifierType& id) const
            {
                return m_asyncOutputVariables.find(id) != m_asyncOutputVariables.end();
            }

            /*!
             * \brief Find synchronous output variable and return its value
             * \param id Object identifier
             * \return The value of the registered variable
             */
            template <typename T>
            T syncOutputVariable(const typename BusType::OutputObjectIdentifierType& id) const
            {
                return m_syncOutputVariables.at(id).unsafeRef().template indirect<T>();
            }

            /*!
             * \brief Find synchronous input variable and return its value
             * \param id Object identifier
             * \return The value of the registered variable
             */
            template <typename T>
            T syncInputVariable(const typename BusType::InputObjectIdentifierType& id) const
            {
                return m_syncInputVariables.at(id).unsafeRef().template indirect<T>();
            }

            /*!
             * \brief Find synchronous variable and set its value
             * \param id Object identifier
             * \param value The value to set
             */
            template <typename T>
            void setSyncInputVariable(const typename BusType::InputObjectIdentifierType& id, T&& value)
            {
                m_syncInputVariables.at(id).unsafeRef().template indirect<std::decay_t<T>>() = value;
            }

            /*!
             * \brief Find asynchronous output variable and return its value
             * \param id Object identifier
             * \return The value of the registered variable
             */
            template <typename T>
            T asyncOutputVariable(const typename BusType::AsyncOutputObjectIdentifierType& id) const
            {
                return m_asyncOutputVariables.at(id).unsafeRef().template indirect<T>();
            }

            /*!
             * \brief Find asynchronous input variable and return its value
             * \param id Object identifier
             * \return The value of the registered variable
             */
            template <typename T>
            T asyncInputVariable(const typename BusType::AsyncInputObjectIdentifierType& id) const
            {
                return m_asyncInputVariables.at(id).unsafeRef().template indirect<T>();
            }

            /*!
             * \brief Find asynchronous variable and set its value
             * \param id Object identifier
             * \param value The value to set
             */
            template <typename T>
            void setAsyncInputVariable(const typename BusType::AsyncInputObjectIdentifierType& id, T&& value)
            {
                m_asyncInputVariables.at(id).unsafeRef().template indirect<std::decay_t<T>>() = value;
            }

        protected:
            std::unordered_map<typename BusType::OutputObjectIdentifierType, const BusVariablePointer> m_syncOutputVariables;
            std::unordered_map<typename BusType::InputObjectIdentifierType, BusVariablePointer> m_syncInputVariables;
            std::unordered_map<typename BusType::AsyncInputObjectIdentifierType, AsyncBusVariablePointer> m_asyncInputVariables;
            std::unordered_map<typename BusType::AsyncOutputObjectIdentifierType, AsyncBusVariablePointer> m_asyncOutputVariables;
            std::vector<AsyncBusVariablePointer> m_emergencyObjects;
        };
    } // namespace testing
} // namespace hwl
} // namespace broccoli
