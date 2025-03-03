/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am/
 */

#pragma once

#include <queue>

namespace broccoli {
namespace hwl {
    namespace internal {
        template <typename T, typename Container>
        const T& frontOrTop(const std::priority_queue<T, Container>& queue)
        {
            return queue.top();
        };

        template <typename QueueType>
        const auto& frontOrTop(const QueueType& queue)
        {
            return queue.front();
        };

        template <typename Registry, typename BusVariableType, std::size_t N>
        static void registerArray(Registry& registry, const std::string& namePrefix, std::array<BusVariableType, N>& arrayOfVariables)
        {
            for (std::size_t i = 0; i < N; i++) {
                registry.registerVariable(arrayOfVariables[i], EtherCAT::ObjectIdentifierType{ namePrefix + std::to_string(i + 1) });
            }
        }
    } // namespace internal
} // namespace hwl
} // namespace broccoli
