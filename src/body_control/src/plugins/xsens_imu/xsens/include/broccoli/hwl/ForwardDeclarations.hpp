/*
 * This file is part of broccoli.
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

// For type injection
#ifndef BROCCOLI_HWL_MUTEX_TYPE
#define BROCCOLI_HWL_MUTEX_TYPE std::shared_timed_mutex
#endif

#ifndef BROCCOLI_HWL_READ_LOCK_TYPE
#define BROCCOLI_HWL_READ_LOCK_TYPE std::shared_lock
#endif

#ifndef BROCCOLI_HWL_WRITE_LOCK_TYPE
#define BROCCOLI_HWL_WRITE_LOCK_TYPE std::unique_lock
#endif

namespace broccoli {

/*!
 * \brief Hardware Layer module namespace
 */
namespace hwl {

    template <typename>
    class BusDriver;

    template <typename>
    class BusVariableRegistryBase;

    template <typename>
    class BusVariableBase;

    template <typename>
    class BusVariableValue;

    template <typename>
    class BusDriverControl;

    template <typename>
    class BusVariableImpl;

    template <typename>
    class AsyncBusVariableValue;

    class AsyncTransferStateHandle;
    class AsyncTransferState;

    template <typename ContainerType_, template <typename> class ImplType_>
    class ReadableBusVariable;

    template <typename ContainerType_, template <typename> class ImplType_>
    class WriteableBusVariable;

    /*!
     * \brief Internal hwl namespace
     * \ingroup broccoli_hwl
     */
    namespace internal {
        //! Bus variable type trait struct
        template <typename>
        struct BusVariableTypes;
    }

} // namespace hwl
} // namespace broccoli
