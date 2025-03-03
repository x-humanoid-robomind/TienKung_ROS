/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once

#include "PlatformHelperMacOs.hpp"
#include "PlatformHelperPOSIX.hpp"
#include "PlatformHelperQNX.hpp"
#include <memory>

namespace broccoli {
namespace core {
    /*!
     * \brief A factory to create a PlatformHelper object for the current platform.
     * \ingroup broccoli_core_platform
     */
    class PlatformHelperFactory {
    public:
        //! Create a new instance of a suitable PlatformHelper
        static std::unique_ptr<PlatformHelper> create()
        {
#if defined(__QNXNTO__)
            return std::unique_ptr<PlatformHelper>(new PlatformHelperQNX());
#elif defined(__APPLE__)
            return std::unique_ptr<PlatformHelper>(new PlatformHelperMacOs());
#endif
            // Standard POSIX is default
            return std::unique_ptr<PlatformHelper>(new PlatformHelperPOSIX());
        }
    };
} // namespace core
} // namespace broccoli
