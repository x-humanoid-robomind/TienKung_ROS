/*
 * This file is part of broccoli
 * Copyright (C) 2020 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once
#ifdef __APPLE__

#include "PlatformHelperPOSIX.hpp"

namespace broccoli {
namespace core {
    /*!
     * \brief PlatformHelper implementation for macOS
     * \ingroup broccoli_core_platform
     */
    class PlatformHelperMacOs : public PlatformHelperPOSIX {
    public:
        bool setThreadName(const std::string& name) const override
        {
            return pthread_setname_np(name.c_str()) == 0;
        }

    protected:
        PlatformHelperMacOs() {}
        friend class PlatformHelperFactory;
    };
} // namespace core
} // namespace broccoli
#endif // __APPLE__
