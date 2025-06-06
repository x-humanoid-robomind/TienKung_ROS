#pragma once

#include <vector>

#include "SoemMessage.h"

class SoemDevice {
public:
    SoemDevice();
    virtual bool IsReady();

protected:
    friend class SoemMaster;
    
    bool ready = false;

    inline void SetReady() {
        ready = true;
    }

    virtual void OnRequest(int passage, DeviceMessage& msg) = 0;
    virtual void OnResponse(int passage, DeviceMessage& msg) = 0;
};