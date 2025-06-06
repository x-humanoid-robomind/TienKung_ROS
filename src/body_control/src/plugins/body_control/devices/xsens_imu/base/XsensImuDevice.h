#pragma once

#include "SoemDevice.h"
#include "../ImuData.h"
#include <mutex>
#include <functional>
#include "mode1/XsensImuDevice.h"

class XsensImuDevice : public xsens_mode1::XsensImuDevice {
public:
    XsensImuDevice(int slave, int passages[4], uint16_t ids[4], std::function<void(ImuData&)> funcOnMessage);
};