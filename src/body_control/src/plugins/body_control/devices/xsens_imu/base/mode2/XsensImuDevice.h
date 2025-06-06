#pragma once

#include "SoemDevice.h"
#include "../../ImuData.h"
#include <mutex>
#include <functional>

namespace xsens_mode2 {

class XsensImuDevice : public SoemDevice {
public:
    XsensImuDevice(int slave, int passages[4], uint16_t ids[4], std::function<void(ImuData&)> funcOnMessage);
    void StartRealTime(float rate);

private:
    std::mutex mtxCmd;
    int passages[4];
    uint16_t ids[4];
    ImuData data;
    bool readyMsg[4];
    int count = 0;
    static const int INIT_TIMES = 100;
    std::function<void(ImuData&)> funcOnMessage;

    XsensImuDevice() = default;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;

    void SendActiveRequest(int passage, DeviceMessage& msg);
    int16_t GetInt16(uint8_t* src);
};

}