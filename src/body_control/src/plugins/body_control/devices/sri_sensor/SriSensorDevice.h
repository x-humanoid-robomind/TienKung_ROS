#pragma once

#include "../../soem_master/SoemDevice.h"
#include <mutex>
#include <functional>

class SriSensorDevice : public SoemDevice {
public:
    SriSensorDevice(int slave, int idCmd, int passageCmd, int idData[3], int passageData[3],
        std::function<void(float fx, float fy, float fz, float mx, float my, float mz)> funcOnStatus);
    void StartRealTime(float rate);

private:
    std::mutex mtxCmd;
    int slave;
    uint16_t idCmd;
    int passageCmd;
    uint16_t idData[3];
    int passageData[3];
    float fx, fy, fz, mx, my, mz;
    bool readyMsg[3];
    int count = 0;
    static const int INIT_TIMES = 5;
    std::function<void(float fx, float fy, float fz, float mx, float my, float mz)> funcOnStatus;

    SriSensorDevice() = default;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;

    void StartRealTime(int passage, DeviceMessage& msg);
};