#pragma once

#include "SoemDevice.h"
#include "../../ImuData.h"
#include <mutex>
#include <functional>

namespace xsens_mode1 {

class XsensImuDevice : public SoemDevice {
public:
    XsensImuDevice(int slave, int passage1, uint16_t id1, int passage2, uint16_t id2, int passage3, uint16_t id3, 
        std::function<void(ImuData&)> funcOnMessage);
    void StartRealTime(float rate);

private:
    std::mutex mtxCmd;
    int passage1, passage2, passage3;
    uint16_t id1, id2, id3;
    ImuData data;
    bool readyMsg[3];
    int count = 0;
    static const int INIT_TIMES = 100;
    std::function<void(ImuData&)> funcOnMessage;

    XsensImuDevice() = default;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;

    void SendActiveRequest(int passage, DeviceMessage& msg);
    void quatToRPY(const double* q, double& roll, double& pitch, double& yaw);
    void quatToRPY2(const double* q, double& roll, double& pitch, double& yaw);
    void toEulerAngles(double& roll, double& pitch, double& yaw);
    int16_t GetInt16(uint8_t* src);
};

}