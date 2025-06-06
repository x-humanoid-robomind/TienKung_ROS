#pragma once

#include "../../soem_master/SoemDevice.h"
#include "util/LockFreeQueue.h"
#include <mutex>
#include <functional>

class MotorDevice : public SoemDevice {
public:
    enum class Type : int {
        S_4310 = 1,
        M_8112,
        L_10020,
        H_13715
    }; 
    MotorDevice(int slave, int passage, uint16_t motorId, Type type, 
        std::function<void(float position, float speed, float current, float temperature)> funcOnStatus,
        std::function<void()> funcOnMiss);

    void SetMotorSpeed(float speed, float current);
    void SetMotorPosition(float position, float speed, float current);
    void SetMotorDistance(float distance, float speed, float current);
    void SendMotorCtrlCmd(float kp, float kd, float position, float speed, float torque);

    inline void SetZeroOffset(double offset) {
        zeroOffset = offset;
    }

    inline const bool is_temperature_high(const float top_limit = 80.0f) {
        return temperature >= top_limit;
    }

private:
    friend class MotorDeviceManager;
    int slave;
    int passage;
    uint16_t motorId;
    DeviceMessage reqBuff;
    LockFreeQueue<DeviceMessage*> reqQueue;
    Type type;
    std::function<void(float, float, float, float)> funcOnStatus;
    std::function<void()> funcOnMiss;
    std::mutex mtxSetting;

    float position;
    float speed;
    float current;
    float temperature;
    double zeroOffset = 0;

    MotorDevice() = default;
    void OnStatus(DeviceMessage& msg);

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;
};