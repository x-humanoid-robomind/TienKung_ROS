#pragma once

#include "MotorDevice.h"
#include <memory>
#include <unordered_map>
#include <bodyctrl_msgs/MotorStatusMsg.h>

class MotorDeviceManager {
public:
    MotorDeviceManager();
    inline std::shared_ptr<MotorDevice> GetDevice(int name) {
        return mapByName.at(name);
    }
    void NewDevice(int name, uint16_t slave, int passage, uint16_t id, int type, double offset = 0);
    inline void SetOnStatusReady(std::function<void(bodyctrl_msgs::MotorStatusMsg::Ptr)> funcOnStatusReady) {
        this->funcOnStatusReady = funcOnStatusReady;
    }

    bool IsReady();
    std::vector<int> GetNotReadyList();

    const bool is_temperature_high(const float top_limit = 80.0f);

private:
    bodyctrl_msgs::MotorStatusMsg::Ptr msgStatus;
    std::unordered_map<uint16_t, std::shared_ptr<MotorDevice>> mapByName;
    std::unordered_map<int, bool> statusReady;
    std::function<void(bodyctrl_msgs::MotorStatusMsg::Ptr)> funcOnStatusReady;
    bool ready = false;

    void CheckAndSendStatus();
};