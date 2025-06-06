#pragma once

#include "TsinghuaHandDevice.h"
#include <memory>
#include <unordered_map>
#include <bodyctrl_msgs/TsHandStatusMsg.h>

class TsinghuaHandDeviceManager {
public:
    TsinghuaHandDeviceManager();
    inline std::shared_ptr<TsinghuaHandDevice> GetDevice(int name) {
        return devs.at(name);
    }
    void NewDevice(int name, uint16_t slave, int passage, uint16_t id);
    inline void SetOnStatusReady(std::function<void(bodyctrl_msgs::TsHandStatusMsg::Ptr)> funcOnStatusReady) {
        this->funcOnStatusReady = funcOnStatusReady;
    }

    bool IsReady();
    std::vector<int> GetNotReadyList();

private:
    bodyctrl_msgs::TsHandStatusMsg::Ptr msgStatus;
    std::unordered_map<int, std::shared_ptr<TsinghuaHandDevice>> devs;
    std::unordered_map<int, bool> statusReady;
    std::function<void(bodyctrl_msgs::TsHandStatusMsg::Ptr)> funcOnStatusReady;
    bool ready = false;

    void CheckAndSendStatus();
};