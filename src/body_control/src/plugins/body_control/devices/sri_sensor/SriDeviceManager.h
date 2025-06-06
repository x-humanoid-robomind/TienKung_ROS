#pragma once

#include "SriSensorDevice.h"
#include <memory>
#include <unordered_map>
#include <bodyctrl_msgs/Sri.h>

class SriDeviceManager {
public:
    SriDeviceManager();
    inline std::shared_ptr<SriSensorDevice> GetDevice(int name) {
        return devs.at(name);
    }
    void NewDevice(int name, int slave, int idCmd, int passageCmd, int idData[3], int passageData[3]);
    inline void SetOnStatusReady(std::function<void(bodyctrl_msgs::Sri::Ptr)> funcOnStatusReady) {
        this->funcOnStatusReady = funcOnStatusReady;
    }

    bool IsReady();
    std::vector<int> GetNotReadyList();

private:
    std::unordered_map<int, std::shared_ptr<SriSensorDevice>> devs;
    std::function<void(bodyctrl_msgs::Sri::Ptr)> funcOnStatusReady;
    bool ready = false;
};