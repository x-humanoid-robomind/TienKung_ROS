#pragma once

#include "EyouMotorDevice.h"
#include <memory>
#include <unordered_map>
#include <bodyctrl_msgs/MotorStatusMsg.h>

namespace eyou {

class EyouMotorDevMgr {
public:
    EyouMotorDevMgr();
    inline std::shared_ptr<EyouMotorDevice> GetDevice(int name) {
        return mapByName.at(name);
    }
    void NewDevice(int name, uint16_t slave, int passageReq, int passageResp, uint16_t id, int mode, int type, float kt = 0);
    inline void SetOnStatusReady(std::function<void(bodyctrl_msgs::MotorStatusMsg::Ptr)> funcOnStatusReady) {
        this->funcOnStatusReady = funcOnStatusReady;
    }

    bool IsReady();
    std::vector<int> GetNotReadyList();

private:
    bodyctrl_msgs::MotorStatusMsg::Ptr msgStatus;
    std::unordered_map<uint16_t, std::shared_ptr<EyouMotorDevice>> mapByName;
    std::unordered_map<int, bool> statusReady;
    std::function<void(bodyctrl_msgs::MotorStatusMsg::Ptr)> funcOnStatusReady;
    bool ready = false;

    void CheckAndSendStatus();
};

}