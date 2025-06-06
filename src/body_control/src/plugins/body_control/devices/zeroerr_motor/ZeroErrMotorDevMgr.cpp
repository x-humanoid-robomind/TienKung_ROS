#include "ZeroErrMotorDevMgr.h"
#include "../../soem_master/SoemMaster.h"

#define SLAVE_INDEX(X) (X << 8 & 0x00FF00)
#define MOTOR_ID(X) (X & 0x00FF)
#define MAP_KEY2(X, Y) (SLAVE_INDEX(X) | MOTOR_ID(Y))
#define MAP_KEY(X, Y) (X * 1000 + Y)

ZeroErrMotorDevMgr::ZeroErrMotorDevMgr() {
    msgStatus = bodyctrl_msgs::MotorStatusMsg::Ptr(new bodyctrl_msgs::MotorStatusMsg()); 
}

void ZeroErrMotorDevMgr::NewDevice(int name, uint16_t slave, int passageReq, int passageResp, uint16_t id, int mode, int type, float kt, float offset) {
    auto motor = std::make_shared<ZeroErrMotorDevice>(
        slave, passageReq, passageResp, id, ZeroErrMotorDevice::Mode(mode), (ZeroErrMotorDevice::Type)type, kt,
        [this, name](MotorStatusMessage& status) {

            bodyctrl_msgs::MotorStatus msg;
            msg.name = name;
            msg.pos = status.pos;
            msg.speed = status.vel;
            msg.current = status.cur;
            
            msgStatus->status.push_back(msg);
            statusReady[name] = true;

            CheckAndSendStatus();
        });

    motor->SetZeroOffset(offset);

    mapByName.insert(std::make_pair(name, motor));
    statusReady[name] = false;
    SoemMaster::Instance().RegisterDevice(slave, {passageReq, passageResp}, std::dynamic_pointer_cast<SoemDevice>(motor));
}

void ZeroErrMotorDevMgr::CheckAndSendStatus() {
    if (IsReady()) {
        bool readyAllStatus = true;
        for (auto ready : statusReady) {
            readyAllStatus = (readyAllStatus && ready.second);
        }
        if (readyAllStatus) {
            funcOnStatusReady(msgStatus);
            for (auto& ready : statusReady) {
                ready.second = false;
            }
            msgStatus = bodyctrl_msgs::MotorStatusMsg::Ptr(new bodyctrl_msgs::MotorStatusMsg()); 
        }
    }
}

bool ZeroErrMotorDevMgr::IsReady() {
    if (!ready) {
        bool readyAllDevices = true;
        for (auto& dev : mapByName) {
            readyAllDevices = (readyAllDevices && dev.second->IsReady());
        }
        ready = readyAllDevices;
    }

    return ready;
}

std::vector<int> ZeroErrMotorDevMgr::GetNotReadyList() {
    std::vector<int> nrList;
    for (auto& dev : mapByName) {
        if(!dev.second->IsReady()) {
            nrList.emplace_back(dev.first);
        }
    }
    return nrList;
}