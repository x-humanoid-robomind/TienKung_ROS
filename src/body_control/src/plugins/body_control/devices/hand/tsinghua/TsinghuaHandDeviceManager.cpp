#include "TsinghuaHandDeviceManager.h"
#include "../../soem_master/SoemMaster.h"
#include <glog/logging.h>

#define SLAVE_INDEX(X) (X << 8 & 0x00FF00)
#define MOTOR_ID(X) (X & 0x00FF)
#define MAP_KEY2(X, Y) (SLAVE_INDEX(X) | MOTOR_ID(Y))
#define MAP_KEY(X, Y) (X * 1000 + Y)

TsinghuaHandDeviceManager::TsinghuaHandDeviceManager() {
    msgStatus = bodyctrl_msgs::TsHandStatusMsg::Ptr(new bodyctrl_msgs::TsHandStatusMsg()); 
}

void TsinghuaHandDeviceManager::NewDevice(int name, uint16_t slave, int passage, uint16_t id) {
    auto dev = std::make_shared<TsinghuaHandDevice>(
        slave, passage, id,
        [this, name](FingerStatusMessage& msg) {

            bodyctrl_msgs::TsHandStatus status;

            status.name = name;
            status.rotation_angle = msg.thumb.rotation.angle;
            status.bend_angle.resize(5);
            status.bend_angle[0] = msg.thumb.bend.angle;
            status.bend_angle[1] = msg.fore.bend.angle;
            status.bend_angle[2] = msg.middle.bend.angle;
            status.bend_angle[3] = msg.ring.bend.angle;
            status.bend_angle[4] = msg.little.bend.angle;
            
            msgStatus->status.push_back(status);
            statusReady[name] = true;

            CheckAndSendStatus();
        });

    devs.insert(std::make_pair(name, dev));
    statusReady[name] = false;

    SoemMaster::Instance().RegisterDevice(slave, passage, std::dynamic_pointer_cast<SoemDevice>(dev), SoemMaster::Mode::EXTENDED);
}

void TsinghuaHandDeviceManager::CheckAndSendStatus() {
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
            msgStatus = bodyctrl_msgs::TsHandStatusMsg::Ptr(new bodyctrl_msgs::TsHandStatusMsg()); 
        }
    }
}

bool TsinghuaHandDeviceManager::IsReady() {
    if (!ready) {
        bool readyAllDevices = true;
        for (auto& dev : devs) {
            readyAllDevices = (readyAllDevices && dev.second->IsReady());
        }
        ready = readyAllDevices;
    }

    return ready;
}

std::vector<int> TsinghuaHandDeviceManager::GetNotReadyList() {
    std::vector<int> nrList;
    for (auto& dev : devs) {
        if(!dev.second->IsReady()) {
            nrList.emplace_back(dev.first);
        }
    }
    return nrList;
}