#include "SriDeviceManager.h"
#include "../../soem_master/SoemMaster.h"

#define SLAVE_INDEX(X) (X << 8 & 0x00FF00)
#define MOTOR_ID(X) (X & 0x00FF)
#define MAP_KEY2(X, Y) (SLAVE_INDEX(X) | MOTOR_ID(Y))
#define MAP_KEY(X, Y) (X * 1000 + Y)

SriDeviceManager::SriDeviceManager() {
}

void SriDeviceManager::NewDevice(int name, int slave, int idCmd, int passageCmd, int idData[3], int passageData[3]) {
    auto dev = std::make_shared<SriSensorDevice>(
        slave, idCmd, passageCmd, idData, passageData, 
        [this, name](float fx, float fy, float fz, float mx, float my, float mz) {

            auto msg = bodyctrl_msgs::Sri::Ptr(new bodyctrl_msgs::Sri()); 
            msg->name = name;
            msg->fx = fx;
            msg->fy = fy;
            msg->fz = fz;
            msg->mx = mx;
            msg->my = my;
            msg->mz = mz;
            funcOnStatusReady(msg);
        });

    devs.insert(std::make_pair(name, dev));

    SoemMaster::Instance().RegisterDevice(slave, {passageCmd, passageData[0], passageData[1], passageData[2]}, std::dynamic_pointer_cast<SoemDevice>(dev));
}

bool SriDeviceManager::IsReady() {
    if (!ready) {
        bool readyAllDevices = true;
        for (auto& dev : devs) {
            readyAllDevices = (readyAllDevices && dev.second->IsReady());
        }
        ready = readyAllDevices;
    }

    return ready;
}

std::vector<int> SriDeviceManager::GetNotReadyList() {
    std::vector<int> nrList;
    for (auto& dev : devs) {
        if(!dev.second->IsReady()) {
            nrList.emplace_back(dev.first);
        }
    }
    return nrList;
}