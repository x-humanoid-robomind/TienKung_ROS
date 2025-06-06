#include "MotorDeviceManager.h"
#include "../../soem_master/SoemMaster.h"

#define SLAVE_INDEX(X) (X << 8 & 0x00FF00)
#define MOTOR_ID(X) (X & 0x00FF)
#define MAP_KEY2(X, Y) (SLAVE_INDEX(X) | MOTOR_ID(Y))
#define MAP_KEY(X, Y) (X * 1000 + Y)

MotorDeviceManager::MotorDeviceManager() {
    msgStatus = bodyctrl_msgs::MotorStatusMsg::Ptr(new bodyctrl_msgs::MotorStatusMsg()); 
}

void MotorDeviceManager::NewDevice(int name, uint16_t slave, int passage, uint16_t id, int type, double offset) {
    auto motor = std::make_shared<MotorDevice>(
        slave, passage, id, (MotorDevice::Type)type,
        [this, name](float position, float speed, float current, float temperature) {

            bodyctrl_msgs::MotorStatus status;
            status.name = name;
            status.pos = position;
            status.speed = speed;
            status.current = current;
            status.temperature = temperature;
            
            msgStatus->status.push_back(status);
            statusReady[name] = true;

            CheckAndSendStatus();
        },
        [this, name]() {
            statusReady[name] = true;
            CheckAndSendStatus();
        });

    motor->SetZeroOffset(offset);

    mapByName.insert(std::make_pair(name, motor));
    statusReady[name] = false;
    SoemMaster::Instance().RegisterDevice(slave, passage, std::dynamic_pointer_cast<SoemDevice>(motor));
}

void MotorDeviceManager::CheckAndSendStatus() {
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

bool MotorDeviceManager::IsReady() {
    if (!ready) {
        bool readyAllDevices = true;
        for (auto& dev : mapByName) {
            readyAllDevices = (readyAllDevices && dev.second->IsReady());
        }
        ready = readyAllDevices;
    }

    return ready;
}

std::vector<int> MotorDeviceManager::GetNotReadyList() {
    std::vector<int> nrList;
    for (auto& dev : mapByName) {
        if(!dev.second->IsReady()) {
            nrList.emplace_back(dev.first);
        }
    }
    return nrList;
}

const bool MotorDeviceManager::is_temperature_high(const float top_limit)
{
    for(auto& dev : mapByName) {
        if(dev.second->is_temperature_high(top_limit)) {
            return true;
        }
    }
    return false;
}
