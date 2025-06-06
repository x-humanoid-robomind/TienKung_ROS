#pragma once

#include <cstdint>
#include <vector>

#pragma pack(push, 1)

struct DeviceMessage {
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];

    DeviceMessage() = default;

    DeviceMessage(uint32_t id, uint8_t rtr, std::vector<uint8_t> data) {
        this->id = id;
        this->rtr = rtr;
        this->dlc = data.size();
        for (auto i = 0; i < data.size(); ++i) {
            this->data[i] = data[i];
        }
    }

    // 委托构造函数
    DeviceMessage(uint32_t id, std::vector<uint8_t> data) : DeviceMessage(id, 0, data) {}
};
 


struct SlaveMessage
{
    uint8_t dev_num;
    uint8_t can_ide;
    DeviceMessage dev[6];
};

#pragma pack(pop)

