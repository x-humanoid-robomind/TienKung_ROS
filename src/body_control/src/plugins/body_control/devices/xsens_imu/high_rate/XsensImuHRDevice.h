#pragma once

#include "../../soem_master/SoemDevice.h"
#include "XsensImuHR.h"

class XsensImuHRDevice : public SoemDevice, protected XsensImuHR {
public:
    XsensImuHRDevice(ros::NodeHandle &nh, int slave, int passage1, uint16_t id1, int passage2, uint16_t id2,
        std::function<void(const ubt_hw::ImuData*)> funcOnMessage);

private:
    int slave;
    int passage1;
    uint16_t id1;
    int passage2; 
    uint16_t id2;
    bool readyMsg[2];
    static const int INIT_TIMES = 100;
    std::function<void(const ubt_hw::ImuData*)> funcOnMessage;
    int count = 0;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;
    
    void SendActiveRequest(int passage, DeviceMessage& msg);
    inline int16_t GetInt16(uint8_t* src) {
        int16_t dest;
        memcpy(&dest, src + 1, 1);
        memcpy((uint8_t*)&dest + 1, src, 1);
        return dest;
    }
};