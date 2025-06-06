#pragma once

#include "../../soem_master/SoemDevice.h"
#include "ubt_hw/drive/imu/rm_imu.h"

class RmImuDevice : public SoemDevice, protected rm_imu::RmImu {
public:
    RmImuDevice(ros::NodeHandle &nh, int slave, int passage1, uint16_t id1, int passage2, uint16_t id2,
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
};