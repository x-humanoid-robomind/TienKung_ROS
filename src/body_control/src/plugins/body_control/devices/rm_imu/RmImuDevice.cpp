#include "RmImuDevice.h"
#include <glog/logging.h>

RmImuDevice::RmImuDevice(ros::NodeHandle &nh, int slave, int passage1, uint16_t id1, int passage2, uint16_t id2,
    std::function<void(const ubt_hw::ImuData*)> funcOnMessage)
    : slave(slave), passage1(passage1), id1(id1), passage2(passage2), id2(id2), funcOnMessage(funcOnMessage)
{
    data_ = new ubt_hw::ImuData();
    init(nh);
    readyMsg[0] = false;
    readyMsg[1] = false;
}

void RmImuDevice::SendActiveRequest(int passage, DeviceMessage& msg) {
    if (passage == passage1) {
        msg.rtr = 0;
        msg.id = id1; 
        msg.dlc = 0;
    }
    else {
        msg.rtr = 0;
        msg.id = id2; 
        msg.dlc = 0;  
    }
}

void RmImuDevice::OnRequest(int passage, DeviceMessage& msg) {
    // make sure that etherCAT & CAN card can receive id command
    if (count < INIT_TIMES) {
        SendActiveRequest(passage, msg);
        ++count;
    }
}

void RmImuDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id == 0)  {
        // if the first CAN package of the imu data is missing, abandon this frame
        if (passage != passage1) {
            if (readyMsg[0]) {
                readyMsg[0] = false;
                LOG(WARNING) << "abandon 1 frame of Imu";
            }
        }
    }
    else if (count >= INIT_TIMES) {
        ubt_hw::CAN_PACKAGE pkg = {0};
        pkg.head.h32 = msg.id;
        for (int i = 0; i < 8; ++i) {
            pkg.data[i] = msg.data[i];
        }
        readStatus(pkg);
        if (pkg.head.h32  == id2) {
            readyMsg[1] = true;
        } else if (pkg.head.h32  == id1) {
            readyMsg[0] = true;
        }
        if (readyMsg[0] && readyMsg[1]) {
            SetReady();
            funcOnMessage(data_);
            readyMsg[0] = readyMsg[1] = false;
        }
    } 
}