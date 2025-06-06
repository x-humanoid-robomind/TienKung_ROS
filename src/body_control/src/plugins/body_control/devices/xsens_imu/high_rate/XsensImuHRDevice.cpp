#include "XsensImuHRDevice.h"
#include <glog/logging.h>

XsensImuHRDevice::XsensImuHRDevice(ros::NodeHandle &nh, int slave, int passage1, uint16_t id1, int passage2, uint16_t id2,
    std::function<void(const ubt_hw::ImuData*)> funcOnMessage)
    : slave(slave), passage1(passage1), id1(id1), passage2(passage2), id2(id2), funcOnMessage(funcOnMessage)
{
    data_ = new ubt_hw::ImuData();
    init(nh);
    readyMsg[0] = false;
    readyMsg[1] = false;
}

void XsensImuHRDevice::SendActiveRequest(int passage, DeviceMessage& msg) {
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

void XsensImuHRDevice::OnRequest(int passage, DeviceMessage& msg) {
    // make sure that etherCAT & CAN card can receive id command
    if (count < INIT_TIMES) {
        SendActiveRequest(passage, msg);
        ++count;
    }
}

void XsensImuHRDevice::OnResponse(int passage, DeviceMessage& msg) {
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

        if (passage == passage1) {
            double gyr[3] = {0};
            gyr[0] = GetInt16(msg.data) * 0.0020;
            gyr[1] = GetInt16(msg.data + 2) * 0.0020;
            gyr[2] = GetInt16(msg.data + 4) * 0.0020;
            UpdateGyr(gyr);
            readyMsg[1] = true;
        }
        else if (passage == passage2) {
            double acc[3] = {0};
            acc[0] = GetInt16(msg.data) * 0.0039;
            acc[1] = GetInt16(msg.data + 2) * 0.0039;
            acc[2] = GetInt16(msg.data + 4) * 0.0039;
            UpdateAcc(acc);
            readyMsg[0] = true;
        }

        if (readyMsg[0] && readyMsg[1]) {
            SetReady();
            funcOnMessage(data_);
            readyMsg[0] = readyMsg[1] = false;
        }
    } 
}