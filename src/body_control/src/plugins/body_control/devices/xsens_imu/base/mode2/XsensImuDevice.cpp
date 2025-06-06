#include "XsensImuDevice.h"
#include <cstring>
#include <cmath>
#include <glog/logging.h>

namespace xsens_mode2 {

XsensImuDevice::XsensImuDevice(int slave, int passages[4], uint16_t ids[4], std::function<void(ImuData&)> funcOnMessage)
        : funcOnMessage(funcOnMessage)
{
    for (int i = 0; i < 4; ++i) {
        this->passages[i] = passages[i];
        this->ids[i] = ids[i];
        readyMsg[i] = false;
    }
}

void XsensImuDevice::SendActiveRequest(int passage, DeviceMessage& msg) {
    if (passage == passages[0]) {
        msg.rtr = 0;
        msg.id = ids[0]; 
        msg.dlc = 0;
    }
    else if (passage == passages[1]) {
        msg.rtr = 0;
        msg.id = ids[1]; 
        msg.dlc = 0;
    }
    else if (passage == passages[2]) {
        msg.rtr = 0;
        msg.id = ids[2]; 
        msg.dlc = 0;
    }
    else if (passage == passages[3]) {
        msg.rtr = 0;
        msg.id = ids[3]; 
        msg.dlc = 0;
    }
}

void XsensImuDevice::OnRequest(int passage, DeviceMessage& msg) {
    // make sure that etherCAT & CAN card can receive id command
    if (count < INIT_TIMES) {
        SendActiveRequest(passage, msg);
        ++count;
    }
}

void XsensImuDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id == 0)  {
        // if the first CAN package of the imu data is missing, abandon this frame
        if (passage != passages[0]) {
            if (readyMsg[0] || readyMsg[1] || readyMsg[2]) {
                readyMsg[0] = readyMsg[1] = readyMsg[2] = false;
                LOG(WARNING) << "abandon 1 frame of Imu";
            }
        }
    }
    else if (count >= INIT_TIMES) {
        if (passage == passages[0]) { 
            auto w = GetInt16(msg.data);
            auto x = GetInt16(msg.data + 2);
            auto y = GetInt16(msg.data + 4);
            auto z = GetInt16(msg.data + 6);
            data.orientation[0] = x * 3.019e-05;
            data.orientation[1] = y * 3.019e-05;
            data.orientation[2] = z * 3.019e-05;
            data.orientation[3] = w * 3.019e-05;
            readyMsg[0] = true;
        }
        else if (passage == passages[1]) {
            auto x = GetInt16(msg.data);
            auto y = GetInt16(msg.data + 2);
            auto z = GetInt16(msg.data + 4);
            data.angular_vel[0] = x * 0.0020;
            data.angular_vel[1] = y * 0.0020;
            data.angular_vel[2] = z * 0.0020;
            readyMsg[1] = true;
        }
        else if (passage == passages[2]) {
            auto x = GetInt16(msg.data);
            auto y = GetInt16(msg.data + 2);
            auto z = GetInt16(msg.data + 4);

            data.linear_accel[0] = x * 0.0039;
            data.linear_accel[1] = y * 0.0039;
            data.linear_accel[2] = z * 0.0039;
            readyMsg[2] = true;
        }
        else if (passage == passages[3]) {
            auto roll = GetInt16(msg.data);
            auto pitch = GetInt16(msg.data + 2);
            auto yaw = GetInt16(msg.data + 4);

            data.rpy[0] = roll * 0.0078 * M_PI / 180;
            data.rpy[1] = pitch * 0.0078 * M_PI / 180;
            data.rpy[2] = yaw * 0.0078 * M_PI / 180;
            readyMsg[3] = true;
        }

        if (readyMsg[0] && readyMsg[1] && readyMsg[2] && readyMsg[3]) {
            SetReady();
            funcOnMessage(data);
            readyMsg[0] = readyMsg[1] = readyMsg[2] = readyMsg[3] = false;
        }
    }
}

int16_t XsensImuDevice::GetInt16(uint8_t* src) {
    int16_t dest;
    memcpy(&dest, src + 1, 1);
    memcpy((uint8_t*)&dest + 1, src, 1);
    return dest;
}

}