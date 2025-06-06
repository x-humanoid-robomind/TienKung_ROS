#include "XsensImuDevice.h"
#include <cstring>
#include <cmath>
#include <glog/logging.h>

namespace xsens_mode1 {

XsensImuDevice::XsensImuDevice(int slave, int passage1, uint16_t id1, int passage2, uint16_t id2, int passage3, uint16_t id3,  
    std::function<void(ImuData&)> funcOnMessage)
        : id1(id1), id2(id2), id3(id3), passage1(passage1), passage2(passage2), passage3(passage3), funcOnMessage(funcOnMessage)
{
    readyMsg[0] = false;
    readyMsg[1] = false;
    readyMsg[2] = false;
}

void XsensImuDevice::SendActiveRequest(int passage, DeviceMessage& msg) {
    if (passage == passage1) {
        msg.rtr = 0;
        msg.id = id1; 
        msg.dlc = 0;
    }
    else if (passage == passage2) {
        msg.rtr = 0;
        msg.id = id2; 
        msg.dlc = 0;
    }
    else if (passage == passage3) {
        msg.rtr = 0;
        msg.id = id3; 
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
        if (passage != passage1) {
            if (readyMsg[0] || readyMsg[1]) {
                readyMsg[0] = readyMsg[1] = false;
                LOG(WARNING) << "abandon 1 frame of Imu";
            }
        }
    }
    else if (count >= INIT_TIMES) {
        if (passage == passage1) { 
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
        else if (passage == passage2) {
            auto x = GetInt16(msg.data);
            auto y = GetInt16(msg.data + 2);
            auto z = GetInt16(msg.data + 4);
            data.angular_vel[0] = x * 0.0020;
            data.angular_vel[1] = y * 0.0020;
            data.angular_vel[2] = z * 0.0020;
            readyMsg[1] = true;
        }
        else if (passage == passage3) {
            auto x = GetInt16(msg.data);
            auto y = GetInt16(msg.data + 2);
            auto z = GetInt16(msg.data + 4);

            data.linear_accel[0] = x * 0.0039;
            data.linear_accel[1] = y * 0.0039;
            data.linear_accel[2] = z * 0.0039;
            readyMsg[2] = true;
        }

        if (readyMsg[0] && readyMsg[1] && readyMsg[2]) {
            quatToRPY2(data.orientation, data.rpy[0], data.rpy[1], data.rpy[2]);
            // toEulerAngles(data.rpy[0], data.rpy[1], data.rpy[2]);
            SetReady();
            funcOnMessage(data);
            readyMsg[0] = readyMsg[1] = readyMsg[2] = false;
        }
    }
}

int16_t XsensImuDevice::GetInt16(uint8_t* src) {
    int16_t dest;
    memcpy(&dest, src + 1, 1);
    memcpy((uint8_t*)&dest + 1, src, 1);
    return dest;
}

void XsensImuDevice::quatToRPY(const double* q, double& roll, double& pitch, double& yaw) {
    // x: q[0] y: q[1] z: q[2] w: q[3]
    double as = fmin(-2. * (q[0] * q[2] - q[3] * q[1]), .99999);
    yaw = std::atan2(2 * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);
    pitch = std::asin(as);
    roll = std::atan2(2 * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
}

void XsensImuDevice::quatToRPY2(const double* q, double& roll, double& pitch, double& yaw) {  
    // 假设q遵循(x, y, z, w)的顺序  
    double qx = q[0];  
    double qy = q[1];  
    double qz = q[2];  
    double qw = q[3];  
  
    // 四元数规范化（如果需要的话）  
    double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);  
    if (norm > 0.0) {  
        qx /= norm;  
        qy /= norm;  
        qz /= norm;  
        qw /= norm;  
    }  
  
    // Roll (X-axis rotation)  
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);  
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);  
    roll = std::atan2(sinr_cosp, cosr_cosp);  
  
    // Pitch (Y-axis rotation)  
    double sinp = 2.0 * (qw * qy - qx * qz);  
    if (std::abs(sinp) >= 1) {  
        // 使用90度来避免除以零  
        pitch = copysign(M_PI / 2, sinp); // PI 是圆周率，需要包含 <cmath>  
    } else {  
        pitch = std::asin(sinp);  
    }  
  
    // Yaw (Z-axis rotation)  
    double siny_cosp = 2.0 * (qw * qz + qx * qy);  
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);  
    yaw = std::atan2(siny_cosp, cosy_cosp);  
} 

void XsensImuDevice::toEulerAngles(double& roll, double& pitch, double& yaw) {  

    auto& x = data.orientation[0];
    auto& y = data.orientation[1];
    auto& z = data.orientation[2];
    auto& w = data.orientation[3];

    // roll (x-axis rotation)  

    double sinr_cosp = 2.0 * (w * x + y * z);  

    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);  

    roll = std::atan2(sinr_cosp, cosr_cosp);  



    // pitch (y-axis rotation)  

    double sinp = 2.0 * (w * y - z * x);  

    if (std::abs(sinp) >= 1)  

        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range  

    else  

        pitch = std::asin(sinp);  



    // yaw (z-axis rotation)  

    double siny_cosp = 2.0 * (w * z + x * y);  

    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);  

    yaw = std::atan2(siny_cosp, cosy_cosp);  

}

}