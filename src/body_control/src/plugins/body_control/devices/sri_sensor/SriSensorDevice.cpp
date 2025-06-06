#include "SriSensorDevice.h"
#include <cstring>
#include <glog/logging.h>

SriSensorDevice::SriSensorDevice(int slave, int idCmd, int passageCmd, int idData[3], int passageData[3],
        std::function<void(float fx, float fy, float fz, float mx, float my, float mz)> funcOnStatus)
        : slave(slave), idCmd(idCmd), passageCmd(passageCmd), funcOnStatus(funcOnStatus)
{
    for (auto i = 0; i < 3; ++i) {
        this->idData[i] = idData[i];
        this->passageData[i] = passageData[i];
    }
    readyMsg[0] = false;
    readyMsg[1] = false;
    readyMsg[2] = false;
}

void SriSensorDevice::StartRealTime(int passage, DeviceMessage& msg) {
    if (passage == passageCmd) {
        msg.rtr = 0;
        msg.id = idCmd; 
        msg.dlc = 3;

        msg.data[0] = 0x8A;
        msg.data[1] = 0x00;
        msg.data[2] = 0x05;
    }
    else if (passage == passageData[0]) {
        msg.rtr = 0;
        msg.id = idData[0]; 
        msg.dlc = 0;
    }
    else if (passage == passageData[1]) {
        msg.rtr = 0;
        msg.id = idData[1]; 
        msg.dlc = 0;
    }
    else if (passage == passageData[2]) {
        msg.rtr = 0;
        msg.id = idData[2]; 
        msg.dlc = 0;
    }
}

void SriSensorDevice::OnRequest(int passage, DeviceMessage& msg) {
    if (count < INIT_TIMES) {
        StartRealTime(passage, msg);
        ++count;
    }
}

void SriSensorDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id != 0) {
        if (msg.id == idCmd) {
            // LOG(INFO) << passage << " " << std::hex << msg.id << " " << passageCmd;
        }
        if (count >= INIT_TIMES) {

            if (passage == passageData[0]) {
                memcpy(&fx, msg.data, 4);
                memcpy(&fy, msg.data + 4, 4);
                readyMsg[0] = true;
            }
            else if (passage == passageData[1]) {
                memcpy(&fz, msg.data, 4);
                memcpy(&mx, msg.data + 4, 4);
                readyMsg[1] = true;
            }
            else if (passage == passageData[2]) {
                memcpy(&my, msg.data, 4);
                memcpy(&mz, msg.data + 4, 4);
                readyMsg[2] = true;
            }

            if (readyMsg[0] && readyMsg[1] && readyMsg[2]) {
                SetReady();
                funcOnStatus(fx, fy, fz, mx, my, mz);
                readyMsg[0] = readyMsg[1] = readyMsg[2] = false;
            }
        }
    }
}