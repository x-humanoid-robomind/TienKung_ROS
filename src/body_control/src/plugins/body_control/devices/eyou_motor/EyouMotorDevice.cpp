#include "EyouMotorDevice.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <thread>
#include <glog/logging.h>
#include <algorithm>

namespace eyou {

using namespace canopen;

EyouMotorDevice::EyouMotorDevice(int slave, int passageReq, int passageResp, uint16_t id, Mode mode, Type type, float kt, 
        std::function<void(MotorStatusMessage&)> funcOnStatus) : 
            slave(slave), id(id), funcOnStatus(funcOnStatus), mode(mode), kt(kt), type(type), 
            passageReq(passageReq), passageResp(passageResp) {}

// void EyouMotorDevice::SetFphc(float kp, float kd, float position, float speed, float torque) {
//     mtxSet.lock();
//     this->kp = kp;
//     this->kd = kd;
//     this->position = position;
//     this->speed = speed;
//     this->torque = torque;
//     mtxSet.unlock();
// }

void EyouMotorDevice::SetPos(float position, float maxSpeed) {
    if (maxSpeed <= 0) {
        return;
    }
    mtxSet.lock();
    switch (mode) {
        case Mode::POS: {
            this->targetPos = position;
            this->maxSpeed = maxSpeed;
            break;
        }
        case Mode::POS_2: {
            motorController.setTargetPositionAndMaxVelocity(position, maxSpeed);
            break;
        }
    }

    mtxSet.unlock();
}

void EyouMotorDevice::DisableMotor()
{
    sdo::Request reqClearErrors(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x80);
    sdo::Request reqDisableMotor(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x06);
    sdo::Request reqGetMotorEnableState(id, sdo::Ins::REQ_READ_STATUS_WORD);

    while(DriverEnable != 0)
    {
        PendRequest(reqClearErrors);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqDisableMotor);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqGetMotorEnableState);
        std::this_thread::sleep_for(std::chrono::milliseconds(350));

        LOG(INFO) << "DisableMotor Driver Disable = " << DriverEnable << ", id: " << id;    
    }
}

void EyouMotorDevice::SetMotorMode(MotionIns modeCode)  // 电机模式
{
    auto mode = (uint32_t)modeCode;
    sdo::Request reqSetMotorMode(id, sdo::Ins::REQ_WRITE_MODES_OF_OPERATION, mode);
    sdo::Request reqGetMotorMode(id, sdo::Ins::REQ_READ_MODES_OF_OPERATION_DISPLAY);

    switch (modeCode) {
        case MotionIns::CS_TORQUE: {
            sdo::Request reqResetTargetTorgue(id, sdo::Ins::REQ_WRITE_TARGET_TORQUE, 0);

            while(DriverMode != mode)
            {
                PendRequest(reqSetMotorMode, 100);
                PendRequest(reqResetTargetTorgue, 100);
                PendRequest(reqGetMotorMode, 100);
                LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode << ", id: " << id;      
            }
            break;
        }
        case MotionIns::CS_POSITION: {
            sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
            sdo::Request reqGetMaxSpeed(id, sdo::Ins::REQ_READ_MAX_PROFILE_VELOCITY, 0x01);

            while(DriverMode != mode)
            {
                PendRequest(reqSetMotorMode, 100);
                PendRequest(reqGetPosition, 100);
                targetPos = ReceivePosDeg / 180 * M_PI;
                LOG(INFO) << "init pos: " << ReceivePosDeg / 180 * M_PI << ", data: " << ReceivePos;
                sdo::Request reqInitPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, ReceivePos);
                PendRequest(reqInitPosition, 100);
                PendRequest(reqGetMaxSpeed, 100);
                PendRequest(reqGetMotorMode, 100);
                LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode << ", id: " << id;     
            }
            // LOG(INFO) << "Max Speed: " << maxSpeedSetting / Trans2Deg / 180 * M_PI;
            LOG(INFO) << "Max Speed: " << maxSpeedSetting;
            break;
        }
        case MotionIns::CS_VELOCITY: {
            while(DriverMode != mode)
            {
                PendRequest(reqSetMotorMode, 100);
                PendRequest(reqGetMotorMode, 100);
                LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode << ", id: " << id;     
            }
            break;
        }
    }

}

void EyouMotorDevice::EnableMotor()  // 电机使能
{
    sdo::Request reqShutdown(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x06);
    sdo::Request reqSetReady(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x07);
    sdo::Request reqSetEnable(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x0F);
    sdo::Request reqGetMotorEnableState(id, sdo::Ins::REQ_READ_STATUS_WORD);

    while(DriverEnable != 1)
    {
        PendRequest(reqShutdown, 100);
        PendRequest(reqSetReady, 100);
        PendRequest(reqSetEnable, 100);
        PendRequest(reqGetMotorEnableState, 100);
        LOG(INFO) << "EnableMotor Driver Enable = " << DriverEnable << ", id: " << id;  
    }
}

void EyouMotorDevice::GetMotorStatus() {

    sdo::Request reqGetSpeed(id, sdo::Ins::REQ_READ_ACTUAL_VELOCITY);
    sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
    sdo::Request reqGetActualCurrent(id, sdo::Ins::REQ_READ_MOTOR_ACTUAL_CURRENT);

    PendRequest(reqGetSpeed);
    PendRequest(reqGetPosition);
    PendRequest(reqGetActualCurrent);
}

void EyouMotorDevice::Run() {
    if (mode == Mode::FPHC) {
        DisableMotor();
        SetMotorMode(MotionIns::CS_TORQUE); // 周期同步电流扭矩模式
        EnableMotor();

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        sdo::Request reqGetRatedCurrent(id, sdo::Ins::REQ_READ_MOTOR_RATED_CURRENT); // 额定电流
        PendRequest(reqGetRatedCurrent);

        while (running) {
            mtxSet.lock();
            // mA
            auto current = (kp * (position - ReceivePosDeg / 180 * M_PI) + kd * (speed - ReceiveVelDeg / 180 * M_PI) + torque) * kt * 10000;
            // LOG(INFO) << "current:" << current;
            auto data = current * 10000 / RatedCurrent;
            auto d2 = ((uint32_t)(int16_t)data) & 0x00FFFF;
            // LOG(INFO)  << std::hex << "RatedCurrent:" << RatedCurrent << ",data2:" << d2;
            sdo::Request reqResetTargetTorgue(id, sdo::Ins::REQ_WRITE_TARGET_TORQUE, (uint32_t)d2);
            mtxSet.unlock();
            PendRequest(reqResetTargetTorgue);
            GetMotorStatus();
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
    else if (mode == Mode::POS) {
        DisableMotor();
        SetMotorMode(MotionIns::CS_POSITION);     // 周期同步位置模式
        EnableMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        SetReady();

        while (running) {
            static int k = 36;      // 速度放大倍数，用于处理意优电机的位置模式速度过慢问题
            static float mv = 0.2;   // 最大速度基础值常量

            mtxSet.lock();
            int data = targetPos / M_PI * 180 / Trans2Deg;
            mtxSet.unlock();

            auto dis = data - ReceivePos;
            auto m = dis / abs(dis);
            auto adis = abs(dis);  
            auto baselimit = maxSpeed / M_PI * 180 / Trans2Deg;
            
            if (adis > baseTolerance) {
                targetTolerance = baseTolerance * (k / 4) * std::max(1.0, (maxSpeed / mv));
                if (adis < targetTolerance) {
                    auto limit = baselimit * 0.002 * std::max(1.0, (double)k * (adis / targetTolerance)); // 每2ms的速度
                    dis = (adis < limit ? adis : limit);
                    auto d = ReceivePos + m * dis;
                    sdo::Request reqSetPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, (uint32_t)d);
                    // LOG(INFO) << "current: " << ReceivePos << ", target: " << d << ", dis:" << dis << ", adis:" << adis;
                    PendRequest(reqSetPosition);
                }
                else {
                    auto limit = baselimit * 0.002; // 每2ms的速度
                    dis = (adis < limit ? adis : limit);
                    auto d = ReceivePos + m * dis * k;
                    sdo::Request reqSetPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, (uint32_t)d);
                    // LOG(INFO) << "current: " << ReceivePos << ", target: " << d << ", dis:" << dis;
                    PendRequest(reqSetPosition);
                }
            }
            else {
                sdo::Request reqSetPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, (uint32_t)ReceivePos);
                // LOG(INFO) << "done, current: " << ReceivePos << ", dis:" << dis;
            }
            sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
            PendRequest(reqGetPosition, 2);
        }
    }
    else if (mode == Mode::POS_2) {
        DisableMotor();
        SetMotorMode(MotionIns::CS_VELOCITY);     // 周期同步速度模式
        EnableMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        GetMotorStatus();  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto initPosition = ReceivePosDeg / 180 * M_PI;

        // 设置目标位置和最大速度  
        motorController.setTargetPositionAndMaxVelocity(initPosition, 0); // 目标位置100，最大速度50  

        sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
        PendRequest(reqGetPosition, 100);

        SetReady();

        // 假设从某处获取电机的当前状态  
        double currentPosition = 0; 
        double currentVelocity = 0; 
        double current = 0;  
    
        // 模拟控制循环  
        while (running) {
            currentPosition = ReceivePosDeg / 180 * M_PI;
            currentVelocity = ReceiveVel / 180 * M_PI;
            mtxSet.lock();
            double plannedVelocity = 0;
            // 添加逻辑以处理达到目标或超调等情况  
            if (std::abs(currentPosition - motorController.targetPosition) > 0.01) {  
                plannedVelocity = motorController.update(currentPosition, currentVelocity, current);
                auto vel = plannedVelocity / M_PI * 180 / Trans2Deg;
                sdo::Request reqSetTargetVelocity(id, sdo::Ins::REQ_WRITE_TARGET_VELOCITY, (uint32_t)(int32_t)vel); 
                PendRequest(reqSetTargetVelocity); 
            }
            
            mtxSet.unlock();  

            LOG(INFO) << "Current position: " << currentPosition << ", Planned velocity: " << plannedVelocity  << ", id: " << id;  

            sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
            sdo::Request reqGetSpeed(id, sdo::Ins::REQ_READ_ACTUAL_VELOCITY);            
            PendRequest(reqGetSpeed);
            PendRequest(reqGetPosition, 3);
        }  
    }

}

void EyouMotorDevice::PendRequest(sdo::Request& req, int sleepMs) {
    auto msg = new DeviceMessage(req.idCan, req.ToBytes());
    {
        std::lock_guard<std::mutex> guard(mtxOpt);
        reqQueue.push(msg);
    } 
    if (sleepMs > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
}

void EyouMotorDevice::OnRequest(int passage, DeviceMessage& msg) {
    if (!started) {
        started = true;
        std::thread([this](){
            Run();
        }).detach();
    }

    if (passage == passageResp) {
        if (passageRespNotActive) {
            msg.rtr = 0;
            msg.id = sdo::Message::ToResponseCanId(id);
            msg.dlc = 0;
            passageRespNotActive = false;
        }
    }
    else if (passage == passageReq) {
        if (!reqQueue.empty()) {
            auto buff = reqQueue.pop(); 
            msg = *buff;
            delete buff;
        }
    }

}

void EyouMotorDevice::GetInfo(DeviceMessage& msg)
{
    if (msg.dlc < 8) {
        LOG(WARNING) << "?";
        return;
    }
    sdo::Response resp(msg.id, msg.data);

    switch (resp.GetIns()) {
        // 速度实际值
        case sdo::Ins::RESP_READ_ACTUAL_VELOCITY: {
            ReceiveVel = resp.data;
            ReceiveVelDeg = ReceiveVel * Trans2Deg;
            statusReady[0] = true;
            break;
        }
        // 位置实际值
        case sdo::Ins::RESP_READ_ACTUAL_POSITION_VALUE: {
            ReceivePos = resp.data;
            ReceivePosDeg = ReceivePos * Trans2Deg;
            statusReady[1] = true;
            break;
        }
        // 额定电流
        case sdo::Ins::RESP_READ_MOTOR_RATED_CURRENT: {
            RatedCurrent = resp.data;
            break;
        }  
        // 电流实际值
        case sdo::Ins::RESP_READ_MOTOR_ACTUAL_CURRENT: {
            ReceiveCurrent = resp.data;
            SignedCurrent = ReceiveCurrent * RatedCurrent  / 10000.0;
            statusReady[2] = true;
            break;
        }  
        // 使能状态
        case sdo::Ins::RESP_READ_STATUS_WORD: {
            auto key = resp.data & 0x00FF;
            if (key == 0x37) {
                DriverEnable = 1;
            }
            else if (key == 0x31) {
                DriverEnable = 0;
            }
            // LOG(INFO) << "RESP_READ_STATUS_WORD: " << std::hex 
            //     << "ins-0x" << resp.index << ", "
            //     << "subins-0x" << resp.subIndex << ", "
            //     << "type-0x" << resp.type << ", "
            //     << "data-0x" << resp.data << ", "
            //     << "key-0x" << key;
            break;
        }
        // 监测电机控制模式
        case sdo::Ins::RESP_READ_MODES_OF_OPERATION_DISPLAY: {
            DriverMode = resp.data;
            break;
        }
        // 最大速度显示
        case sdo::Ins::RESP_READ_MAX_PROFILE_VELOCITY: {
            maxSpeedSetting = resp.data;
            break;
        }
        case sdo::Ins::RESP_READ_CONROL_WORD: {
            // LOG(INFO) << "RESP_READ_CONROL_WORD: " << std::hex 
            //     << "ins-0x" << resp.index << ", "
            //     << "subins-0x" << resp.subIndex << ", "
            //     << "type-0x" << resp.type << ", "
            //     << "data-0x" << resp.data;
            // break;
        }
        default: {
            // LOG(INFO) << "RESP_UNKNOWN: " << std::hex 
            //     << "ins-0x" << resp.index << ", "
            //     << "subins-0x" << resp.subIndex << ", "
            //     << "type-0x" << resp.type << ", "
            //     << "data-0x" << resp.data;
        }
    }

    // publish data
    if (mode == Mode::FPHC) {
        if (statusReady[0] && statusReady[1] && statusReady[2]) {
            MotorStatusMessage status;
            status.vel = ReceiveVelDeg / 180 * M_PI;
            status.pos = ReceivePosDeg / 180 * M_PI;
            status.cur = SignedCurrent / 1000.0;

            statusReady[0] = statusReady[1] = statusReady[2] = false;
            SetReady();
            funcOnStatus(status);
        }
    }
    else if (mode == Mode::POS) {
        if (statusReady[1]) {
            MotorStatusMessage status;
            status.vel = 0;
            status.pos = ReceivePosDeg / 180 * M_PI;
            status.cur = 0;

            statusReady[1] = false;
            funcOnStatus(status);
        }
    }
    else if (mode == Mode::POS_2) {
        if (statusReady[0] && statusReady[1]) {
            MotorStatusMessage status;
            status.vel = ReceiveVelDeg / 180 * M_PI;
            status.pos = ReceivePosDeg / 180 * M_PI;
            status.cur = 0;

            statusReady[0] = false;
            statusReady[1] = false;
            funcOnStatus(status);
        }
    }

}

void EyouMotorDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id != 0) {
        if (passage == passageResp) {
            GetInfo(msg);
        }   
    }
}

}