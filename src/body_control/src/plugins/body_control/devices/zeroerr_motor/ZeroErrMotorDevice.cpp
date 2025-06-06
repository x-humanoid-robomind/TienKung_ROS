#include "ZeroErrMotorDevice.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <thread>
#include <glog/logging.h>

using namespace canopen;

ZeroErrMotorDevice::ZeroErrMotorDevice(int slave, int passageReq, int passageResp, uint16_t id, Mode mode, Type type, float kt, 
        std::function<void(MotorStatusMessage&)> funcOnStatus) : 
            slave(slave), id(id), funcOnStatus(funcOnStatus), mode(mode), kt(kt), type(type), 
            passageReq(passageReq), passageResp(passageResp) {}

void ZeroErrMotorDevice::SetFphc(float kp, float kd, float position, float speed, float torque) {
    mtxSet.lock();
    this->kp = kp;
    this->kd = kd;
    this->position = position + zeroOffset;
    this->speed = speed;
    this->torque = torque;
    mtxSet.unlock();
}

void ZeroErrMotorDevice::SetPos(float position, float maxSpeed) {
    if (maxSpeed <= 0) {
        return;
    }
    mtxSet.lock();
    this->targetPos = position + zeroOffset;
    this->maxSpeed = maxSpeed;
    mtxSet.unlock();
}

void ZeroErrMotorDevice::DisableMotor()
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

        LOG(INFO) << "DisableMotor Driver Disable = " << DriverEnable;    
    }
}

void ZeroErrMotorDevice::SetMotorMode()  // 电机模式
{
    static uint32_t modeCode = 0x0A; // 周期同步电流扭矩模式
    sdo::Request reqSetMotorMode(id, sdo::Ins::REQ_WRITE_MODES_OF_OPERATION, modeCode);
    sdo::Request reqResetTargetTorgue(id, sdo::Ins::REQ_WRITE_TARGET_TORQUE, 0);
    sdo::Request reqGetMotorMode(id, sdo::Ins::REQ_READ_MODES_OF_OPERATION_DISPLAY);


    while(DriverMode != modeCode)
    {
        PendRequest(reqSetMotorMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqResetTargetTorgue);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqGetMotorMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode;    
    }
}

void ZeroErrMotorDevice::EnableMotor()  // 电机使能
{
    sdo::Request reqShutdown(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x06);
    sdo::Request reqSetReady(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x07);
    sdo::Request reqSetEnable(id, sdo::Ins::REQ_WRITE_CONROL_WORD, 0x0F);
    sdo::Request reqGetMotorEnableState(id, sdo::Ins::REQ_READ_STATUS_WORD);

    while(DriverEnable != 1)
    {
        PendRequest(reqShutdown);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqSetReady);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqSetEnable);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        PendRequest(reqGetMotorEnableState);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (DriverEnable == 4) {
            LOG(WARNING) << "EnableMotor Driver Enable = " << DriverEnable << ", maybe the battery is of low power.";
        }
        else {
            LOG(INFO) << "EnableMotor Driver Enable = " << DriverEnable;
        }
    }
}

void ZeroErrMotorDevice::GetMotorStatus() {

    sdo::Request reqGetSpeed(id, sdo::Ins::REQ_READ_ACTUAL_VELOCITY);
    sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
    sdo::Request reqGetActualCurrent(id, sdo::Ins::REQ_READ_MOTOR_ACTUAL_CURRENT);

    PendRequest(reqGetSpeed);
    PendRequest(reqGetPosition);
    PendRequest(reqGetActualCurrent);
}

void ZeroErrMotorDevice::Run() {

    if (mode == Mode::FPHC) {
        DisableMotor();
        SetMotorMode();
        EnableMotor();
        
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

        {
            // 位置模式
            static uint32_t modeCode = 0x08; // 周期同步电流扭矩模式
            sdo::Request reqSetMotorMode(id, sdo::Ins::REQ_WRITE_MODES_OF_OPERATION, modeCode);
            sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
            sdo::Request reqGetMotorMode(id, sdo::Ins::REQ_READ_MODES_OF_OPERATION_DISPLAY);


            while(DriverMode != modeCode)
            {
                PendRequest(reqSetMotorMode, 100);
                PendRequest(reqGetPosition, 100);
                targetPos = ReceivePosDeg / 180 * M_PI;
                LOG(INFO) << "init pos: " << ReceivePosDeg / 180 * M_PI << ", data: " << ReceivePos;
                sdo::Request reqInitPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, ReceivePos);
                PendRequest(reqInitPosition, 100);
                PendRequest(reqGetMotorMode, 100);
                LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode;    
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        EnableMotor();

        SetReady();

        while (running) {
            mtxSet.lock();
            int data = targetPos / M_PI * 180 / Trans2Deg;
            mtxSet.unlock();

            auto dis = data - ReceivePos;
            auto adis = abs(dis);  
            if (adis > 5 && maxSpeed > 0) {
                auto m = dis / abs(dis);
                auto limit = maxSpeed / M_PI * 180 / Trans2Deg * 0.002; // 每2ms的速度
                dis = (adis < limit ? adis : limit);
                auto d = ReceivePos + m * dis;
                // LOG(INFO) << "target:" << targetPos << ", data:" << data << ", ReceivePos:" << ReceivePos << ", d: " << d;
                sdo::Request reqSetPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, (uint32_t)d);
                PendRequest(reqSetPosition);
                sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
                PendRequest(reqGetPosition);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            else {
                sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
                PendRequest(reqGetPosition);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }
}

void ZeroErrMotorDevice::PendRequest(sdo::Request& req, int sleepMs) {
    auto msg = new DeviceMessage(req.idCan, req.ToBytes());
    {
        std::lock_guard<std::mutex> guard(mtxOpt);
        reqQueue.push(msg);
    } 
    if (sleepMs > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
}

void ZeroErrMotorDevice::OnRequest(int passage, DeviceMessage& msg) {
    if (!started) {
        started = true;
        std::thread([this](){
            Run();
        }).detach();
    }

    if (passage == passageResp) {
        static bool send = true;
        if (send) {
            msg.rtr = 0;
            msg.id = sdo::Message::ToResponseCanId(id);
            msg.dlc = 0;
            send = false;
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

void ZeroErrMotorDevice::GetInfo(DeviceMessage& msg)
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
            // LOG(INFO) << "key:" << key;
            if (key == 0x37) {
                DriverEnable = 1;
            }
            else if (key == 0x21) {
                DriverEnable = 0;
            }
            else if (key == 0x08) {
                DriverEnable = 4;
            }
            break;
        }
        // 监测电机控制模式
        case sdo::Ins::RESP_READ_MODES_OF_OPERATION_DISPLAY: {
            DriverMode = resp.data;
            break;
        }
    }

    // publish data
    if (mode == Mode::FPHC) {
        if (statusReady[0] && statusReady[1] && statusReady[2]) {
            MotorStatusMessage status;
            status.pos = ReceivePosDeg / 180 * M_PI - zeroOffset;
            status.vel = ReceiveVelDeg / 180 * M_PI;
            status.cur = SignedCurrent / 1000.0;

            statusReady[0] = statusReady[1] = statusReady[2] = false;
            SetReady();
            funcOnStatus(status);
        }
    }
    else if (mode == Mode::POS) {
        if (statusReady[1]) {
            MotorStatusMessage status;
            status.pos = ReceivePosDeg / 180 * M_PI  - zeroOffset;
            status.vel = 0;
            status.cur = 0;

            statusReady[1] = false;
            funcOnStatus(status);
        }
    }

}

void ZeroErrMotorDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id != 0) {
        if (passage == passageResp) {
            GetInfo(msg);
        }   
    }
}