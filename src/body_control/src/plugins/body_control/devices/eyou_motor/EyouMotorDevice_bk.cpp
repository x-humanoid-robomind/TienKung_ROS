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
    this->targetPos = position;
    this->maxSpeed = maxSpeed;
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

        LOG(INFO) << "DisableMotor Driver Disable = " << DriverEnable;    
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
                LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode;    
            }
            break;
        }
        case MotionIns::CS_POSITION: {
            sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);

            while(DriverMode != mode)
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
            break;
        }
        case MotionIns::CS_VELOCITY: {
            while(DriverMode != mode)
            {
                PendRequest(reqSetMotorMode, 100);
                PendRequest(reqGetMotorMode, 100);
                LOG(INFO) << "Set MotorMode Driver Mode = 0x" << std::hex << DriverMode;    
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
        LOG(INFO) << "EnableMotor Driver Enable = " << DriverEnable;
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

double TribleOrder(double x0 , double x1 , double v0 , double v1 , double T , double t);
double TribleOrder(double x0 , double x1 , double v0 , double v1 , double a0 , double a1 , double T , double t);

void EyouMotorDevice::TrajPlan(double time)
{
    auto i = 0;
    ////  位置规划
    if(time < 5)
    {
        PositionPlan = TribleOrder(PosDegInit - ZeroPosition, 0 , 0 , 0 , 0 , 0 , 5 , time);
    }
    else if(time < 8)
    {
        PositionPlan = 0;
    }
    else
    {
        PositionPlan = 1*(0.3*i+1)*0.5*(30*(1-cos(0.1*M_PI*(time-8))) + 30*sin(0.2*M_PI*(time-8))) + 0 * (time > 8);
    }

    // PositionPlan = PosDegInit + 1*(100*(1-cos(0.1*PI*time)) + 0*sin(0.1*PI*time)) + 0 * (time > 5);

    PositionPlan = PositionPlan + ZeroPosition;
    ////    速度规划
    VelocityPlan = (i+1)*10*sin(0.1*M_PI*time);
    
    ////    电流模式下电流规划
    CurrentPlan =  -20.0 * 0 + 30*sin(2*M_PI*time); 
}

double TribleOrder(double x0 , double x1 , double v0 , double v1 , double T , double t)   //  三次曲线规划
{
    double a , b , c , d , y;
    a=(2*x0-2*x1+v0*T+v1*T)/T/T/T;
    b=(-3*x0+3*x1-2*v0*T-v1*T)/T/T;
    c=v0;
    d=x0;
    y=a*t*t*t+b*t*t+c*t+d;

    return y;
}

double TribleOrder(double x0 , double x1 , double v0 , double v1 , double a0 , double a1 , double T , double t)   //  五次曲线规划
{
    double a , b , c , d , e , f , y;
    a= (a1 - a0)/(2*T*T*T) - 3*(v0+v1)/(T*T*T*T) + 6*(x1-x0)/(T*T*T*T*T);
    b= (3*a0-2*a1)/(2*T*T) + (8*v0+7*v1)/(T*T*T)  + 15*(x0-x1)/(T*T*T*T) ;
    c= (a1-3*a0)/(2*T) - (6*v0+4*v1)/(T*T)  + 10*(x1-x0)/(T*T*T);
    d=a0/2;
    e=v0;
    f=x0;

    y=(a*t*t*t*t*t+b*t*t*t*t+c*t*t*t+d*t*t+e*t+f);

    return y;
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
            mtxSet.lock();
            int data = targetPos / M_PI * 180 / Trans2Deg;
            mtxSet.unlock();

            auto dis = data - ReceivePos;
            auto m = dis / abs(dis);
            auto adis = abs(dis);  
            if (adis > targetTolerance) {
                auto limit = maxSpeed / M_PI * 180 / Trans2Deg * 0.002; // 每2ms的速度
                dis = (adis < limit ? adis : limit);
                auto d = ReceivePos + m * dis;
                sdo::Request reqSetPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, (uint32_t)d);
                PendRequest(reqSetPosition);
                LOG(INFO) << "current: " << ReceivePos << ", target: " << d << ", dis:" << dis;
            }
            else {
                sdo::Request reqSetPosition(id, sdo::Ins::REQ_WRITE_TARGET_POSITION, (uint32_t)ReceivePos);
                LOG(INFO) << "done, current: " << ReceivePos << ", dis:" << dis;
            }
            sdo::Request reqGetPosition(id, sdo::Ins::REQ_READ_ACTUAL_POSITION_VALUE);
            PendRequest(reqGetPosition);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    else if (mode == Mode::POS_2) {
        DisableMotor();
        SetMotorMode(MotionIns::CS_VELOCITY);     // 周期同步速度模式
        EnableMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        GetMotorStatus();
        PosDegInit = ReceivePosDeg;
        auto VelDegInit = ReceivePosDeg;
        auto PositionPlanMemory = PosDegInit; //  记录前一帧的规划位置，用于计算前馈速度
        auto PositionPlanCan = PosDegInit / Trans2Deg;
        double VelocityForword;    //各电机规划前馈速度信息
        double AdmittanceVelocity = 0;    //导纳积分速度

        double RunTime; //  程序运行时间
        double Error_sum = 0 , Duratime;
        auto start = std::chrono::system_clock::now();
        auto now = std::chrono::system_clock::now();

        while (running) {
            mtxSet.lock();

            now = std::chrono::system_clock::now();
            Duratime = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - start).count()/1000/1000 -RunTime;
            RunTime = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - start).count()/1000/1000;

            TrajPlan(RunTime);

            ////  位置规划
            VelocityForword = std::max(-180.0, std::min((PositionPlan - PositionPlanMemory) / Duratime,180.0));     //  位置控制下速度前馈量
            PositionPlanMemory = PositionPlan;    //  位置数据寄存

            ////    速度规划
            // AdmittanceVelocity = AdmittanceVelocity + 0*(- 100*SignedCurrent / 30000.0 + 0 * 0.01 * (0 -AdmittanceVelocity));     //  导纳控制速度量
            VelocityPlan =  0 * AdmittanceVelocity + VelocityForword *1 +  5*(PositionPlan - ReceivePosDeg);   //  前馈加反馈控制
            sdo::Request reqSetTargetTorgue(id, sdo::Ins::REQ_WRITE_TARGET_VELOCITY, (uint32_t)(int32_t)VelocityPlan);
            mtxSet.unlock();
            PendRequest(reqSetTargetTorgue);
            GetMotorStatus();
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
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
            LOG(INFO) << "RESP_READ_STATUS_WORD: " << std::hex 
                << "ins-0x" << resp.index << ", "
                << "subins-0x" << resp.subIndex << ", "
                << "type-0x" << resp.type << ", "
                << "data-0x" << resp.data << ", "
                << "key-0x" << key;
            break;
        }
        // 监测电机控制模式
        case sdo::Ins::RESP_READ_MODES_OF_OPERATION_DISPLAY: {
            DriverMode = resp.data;
            break;
        }
        case sdo::Ins::RESP_READ_CONROL_WORD: {
            LOG(INFO) << "RESP_READ_CONROL_WORD: " << std::hex 
                << "ins-0x" << resp.index << ", "
                << "subins-0x" << resp.subIndex << ", "
                << "type-0x" << resp.type << ", "
                << "data-0x" << resp.data;
        }
    }

    // publish data
    if (mode == Mode::FPHC) {
        if (statusReady[0] && statusReady[1] && statusReady[2]) {
            MotorStatusMessage status;
            status.pos = ReceivePosDeg / 180 * M_PI;
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
            status.pos = ReceivePosDeg / 180 * M_PI;
            status.vel = 0;
            status.cur = 0;

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