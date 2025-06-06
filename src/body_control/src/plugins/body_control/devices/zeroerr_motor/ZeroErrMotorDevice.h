#pragma once

#include "../../soem_master/SoemDevice.h"
#include "util/LockFreeQueue.h"
#include <mutex>
#include <functional>
#include <list>
#include "zemotor_types.h"
#include "canopen/canopen.h"

class ZeroErrMotorDevice : public SoemDevice {
public:
    enum class Type : int {
        S = 1,
        M,
    }; 
    enum class Mode : int {
        POS  = 1,   // 纯位置模式，500Hz
        FPHC = 2    // 力位混合模式，250Hz
    }; 
    ZeroErrMotorDevice(int slave, int passageReq, int passageResp, uint16_t id, Mode mode, Type type, float kt,
        std::function<void(MotorStatusMessage&)> funcOnStatus);

    void SetFphc(float kp, float kd, float position, float speed, float torque);
    void SetPos(float position, float maxSpeed);

    inline void SetZeroOffset(float offset) {
        LOG(INFO) << "zero pos:" << offset;
        zeroOffset = offset;
    }

private:
    int slave;
    int passageReq;
    int passageResp;
    uint16_t id;
    uint16_t idReq200, idReq300, idReq600;
    uint16_t idResp180, idResp280, idResp580;
    LockFreeQueue<DeviceMessage*> reqQueue;
    std::mutex mtxOpt;
    std::function<void(MotorStatusMessage&)> funcOnStatus;
    bool started = false;
    Mode mode;
    Type type;
    float kt;
    float zeroOffset = 0;

    // motor data cache
    int DriverEnable = 3;// =1时表示电机使能，=0时表示电机失能,初始化一个非状态数, =4 maybe low power
    int DriverMode = 0; // 记录每个电机当前的运行模式
    int ReceivePos = 0;  //接收到的各电机位置信息，编码器脉冲数
    int ReceiveVel = 0;  //接收到的各电机速度信息，编码器脉冲数
    int RatedCurrent = 0;  // 额定电流
    int ReceiveCurrent = 0;  //接收到的各电机速度信息
    int EncoderRes = 524288 ; // 编码器分辨率
    double Trans2Deg = (double)360/ EncoderRes;
    double ReceivePosDeg = 0;    //各电机位置信息转换为deg
    double ReceiveVelDeg = 0;    //各电机位置信息转换为deg/s
    signed short int SignedCurrent = 0;  //电流由无符号数转为有符号数
    bool MotorCon = false;   // 用于记录通讯正常的电机数
    int ErrorCode;  //电机错误码603Fh
    bool statusReady[3] = {0};
    bool running = true;

    std::mutex mtxSet;
    float kp = 0;
    float kd = 0;
    float position = 0;
    float speed = 0;
    float torque = 0;
    float targetPos = 0;
    float maxSpeed = 0;
    int lastTargetPos = 0;

    ZeroErrMotorDevice() = default;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;

    void GetInfo(DeviceMessage& msg);

    void Run();
    void EnableMotor();
    void SetMotorMode();
    void DisableMotor();
    void GetMotorStatus();

    void PendRequest(canopen::sdo::Request& req, int sleepMs = 0);
};