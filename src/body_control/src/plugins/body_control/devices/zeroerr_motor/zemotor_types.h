#pragma once

#include <cstdint>

using WaistFrameId = uint16_t;

struct WaistCommand {};

// -----------------------------------------

struct WaistMotionCtrlCmd : public WaistCommand  {

};


// -----------------------------------------


struct WaistSetPositionCmd : public WaistCommand {
    double pos;
    double spd;
    double accel;
    double decel;
};

// -----------------------------------------



struct WaistStopCmd : public WaistCommand {

};

// -----------------------------------------


struct MotorStatusMessage {
    double pos = 0;    //各电机位置信息转换为deg
    double vel = 0;    //各电机位置信息转换为deg/s
    double cur = 0;  //电流 mA
};