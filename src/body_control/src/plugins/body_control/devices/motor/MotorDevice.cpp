#include "MotorDevice.h"
#include "math_ops.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <thread>

#define KP_MIN 0.0f
#define KP_MAX 1500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f

#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

//  - 4310
#define TOR_MIN_4310 -30.0f
#define TOR_MAX_4310 30.0f
#define CUR_MIN_4310 -30.0f
#define CUR_MAX_4310 30.0f
//  - 8112
#define TOR_MIN_8112 -90.0f
#define TOR_MAX_8112 90.0f
#define CUR_MIN_8112 -60.0f
#define CUR_MAX_8112 60.0f
//  - 10020
#define TOR_MIN_10020 -150.0f
#define TOR_MAX_10020 150.0f
#define CUR_MIN_10020 -70.0f
#define CUR_MAX_10020 70.0f
//  - 13715
#define TOR_MIN_13715 -320.0f
#define TOR_MAX_13715 320.0f
#define CUR_MIN_13715 -220.0f
#define CUR_MAX_13715 220.0f

union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

MotorDevice::MotorDevice(int slave, int passage, uint16_t motorId, Type type, 
        std::function<void(float position, float speed, float current, float temperature)> funcOnStatus,
        std::function<void()> funcOnMiss) {
    this->slave = slave;
    this->motorId = motorId;
    this->type = type;
    this->funcOnStatus = funcOnStatus;
    this->funcOnMiss = funcOnMiss;
    this->passage = passage;
    
    ready = false;

    memset(&reqBuff, 0x00, sizeof(reqBuff));
    SetMotorSpeed(0, 1);
}

void MotorDevice::SetMotorSpeed(float speed, float current) {
    auto cur = uint16_t(current * 10);
    
    auto buff = new DeviceMessage();
    buff->rtr = 0;
    buff->id = motorId;
    buff->dlc = 7;

    rv_type_convert.to_float = speed;
    buff->data[0] = 0x41;
    buff->data[1] = rv_type_convert.buf[3];
    buff->data[2] = rv_type_convert.buf[2];
    buff->data[3] = rv_type_convert.buf[1];
    buff->data[4] = rv_type_convert.buf[0];
    buff->data[5] = cur >> 8;
    buff->data[6] = cur & 0xff;

    reqQueue.push(buff);
}

void MotorDevice::SetMotorPosition(float position, float speed, float current) {
    auto oPos = position + zeroOffset;
    if (oPos >= POS_MAX) {
        oPos = POS_MAX;
    }
    else if (oPos <= POS_MIN) {
        oPos = POS_MIN;
    }

    auto cur = uint16_t(current * 10);
    auto spd = uint16_t(speed * 10);
    auto pos = oPos / M_PI * 180;

    auto buff = new DeviceMessage();
    buff->rtr = 0;
    buff->id = motorId;
    buff->dlc = 8;

    rv_type_convert.to_float = pos;
    buff->data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    buff->data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    buff->data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    buff->data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    buff->data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    buff->data[5] = (spd & 0x3FC) >> 2;
    buff->data[6] = (spd & 0x03) << 6 | (cur >> 6);
    buff->data[7] = (cur & 0x3F) << 2 | 0x01;

    reqQueue.push(buff);
}

void MotorDevice::SetMotorDistance(float distance, float speed, float current) {
    auto tarPos = position + distance;
    SetMotorPosition(tarPos, speed, current);
    // std::cout << "tarPos:" << tarPos << ",speed:" << speed << ",cur:" << current << std::endl;
}

void MotorDevice::SendMotorCtrlCmd(float kp, float kd, float position, float speed, float torque) {
    auto pos = position + zeroOffset;
    auto& spd = speed;
    auto& tor = torque;

    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    auto buff = new DeviceMessage();
    buff->rtr = 0;
    buff->id = motorId;
    buff->dlc = 8;

    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;

    float torMin = 0, torMax = 0;
    switch (type) {
        case Type::S_4310: {
            torMin = TOR_MIN_4310;
            torMax = TOR_MAX_4310;
            break;
        }
        case Type::M_8112: {
            torMin = TOR_MIN_8112;
            torMax = TOR_MAX_8112;
            break;
        }
        case Type::L_10020: {
            torMin = TOR_MIN_10020;
            torMax = TOR_MAX_10020;
            break;
        }
        case Type::H_13715: {
            torMin = TOR_MIN_13715;
            torMax = TOR_MAX_13715;
            break;
        }
        default: {
            /* do nothing */
        }
    }

    if (tor > torMax)
        tor = torMax;
    else if (tor < torMin)
        tor = torMin;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, torMin, torMax, 12);

    buff->data[0] = 0x00 | (kp_int >> 7);                             // kp5
    buff->data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    buff->data[2] = kd_int & 0xFF;
    buff->data[3] = pos_int >> 8;
    buff->data[4] = pos_int & 0xFF;
    buff->data[5] = spd_int >> 4;
    buff->data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    buff->data[7] = tor_int & 0xff;

    reqQueue.push(buff);
}

void MotorDevice::OnRequest(int passage, DeviceMessage& msg) {
    if (!reqQueue.empty()) {
        auto buff = reqQueue.pop(); 
        reqBuff = *buff;
        delete buff;
    }

    msg = reqBuff;
}

void MotorDevice::OnStatus(DeviceMessage& msg) {
    int pos_int = msg.data[1] << 8 | msg.data[2];
    int spd_int = msg.data[3] << 4 | (msg.data[4] & 0xF0) >> 4;
    int cur_int = (msg.data[4] & 0x0F) << 8 | msg.data[5];

    position = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
    speed = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
    current = 0;
    switch (type) {
        case Type::S_4310: {
            current = uint_to_float(cur_int, CUR_MIN_4310, CUR_MAX_4310, 12);
            break;
        }
        case Type::M_8112: {
            current = uint_to_float(cur_int, CUR_MIN_8112, CUR_MAX_8112, 12);
            break;
        }
        case Type::L_10020: {
            current = uint_to_float(cur_int, CUR_MIN_10020, CUR_MAX_10020, 12);
            break;
        }
        case Type::H_13715: {
            current = uint_to_float(cur_int, CUR_MIN_13715, CUR_MAX_13715, 12);
            break;
        }
        default: {
            /* do nothing */
        }
    }
    temperature = (msg.data[6] - 50) / 2;
    SetReady();
    funcOnStatus(position - zeroOffset, speed, current, temperature);
}

void MotorDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id != 0) {
        if (msg.id == motorId) {
            OnStatus(msg);
        }
    }
    else {
        funcOnMiss();
    }
}