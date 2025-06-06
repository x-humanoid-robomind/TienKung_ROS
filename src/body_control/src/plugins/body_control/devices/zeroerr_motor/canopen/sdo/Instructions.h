#pragma once

#include "Type.h"

#define DEF_RON(NAME, INDEX, SUBINDEX) \
    REQ_READ_##NAME = Type::REQ_READ << 24 | INDEX << 8 | SUBINDEX, \
    RESP_READ_##NAME = INDEX << 8 | SUBINDEX, 

#define DEF_RW1(NAME, INDEX, SUBINDEX) \
    REQ_READ_##NAME = Type::REQ_READ << 24 | INDEX << 8 | SUBINDEX, \
    RESP_READ_##NAME = INDEX << 8 | SUBINDEX, \
    REQ_WRITE_##NAME = Type::REQ_WRITE_1BYTE << 24 | INDEX << 8 | SUBINDEX, \
    RESP_WRITE_##NAME = INDEX << 8 | SUBINDEX, 

#define DEF_RW2(NAME, INDEX, SUBINDEX) \
    REQ_READ_##NAME = Type::REQ_READ << 24 | INDEX << 8 | SUBINDEX, \
    RESP_READ_##NAME = INDEX << 8 | SUBINDEX, \
    REQ_WRITE_##NAME = Type::REQ_WRITE_2BYTES << 24 | INDEX << 8 | SUBINDEX, \
    RESP_WRITE_##NAME = INDEX << 8 | SUBINDEX, 

#define DEF_RW3(NAME, INDEX, SUBINDEX) \
    REQ_READ_##NAME = Type::REQ_READ << 24 | INDEX << 8 | SUBINDEX, \
    RESP_READ_##NAME = INDEX << 8 | SUBINDEX, \
    REQ_WRITE_##NAME = Type::REQ_WRITE_3BYTES << 24 | INDEX << 8 | SUBINDEX, \
    RESP_WRITE_##NAME = INDEX << 8 | SUBINDEX, 

#define DEF_RW4(NAME, INDEX, SUBINDEX) \
    REQ_READ_##NAME = Type::REQ_READ << 24 | INDEX << 8 | SUBINDEX, \
    RESP_READ_##NAME = INDEX << 8 | SUBINDEX, \
    REQ_WRITE_##NAME = Type::REQ_WRITE_4BYTES << 24 | INDEX << 8 | SUBINDEX, \
    RESP_WRITE_##NAME = INDEX << 8 | SUBINDEX, 
    
namespace canopen {
namespace sdo {

enum Instruction {
    DEF_RON(ACTUAL_VELOCITY, 0x606C, 00)                 // 实际速度
    DEF_RON(ACTUAL_POSITION_VALUE, 0x6064, 00)           // 实际位置
    DEF_RW4(MOTOR_RATED_CURRENT, 0x6075, 00)            // 额定电流
    DEF_RON(MOTOR_ACTUAL_CURRENT, 0x6078, 00)            // 实际电流
    DEF_RW2(CONROL_WORD, 0x6040, 00)             // 控制字
    DEF_RW2(STATUS_WORD, 0x6041, 00)                     // 状态字字
    DEF_RW1(MODES_OF_OPERATION, 0x6060, 00)      // 运行模式
    DEF_RON(MODES_OF_OPERATION_DISPLAY, 0x6061, 00)      // 运行模式显示
    DEF_RW2(TARGET_TORQUE, 0x6071, 00)           // 目标扭矩
    DEF_RW4(TARGET_POSITION, 0x607A, 00)         // 目标位置
};

using Ins = Instruction;
using ReqIns = Instruction;
using RespIns = Instruction;

}
}


#undef DEF_RON
#undef DEF_RW1
#undef DEF_RW2
#undef DEF_RW3
#undef DEF_RW4

