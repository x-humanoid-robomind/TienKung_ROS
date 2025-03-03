/**
 * @file RobotFSM.h
 * @brief
 * @version 1.0
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024 x-humanoid_
 *
 */

#ifndef ROBOT_FSM_H_
#define ROBOT_FSM_H_

#include "RobotInterface.h"
#include "FSMState.h"

class RobotFSM {
 public:
  explicit RobotFSM(RobotData &robot_data){}
  virtual ~RobotFSM() = default;
  // run the fsm
  virtual void RunFSM(xbox_flag& flag) = 0;
  // get current fsm state 0: stop 1: zero 2: mlp
  virtual FSMStateName getCurrentState() = 0;
  // trigger by joystick
  bool disable_joints_ = false;
};

RobotFSM *get_robot_FSM(RobotData &robot_data);

#endif //ROBOT_FSM_H_
