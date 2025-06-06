/*******************************************************************************
 * Copyright (c) 2023/10/19, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once
#include <ros/ros.h>

#include <chrono>
#include <string>
namespace ubt_hw {

enum ServoMode {
  SERVO_MODE_INIT = 0,  // 初始化，检测通信系统
  SERVO_MODE_HOME,      // 位置模式回零操作
  SERVO_MODE_STOP,      // 停机
  SERVO_MODE_CTRL,      // 控制模式
  SERVO_MODE_SETZERO,   // 设置零点
  SERVO_GET_STATUS      // 获得当前状态
};

enum ServoCtrlMode {
  SERVO_POS = 0,  // 位置模式
  SERVO_VEL,      // 速度模式
  SERVO_TOR       // 力矩模式
};

struct ActData {
  ros::Time status_stamp;
  uint64_t seq;
  uint16_t q_raw;
  int16_t qd_raw;
  int16_t tor_raw;
  int64_t q_circle;
  uint16_t q_last;
  double temp;
  int status;
  double pos_raw;
  double pos, vel, effort;
  ros::Time cmd_stamp;
  ServoMode servo_mode;
  ServoCtrlMode ctrl_mode;
  double cmd_pos, cmd_vel, cmd_effort;
  double cmd_pos_unlimit, cmd_vel_unlimit, cmd_effort_unlimit;
};

struct ImuData {
  ros::Time stamp;
  uint64_t seq;
  double orientation[4], angular_vel[3], linear_accel[3], rpy[3];
  double orientation_covariance[3], angular_velocity_covariance[3], linear_acceleration_covariance[3];
};

}  // namespace ubt_hw
