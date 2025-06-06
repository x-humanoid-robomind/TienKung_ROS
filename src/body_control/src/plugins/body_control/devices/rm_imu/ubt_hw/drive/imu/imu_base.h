/*******************************************************************************
 * Copyright (c) 2023/09/06, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once
#include "ubt_hw/data_type.h"
#include "ubt_hw/type.h"
namespace ubt_hw {

class ImuBase {
 public:
  ImuBase() = default;
  ~ImuBase() = default;
  virtual bool init(ros::NodeHandle& imu_nh) = 0;
  virtual void readStatus(ubt_hw::CAN_PACKAGE can_package) = 0;
  void quatToRPY(const double* q, double& roll, double& pitch, double& yaw) {
    // x: q[0] y: q[1] z: q[2] w: q[3]
    double as = fmin(-2. * (q[0] * q[2] - q[3] * q[1]), .99999);
    yaw = std::atan2(2 * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);
    pitch = std::asin(as);
    roll = std::atan2(2 * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2]);
  }
  std::string name_;
  std::string type_;
  std::string info_;
  uint32_t can_bus_;
  ImuData* data_{};
};
}  // namespace ubt_hw
