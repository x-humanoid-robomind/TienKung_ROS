/*******************************************************************************
 * Copyright (c) 2023/09/06, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include "ros/ros.h"
#include "ubt_hw/drive/imu/imu_base.h"
#include "ubt_hw/filters/complementary_filter.h"
#include "ubt_hw/filters/lp_filter.h"
#include "ubt_hw/filters/hp_filter.h"

namespace rm_imu {

class RmImu : public ubt_hw::ImuBase {
 public:
  RmImu() = default;
  ~RmImu() = default;
  virtual bool init(ros::NodeHandle &imu_nh) override;
  virtual void readStatus(ubt_hw::CAN_PACKAGE can_package) override;

 protected:
  void filterUpdate(const ros::Time &time, double *gyro, double *accel);
  void filterReset();
  void accFilter(double *gyro, const ros::Time &time);
  void gyroFilter(double *gyro, const ros::Time &time);

  double angular_vel_offset[3] = {-0.0, 0.0, -0.0}; //  {-0.0022, 0.0001, -0.0001}
  double ori_cov[3] = {0.0012, 0.0012, 0.0012};
  double angular_vel_cov[3] = {0.0004, 0.0004, 0.0004};
  double linear_acc_cov[3] = {0.01, 0.01, 0.01};
  double angular_vel_coeff = 0.0010652644;
  double accel_coeff = 0.0017944335;
  double av[3], la[3], ori[4];
  double roll_, pitch_, yaw_;
  double la_f_[3];
  double av_f_[3];

  // imu filter
  std::shared_ptr<ubt_hw::ComplementaryFilter> comple_filters_;
  std::vector<std::unique_ptr<ubt_hw::LowPassFilter>> acc_filters_;
  std::vector<std::unique_ptr<ubt_hw::HighPassFilter>> gyro_filters_;
  double lp_freq = 40;
  double hp_freq = 10;
  bool do_bias_estimation_{true};
  bool do_adaptive_gain_{true};
  double gain_acc_ = 0.003; // 0.0003
  double bias_alpha_ = 0.01;
  bool initialized_filter_{false};
  ros::Time last_update_;
};
}  // namespace rm_imu
