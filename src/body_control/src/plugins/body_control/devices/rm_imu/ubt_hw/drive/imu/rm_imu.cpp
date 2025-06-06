/*******************************************************************************
 * Copyright (c) 2023/09/06, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "ubt_hw/drive/imu/rm_imu.h"

#include <ros/ros.h>
namespace rm_imu {
bool RmImu::init(ros::NodeHandle &imu_nh) {
  for (int i = 0; i < 3; i++) {
    acc_filters_.push_back(std::make_unique<ubt_hw::LowPassFilter>(lp_freq));
    acc_filters_[i]->reset();
    gyro_filters_.push_back(std::make_unique<ubt_hw::HighPassFilter>(hp_freq));
    gyro_filters_[i]->reset();
  }
  return true;
}

void RmImu::readStatus(ubt_hw::CAN_PACKAGE can_package) {
  // modified by Max 2024.1.30
  // if (can_package.head.rx_can_id.id == 100) {
  if (can_package.head.h32 == 5) {
    av[0] = (((int16_t)((can_package.data[1]) << 8) | can_package.data[0]) * angular_vel_coeff) + angular_vel_offset[0];
    av[1] = (((int16_t)((can_package.data[3]) << 8) | can_package.data[2]) * angular_vel_coeff) + angular_vel_offset[1];
    av[2] = (((int16_t)((can_package.data[5]) << 8) | can_package.data[4]) * angular_vel_coeff) + angular_vel_offset[2];
  }
  // if (can_package.head.rx_can_id.id == 101) {
  if (can_package.head.h32 == 6) {
    la[0] = ((int16_t)((can_package.data[1]) << 8) | can_package.data[0]) * accel_coeff;
    la[1] = ((int16_t)((can_package.data[3]) << 8) | can_package.data[2]) * accel_coeff;
    la[2] = ((int16_t)((can_package.data[5]) << 8) | can_package.data[4]) * accel_coeff;

    auto stamp = ros::Time::now();
    filterUpdate(stamp, av, la);
    comple_filters_->getOrientation(ori[3], ori[0], ori[1], ori[2]);
    data_->stamp = stamp;
    data_->seq++;
    data_->orientation[0] = ori[0];
    data_->orientation[1] = ori[1];
    data_->orientation[2] = ori[2];
    data_->orientation[3] = ori[3];
    data_->angular_vel[0] = av_f_[0];
    data_->angular_vel[1] = av_f_[1];
    data_->angular_vel[2] = av_f_[2];
    data_->linear_accel[0] = la_f_[0];
    data_->linear_accel[1] = la_f_[1];
    data_->linear_accel[2] = la_f_[2];
    quatToRPY(data_->orientation, data_->rpy[0], data_->rpy[1], data_->rpy[2]);
  }
}

void RmImu::filterReset() {
  comple_filters_ = std::make_shared<ubt_hw::ComplementaryFilter>();
  comple_filters_->setDoBiasEstimation(do_bias_estimation_);
  comple_filters_->setDoAdaptiveGain(do_adaptive_gain_);

  if (!comple_filters_->setGainAcc(gain_acc_)) {
    ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
  }
  if (do_bias_estimation_) {
    if (!comple_filters_->setBiasAlpha(bias_alpha_)) {
      ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
    }
  }
  comple_filters_->reset();
  for (auto &item : acc_filters_) {
    item->reset();
  }
}

void RmImu::filterUpdate(const ros::Time &time, double *gyro, double *accel) {
  if (!initialized_filter_) {
    last_update_ = time;
    filterReset();
    initialized_filter_ = true;
    return;
  }
  // offline
  if ((time - last_update_).toSec() > 1) {
    last_update_ = time;
    filterReset();
    return;
  }

  // update
  accFilter(accel, time);
  gyroFilter(gyro, time);
  comple_filters_->update(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], (time - last_update_).toSec());
  last_update_ = time;
}

void RmImu::accFilter(double *gyro, const ros::Time &time) {
  for (size_t i = 0; i < acc_filters_.size(); i++) {
    acc_filters_[i]->input(gyro[i], time);
    la_f_[i] = acc_filters_[i]->output();
  }
}

void RmImu::gyroFilter(double *gyro, const ros::Time &time) {
  for (size_t i = 0; i < gyro_filters_.size(); i++) {
    gyro_filters_[i]->input(gyro[i], time);
    av_f_[i] = gyro_filters_[i]->output();
  }
}

}  // namespace rm_imu
