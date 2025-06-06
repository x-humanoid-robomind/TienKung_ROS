#pragma once

#include "../../rm_imu/ubt_hw/drive/imu/rm_imu.h"

class XsensImuHR : public rm_imu::RmImu {    
public:
    void UpdateAcc(double acc[3]);
    void UpdateGyr(double gyr[3]);
};