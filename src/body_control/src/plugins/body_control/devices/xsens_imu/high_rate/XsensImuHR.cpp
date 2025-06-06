#include "XsensImuHR.h"

void XsensImuHR::UpdateAcc(double acc[3]) {

    la[0] = acc[0];
    la[1] = acc[1];
    la[2] = acc[2];
    // TODO
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

void XsensImuHR::UpdateGyr(double gyr[3]) {

    av[0] = gyr[0];
    av[1] = gyr[1];
    av[2] = gyr[2];


}