#include "XsensImuDevice.h"

XsensImuDevice::XsensImuDevice(int slave, int passages[4], uint16_t ids[4], std::function<void(ImuData&)> funcOnMessage)
    : xsens_mode1::XsensImuDevice(slave, ids[0], passages[0], ids[1], passages[1], ids[2], passages[2], funcOnMessage) {

}