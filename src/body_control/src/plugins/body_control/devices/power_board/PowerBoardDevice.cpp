#include "PowerBoardDevice.h"
#include <glog/logging.h>
#include <iostream>
#include <chrono>

PowerBoardDevice::PowerBoardDevice(int slave, int passage, uint16_t id,
        std::function<void(const PowerMgr& status)> funcOnStatus)
        : id(id), passage(passage), funcOnStatus(funcOnStatus)
{

}
void PowerBoardDevice::enable_beep()
{
    is_beeping_.store(true);
}

void PowerBoardDevice::disable_beep()
{
    is_beeping_.store(false);
}
void PowerBoardDevice::OnRequest(int passage, DeviceMessage& msg) {

    auto now = std::chrono::system_clock::now();
    msg.id = id;
    msg.rtr = 0;
    msg.dlc = 0;
    msg.data[0] = 0x0;

    if((now - last_request_time_) >= std::chrono::seconds(1))
    {
        last_request_time_ = now;
        mgr.update_req_id(0x80 | (uint8_t)PbIns::GET_WAIST_TEMP);
    }

    uint8_t req_id = mgr.get_req_id();
    if(last_req_id_ != req_id || now - last_req_time_ >= std::chrono::milliseconds(10))
    {
        req_cnt_ = 0;
        last_req_id_ = req_id;
        msg.data[0] = req_id;
        if(msg.data[0] != 0x0)
        {
            last_req_time_ = now;
            msg.dlc = 1;
        }
    }
    if(0 == msg.data[0])
    {
        if(is_beeping_.load() && now - last_beep_time_ >= std::chrono::milliseconds(100))
        {
            msg.data[0] = 0x8F;
            last_beep_time_ = now;
            msg.dlc = 1;
        }
    }
    // msg.data[0] = mgr.NextIns();
}

void PowerBoardDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id != 0)
    {
        mgr.ParseCanPackage(msg.data);

        if (mgr.IsReady()) {
            SetReady();

            // bodyctrl_msgs::PowerStatus status;
            // for (auto i : mgr.GetDataIndex()) {
            //     status.data.push_back(std::any_cast<float>(mgr.values[i]));
            // }

            funcOnStatus(mgr);
            mgr.ClearReady();
        }
    }
}