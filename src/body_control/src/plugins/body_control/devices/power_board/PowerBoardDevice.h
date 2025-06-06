#pragma once

#include "../../soem_master/SoemDevice.h"
#include "PowerMgr.h"
#include <atomic>
#include <mutex>
#include <functional>

class PowerBoardDevice : public SoemDevice {
public:
    PowerBoardDevice(int slave, int passage, uint16_t id,
        std::function<void(const PowerMgr& status)> funcOnStatus);

    void enable_beep();
    void disable_beep();

private:
    std::mutex mtxCmd;
    PowerMgr mgr;
    int passage;
    uint16_t id;
    std::function<void(const PowerMgr&)> funcOnStatus;

    std::atomic<bool> is_beeping_ = false;

    std::chrono::system_clock::time_point last_request_time_;
    std::chrono::system_clock::time_point last_beep_time_;
    int beep_count_ = 0;

    uint8_t last_req_id_;
    uint8_t req_cnt_ = 0;
    std::chrono::system_clock::time_point last_req_time_;


    PowerBoardDevice() = default;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;
};