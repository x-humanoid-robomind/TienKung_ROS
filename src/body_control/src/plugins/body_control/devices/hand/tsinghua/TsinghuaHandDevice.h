#pragma once

#include "../../soem_master/SoemDevice.h"
#include "util/LockFreeQueue.h"
#include <mutex>
#include <functional>
#include <list>
#include "finger_types.h"

class TsinghuaHandDevice : public SoemDevice {
public:
    enum class Type : int {
        S_4310 = 1,
        M_8112,
        L_10020
    }; 
    TsinghuaHandDevice(int slave, int passage, uint16_t id,
        std::function<void(FingerStatusMessage&)> funcOnStatus);

    void SetPos(FingerSetPositionCmd& cmd);
    void SetEmergencyStop(FingerStopCmd& cmd);
    void SetMotionCtrl(FingerMotionCtrlCmd& cmd);

private:
    friend class TsinghuaHandDeviceManager;
    int slave;
    int passage;
    uint16_t id;
    LockFreeQueue<std::list<DeviceMessage*>> cmdQueue;
    std::list<DeviceMessage*> curReqFrames;
    std::mutex mtxOpt;
    std::function<void(FingerStatusMessage&)> funcOnStatus;

    TsinghuaHandDevice() = default;

protected:
    virtual void OnRequest(int passage, DeviceMessage& msg) override;
    virtual void OnResponse(int passage, DeviceMessage& msg) override;

    void GenerateQueryStatus(std::list<DeviceMessage*>& reqFrames);
    void GenerateQueryCtrlState(std::list<DeviceMessage*>& reqFrames);
    void GenerateQueryTouchSensors(std::list<DeviceMessage*>& reqFrames);
    void GenerateQueryRequest(std::list<DeviceMessage*>& reqFrames);

    void OnStatus(FrameId& fid, DeviceMessage& msg);
    void OnTouchSensors(FrameId& fid, DeviceMessage& msg);
};