#include "TsinghuaHandDevice.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <thread>
#include <glog/logging.h>

TsinghuaHandDevice::TsinghuaHandDevice(int slave, int passage, uint16_t id,
        std::function<void(FingerStatusMessage&)> funcOnStatus) : 
            slave(slave), passage(passage), id(id), funcOnStatus(funcOnStatus) 
{

}

void TsinghuaHandDevice::SetPos(FingerSetPositionCmd& cmd) {
    std::lock_guard<std::mutex> guard(mtxOpt);

    std::list<DeviceMessage*> reqFrames;
    
    // frame 1
    auto msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    FrameId fid;
    fid.id = id;
    fid.cmd = 0x10;
    fid.frameCount = 2;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x08;

    msg->data[0] = cmd.thumb.rotation.enable;
    msg->data[1] = cmd.thumb.rotation.angle;
    msg->data[2] = cmd.thumb.bend.enable;
    msg->data[3] = cmd.thumb.bend.angle;
    msg->data[4] = cmd.fore.bend.enable;
    msg->data[5] = cmd.fore.bend.angle;
    msg->data[6] = cmd.middle.bend.enable;
    msg->data[7] = cmd.middle.bend.angle; 

    reqFrames.push_back(msg);

    // frame 2
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.frameSeq = 2;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x04;

    msg->data[0] = cmd.ring.bend.enable;
    msg->data[1] = cmd.ring.bend.angle;
    msg->data[2] = cmd.little.bend.enable;
    msg->data[3] = cmd.little.bend.angle;

    reqFrames.push_back(msg);
    
    // add a query request each time the CTRL command got
    GenerateQueryStatus(reqFrames);

    cmdQueue.push(reqFrames);
}

void TsinghuaHandDevice::SetEmergencyStop(FingerStopCmd& cmd) {
    std::lock_guard<std::mutex> guard(mtxOpt);

    std::list<DeviceMessage*> reqFrames;

    auto msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    FrameId fid;
    fid.id = id;
    fid.cmd = (cmd.stop ? 0x20 : 0x30);     // 0x20 - stop, 0x30 - clear stop
    fid.frameCount = 1;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x06;

    msg->data[0] = cmd.thumb.rotation.enable;
    msg->data[1] = cmd.thumb.bend.enable;
    msg->data[2] = cmd.fore.bend.enable;
    msg->data[3] = cmd.middle.bend.enable;
    msg->data[4] = cmd.ring.bend.enable;
    msg->data[5] = cmd.little.bend.enable;

    reqFrames.push_back(msg);

    cmdQueue.push(reqFrames);
}

void TsinghuaHandDevice::SetMotionCtrl(FingerMotionCtrlCmd& cmd) {
    std::lock_guard<std::mutex> guard(mtxOpt);

    std::list<DeviceMessage*> reqFrames;

    // frame-0 1/1 for stop
    auto msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    FrameId fid;
    fid.id = id;
    fid.cmd = 0x4A;
    fid.frameCount = 1;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x01;

    msg->data[0] = 0x00;

    reqFrames.push_back(msg);

    // frame-1 1/5 for set
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.cmd = 0x40;
    fid.frameCount = 5;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x08;

    msg->data[0] = cmd.thumb.rotation.enable;
    msg->data[1] = cmd.thumb.rotation.vel;
    msg->data[2] = cmd.thumb.rotation.startAngle;
    msg->data[3] = cmd.thumb.rotation.maxAngle;
    msg->data[4] = cmd.thumb.bend.enable;
    msg->data[5] = cmd.thumb.bend.vel;
    msg->data[6] = cmd.thumb.bend.startAngle;
    msg->data[7] = cmd.thumb.bend.maxAngle;

    reqFrames.push_back(msg);

    // frame-2 2/5 for set
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.frameSeq = 2;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x08;

    msg->data[0] = (cmd.thumb.threshold & 0x0FF00) >> 8;
    msg->data[1] = (cmd.thumb.threshold & 0x0FF);
    msg->data[2] = cmd.fore.bend.enable;
    msg->data[3] = cmd.fore.bend.vel;
    msg->data[4] = cmd.fore.bend.startAngle;
    msg->data[5] = cmd.fore.bend.maxAngle;
    msg->data[6] = (cmd.fore.threshold & 0x0FF00) >> 8;
    msg->data[7] = (cmd.fore.threshold & 0x0FF);

    reqFrames.push_back(msg);

    // frame-3 3/5 for set
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.frameSeq = 3;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x08;

    msg->data[0] = cmd.middle.bend.enable;
    msg->data[1] = cmd.middle.bend.vel;
    msg->data[2] = cmd.middle.bend.startAngle;
    msg->data[3] = cmd.middle.bend.maxAngle;
    msg->data[4] = (cmd.middle.threshold & 0x0FF00) >> 8;
    msg->data[5] = (cmd.middle.threshold & 0x0FF);
    msg->data[6] = cmd.ring.bend.enable;
    msg->data[7] = cmd.ring.bend.vel;

    reqFrames.push_back(msg);

    // frame-4 4/5 for set
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.frameSeq = 4;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x08;

    msg->data[0] = cmd.ring.bend.startAngle;
    msg->data[1] = cmd.ring.bend.maxAngle;
    msg->data[2] = (cmd.ring.threshold & 0x0FF00) >> 8;
    msg->data[3] = (cmd.ring.threshold & 0x0FF);
    msg->data[4] = cmd.little.bend.enable;
    msg->data[5] = cmd.little.bend.vel;
    msg->data[6] = cmd.little.bend.startAngle;
    msg->data[7] = cmd.little.bend.maxAngle;

    reqFrames.push_back(msg);

    // frame-5 4/5 for set
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.frameSeq = 5;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x02;

    msg->data[0] = (cmd.little.threshold & 0x0FF00) >> 8;
    msg->data[1] = (cmd.little.threshold & 0x0FF);

    reqFrames.push_back(msg);

    // frame-6 1/1 for start
    msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    fid.id = id;
    fid.cmd = 0x4A;
    fid.frameCount = 1;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x01;

    msg->data[0] = 0x01;

    reqFrames.push_back(msg);

    // add a query request each time the CTRL command got
    GenerateQueryStatus(reqFrames);

    cmdQueue.push(reqFrames); 
}

void TsinghuaHandDevice::GenerateQueryStatus(std::list<DeviceMessage*>& reqFrames) {
    // query status
    auto msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    FrameId fid;
    fid.id = id;
    fid.cmd = 0xF1;
    fid.frameCount = 1;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x01;

    msg->data[0] = 0x00;

    reqFrames.push_back(msg);
    // LOG(INFO) << "====================================";
}

void TsinghuaHandDevice::GenerateQueryCtrlState(std::list<DeviceMessage*>& reqFrames) {
    // query status
    auto msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    FrameId fid;
    fid.id = id;
    fid.cmd = 0x4B;
    fid.frameCount = 1;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x01;

    msg->data[0] = 0x00;

    reqFrames.push_back(msg);
}

void TsinghuaHandDevice::GenerateQueryTouchSensors(std::list<DeviceMessage*>& reqFrames) {
    // query touch sensor
    auto msg = new DeviceMessage();
    memset((void*)msg, 0x00, sizeof(DeviceMessage));
    FrameId fid;
    fid.id = id;
    fid.cmd = 0x01;
    fid.frameCount = 1;
    fid.frameSeq = 1;
    memcpy(&msg->id, &fid, sizeof(FrameId));
    msg->dlc = 0x01;

    msg->data[0] = 0x00;

    reqFrames.push_back(msg);
}

void TsinghuaHandDevice::GenerateQueryRequest(std::list<DeviceMessage*>& reqFrames) {
    GenerateQueryStatus(reqFrames);
    GenerateQueryCtrlState(reqFrames);
    // so far, we do not need the touch sensor
    // GenerateQueryTouchSensors(reqFrames);
}

void TsinghuaHandDevice::OnRequest(int passage, DeviceMessage& msg) {
    if (curReqFrames.empty()) {
        if (!cmdQueue.empty()) {
            curReqFrames = cmdQueue.pop();
        }
        else {
            // auto generate query request when no command received
            GenerateQueryStatus(curReqFrames);
        }
    }

    if (!curReqFrames.empty()) {
        auto first = curReqFrames.front();
        msg = *first;
        curReqFrames.remove(first);
        delete first;
    }
}

void TsinghuaHandDevice::OnStatus(FrameId& fid, DeviceMessage& msg) {
    FingerStatusMessage fmsg;
    fmsg.thumb.rotation.angle = (uint32_t)msg.data[0];
    fmsg.thumb.bend.angle = (uint32_t)msg.data[1];
    fmsg.fore.bend.angle = (uint32_t)msg.data[2];
    fmsg.middle.bend.angle = (uint32_t)msg.data[3];
    fmsg.ring.bend.angle = (uint32_t)msg.data[4];
    fmsg.little.bend.angle = (uint32_t)msg.data[5];
    
    SetReady();
    funcOnStatus(fmsg);
}

void TsinghuaHandDevice::OnTouchSensors(FrameId& fid, DeviceMessage& msg) {
    LOG(INFO) << "Touch Sensor:";
    LOG(INFO) << "frameSeq:" << fid.frameCount;
    for (int i = 0; i < msg.dlc; ++i) {
        LOG(INFO) << "data:" << std::hex << (int)msg.data[i];
    }
}

void TsinghuaHandDevice::OnResponse(int passage, DeviceMessage& msg) {
    if (msg.id != 0) {
        
        // static long c = 0;
        // LOG(INFO) << ++c;
        FrameId fid;
        memcpy(&fid, &msg.id, sizeof(FrameId));
        
        if (fid.id == id) {
            switch (fid.cmd) {
                // status response
                case 0x0F1: {
                    OnStatus(fid, msg);
                    break;
                }
                // touch sensors response
                case 0x01: {
                    OnTouchSensors(fid, msg);
                    break;
                }
            }
        }
    }
    else {
        // ---
    }
}