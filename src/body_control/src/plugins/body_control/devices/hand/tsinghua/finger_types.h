#pragma once

#include <cstdint>

struct FrameId {
    uint32_t frameSeq: 8;       // bit0 ~ bit7
    uint32_t frameCount: 8;     // bit8 ~ bit15
    uint32_t cmd: 8;            // bit16 ~ bit23
    uint32_t id: 5;             // bit24 ~ bit28
};

struct FingerCommand {};

// -----------------------------------------

struct FingerMotionCtrl {
    struct Bend {
        uint8_t enable;
        uint8_t vel;
        uint8_t startAngle;
        uint8_t maxAngle;           
    } bend;

    uint16_t threshold;
};

struct ThumbMotionCtrl : public FingerMotionCtrl {
    struct Rotation {
        uint8_t enable;
        uint8_t vel;
        uint8_t startAngle;
        uint8_t maxAngle;
    } rotation;   
};

struct FingerMotionCtrlCmd : public FingerCommand  {
    ThumbMotionCtrl thumb;
    FingerMotionCtrl fore;
    FingerMotionCtrl middle;
    FingerMotionCtrl ring;
    FingerMotionCtrl little;
};


// -----------------------------------------

struct FingerMotion {
    struct Bend {
        uint8_t enable;    
        uint8_t angle;
    } bend;
};

struct ThumbMotion : public FingerMotion {
    struct Rotation {
        uint8_t enable;    
        uint8_t angle;
    } rotation; 
};

struct FingerSetPositionCmd : public FingerCommand {
    ThumbMotion thumb;
    FingerMotion fore;
    FingerMotion middle;
    FingerMotion ring;
    FingerMotion little;
};

// -----------------------------------------

struct FingerStop {
    struct Bend {
        bool enable;
    } bend;
};

struct ThumbStop : public FingerStop {
    struct Rotation {
        bool enable;
    } rotation; 
};

struct FingerStopCmd : public FingerCommand {
    bool stop;
    ThumbStop thumb;
    FingerStop fore;
    FingerStop middle;
    FingerStop ring;
    FingerStop little;
};

// -----------------------------------------

struct FingerStatusData {
    struct Bend {
        uint32_t angle;
    } bend;
};

struct ThumbStatusData : public FingerStatusData {
    struct Rotation {
        uint32_t angle;
    } rotation;
};

struct FingerStatusMessage : public FingerCommand {
    ThumbStatusData thumb;
    FingerStatusData fore;
    FingerStatusData middle;
    FingerStatusData ring;
    FingerStatusData little;
};