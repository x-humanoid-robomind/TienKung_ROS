#ifndef UBT_USB_SBUS_PROCESS_HPP_
#define UBT_USB_SBUS_PROCESS_HPP_
#include "ros/ros.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <string>

#include "CerealPort.h"

using namespace std;

// SBUS protocol constants
#define USBSBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS2_FOOTER 0x04
#define SBUS_LOST_FRAME 0x20
#define SBUS_FAILSAFE 0x10
#define SBUS_BAUD_RATE 115200
#define USBSBUS_PACKET_SZIE 35
#define USBSBUS_XOR_END     34
#define USBSBUS_XOR_BEGIN   1
#define USBSBUS_FLAGS_INDEX 33//第33个字节
#define SBUS_CH_MAXDATA     0X7FF
#define RC_MIN              0
#define RC_MAX              0x7ff

class usbSbus
{
public:
  usbSbus(){}
  ~usbSbus() { }

public:
    bool init(const char *cDevName);
    int sbus_read(uint16_t* channels_out);
    int sbus_write(uint16_t* channels_in);
    int sbus_failsafe(void);
    int sbus_lost_frames(void);

private:
    bool USBSBUS_XOR_CHECK(char *data);
    void usbSbus_decode(char *data,uint16_t* channels_out);

private:
    cereal::CerealPort m_serialHandle;
    uint8_t buffer[USBSBUS_PACKET_SZIE];
    int timeout_ms;
    int lost_frames;
    int failsafe;
    struct timespec packet_start;

};

#endif
