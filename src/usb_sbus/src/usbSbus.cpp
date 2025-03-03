#include "usbSbus.h"

bool usbSbus::init(const char *cDevName)
{
    try 
    {
        m_serialHandle.open(cDevName, SBUS_BAUD_RATE);
        ROS_INFO("usb_sbus open device : %s.",cDevName);
    } 
    catch(const std::exception& e) 
    {
        ROS_ERROR("m_serialHandle open %s is error! : %s \n" ,cDevName,e.what());
        return false;
    }
    return true;
}

int usbSbus::sbus_failsafe(void)
{
    return failsafe;
}

int usbSbus::sbus_lost_frames(void)
{
    return lost_frames;
}

bool usbSbus::USBSBUS_XOR_CHECK(char *data)
{
    char cXorValue = 0;
    for(int i=USBSBUS_XOR_BEGIN; i<USBSBUS_XOR_END;i++)
    {
        cXorValue ^= data[i];
    }
    if(cXorValue==data[34])
        return true;
    else
        return false;
}

void usbSbus::usbSbus_decode(char *data,uint16_t* channels_out)
{
    uint8_t * packet = (uint8_t *)data;
    for(int i=0;i<16;i++)
    {
        channels_out[i] = ((packet[i*2]) * 256) + packet[i*2 + 1];
        channels_out[i] &= SBUS_CH_MAXDATA;
    }
}

int usbSbus::sbus_read(uint16_t* channels_out)
{
    char packet[USBSBUS_PACKET_SZIE];
    char cXorValue;
    int nCount = 0;
    int nSize = 0;

    if(!m_serialHandle.portOpen())
    {
        // 设备未打开
        ROS_ERROR("serial port not open !");
        return -1;
    }

    try
    {
    #if 1
        //查找帧头
        packet[0] = 0;
        while(packet[0]!=USBSBUS_HEADER && ++nCount < USBSBUS_PACKET_SZIE)
        {
            nSize = m_serialHandle.read(packet,1,100);
            if(nSize!=1)
                usleep(100*1000);
        }
        if(packet[0]==USBSBUS_HEADER)
        {
            nSize = m_serialHandle.readBytes(packet+1, (USBSBUS_PACKET_SZIE-1));
            if(nSize==(USBSBUS_PACKET_SZIE-1))
            {
                if(USBSBUS_XOR_CHECK(packet)==true)
                {
                    failsafe = packet[USBSBUS_FLAGS_INDEX] & SBUS_FAILSAFE;
                    if (packet[USBSBUS_FLAGS_INDEX] & SBUS_LOST_FRAME) { lost_frames++; }
                    usbSbus_decode(packet+1,channels_out);
                }
                else
                    return -1;
            }
            else
                return -1;
        }
        else
            return -1;
    #else
        nSize = m_serialHandle.read(packet,1,100);
        if(nSize==1)
        {
            if(packet[0]==USBSBUS_HEADER)
            {
                nSize = m_serialHandle.readBytes(packet+1, (USBSBUS_PACKET_SZIE-1));
                if(nSize==(USBSBUS_PACKET_SZIE-1))
                {
                    if(USBSBUS_XOR_CHECK(packet)==true)
                    {
                        failsafe = packet[USBSBUS_FLAGS_INDEX] & SBUS_FAILSAFE;
                        if (packet[USBSBUS_FLAGS_INDEX] & SBUS_LOST_FRAME) { lost_frames++; }
                        usbSbus_decode(packet+1,channels_out);
                    }
                    else
                        return -1;
                }
            }
            else
                return -1;
        }
        else
            return -1;
    #endif
    }
    catch(const std::exception& e)
    {
      //  ROS_ERROR("m_serialHandle read error! : %s \n", e.what());
        return -1;
    }

    return 16;
}

int usbSbus::sbus_write(uint16_t* channels_in)
{
    if(!m_serialHandle.portOpen())
    {
        // 设备未打开
        ROS_ERROR("serial port not open !");
        return -1;
    }
    
    return 16;
}
