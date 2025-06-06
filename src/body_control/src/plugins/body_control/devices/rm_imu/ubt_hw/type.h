/******************************************************************************* 
* Copyright (c) 2023/07/14, Liao LunJia. 
* All rights reserved. 
*******************************************************************************/ 
#pragma once 
	 
#include <chrono> 
#include <string> 
namespace ubt_hw { 
	 
#pragma pack(1) 
struct CAN_ID { 
    uint32_t channel : 16; 
    uint32_t index : 8; 
    uint32_t end : 1; 
    uint32_t can : 1; 
    uint32_t mode : 3; 
    uint32_t size : 3; 
}; 
	 
struct RX_CAN_ID { 
    uint32_t id : 8; 
    uint32_t index : 16; 
    uint32_t end : 1; 
    uint32_t bus : 1; 
    uint32_t mode : 3; 
    uint32_t size : 3; 
}; 
	 
struct TMOTOR_TX_CAN_ID { 
    uint32_t id : 8; 
    uint32_t control_mode : 8; 
    uint32_t index : 8; 
    uint32_t end : 1; 
    uint32_t bus : 1; 
    uint32_t mode : 3; 
    uint32_t size : 3; 
}; 
 
struct CAN_PACKAGE { 
    uint8_t can_type;  // 0：standard； 1： extend 
    uint8_t can_bus;   // 0：can0； 1： can1 
    union HEAD { 
        struct CAN_ID can_id; 
        struct RX_CAN_ID rx_can_id; 
        struct TMOTOR_TX_CAN_ID tmotor_tx_can_id; 
        uint32_t h32; 
    } head; 
	uint8_t data[8]; 
}; 
	 
struct CanPackageStamp { 
    ros::Time stamp; 
    CAN_PACKAGE can_package{}; 
}; 
    
struct UDP_PACKAGE { 
    uint8_t index; 
    uint8_t packageNum; 
    CAN_PACKAGE package[100]; 
}; 
	 
struct IpUdpPackage { 
    std::string ip; 
    UDP_PACKAGE udp_package{}; 
}; 
	 
#pragma pack() 
	 
}  // namespace ubt_hw