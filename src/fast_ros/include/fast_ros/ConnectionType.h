#pragma once

namespace fast_ros {

enum ConnectionType : int {
    FAST_ROS    = 0x01,
    NATIVE_ROS  = 0x02,
    ALL         = FAST_ROS | NATIVE_ROS
};

}