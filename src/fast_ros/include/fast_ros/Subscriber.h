#pragma once

#include <ros/subscriber.h>
#include "ConnectionType.h"

namespace fast_ros {

class Subscriber {

public:
    void shutdown();

private:
    friend class NodeHandle;

    ConnectionType type;
    std::function<void()> funcShutdown;
    ros::Subscriber subRos;
};

}