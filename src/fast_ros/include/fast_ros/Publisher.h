#pragma once

#include <ros/publisher.h>
#include <any>
#include "ConnectionType.h"

namespace fast_ros {

class Publisher {
public:
    Publisher();
    template <typename M>
    void publish(const boost::shared_ptr<M>& message) const
    {
        if (type & ConnectionType::FAST_ROS) {
            funcPubFast(message);      
        }
        
        if (type & ConnectionType::NATIVE_ROS) {
            pubRos.publish(message);
        }
    }
private:
    friend class NodeHandle;

    ConnectionType type;
    ros::Publisher pubRos;
    std::function<void(const std::any&)> funcPubFast;
};

}