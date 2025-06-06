#pragma once

#include <ros/ros.h>
#include "Publisher.h"
#include "Subscriber.h"
#include "FastConnection.h"
#include <glog/logging.h>

namespace fast_ros {

class NodeHandle {
public:
    NodeHandle(ros::NodeHandle& nh);

    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size, ConnectionType type = ConnectionType::ALL, bool latch = false)
    {
        Publisher pub;
        pub.type = type;
        pub.pubRos = nh.advertise<M>(topic, queue_size, latch);
        auto nsTopic = nh.getNamespace() + "/" + topic;
        pub.funcPubFast = [nsTopic](const std::any& message) {
            static FastConnection& fc = FastConnection::Instance();
            fc.publish(nsTopic, message);
        };
        return pub;
    }

    template<class M, class T>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
        ConnectionType type = ConnectionType::FAST_ROS)
    {
        Subscriber sub;
        if (type & ConnectionType::FAST_ROS) {
            auto func = std::bind(fp, obj, std::placeholders::_1);
            auto funcOnMessage = [func](const std::any& message){
                auto rawMessage = std::any_cast<boost::shared_ptr<M>>(message);
                func(rawMessage);
            };
            FastConnection::Instance().subscribe(topic, funcOnMessage);        
        }

        if (type & ConnectionType::NATIVE_ROS) {
            sub.subRos = nh.subscribe(topic, queue_size, fp, obj);
        }
 
        return sub;
    }

private:
    ros::NodeHandle nh;
};

}