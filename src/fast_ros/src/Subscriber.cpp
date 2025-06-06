#include "Subscriber.h"

namespace fast_ros {

void Subscriber::shutdown() {
    if (type & ConnectionType::FAST_ROS) {
        funcShutdown();    
    }
    
    if (type & ConnectionType::NATIVE_ROS) {
        subRos.shutdown();
    }
}

}