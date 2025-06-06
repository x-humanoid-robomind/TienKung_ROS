#include "FastConnection.h"
#include <glog/logging.h>

namespace fast_ros {

FastConnection& FastConnection::Instance() {
    static FastConnection singleton;
    return singleton;
}

FastConnection::FastConnection() {
    subsLast = std::make_shared<SubscriberMatrix>();
}

void FastConnection::publish(const std::string& topic, const std::any& message) {
    while (!queueSubs.empty()) {
        subsCurrent = queueSubs.pop();
    }

    if (subsCurrent) {
        try {
            auto& fps = subsCurrent->at(topic);
            for (auto& fp : fps) {
                fp.func(message);
            }
        } catch(std::exception& e) {}
    }
}

std::function<void()> FastConnection::subscribe(const std::string& topic, std::function<void(const std::any&)> funcOnMessage) {
    std::lock_guard<std::mutex> guard(mtxSubsLast);
    auto subsLastNew = std::make_shared<SubscriberMatrix>();
    *subsLastNew = *subsLast;
    auto id = ++idGen;
    auto shutdown = [this, topic, id]() {
        std::lock_guard<std::mutex> guard(mtxSubsLast);
        try {
            auto& fps = subsLast->at(topic);
            for (auto fp = fps.begin(); fp != fps.end(); ++fp) {
                if (fp->id == id) {
                    fps.erase(fp);
                    queueSubs.push(subsLast);
                    break;
                }
            }
        } catch(std::exception& e) {}
    };
    auto& subs = (*subsLastNew)[topic];
    subs.push_back({id, funcOnMessage});
    subsLast = subsLastNew;
    queueSubs.push(subsLast);

    return shutdown;
}

}