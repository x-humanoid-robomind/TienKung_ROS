#pragma once

#include <string>
#include <any>
#include <functional>
#include <unordered_map>
#include <vector>
#include <memory>
#include <mutex>

#include "LockFreeQueue.h"

namespace fast_ros {

class FastConnection {
public:
    static FastConnection& Instance();

    void publish(const std::string& topic, const std::any& message);
    std::function<void()> subscribe(const std::string& topic, std::function<void(const std::any&)> funcOnMessage);
private:
    FastConnection();

    using SubscribeId = uint32_t;
    struct FunctionPair {
        SubscribeId id;
        std::function<void(const std::any&)> func;
    };

    using SubscriberMatrix = std::unordered_map<std::string, std::vector<FunctionPair>>;
    std::mutex mtxSubsLast;
    std::shared_ptr<SubscriberMatrix> subsCurrent, subsLast;
    LockFreeQueue<std::shared_ptr<SubscriberMatrix>> queueSubs;
    SubscribeId idGen = 0;
};

}