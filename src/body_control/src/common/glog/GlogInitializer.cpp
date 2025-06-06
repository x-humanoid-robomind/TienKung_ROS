#include "GlogInitializer.h"

#include <atomic>

void FailureFunc(const char* data, int size) {
    LOG(ERROR) << data;
}

void GlogInitializer::Init(std::string fileDir) {
    static std::atomic_bool inited = false;
    bool targetState = false;
    if (inited.compare_exchange_strong(targetState, true)) {
        google::InitGoogleLogging("body_control");
        FLAGS_log_dir = fileDir.c_str();
        FLAGS_max_log_size = 100;
        FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr=true;
        google::InstallFailureSignalHandler();
        //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr，可以通过下面的方法自定义输出方式：
        google::InstallFailureWriter(&FailureFunc);
    }
}