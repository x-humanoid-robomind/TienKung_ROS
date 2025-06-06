#pragma once

#include <string>
#include <glog/logging.h>

class GlogInitializer {
public:
    static void Init(std::string fileDir);
};

#define INIT_GLOG(FILE_DIR) GlogInitializer::Init(FILE_DIR); LOG(INFO) << "Try to initialize glog.";