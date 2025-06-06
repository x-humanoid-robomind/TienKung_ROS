#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <list>
#include <memory>

#include "SoemDevice.h"
#include "SoemMessage.h"

class SoemMaster {
public:
    enum class State : int {
        NOT_INITIALIZED,
        INITIALIZING,
        INITIALIZED
    };

    enum class Mode : int {
        STANDARD,       // support standard can frame, for both SoemMaster and SomeDevice
        EXTENDED,       // support extended can frame, for both SoemMaster and SomeDevice
        TIME_SHARING    // support both standard and extended can frame by time sharing system, only for SomeMaster
    };

    bool Init(std::string ifname, double rate, std::vector<Mode> slaveModes = {});
    void Stop() { running = false; }
    // Attention: "mode" should by STANDARD or EXTENDED, do not set it to TIME_SHARING in register functions
    bool RegisterDevice(int slave, int passage, std::shared_ptr<SoemDevice> dev, Mode mode = Mode::STANDARD);
    bool RegisterDevice(int slave, std::vector<int> passages, std::shared_ptr<SoemDevice> dev, Mode mode = Mode::STANDARD);
    
    inline State GetState() {
        return state;
    }
    
    static SoemMaster& Instance();
private:

    struct RegisteredDevice {
        int slave;
        int passage;
        std::shared_ptr<SoemDevice> dev;
        Mode mode;
    };
    std::list<RegisteredDevice> devices;

    std::vector<Mode> vecCurTsMode;
    std::vector<Mode> slaveModes;

    static const int SLAVE_NUMBER = 6;
    static const int K_ETHERCAT_ERR_PERIOD = 100;
    static const int K_ETHERCAT_ERR_MAX = 20;
    int wkc_err_count = 0;
    int wkc_err_iteration_count = 0;
    std::atomic<State> state;
    double rate;
    bool running = true;
    SlaveMessage reqBuff[SLAVE_NUMBER];
    SlaveMessage respBuff[SLAVE_NUMBER];
    bool SlaveExtCan[SLAVE_NUMBER] = {0};

    SoemMaster() = default;

    bool SetSlaveExtCan(int slave);
    int Start(std::string& ifname);
    bool InitSoem(std::string ifname);
    void Run();
    void GetRequestFromDevices();
    void SendResponseToDevices();
};