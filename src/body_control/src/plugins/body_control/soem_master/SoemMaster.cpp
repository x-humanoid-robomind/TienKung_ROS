#include "SoemMaster.h"

#include "ethercat.h"

#include <thread>
#include <cstring>
#include "util/Rate2.h"
#include "util/humanRate.hpp"
#include <glog/logging.h>

#define EC_TIMEOUTM

char IOmap[4096];
OSAL_THREAD_HANDLE checkThread;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint64_t num;

#define EC_TIMEOUTMON 500

SoemMaster& SoemMaster::Instance() {
    static SoemMaster singleton;
    return singleton;
}

static void degraded_handler()
{
    LOG(INFO) << "[EtherCAT Error] Logging error...";
    time_t current_time = time(NULL);
    char *time_str = ctime(&current_time);
    LOG(INFO) << "ESTOP. EtherCAT became degraded at " << time_str;
    LOG(INFO) << "[EtherCAT Error] Stopping RT process.";
}

int SoemMaster::Start(std::string& ifname)
{
    int i;
    int oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    num = 1;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname.c_str()))
    {
        LOG(INFO) << "[EtherCAT Init] Initialization on device " << ifname << " succeeded.";
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            LOG(INFO) << "[EtherCAT Init] " << ec_slavecount << " slaves found and configured.";
            if (ec_slavecount < SLAVE_NUMBER)
            {
                LOG(INFO) << "[RT EtherCAT] Warning: Expected " << SLAVE_NUMBER << " slaves, found " << ec_slavecount << ".";
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx+1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_config_map(&IOmap);
            ec_configdc();

            LOG(INFO) << "[EtherCAT Init] Mapped slaves.";
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
            {
                LOG(INFO) << "[SLAVE " << slave_idx << "]"
                    << "  IN  " << ec_slave[slave_idx].Ibytes << " bytes, " << ec_slave[slave_idx].Ibits << " bits"
                    << "  OUT " << ec_slave[slave_idx].Obytes << " bytes, " << ec_slave[slave_idx].Obits << " bits";
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            LOG(INFO) << "[EtherCAT Init] segments : "
                << ec_group[0].nsegments << ":"
                << ec_group[0].IOsegment[0] << " "
                << ec_group[0].IOsegment[1] << " "
                << ec_group[0].IOsegment[2] << " "
                << ec_group[0].IOsegment[3];

            LOG(INFO) << "[EtherCAT Init] Requesting operational state for all slaves...";
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            LOG(INFO) << "[EtherCAT Init] Calculated workcounter " << expectedWKC;
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                LOG(INFO) << "[EtherCAT Init] Operational state reached for all slaves.";
                inOP = TRUE;
                return 1;
            }
            else
            {
                LOG(WARNING) << "[EtherCAT Error] Not all slaves reached operational state.";
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        // printf"[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s",
                        //        i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                        LOG(WARNING) << "[EtherCAT Error] Slave " << i
                            << " State=0x" << std::hex << ec_slave[i].state
                            << " StatusCode=0x" << ec_slave[i].ALstatuscode 
                            << ": " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode);
                    }
                }
            }
        }
        else
        {
            LOG(ERROR) << "[EtherCAT Error] No slaves found!";
        }
    }
    else
    {
        LOG(ERROR) << "[EtherCAT Error] No socket connection on " << ifname << " - are you running run.sh?";
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

static OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    (void)ptr;
    int slave = 0;
    while (1)
    {
        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX)
        {
            // possibly shut down
            LOG(ERROR) << "[EtherCAT Error] EtherCAT connection degraded.";
            LOG(ERROR) << "[Simulink-Linux] Shutting down....";
            degraded_handler();
            break;
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                LOG(INFO) << "";
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        LOG(ERROR) << "[EtherCAT Error] Slave " << slave << " is in SAFE_OP + ERROR, attempting ack.";
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        LOG(ERROR) << "[EtherCAT Error] Slave " << slave << " is in SAFE_OP, change to OPERATIONAL.";
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            LOG(INFO) << "[EtherCAT Status] Slave " << slave << " reconfigured";
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            LOG(ERROR) << "[EtherCAT Error] Slave " << slave << " lost";
                            {
                                LOG(ERROR) << "[EtherCAT Error] Try to stop ethercat master! ";
                                SoemMaster::Instance().Stop();
                                return;
                            }

                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            LOG(INFO) << "[EtherCAT Status] Slave " << slave << " recovered";
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        LOG(INFO) << "[EtherCAT Status] Slave " << slave << " found";
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                LOG(INFO) << "[EtherCAT Status] All slaves resumed OPERATIONAL.";
        }
        osal_usleep(50000);
    }
}

bool SoemMaster::InitSoem(std::string ifname) {
    int i;
    int rc;
    LOG(INFO) << "[EtherCAT] Initializing EtherCAT";
    osal_thread_create((void *)&checkThread, 128000, (void *)&ecatcheck, (void *)&ctime);
    for (i = 1; i < 10; i++)
    {
        LOG(INFO) << "[EtherCAT] Attempting to start EtherCAT, try " << i << " of 10.";
        rc = Start(ifname);
        if (rc) {
            break;
        }
        osal_usleep(1000000);
    }
    if (rc)
        LOG(INFO) << "[EtherCAT] EtherCAT successfully initialized on attempt " << i;
    else
    {
        LOG(ERROR) << "[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. ";
        return false;
    }
    return true;
}

bool SoemMaster::Init(std::string ifname, double rate, std::vector<Mode> slaveModes) {
    auto reqState = State::NOT_INITIALIZED;
    if (state.compare_exchange_strong(reqState, State::INITIALIZING)) {
        this->slaveModes = slaveModes;
        vecCurTsMode.resize(slaveModes.size());
        for (auto& state: vecCurTsMode) {
            state = Mode::STANDARD;
        }
        this->rate = rate;
        auto rlt = InitSoem(ifname);
        state = rlt ? State::INITIALIZED : State::NOT_INITIALIZED;
        if (rlt) {
            std::thread(&SoemMaster::Run, this).detach();
        }
        return true;
    }
    return false;
}

void SoemMaster::Run()
{
    LOG(INFO) << "start publisher thread: " << gettid() << std::endl;
#if 0
    Rate r(rate);
#else
    body_human_rate_control::humanRateControl r(rate);
#endif
    while (running) {
        if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            wkc_err_count = 0;
            wkc_err_iteration_count = 0;
        }
        if (wkc_err_count > K_ETHERCAT_ERR_MAX)
        {
            LOG(ERROR) << "[EtherCAT Error] Error count too high!";
            degraded_handler();
        }
        // send
        GetRequestFromDevices();
        ec_send_processdata();
        // receive
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        SendResponseToDevices();
        //  check for dropped packet
        if (wkc < expectedWKC)
        {
            LOG(ERROR) << "[EtherCAT Error] Dropped packet (Bad WKC!)";
            wkc_err_count++;
        }
        else
        {
            needlf = TRUE;
        }
        wkc_err_iteration_count++;
        // usleep(1000);
        r.sleep();
    }
}

bool SoemMaster::RegisterDevice(int slave, int passage, std::shared_ptr<SoemDevice> dev, Mode mode) {
    // TODO: check slave & id
    RegisteredDevice rd = {
        slave,
        passage,
        dev,
        mode
    };
    devices.push_back(rd);
    return true;
}

bool SoemMaster::RegisterDevice(int slave, std::vector<int> passages, std::shared_ptr<SoemDevice> dev, Mode mode) {
    auto rlt = true;
    for (auto passage : passages) {
        rlt = (rlt && RegisterDevice(slave, passage, dev, mode));
    }
    return rlt;
}

void SoemMaster::GetRequestFromDevices() {
    memset(reqBuff, 0x00, SLAVE_NUMBER * sizeof(SlaveMessage));
    for (auto dev : devices) {
        switch (slaveModes[dev.slave]) {
            case Mode::STANDARD: 
            case Mode::EXTENDED: {
                if (slaveModes[dev.slave] == dev.mode) {
                    DeviceMessage& msgDev = reqBuff[dev.slave].dev[dev.passage - 1];
                    dev.dev->OnRequest(dev.passage, msgDev);
                }
                break;
            }
            case Mode::TIME_SHARING: {
                if (vecCurTsMode[dev.slave] == dev.mode) {
                    DeviceMessage& msgDev = reqBuff[dev.slave].dev[dev.passage - 1];
                    dev.dev->OnRequest(dev.passage, msgDev);
                }
                break;
            }
        }
    }

    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        switch (slaveModes[slave]) {
            case Mode::STANDARD: {
                // do nothing
                break;
            }
            case Mode::EXTENDED: {
                reqBuff[slave].can_ide = 1;
                break;
            }
            case Mode::TIME_SHARING: {
                if (vecCurTsMode[slave] == Mode::EXTENDED) {
                    reqBuff[slave].can_ide = 1;
                }
                break;
            }
        }
        auto msgSlave = (SlaveMessage*)(ec_slave[slave + 1].outputs);
        *msgSlave = reqBuff[slave];
    }
}

void SoemMaster::SendResponseToDevices() {
    memset(respBuff, 0x00, SLAVE_NUMBER * sizeof(SlaveMessage));
    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        auto msgSlave = (SlaveMessage*)(ec_slave[slave + 1].inputs);
        respBuff[slave] = *msgSlave;
    }

    for (auto dev : devices) {
        switch (slaveModes[dev.slave]) {
            case Mode::STANDARD: 
            case Mode::EXTENDED: {
                if (slaveModes[dev.slave] == dev.mode) {
                    DeviceMessage& msgDev = respBuff[dev.slave].dev[dev.passage - 1];
                    dev.dev->OnResponse(dev.passage, msgDev);
                }
                break;
            }
            case Mode::TIME_SHARING: {
                if (vecCurTsMode[dev.slave] == dev.mode) {
                    DeviceMessage& msgDev = respBuff[dev.slave].dev[dev.passage - 1];
                    dev.dev->OnResponse(dev.passage, msgDev);

                }
                break;
            }
        }
    }

    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        if (slaveModes[slave] == Mode::TIME_SHARING) {
            vecCurTsMode[slave] = (vecCurTsMode[slave] == Mode::STANDARD ? Mode::EXTENDED : Mode::STANDARD);
        }
    }
}