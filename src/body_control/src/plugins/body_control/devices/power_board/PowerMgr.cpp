#include "PowerMgr.h"
#include <atomic>
#include <string.h>
#include <glog/logging.h>

#define CMD(INS, TYPE) {INS, {INS << (INS - 1), [this](uint8_t* data){ values[INS] = parse##TYPE(data + 1);}}}

static std::unordered_map<PbIns, std::shared_ptr<CanData>> make_cmds()
{
    std::unordered_map<PbIns, std::shared_ptr<CanData>> cmds;
    cmds.emplace(PbIns::GET_WAIST_TEMP         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_ARM_A_TEMP         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_ARM_B_TEMP         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_LEG_A_TEMP         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_LEG_B_TEMP         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_ARM_A_CURR         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_ARM_B_CURR         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_LEG_A_CURR         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_LEG_B_CURR         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_WAIST_CURR         , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_HEAD_CURR          , std::make_shared<PowerStatus>());
    cmds.emplace(PbIns::GET_BATTERY_INFO       , std::make_shared<BatteryInfomation>());
    cmds.emplace(PbIns::GET_VERSION_INFO       , std::make_shared<PowerVersionInfo>());
    cmds.emplace(PbIns::GET_POWER_BOARD_STATUS , std::make_shared<PowerBoardStatus>());
    return std::move(cmds);
}

PowerMgr::PowerMgr()
  : cmds{make_cmds()}
  , power_board_status_(std::static_pointer_cast<PowerBoardStatus>( cmds[PbIns::GET_POWER_BOARD_STATUS] ))
{
    statusCmds = {
        (std::int8_t)PbIns::GET_WAIST_TEMP,
        (std::int8_t)PbIns::GET_ARM_A_TEMP,
        (std::int8_t)PbIns::GET_ARM_B_TEMP,
        (std::int8_t)PbIns::GET_LEG_A_TEMP,
        (std::int8_t)PbIns::GET_LEG_B_TEMP,
        (std::int8_t)PbIns::GET_BATTERY_INFO
    };
}

void PowerMgr::ParseCanPackage(uint8_t* data) {
    auto ins = data[0];

    if(ins == 0x00)
    {
        return;
    }
    if(ins <= readyBits.size())
    {
        readyBits.set(ins - 1);
    }

    if(ins != (req_id_ & 0x0F))
    {
        return;
    }

    auto itor = cmds.find((PbIns)ins);
    if(itor == cmds.end()) {
        return;
    }
    itor->second->update(&data[1], 6);

    uint8_t idx = ins + 1;
    if(idx > 0x0E)
    {
        idx = 0x00;
        update_req_id(idx);
    }
    else
    {
        update_req_id(idx | 0x80);
    }
    // std::cout << "update idx: " << idx << std::endl;
}

bool PowerMgr::IsReady() {
    if(readyBits.count() == readyBits.size())
    {
        return true;
    }
    if(power_board_status_->is_update())
    {
        return true;
    }
    return false;
}
char PowerMgr::NextIns() {
    auto i = indexStatusCmd++ % statusCmds.size();
    return statusCmds[i];
}
void PowerMgr::ClearReady()
{
    readyBits.reset();
    power_board_status_->clear_statue();
}
const std::shared_ptr<const BatteryInfomation> PowerMgr::GetBatteryInfomation() const
{
    auto itor = cmds.find(PbIns::GET_BATTERY_INFO);
    if(itor != cmds.end()) {
        return std::static_pointer_cast<const BatteryInfomation>(itor->second);
    }
    return nullptr;
}

const std::shared_ptr<const PowerVersionInfo> PowerMgr::GetPowerVersionInfo() const
{
    auto itor = cmds.find(PbIns::GET_VERSION_INFO);
    if(itor != cmds.end()) {
        return std::static_pointer_cast<const PowerVersionInfo>(itor->second);
    }
    return nullptr;
}

const std::shared_ptr<const PowerBoardStatus> PowerMgr::GetPowerBoardStatus() const
{
    return power_board_status_;
}

void PowerStatus::update(uint8_t *data, const std::size_t len)
{
    if(len != 6) {
        std::cout << "PowerStatus update failed, len = " << len << std::endl;
        return;
    }
    max_ = (float)((int16_t)(data[0] << 8 | data[1])) * 0.1f;
    min_ = (float)((int16_t)(data[2] << 8 | data[3])) * 0.1f;
    value_ = (float)((int16_t)(data[4] << 8 | data[5])) * 0.1f;
}

void PowerVersionInfo::update(uint8_t *data, const std::size_t len)
{
    if(len != 6)
    {
        std::cout << "PowerVersionInfo update failed, len = " << len << std::endl;
        return;
    }
    software_version_ = (uint32_t)(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]);
    hardware_version_ = (uint16_t)(data[4] << 24 | data[5] << 16);
}

void BatteryInfomation::update(uint8_t *data, const std::size_t len)
{
    if(len != 6)
    {
        std::cout << "BatteryInfomation update failed, len = " << len << std::endl;
        return;
    }
    voltage_ = (float)((int16_t)(data[0] << 8 | data[1])) * 0.1f;
    power_ = (float)((int16_t)(data[2] << 8 | data[3])) * 0.1f;
    current_ = (float)((int16_t)(data[4] << 8 | data[5])) * 0.1f;
}

void PowerBoardStatus::update(uint8_t *data, const std::size_t len)
{
    if(len != 6)
    {
        std::cout << "PowerBoardStatus update failed, len = " << len << std::endl;
        return;
    }

    work_time_ = (uint32_t)(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]);
    bool is_estop = data[4] & 0x01;
    bool is_remote_estop = data[4] & 0x02;
    bool is_power_on = data[4] & 0x04;

    if(is_estop != is_estop_)
    {
        is_estop_ = is_estop;
        is_update_ = true;
    }
    if(is_remote_estop != is_remote_estop_)
    {
        is_remote_estop_ = is_remote_estop;
        is_update_ = true;
    }
    if(is_power_on != is_power_on_)
    {
        is_power_on_ = is_power_on;
        is_update_ = true;
    }
}

void PowerBoardStatus::clear_statue()
{
    is_update_ = false;
}

