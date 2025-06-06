#pragma once

#include <bodyctrl_msgs/PowerStatus.h>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <bitset>
#include <memory>
#include <any>

// power board instruction
enum class PbIns {
    GET_WAIST_TEMP         = 0x01,
    GET_ARM_A_TEMP         = 0x02,
    GET_ARM_B_TEMP         = 0x03,
    GET_LEG_A_TEMP         = 0x04,
    GET_LEG_B_TEMP         = 0x05,
    GET_ARM_A_CURR         = 0x06,
    GET_ARM_B_CURR         = 0x07,
    GET_LEG_A_CURR         = 0x08,
    GET_LEG_B_CURR         = 0x09,
    GET_WAIST_CURR         = 0x0A,
    GET_HEAD_CURR          = 0x0B,
    GET_BATTERY_INFO       = 0x0C,
    GET_VERSION_INFO       = 0x0D,
    GET_POWER_BOARD_STATUS = 0x0E
};
class CanData
{
public:
    CanData() = default;
    ~CanData() = default;

    virtual void update(uint8_t* data, const std::size_t len) = 0;
};
class PowerStatus : public CanData
{
public:
    PowerStatus() = default;
    ~PowerStatus() = default;

    virtual void update(uint8_t* data, const std::size_t len) override final;

    inline float max() const { return max_; }
    inline float min() const { return min_; }
    inline float value() const { return value_; }

private:
    float max_ = 0.0f;
    float min_ = 0.0f;
    float value_ = 0.0f;
};
class BatteryInfomation : public CanData
{
public:
    BatteryInfomation() = default;
    ~BatteryInfomation() = default;

    virtual void update(uint8_t* data, const std::size_t len) override final;

    inline float voltage() const { return voltage_; }
    inline float power() const { return power_; }
    inline float current() const { return current_; }
private:
    float voltage_ = 0.0f;
    float power_ = 0.0f;
    float current_ = 0.0f;
};
class PowerVersionInfo : public CanData
{
public:
    PowerVersionInfo() = default;
    ~PowerVersionInfo() = default;

    virtual void update(uint8_t* data, const std::size_t len) override final;

    inline uint32_t hardware_version() const { return hardware_version_; }
    inline uint32_t software_version() const { return software_version_; }

private:
    uint32_t hardware_version_ = 0;
    uint32_t software_version_ = 0;
};
class PowerBoardStatus : public CanData
{
public:
    PowerBoardStatus() = default;
    ~PowerBoardStatus() = default;

    virtual void update(uint8_t* data, const std::size_t len) override final;

    inline bool is_update() const { return is_update_;};
    void clear_statue();

    inline uint32_t work_time() const { return work_time_; }
    inline bool is_estop() const { return is_estop_; }
    inline bool is_remote_estop() const { return is_remote_estop_; }
    inline bool is_power_on() const { return is_power_on_; }

private:
    uint32_t work_time_ = 0;
    bool is_estop_ = false;
    bool is_remote_estop_ = false;
    bool is_power_on_ = false;

    bool is_update_ = false;
};

class PowerMgr {
public:
    PowerMgr();
    void ParseCanPackage(uint8_t* data);
    bool IsReady();

    void ClearReady();
    
    char NextIns();
    std::vector<std::int8_t> GetDataIndex() {
        return statusCmds;
    }

    int slave;
    int id;
    bool enable = false;

    // ins - value
    std::unordered_map<std::int8_t, std::any> values;
    
    template<PbIns ins>
    const std::shared_ptr<const PowerStatus> GetPowerStatus() const;

    const std::shared_ptr<const BatteryInfomation> GetBatteryInfomation() const;
    const std::shared_ptr<const PowerVersionInfo> GetPowerVersionInfo() const;
    const std::shared_ptr<const PowerBoardStatus> GetPowerBoardStatus() const;

    inline const uint8_t get_req_id() { std::lock_guard<std::mutex> lock(req_id_mutex_); return req_id_; }
    inline void update_req_id(const uint8_t id)
    {
        std::lock_guard<std::mutex> lock(req_id_mutex_); req_id_ = id;
    }

private:
    struct PowerCmd {
        std::uint32_t bitReady;
        std::function<void(uint8_t*)> funcParseValue;
    };

    std::vector<std::int8_t> statusCmds;
    int indexStatusCmd = 0;

    std::mutex req_id_mutex_;
    uint8_t req_id_ = 0;

    std::bitset<14> readyBits = 0;
    std::unordered_map<PbIns, std::shared_ptr<CanData>> cmds;
    std::shared_ptr<PowerBoardStatus> power_board_status_ = nullptr;
};

template <PbIns ins>
inline const std::shared_ptr<const PowerStatus> PowerMgr::GetPowerStatus() const
{
    auto itor = cmds.find(ins);
    if (itor != cmds.end())
    {
        return std::static_pointer_cast<const PowerStatus>(itor->second);
    }

    return nullptr;
}
