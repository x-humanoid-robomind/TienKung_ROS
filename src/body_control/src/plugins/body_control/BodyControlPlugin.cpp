#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <any>
#include <unordered_map>
#include <bodyctrl_msgs/MotorInit.h>
#include <bodyctrl_msgs/MotorStart.h>
#include <bodyctrl_msgs/MotorStop.h>
#include <bodyctrl_msgs/MotorName.h>
#include <bodyctrl_msgs/MotorStatusMsg.h>
#include <bodyctrl_msgs/CmdMotorCtrl.h>
#include <bodyctrl_msgs/CmdSetMotorPosition.h>
#include <bodyctrl_msgs/CmdSetMotorSpeed.h>
#include <bodyctrl_msgs/CmdSetMotorDistance.h>
#include <bodyctrl_msgs/CmdSetMotorCurTor.h>
#include <bodyctrl_msgs/CmdSetTsHandPosition.h>
#include <bodyctrl_msgs/TsHandStatusMsg.h>
#include <bodyctrl_msgs/TsHandName.h>
#include <bodyctrl_msgs/CmdSetTsHandCtrl.h>
#include <bodyctrl_msgs/Imu.h>
#include <bodyctrl_msgs/Sri.h>
#include <bodyctrl_msgs/PowerStatus.h>
#include <bodyctrl_msgs/PowerBoardKeyStatus.h>
#include <bodyctrl_msgs/NodeState.h>
#include <bodyctrl_msgs/MotorStatus.h>
#include <fast_ros/fast_ros.h>

#include <sstream>
#include <math.h> //fabs

#include "util/LockFreeQueue.h"
#include "glog/GlogInitializer.h"

#include "soem_master/SoemMaster.h"
#include "devices/motor/MotorDeviceManager.h"
#include "devices/rm_imu/RmImuDevice.h"
#include "devices/sri_sensor/SriDeviceManager.h"
#include "devices/power_board/PowerBoardDevice.h"
#include "devices/xsens_imu/base/XsensImuDevice.h"
#include "devices/xsens_imu/high_rate/XsensImuHRDevice.h"
#include "devices/hand/tsinghua/TsinghuaHandDeviceManager.h"
#include "devices/zeroerr_motor/ZeroErrMotorDevMgr.h"
#include "devices/eyou_motor/EyouMotorDevMgr.h"



static bool cvt_msg(const PowerMgr& mgr, bodyctrl_msgs::PowerStatus::Ptr& status)
{
  // temperature information
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_WAIST_TEMP>();
    status->waist_temp = obj->value();
    status->waist_temp_max = obj->max();
    status->waist_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_A_TEMP>();
    status->arm_a_temp = obj->value();
    status->arm_a_temp_max = obj->max();
    status->arm_a_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_B_TEMP>();
    status->arm_b_temp = obj->value();
    status->arm_b_temp_max = obj->max();
    status->arm_b_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_A_TEMP>();
    status->leg_a_temp = obj->value();
    status->leg_a_temp_max = obj->max();
    status->leg_a_temp_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_B_TEMP>();
    status->leg_b_temp = obj->value();
    status->leg_b_temp_max = obj->max();
    status->leg_b_temp_min = obj->min();
  }
  // current information
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_A_CURR>();
    status->arm_a_curr = obj->value();
    status->arm_a_curr_max = obj->max();
    status->arm_a_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_ARM_B_CURR>();
    status->arm_b_curr = obj->value();
    status->arm_b_curr_max = obj->max();
    status->arm_b_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_A_CURR>();
    status->leg_a_curr = obj->value();
    status->leg_a_curr_max = obj->max();
    status->leg_a_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_LEG_B_CURR>();
    status->leg_b_curr = obj->value();
    status->leg_b_curr_max = obj->max();
    status->leg_b_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_WAIST_CURR>();
    status->waist_curr = obj->value();
    status->waist_curr_max = obj->max();
    status->waist_curr_min = obj->min();
  }
  {
    auto obj = mgr.GetPowerStatus<PbIns::GET_HEAD_CURR>();
    status->head_curr = obj->value();
    status->head_curr_max = obj->max();
    status->head_curr_min = obj->min();
  }
  // version information
  {
    auto obj = mgr.GetPowerVersionInfo();
    {
      std::stringstream ss;
      ss << std::hex << obj->hardware_version();
      status->hardware_version = ss.str();
    }
    {
      std::stringstream ss;
      ss << std::hex << obj->software_version();
      status->software_version = ss.str();
    }
  }

  // battery infomation
  {
    auto obj = mgr.GetBatteryInfomation();
    status->battery_voltage = obj->voltage();
    status->battery_current = obj->current();
    status->battery_power = obj->power();
  }

  return true;
}
static bool cvt_msg(const PowerMgr& mgr, bodyctrl_msgs::PowerBoardKeyStatus::Ptr& status)
{
  auto obj                     = mgr.GetPowerBoardStatus();
  status->work_time            = obj->work_time();
  status->is_estop.data        = obj->is_estop();
  status->is_remote_estop.data = obj->is_remote_estop();
  status->is_power_on.data     = obj->is_power_on();
  return true;
}


namespace body_control
{

class BodyControl : public nodelet::Nodelet
{
public:
  BodyControl() {
    INIT_GLOG("./glogs");
  }

  ~BodyControl() {
    SoemMaster::Instance().Stop();
  }

  void RunPubMsg() {
    LOG(INFO) << "start publisher thread: " << gettid();
 
    std::unique_lock<std::mutex> lck(mtxPubMsgCv);
    while (ros::ok()) 
    {
      while (!msgQueue.empty()) {
        auto msg = msgQueue.pop();
        if (msg.type == CacheMessage::MessageType::MOTOR) {
          pubMotorsState.publish(std::any_cast<bodyctrl_msgs::MotorStatusMsg::Ptr>(msg.msg));
          
        }
        else if (msg.type == CacheMessage::MessageType::IMU) {
          pubImu.publish(std::any_cast<bodyctrl_msgs::Imu::Ptr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::IMU_HR) {
          pubXImuHr.publish(std::any_cast<bodyctrl_msgs::Imu::Ptr>(msg.msg));
        }
        else if(msg.type == CacheMessage::MessageType::POWER_KEY)
        {
          pubPowerBoardKeyStatus.publish(std::any_cast<bodyctrl_msgs::PowerBoardKeyStatus::Ptr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::POWER) {
          pubPowerStatus.publish(std::any_cast<bodyctrl_msgs::PowerStatus::Ptr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::SRI) {
          pubSri.publish(std::any_cast<bodyctrl_msgs::Sri::Ptr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::HAND_TSINGHUA) {
          pubHandsStatus.publish(std::any_cast<bodyctrl_msgs::TsHandStatusMsg::Ptr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::ZE_MOTOR) {
          pubZeMotorStatus.publish(std::any_cast<bodyctrl_msgs::MotorStatusMsg::Ptr>(msg.msg));
        }
        else if (msg.type == CacheMessage::MessageType::EYOU_MOTOR) {
          pubEyouMotorStatus.publish(std::any_cast<bodyctrl_msgs::MotorStatusMsg::Ptr>(msg.msg));
        }
      }
      cvPubMsg.wait_for(lck, std::chrono::microseconds(500));
    }
  }

  void LoadEthercatParam() {
    auto& nh = getPrivateNodeHandle();
    XmlRpc::XmlRpcValue slaveMode;  
    if (nh.getParam ("slave_mode", slaveMode)) {  
        std::string logTxt = "slave_mode: ";
        for (int i = 0; i < slaveMode.size(); ++i) {
          auto id = static_cast<int>(slaveMode[i]);
          vecSlaveMode.push_back((SoemMaster::Mode)id);
          logTxt = logTxt + std::to_string(id) + " "; 
        }
        LOG(INFO) << logTxt;
    } else {  
        LOG(WARNING) << ("no ethercat param setting.");  
    }
  }

  void LoadMotors() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    float top_temp_limit = 90.0;
    if(not nh.getParam("top_temp_limit", top_temp_limit))
    {
      top_temp_limit = 90.0;
    }
    LOG(INFO) << "top_temp_limit: " << top_temp_limit;
    mdm.reset(new MotorDeviceManager);
    mdm->SetOnStatusReady([this, top_temp_limit](bodyctrl_msgs::MotorStatusMsg::Ptr msg){
        msg->header.stamp = ros::Time::now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::MOTOR;
        cmsg.msg = msg;
        msgQueue.push(cmsg);

        if(power)
        {
              if(this->mdm->is_temperature_high(top_temp_limit))
              {
                this->power->enable_beep();
              }
              else
              {
                this->power->disable_beep();
              }
        }

        cvPubMsg.notify_one();
    });
    // get mapping for moters
    XmlRpc::XmlRpcValue mappings_list;  
    if (nh.getParam ("motors_mapping", mappings_list)) {
        // motor interfaces
        pubMotorsState = fnh.advertise<bodyctrl_msgs::MotorStatusMsg>("motor_state", 1000);
        subCmdMotorCtrl = nh.subscribe("motor_ctrl", 10, &BodyControl::OnCmdMotorCtrlMsg, this);
        subCmdSetMotorPosition = nh.subscribe("set_motor_position", 10, &BodyControl::OnCmdSetMotorPosition, this);
        subCmdSetMotorDistance = nh.subscribe("set_motor_distance", 10, &BodyControl::OnCmdSetMotorDistance, this);
        subCmdSetMotorSpeed = nh.subscribe("set_motor_speed", 10, &BodyControl::OnCmdSetMotorSpeed, this);  
        LOG(INFO) << "Mappings loaded:";   
        for (int i = 0; i < mappings_list.size(); ++i) {    
            if (mappings_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {  
                XmlRpc::XmlRpcValue motor_mapping = mappings_list[i];  
                if (motor_mapping.size() == 5) {  
                    LOG(INFO) 
                      << " " << static_cast<int>(motor_mapping[0]) << ","
                      << " " << static_cast<int>(motor_mapping[1]) << "," 
                      << " " << static_cast<int>(motor_mapping[2]) << ","
                      << " " << static_cast<int>(motor_mapping[3]) << ","
                      << " " << static_cast<int>(motor_mapping[4]);
                    mdm->NewDevice(
                      static_cast<int>(motor_mapping[0]),
                      static_cast<int>(motor_mapping[1]),
                      static_cast<int>(motor_mapping[2]),
                      static_cast<int>(motor_mapping[3]),
                      static_cast<int>(motor_mapping[4])
                    );
                    bEnableMotors = true;
                } else if (motor_mapping.size() == 6) {
                    LOG(INFO) 
                      << " " << static_cast<int>(motor_mapping[0]) << ","
                      << " " << static_cast<int>(motor_mapping[1]) << "," 
                      << " " << static_cast<int>(motor_mapping[2]) << ","
                      << " " << static_cast<int>(motor_mapping[3]) << ","
                      << " " << static_cast<int>(motor_mapping[4]) << ","
                      << " " << static_cast<double>(motor_mapping[5]);
                    mdm->NewDevice(
                      static_cast<int>(motor_mapping[0]),
                      static_cast<int>(motor_mapping[1]),
                      static_cast<int>(motor_mapping[2]),
                      static_cast<int>(motor_mapping[3]),
                      static_cast<int>(motor_mapping[4]),
                      static_cast<double>(motor_mapping[5])
                    );
                    bEnableMotors = true;
                }
                else {  
                    LOG(ERROR) << ("Malformed motor mapping at index %d", i);  
                }  
            } else {  
                LOG(ERROR) << ("Malformed motor mapping at index %d", i);  
            }  
        }
    } else {  
        LOG(WARNING) << ("no motor setting.");  
    }
  }

  void LoadTsinghuaHand() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    tsHandMgr.reset(new TsinghuaHandDeviceManager);
    tsHandMgr->SetOnStatusReady([this](bodyctrl_msgs::TsHandStatusMsg::Ptr msg){
        msg->header.stamp = ros::Time::now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::HAND_TSINGHUA;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });
    // get mapping for moters
    XmlRpc::XmlRpcValue mappings_list;  
    if (nh.getParam ("hand_mapping", mappings_list)) {  
        // tshand interface
        pubHandsStatus = nh.advertise<bodyctrl_msgs::TsHandStatusMsg>("tshand_status", 1000);
        subCmdTsHandSetPos = nh.subscribe("set_tshand_position", 10, &BodyControl::OnCmdSetSetTsHandPosition, this);
        subCmdTsHandSetCtrl = nh.subscribe("set_tshand_ctrl", 10, &BodyControl::OnCmdSetSetTsHandCtrl, this);
        LOG(INFO) << "hand mappings loaded:";   
        for (int i = 0; i < mappings_list.size(); ++i) {    
            if (mappings_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {  
                XmlRpc::XmlRpcValue motor_mapping = mappings_list[i];  
                if (motor_mapping.size() == 4) {  
                    LOG(INFO) 
                      << " " << static_cast<int>(motor_mapping[0]) << ","
                      << " " << static_cast<int>(motor_mapping[1]) << "," 
                      << " " << static_cast<int>(motor_mapping[2]) << ","
                      << " " << static_cast<int>(motor_mapping[3]);
                    tsHandMgr->NewDevice(
                      static_cast<int>(motor_mapping[0]),
                      static_cast<int>(motor_mapping[1]),
                      static_cast<int>(motor_mapping[2]),
                      static_cast<int>(motor_mapping[3])
                    );
                    bEnableTsHands = true;
                } else {  
                    LOG(ERROR) << ("Malformed tshand mapping at index %d", i);  
                }  
            } else {  
                LOG(ERROR) << ("Malformed tshand mapping at index %d", i);  
            }  
        }
    } else {  
        LOG(WARNING) << ("no tshand setting.");  
    }
  }

  void LoadRmImu() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    int slave, id1, id2, id3;
    // get mapping for imu
    if (nh.getParam ("imu_mapping/slave", slave)) {
      // imu interface
      pubImu = fnh.advertise<bodyctrl_msgs::Imu>("imu", 1000);
      if (nh.getParam ("imu_mapping/id1", id1)) { 
        if (nh.getParam ("imu_mapping/id2", id2)) {
          if (nh.getParam ("imu_mapping/id3", id3)) {  
            LOG(INFO) << "Imu slave: " << slave << ", id1: " << id1 << ", id2: " << id2;
            rmImu.reset(new RmImuDevice(nh, slave, id1, id1, id2, id2,
              [this](const ubt_hw::ImuData* data) {
                bodyctrl_msgs::Imu::Ptr msg(new bodyctrl_msgs::Imu());
                msg->header.stamp = data->stamp;
                msg->orientation.x = data->orientation[0];
                msg->orientation.y = data->orientation[1];
                msg->orientation.z = data->orientation[2];
                msg->orientation.w = data->orientation[3];
                msg->angular_velocity.x = data->angular_vel[0];
                msg->angular_velocity.y = data->angular_vel[1];
                msg->angular_velocity.z = data->angular_vel[2];
                msg->linear_acceleration.x = data->linear_accel[0];
                msg->linear_acceleration.y = data->linear_accel[1];
                msg->linear_acceleration.z = data->linear_accel[2];
                msg->euler.roll = data->rpy[0];
                msg->euler.pitch = data->rpy[1];
                msg->euler.yaw = data->rpy[2];
                msg->orientation_covariance[0] = data->orientation_covariance[0];
                msg->orientation_covariance[1] = data->orientation_covariance[1];
                msg->orientation_covariance[2] = data->orientation_covariance[2];
                msg->angular_velocity_covariance[0] = data->angular_velocity_covariance[0];
                msg->angular_velocity_covariance[1] = data->angular_velocity_covariance[1];
                msg->angular_velocity_covariance[2] = data->angular_velocity_covariance[2];
                msg->linear_acceleration_covariance[0] = data->linear_acceleration_covariance[0];
                msg->linear_acceleration_covariance[1] = data->linear_acceleration_covariance[1];
                msg->linear_acceleration_covariance[2] = data->linear_acceleration_covariance[2];
                CacheMessage amsg;
                amsg.type = CacheMessage::MessageType::IMU;
                amsg.msg = msg;
                msgQueue.push(amsg);
                cvPubMsg.notify_one();
              }
            ));
            SoemMaster::Instance().RegisterDevice(slave, {id1, id2}, rmImu);
            bEnableRmImu = true;
          } else {  
              LOG(ERROR) << ("Failed to load imu parameter");  
              return;
          }   
        } else {  
            LOG(ERROR) << ("Failed to load imu parameter"); 
            return; 
        } 
      } else {
          LOG(ERROR) << ("Failed to load imu parameter");  
          return;
      }
    } else {  
        LOG(WARNING) << ("No imu setting.");
    }
  }

  void LoadXsensImu() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    int slave;
    int passages[4] = {0};
    uint16_t ids[4] = {0};

    if (!nh.getParam("xsens_imu_mapping/slave", slave)) {
      LOG(WARNING) << ("No Xsens Imu setting."); 
      return;
    }

    // imu interface
    pubImu = fnh.advertise<bodyctrl_msgs::Imu>("imu", 1000);

    std::string echoInfo = "Xsens Imu slave: ";
    for (int i = 0; i < 4; ++i) {
      std::string name = "xsens_imu_mapping/id" + std::to_string(i + 1);
      int value;
      if (!nh.getParam(name, value)) {
        LOG(ERROR) << "Failed to load Xsens Imu parameter: " << name;  
        return;
      }
      ids[i] = passages[i] = value;
      echoInfo = echoInfo + "id" + std::to_string(i + 1) + ": " + std::to_string(ids[i]) + " ";
    }

    LOG(INFO) << echoInfo;

    xImu.reset(new XsensImuDevice(slave, passages, ids,
      [this](ImuData& data) {
        bodyctrl_msgs::Imu::Ptr msg(new bodyctrl_msgs::Imu());
        msg->header.stamp = ros::Time::now();
        msg->orientation.x = data.orientation[0];
        msg->orientation.y = data.orientation[1];
        msg->orientation.z = data.orientation[2];
        msg->orientation.w = data.orientation[3];
        msg->angular_velocity.x = data.angular_vel[0];
        msg->angular_velocity.y = data.angular_vel[1];
        msg->angular_velocity.z = data.angular_vel[2];
        msg->linear_acceleration.x = data.linear_accel[0];
        msg->linear_acceleration.y = data.linear_accel[1];
        msg->linear_acceleration.z = data.linear_accel[2];
        msg->euler.roll = data.rpy[0];
        msg->euler.pitch = data.rpy[1];
        msg->euler.yaw = data.rpy[2];
        msg->orientation_covariance[0] = 0;
        msg->orientation_covariance[1] = 0;
        msg->orientation_covariance[2] = 0;
        msg->angular_velocity_covariance[0] = 0;
        msg->angular_velocity_covariance[1] = 0;
        msg->angular_velocity_covariance[2] = 0;
        msg->linear_acceleration_covariance[0] = 0;
        msg->linear_acceleration_covariance[1] = 0;
        msg->linear_acceleration_covariance[2] = 0;
        CacheMessage amsg;
        amsg.type = CacheMessage::MessageType::IMU;
        amsg.msg = msg;
        msgQueue.push(amsg);
        cvPubMsg.notify_one();
      }
    ));
    SoemMaster::Instance().RegisterDevice(slave, {passages[0], passages[1], passages[2], passages[3]}, xImu);
    bEnableXsensImu = true;
  } 

  void LoadXsensHrImu() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    int slave, id1, id2, id3;
    // get mapping for imu
    if (nh.getParam ("xhr_imu_mapping/slave", slave)) {  
      pubXImuHr = fnh.advertise<bodyctrl_msgs::Imu>("imu_hr", 1000);
      if (nh.getParam ("xhr_imu_mapping/id1", id1)) { 
        if (nh.getParam ("xhr_imu_mapping/id2", id2)) {
          if (nh.getParam ("xhr_imu_mapping/id3", id3)) {  
            LOG(INFO) << "XHR Imu slave: " << slave << ", id1: " << id1 << ", id2: " << id2;
            xhrImu.reset(new XsensImuHRDevice(nh, slave, id1, id1, id2, id2,
              [this](const ubt_hw::ImuData* data) {
                bodyctrl_msgs::Imu::Ptr msg(new bodyctrl_msgs::Imu());
                msg->header.stamp = data->stamp;
                msg->orientation.x = data->orientation[0];
                msg->orientation.y = data->orientation[1];
                msg->orientation.z = data->orientation[2];
                msg->orientation.w = data->orientation[3];
                msg->angular_velocity.x = data->angular_vel[0];
                msg->angular_velocity.y = data->angular_vel[1];
                msg->angular_velocity.z = data->angular_vel[2];
                msg->linear_acceleration.x = data->linear_accel[0];
                msg->linear_acceleration.y = data->linear_accel[1];
                msg->linear_acceleration.z = data->linear_accel[2];
                msg->euler.roll = data->rpy[0];
                msg->euler.pitch = data->rpy[1];
                msg->euler.yaw = data->rpy[2];
                msg->orientation_covariance[0] = data->orientation_covariance[0];
                msg->orientation_covariance[1] = data->orientation_covariance[1];
                msg->orientation_covariance[2] = data->orientation_covariance[2];
                msg->angular_velocity_covariance[0] = data->angular_velocity_covariance[0];
                msg->angular_velocity_covariance[1] = data->angular_velocity_covariance[1];
                msg->angular_velocity_covariance[2] = data->angular_velocity_covariance[2];
                msg->linear_acceleration_covariance[0] = data->linear_acceleration_covariance[0];
                msg->linear_acceleration_covariance[1] = data->linear_acceleration_covariance[1];
                msg->linear_acceleration_covariance[2] = data->linear_acceleration_covariance[2];
                CacheMessage amsg;
                amsg.type = CacheMessage::MessageType::IMU_HR;
                amsg.msg = msg;
                msgQueue.push(amsg);
                cvPubMsg.notify_one();
              }
            ));
            SoemMaster::Instance().RegisterDevice(slave, {id1, id2}, xhrImu);
            bEnableXsensHrImu = true;
          } else {  
              LOG(ERROR) << ("Failed to load xhr imu parameter");  
              return;
          }   
        } else {  
            LOG(ERROR) << ("Failed to load xhr imu parameter"); 
            return; 
        } 
      } else {
          LOG(ERROR) << ("Failed to load xhr imu parameter");  
          return;
      }
    } else {  
        LOG(WARNING) << ("No xhr imu setting.");
    }
  }

  void LoadSriSensor() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    sriMgr.reset(new SriDeviceManager);
    sriMgr->SetOnStatusReady([this](bodyctrl_msgs::Sri::Ptr msg){
        msg->header.stamp = ros::Time::now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::SRI;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });

    int name, slave, id_cmd, passage_cmd, id_data[3], passage_data[3];
    XmlRpc::XmlRpcValue mappings_list;  
    if (nh.getParam("sri_mapping", mappings_list)) {
        // sri interface
        pubSri = nh.advertise<bodyctrl_msgs::Sri>("sri", 1000);
        LOG(INFO) << "sri mappings loaded:";   
        for (int i = 0; i < mappings_list.size(); ++i) {    
            if (mappings_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {  
                XmlRpc::XmlRpcValue one = mappings_list[i];  
                if (one.size() == 10) {
                  std::string logtxt = "";
                  for (int j = 0; j < 10; ++j) {
                    logtxt = logtxt +  " " + std::to_string(static_cast<int>(one[j])) + ",";
                  }
                  LOG(INFO) << logtxt;
                  name = static_cast<int>(one[0]);
                  slave = static_cast<int>(one[1]);
                  id_cmd = static_cast<int>(one[2]);
                  passage_cmd = static_cast<int>(one[3]);
                  for (int j = 0; j < 3; ++j) {
                    id_data[j] = static_cast<int>(one[4 + j * 2]);
                    passage_data[j] = static_cast<int>(one[4 + j * 2 + 1]);
                  }
                  LOG(INFO) << "ids:" << id_data[0] << " " << id_data[1] << " " << id_data[2];
                  LOG(INFO) << "ps:" << passage_data[0] << " " << passage_data[1] << " " << passage_data[2];
                  sriMgr->NewDevice(name, slave, id_cmd, passage_cmd, id_data, passage_data);
                  bEnableSri = true;
                } else {  
                    LOG(ERROR) << ("Malformed sri mapping at index %d", i);  
                }  
            } else {  
                LOG(ERROR) << ("Malformed sri mapping at index %d", i);  
            }  
        }
    } else {  
        LOG(WARNING) << ("no sri setting.");  
    }
  }

  void LoadPowerBoard() {
    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    int slave, passage, id;
    // get mapping for power board
    if (nh.getParam ("power_mapping/slave", slave)) {  
      if (nh.getParam ("power_mapping/passage", passage)) {  
        // power manager interface
        pubPowerStatus = nh.advertise<bodyctrl_msgs::PowerStatus>("power_status", 1000);
        pubPowerBoardKeyStatus = nh.advertise<bodyctrl_msgs::PowerBoardKeyStatus>("power_board_key_status", 1000); // @brief 发布电源板按键状态
        if (nh.getParam ("power_mapping/id", id)) {  
              LOG(INFO) << "PowerBoard slave: " << slave << ", passage: " << passage << ", id: " << id;
              power.reset(new PowerBoardDevice(slave, passage, id,
                [this](const PowerMgr& status) {
                  bodyctrl_msgs::PowerStatus::Ptr msg(new bodyctrl_msgs::PowerStatus());
                  bodyctrl_msgs::PowerBoardKeyStatus::Ptr key_msg(new bodyctrl_msgs::PowerBoardKeyStatus());
                  {
                    cvt_msg(status, msg);
                    msg->header.stamp = ros::Time::now();
                    // msg->data = status.data;
                    CacheMessage amsg;
                    amsg.type = CacheMessage::MessageType::POWER;
                    amsg.msg = msg;
                    msgQueue.push(amsg);
                    cvPubMsg.notify_one();
                  }
                  {
                    cvt_msg(status, key_msg);
                    key_msg->header.stamp = ros::Time::now();
                    CacheMessage amsg;
                    amsg.type = CacheMessage::MessageType::POWER_KEY;
                    amsg.msg = key_msg;
                    msgQueue.push(amsg);
                    cvPubMsg.notify_one();
                  }
                }
              ));
              SoemMaster::Instance().RegisterDevice(slave, passage, power);
              bEnablePowerBoard = true;
        } else {
            LOG(ERROR) << ("Failed to load PowerBoard parameter");  
            return;
        }
      }
      else {
          LOG(ERROR) << ("Failed to load PowerBoard parameter");  
          return;
      }
    } else {  
        LOG(WARNING) << ("No PowerBoard setting.");
    }
  }

  void LoadZeroErrorMotors() {

    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    XmlRpc::XmlRpcValue mappings_list;  
    if (!nh.getParam ("ze_motors_mapping", mappings_list)) {
      LOG(WARNING) << ("No ze motor setting.");
      return;
    }

    zMgr.reset(new ZeroErrMotorDevMgr());
    zMgr->SetOnStatusReady([this](bodyctrl_msgs::MotorStatusMsg::Ptr msg){
        msg->header.stamp = ros::Time::now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::ZE_MOTOR;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });

    pubZeMotorStatus = nh.advertise<bodyctrl_msgs::MotorStatusMsg>("ze/status", 1000);
    subCmdZeMotorSetFphc = nh.subscribe("ze/set_fphc", 10, &BodyControl::OnCmdSetZeMotorFphc, this);
    subCmdZeMotorSetPos = nh.subscribe("ze/set_pos", 10, &BodyControl::OnCmdSetZeMotorPosition, this); 

    // # motor_name_id, slave_index, passage_req, passage_resp, motor_id, control_mode, motor_type, kt_default
    LOG(INFO) << "ZE Mappings loaded:";   
    for (int i = 0; i < mappings_list.size(); ++i) {    
        if (mappings_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {  
            XmlRpc::XmlRpcValue motor_mapping = mappings_list[i];  
            if (motor_mapping.size() == 9) {  
                LOG(INFO) 
                  << " " << static_cast<int>(motor_mapping[0]) << ","
                  << " " << static_cast<int>(motor_mapping[1]) << "," 
                  << " " << static_cast<int>(motor_mapping[2]) << ","
                  << " " << static_cast<int>(motor_mapping[3]) << ","
                  << " " << static_cast<int>(motor_mapping[4]) << ","
                  << " " << static_cast<int>(motor_mapping[5]) << ","
                  << " " << static_cast<int>(motor_mapping[6]) << ","
                  << " " << (float)static_cast<double>(motor_mapping[7]) << ","
                  << " " << (float)static_cast<double>(motor_mapping[8]);

                zMgr->NewDevice(
                  static_cast<int>(motor_mapping[0]),
                  static_cast<int>(motor_mapping[1]),
                  static_cast<int>(motor_mapping[2]),
                  static_cast<int>(motor_mapping[3]),
                  static_cast<int>(motor_mapping[4]),
                  static_cast<int>(motor_mapping[5]),
                  static_cast<int>(motor_mapping[6]),
                  (float)static_cast<double>(motor_mapping[7]),
                  (float)static_cast<double>(motor_mapping[8])
                );
                bEnableZeMotors = true;
            } else if (motor_mapping.size() == 8) {  
                LOG(INFO) 
                  << " " << static_cast<int>(motor_mapping[0]) << ","
                  << " " << static_cast<int>(motor_mapping[1]) << "," 
                  << " " << static_cast<int>(motor_mapping[2]) << ","
                  << " " << static_cast<int>(motor_mapping[3]) << ","
                  << " " << static_cast<int>(motor_mapping[4]) << ","
                  << " " << static_cast<int>(motor_mapping[5]) << ","
                  << " " << static_cast<int>(motor_mapping[6]) << ","
                  << " " << (float)static_cast<double>(motor_mapping[7]);

                zMgr->NewDevice(
                  static_cast<int>(motor_mapping[0]),
                  static_cast<int>(motor_mapping[1]),
                  static_cast<int>(motor_mapping[2]),
                  static_cast<int>(motor_mapping[3]),
                  static_cast<int>(motor_mapping[4]),
                  static_cast<int>(motor_mapping[5]),
                  static_cast<int>(motor_mapping[6]),
                  (float)static_cast<double>(motor_mapping[7])
                );
                bEnableZeMotors = true;
            }
            else {  
                LOG(ERROR) << ("Malformed ze-motor mapping at index %d", i);  
            }  
        } else {  
            LOG(ERROR) << ("Malformed ze-motor mapping at index %d", i);  
        }  
    }
  }

  void LoadEyouMotors() {

    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    XmlRpc::XmlRpcValue mappings_list;  
    if (!nh.getParam ("ey_motors_mapping", mappings_list)) {
      LOG(WARNING) << ("No ey motor setting.");
      return;
    }

    eyouMotorMgr.reset(new eyou::EyouMotorDevMgr());
    eyouMotorMgr->SetOnStatusReady([this](bodyctrl_msgs::MotorStatusMsg::Ptr msg){
        msg->header.stamp = ros::Time::now();
        CacheMessage cmsg;
        cmsg.type = CacheMessage::MessageType::EYOU_MOTOR;
        cmsg.msg = msg;
        msgQueue.push(cmsg);
        cvPubMsg.notify_one();
    });

    pubEyouMotorStatus = nh.advertise<bodyctrl_msgs::MotorStatusMsg>("ey/status", 1000);
    subCmdEyouMotorSetPos = nh.subscribe("ey/set_pos", 10, &BodyControl::OnCmdSetEyouMotorPosition, this); 

    // # motor_name_id, slave_index, passage_req, passage_resp, motor_id, control_mode, motor_type, kt_default
    LOG(INFO) << "EY Mappings loaded:";   
    for (int i = 0; i < mappings_list.size(); ++i) {    
        if (mappings_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {  
            XmlRpc::XmlRpcValue motor_mapping = mappings_list[i];  
            if (motor_mapping.size() == 5) {  
                LOG(INFO) 
                  << " " << static_cast<int>(motor_mapping[0]) << ","
                  << " " << static_cast<int>(motor_mapping[1]) << "," 
                  << " " << static_cast<int>(motor_mapping[2]) << ","
                  << " " << static_cast<int>(motor_mapping[3]) << ","
                  << " " << static_cast<int>(motor_mapping[4]) << ",";

                eyouMotorMgr->NewDevice(
                  static_cast<int>(motor_mapping[0]),
                  static_cast<int>(motor_mapping[1]),
                  static_cast<int>(motor_mapping[2]),
                  static_cast<int>(motor_mapping[3]),
                  static_cast<int>(motor_mapping[4]),
                  (int)eyou::EyouMotorDevice::Mode::POS,
                  (int)eyou::EyouMotorDevice::Type::S,
                  0.0
                );
                bEnableEyouMotors = true;
            } else {  
                LOG(ERROR) << ("Malformed ey-motor mapping at index %d", i);  
            }  
        } else {  
            LOG(ERROR) << ("Malformed ey-motor mapping at index %d", i);  
        }  
    }
  }

  virtual void onInit()
  {
    LOG(INFO) << "BodyControl onInit()";

    LoadEthercatParam();
    LoadRmImu();
    LoadXsensImu();
    LoadXsensHrImu();
    LoadMotors();
    LoadSriSensor();
    LoadPowerBoard();
    LoadTsinghuaHand();
    LoadZeroErrorMotors();
    LoadEyouMotors();

    auto& nh = getPrivateNodeHandle();
    auto fnh = fast_ros::NodeHandle(nh);

    // get default netcard name
    nh.getParam("net_card_name", nameOfNet);

    pubNodeState = nh.advertise<bodyctrl_msgs::NodeState>("node_state", 1);

    // publish node state
    std::thread([this](){
      while (ros::ok()) {
      ros::Rate r(1);
        bodyctrl_msgs::NodeState::Ptr msg(new bodyctrl_msgs::NodeState());
        msg->header.stamp = ros::Time::now();
        msg->state = nodeState;
        pubNodeState.publish(msg);
        r.sleep();
      }
    }).detach();

    bool bAutoInit = false;
    nh.getParam("auto_init", bAutoInit);
    if (bAutoInit) {
      autoInit();
    }
  }

  void CheckReady() {
      ros::Rate r(1);
      auto timeStart = ros::Time::now();
      while (ros::ok()) {
        auto timeNow = ros::Time::now();
        
        // check all devices
        bool ready = true;
        if (bEnableMotors) {
          ready &= mdm->IsReady();
        }
        if (bEnableTsHands) {
          ready &= tsHandMgr->IsReady();
        }
        if (bEnableRmImu) {
          ready &= rmImu->IsReady();
        }
        if (bEnableXsensImu) {
          ready &= xImu->IsReady();
        }
        if (bEnableXsensHrImu) {
          ready &= xhrImu->IsReady();
        }
        if (bEnablePowerBoard) {
          ready &= power->IsReady();
        }
        if (bEnableSri) {
          ready &= sriMgr->IsReady();
        }
        if (bEnableZeMotors) {
          ready &= zMgr->IsReady();
        }
        if (bEnableEyouMotors) {
          ready &= eyouMotorMgr->IsReady();
        }

        if (ready) {
          LOG(INFO) << "All devices ready.";
          nodeState = bodyctrl_msgs::NodeState::NODE_STATE_RUNNING;
          break;
        }

        if ((timeNow - timeStart).toSec() > 3) {

          std::unordered_map<int, std::string> motorNames = {
            {bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_1, "MOTOR_LEG_LEFT_1"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_2, "MOTOR_LEG_LEFT_2"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_3, "MOTOR_LEG_LEFT_3"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_4, "MOTOR_LEG_LEFT_4"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_5, "MOTOR_LEG_LEFT_5"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_6, "MOTOR_LEG_LEFT_6"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_1, "MOTOR_LEG_RIGHT_1"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_2, "MOTOR_LEG_RIGHT_2"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_3, "MOTOR_LEG_RIGHT_3"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_4, "MOTOR_LEG_RIGHT_4"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_5, "MOTOR_LEG_RIGHT_5"},
            {bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6, "MOTOR_LEG_RIGHT_6"},
            {bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_1, "MOTOR_ARM_LEFT_1"},
            {bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_2, "MOTOR_ARM_LEFT_2"},
            {bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_3, "MOTOR_ARM_LEFT_3"},
            {bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_1, "MOTOR_ARM_RIGHT_1"},
            {bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_2, "MOTOR_ARM_RIGHT_2"},
            {bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_3, "MOTOR_ARM_RIGHT_3"},
            {bodyctrl_msgs::MotorName::MOTOR_HEAD_TOP, "MOTOR_HEAD_1"},
            {bodyctrl_msgs::MotorName::MOTOR_HEAD_LEFT, "MOTOR_HEAD_2"},
            {bodyctrl_msgs::MotorName::MOTOR_HEAD_RIGHT, "MOTOR_HEAD_3"},
            {bodyctrl_msgs::MotorName::MOTOR_WAIST, "MOTOR_WAIST"},
          };

          std::unordered_map<int, std::string> handNames = {
            {bodyctrl_msgs::TsHandName::TSINGHUA_HAND_LEFT, "TSINGHUA_HAND_LEFT"},
            {bodyctrl_msgs::TsHandName::TSINGHUA_HAND_RIGHT, "TSINGHUA_HAND_RIGHT"},
          };

          if (bEnableMotors) {
            LOG(WARNING) << "motors state:"  << (mdm->IsReady() ? "ok" : "not");
            for (auto& name : mdm->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready motor: %s", motorNames[name].c_str());
            }
          }

          if (bEnableZeMotors) {
            LOG(WARNING) << "ze motors state:"  << (zMgr->IsReady() ? "ok" : "not");
            for (auto& name : zMgr->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready ze motor: %s", motorNames[name].c_str());
            }
          }

          if (bEnableEyouMotors) {
            LOG(WARNING) << "ey motors state:"  << (eyouMotorMgr->IsReady() ? "ok" : "not");
            for (auto& name : eyouMotorMgr->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready ey motor: %s", motorNames[name].c_str());
            }
          }

          if (bEnableTsHands) {
            LOG(WARNING) << "tshand state:"  << (tsHandMgr->IsReady() ? "ok" : "not");
            for (auto& name : tsHandMgr->GetNotReadyList()) {
              LOG(WARNING) << ("Not ready hand: %s", handNames[name].c_str());
            }
          }

          if (bEnableRmImu) {
            LOG(WARNING) << "rm_imu state:"  << (rmImu->IsReady() ? "ok" : "not");
          }
          if (bEnableXsensImu) {
            LOG(WARNING) << "xsens imu state:"  << (xImu->IsReady() ? "ok" : "not");
          }
          if (bEnableXsensHrImu) {
            LOG(WARNING) << "xsens hr imu state:"  << (xhrImu->IsReady() ? "ok" : "not");
          }
          if (bEnablePowerBoard) {
            LOG(WARNING) << "power board state:"  << (power->IsReady() ? "ok" : "not");
          }
          if (bEnableSri) {
            LOG(WARNING) << "sri state:"  << (sriMgr->IsReady() ? "ok" : "not");
          }


          break;
        }
        r.sleep();
      } 
  }

  void WaitAndSendReady() {
    LOG(INFO) << "Wait 3 sec for all devices ready...";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    LOG(INFO) << "Send ready state.";
    nodeState = bodyctrl_msgs::NodeState::NODE_STATE_RUNNING;
  }

  bool autoInit() {
    std::thread(&BodyControl::RunPubMsg, this).detach();
    std::thread([this](){
      LOG(INFO) << "Start to initialize SOEM master.";
      // TO DO: load ext-can slave from parameters
      auto rlt = SoemMaster::Instance().Init(nameOfNet, 1100, vecSlaveMode);
      if (!rlt) {
        LOG(ERROR) << ("SOEM init failed.");
      }
      auto& nh = getPrivateNodeHandle();
      // get default netcard name
      auto enableCheckReady = false;
      nh.getParam("enable_check_ready", enableCheckReady);
      if (enableCheckReady) {
        CheckReady();
      }
      else {
        WaitAndSendReady();
      }
    }).detach();
    return true;
  }

  void OnCmdMotorCtrlMsg(const bodyctrl_msgs::CmdMotorCtrl::ConstPtr& msg)
  {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SendMotorCtrlCmd(cmd.kp, cmd.kd, cmd.pos, cmd.spd, cmd.tor);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetMotorPosition(const bodyctrl_msgs::CmdSetMotorPosition::ConstPtr& msg)
  {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetMotorPosition(cmd.pos, cmd.spd, cmd.cur);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetMotorDistance(const bodyctrl_msgs::CmdSetMotorDistance::ConstPtr& msg)
  {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetMotorDistance(cmd.distance, cmd.spd, cmd.cur);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetMotorSpeed(const bodyctrl_msgs::CmdSetMotorSpeed::ConstPtr& msg)
  {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = mdm->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetMotorSpeed(cmd.spd, cmd.cur);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetZeMotorFphc(const bodyctrl_msgs::CmdMotorCtrl::ConstPtr& msg) {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = zMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetFphc(cmd.kp, cmd.kd, cmd.pos, cmd.spd, cmd.tor);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetZeMotorPosition(const bodyctrl_msgs::CmdSetMotorPosition::ConstPtr& msg) {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = zMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetPos(cmd.pos, cmd.spd);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetEyouMotorPosition(const bodyctrl_msgs::CmdSetMotorPosition::ConstPtr& msg) {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = eyouMotorMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          dev->SetPos(cmd.pos, cmd.spd);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetSetTsHandPosition(const bodyctrl_msgs::CmdSetTsHandPosition::ConstPtr& msg)
  {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = tsHandMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          FingerSetPositionCmd fcmd;
          fcmd.thumb.rotation.enable = true;
          fcmd.thumb.rotation.angle = cmd.rotation_angle;
          fcmd.thumb.bend.enable = true;
          fcmd.thumb.bend.angle = cmd.bend_angle[0];
          fcmd.fore.bend.enable = true;
          fcmd.fore.bend.angle = cmd.bend_angle[1];
          fcmd.middle.bend.enable = true;
          fcmd.middle.bend.angle = cmd.bend_angle[2];
          fcmd.ring.bend.enable = true;
          fcmd.ring.bend.angle = cmd.bend_angle[3];
          fcmd.little.bend.enable = true;
          fcmd.little.bend.angle = cmd.bend_angle[4];
          dev->SetPos(fcmd);
        }
      } catch(std::exception& e) {}
    }
  }

  void OnCmdSetSetTsHandCtrl(const bodyctrl_msgs::CmdSetTsHandCtrl::ConstPtr& msg)
  {
    if (nodeState != bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      return;
    }
    for (auto& cmd : msg->cmds) {
      try {
        const auto& dev = tsHandMgr->GetDevice(cmd.name);
        if (dev->IsReady()) {
          FingerMotionCtrlCmd fcmd;
          memset((void*)&fcmd, 0x00, sizeof(FingerMotionCtrlCmd));
          fcmd.thumb.rotation.enable = true;
          fcmd.thumb.rotation.vel = cmd.rotation.vel;
          fcmd.thumb.rotation.startAngle = cmd.rotation.start_angle;
          fcmd.thumb.rotation.maxAngle = cmd.rotation.max_angle;
          fcmd.thumb.bend.enable = true;
          fcmd.thumb.bend.vel = cmd.bend[0].vel;
          fcmd.thumb.bend.startAngle = cmd.bend[0].start_angle;
          fcmd.thumb.bend.maxAngle = cmd.bend[0].max_angle;
          fcmd.thumb.threshold = cmd.threshold[0];
          fcmd.fore.bend.enable = true;
          fcmd.fore.bend.vel = cmd.bend[1].vel;
          fcmd.fore.bend.startAngle = cmd.bend[1].start_angle;
          fcmd.fore.bend.maxAngle = cmd.bend[1].max_angle;
          fcmd.fore.threshold = cmd.threshold[1];
          fcmd.middle.bend.enable = true;
          fcmd.middle.bend.vel = cmd.bend[2].vel;
          fcmd.middle.bend.startAngle = cmd.bend[2].start_angle;
          fcmd.middle.bend.maxAngle = cmd.bend[2].max_angle;
          fcmd.middle.threshold = cmd.threshold[2];
          fcmd.ring.bend.enable = true;
          fcmd.ring.bend.vel = cmd.bend[3].vel;
          fcmd.ring.bend.startAngle = cmd.bend[3].start_angle;
          fcmd.ring.bend.maxAngle = cmd.bend[3].max_angle;
          fcmd.ring.threshold = cmd.threshold[3];
          fcmd.little.bend.enable = true;
          fcmd.little.bend.vel = cmd.bend[4].vel;
          fcmd.little.bend.startAngle = cmd.bend[4].start_angle;
          fcmd.little.bend.maxAngle = cmd.bend[4].max_angle;
          fcmd.little.threshold = cmd.threshold[4];
          dev->SetMotionCtrl(fcmd);
        }
      } catch(std::exception& e) {}
    }
  }

  

  ros::ServiceServer ssInit;
  ros::ServiceServer ssResetPosition;
  ros::ServiceServer ssStart;
  ros::ServiceServer ssStop;
  ros::Subscriber subCmdMotorCtrl;
  ros::Subscriber subCmdSetMotorPosition;
  ros::Subscriber subCmdSetMotorDistance;
  ros::Subscriber subCmdSetMotorSpeed;
  ros::Subscriber subCmdSetMotorCurTor;
  ros::Subscriber subCmdTsHandSetPos;
  ros::Subscriber subCmdTsHandSetCtrl;
  ros::Subscriber subCmdZeMotorSetPos;
  ros::Subscriber subCmdEyouMotorSetPos;
  ros::Subscriber subCmdZeMotorSetFphc;
  ros::Publisher pubNodeState;
  fast_ros::Publisher pubMotorsState;
  fast_ros::Publisher pubImu;
  fast_ros::Publisher pubXImuHr;
  ros::Publisher pubSri;
  ros::Publisher pubPowerStatus;
  ros::Publisher pubPowerBoardKeyStatus;
  ros::Publisher pubHandsStatus;
  ros::Publisher pubZeMotorStatus;
  ros::Publisher pubEyouMotorStatus;

  std::string nameOfNet;

  std::mutex mtxPubMsgCv;
  std::condition_variable cvPubMsg;
  LockFreeQueue<bodyctrl_msgs::MotorStatusMsg> msgStatusQueue;
  struct CacheMessage {
    enum class MessageType : int {
      MOTOR,
      IMU,
      IMU_HR,
      POWER,
      POWER_KEY,
      SRI,
      HAND_TSINGHUA,
      ZE_MOTOR,
      EYOU_MOTOR
    } type;
    std::any msg;
  };
  LockFreeQueue<CacheMessage> msgQueue;

  std::unique_ptr<MotorDeviceManager> mdm;
  std::unique_ptr<TsinghuaHandDeviceManager> tsHandMgr;
  std::shared_ptr<RmImuDevice> rmImu;
  std::shared_ptr<XsensImuDevice> xImu;
  std::shared_ptr<XsensImuHRDevice> xhrImu;
  std::shared_ptr<SriDeviceManager> sriMgr;
  std::shared_ptr<PowerBoardDevice> power;
  std::shared_ptr<ZeroErrMotorDevMgr> zMgr;
  std::shared_ptr<eyou::EyouMotorDevMgr> eyouMotorMgr;
  
  uint16_t nodeState = bodyctrl_msgs::NodeState::NODE_STATE_IDLE;

  std::vector<SoemMaster::Mode> vecSlaveMode;

  bool bEnableMotors = false;
  bool bEnableTsHands = false;
  bool bEnableRmImu = false;
  bool bEnableXsensImu = false;
  bool bEnableXsensHrImu = false;
  bool bEnableSri = false;
  bool bEnableZeMotors = false;
  bool bEnableEyouMotors = false;
  bool bEnablePowerBoard = false;
};
}

PLUGINLIB_EXPORT_CLASS(body_control::BodyControl, nodelet::Nodelet);

