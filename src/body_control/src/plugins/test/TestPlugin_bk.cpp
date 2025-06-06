#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <bodyctrl_msgs/MotorStatusMsg.h>
#include <bodyctrl_msgs/CmdSetMotorSpeed.h>
#include <bodyctrl_msgs/CmdSetMotorPosition.h>
#include <bodyctrl_msgs/MotorName.h>
#include <bodyctrl_msgs/Imu.h>
#include <bodyctrl_msgs/NodeState.h>
#include <bodyctrl_msgs/TsHandName.h>
#include <bodyctrl_msgs/CmdSetTsHandPosition.h>
#include <bodyctrl_msgs/CmdSetTsHandCtrl.h>
#include <bodyctrl_msgs/TsHandStatusMsg.h>

#include <math.h> //fabs

#include <thread>
#include <mutex>
#include <cmath>

#include "PreciseRate.h"
#include "HighPrecisionRate.h"
#include "util/SinTime.h"

namespace body_control  // The usage of the namespace is a good practice but not mandatory
{

class TestPlugin : public nodelet::Nodelet
{
public:
  TestPlugin()
  {}

private:
  virtual void onInit()
  {
    auto& nh = getPrivateNodeHandle();

    pubSetMotorSpeed = nh.advertise<bodyctrl_msgs::CmdSetMotorSpeed>("/BodyControl/set_motor_speed", 1000);
    pubSetMotorPos = nh.advertise<bodyctrl_msgs::CmdSetMotorPosition>("/BodyControl/set_motor_position", 1000);
    pubForPlotSetMotorSpeed = nh.advertise<std_msgs::Float64>("/BodyControl/set_motor_speed_plot", 1000);
    pubForPlotMotorStatus = nh.advertise<std_msgs::Float64>("/motor_status_1", 1000); 
    pubSetTshandPos = nh.advertise<bodyctrl_msgs::CmdSetTsHandPosition>("/BodyControl/set_tshand_position", 1000);
    pubSetTshandCtrl = nh.advertise<bodyctrl_msgs::CmdSetTsHandCtrl>("/BodyControl/set_tshand_ctrl", 1000);

    subNodeState = nh.subscribe("/BodyControl/node_state", 1000, &TestPlugin::OnNodeState, this);
    subHandStatus = nh.subscribe("/BodyControl/tshand_status", 1000, &TestPlugin::OnHandStatus, this);
  }

  void OnNodeState(const bodyctrl_msgs::NodeState::ConstPtr& msg)
  {
    static bool started = false;
    if (!started && msg->state == bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      NODELET_INFO("Start Tester.");
      started = true;
      std::thread([this]() {
        case4();
      }).detach();
      std::thread([this]() {
        case3();
      }).detach();
    }
  }

  void OnHandStatus(const bodyctrl_msgs::TsHandStatusMsg::ConstPtr& msg)
  {
    lastStatus = msg;
  }

  void case5() {
    ros::Rate rate(1);
    SinTime st;
    double start_angle = 0;
    auto max_angle = 55;
    double spd_base = 30;
    auto threshold = 100;
    st.init(1, 0.5);
    while (1) {
      start_angle = lastStatus->status[0].bend_angle[0];
      bodyctrl_msgs::CmdSetTsHandCtrl msg;
      {
        bodyctrl_msgs::SetTsHandCtrl cmd;
        cmd.name = bodyctrl_msgs::TsHandName::TSINGHUA_HAND_LEFT;
        // auto max_angle = fabs(st.update(55));
        
        // auto spd = st.update(spd_base);
        auto spd = spd_base;

        cmd.rotation.vel = spd;
        cmd.rotation.start_angle = start_angle;
        cmd.rotation.max_angle = max_angle;
        cmd.bend.resize(5);
        cmd.threshold.resize(5);
        for (int i = 0; i < 5; ++i) {
          cmd.bend[i].vel = spd;
          cmd.bend[i].start_angle = start_angle;
          cmd.bend[i].max_angle = max_angle;
          cmd.threshold[i] = threshold;
        }
        msg.header.stamp = ros::Time::now();
        msg.cmds.push_back(cmd);
      }

      pubSetTshandCtrl.publish(msg);
      rate.sleep();
    }
  }

  void case4() {
    ros::Rate rate(100);
    SinTime st;
    st.init(55, 2);
    while (1) {
      auto pos = fabs(st.update());
      bodyctrl_msgs::CmdSetTsHandPosition msg;

      {
        bodyctrl_msgs::SetTsHandPosition cmd;
        cmd.name = bodyctrl_msgs::TsHandName::TSINGHUA_HAND_LEFT;
        cmd.rotation_angle = pos;
        cmd.bend_angle.resize(5);
        for (int i = 0; i < 5; ++i) {
          cmd.bend_angle[i] = pos;
        }
        msg.header.stamp = ros::Time::now();
        msg.cmds.push_back(cmd);
      }
      {
        bodyctrl_msgs::SetTsHandPosition cmd;
        cmd.name = bodyctrl_msgs::TsHandName::TSINGHUA_HAND_RIGHT;
        cmd.rotation_angle = pos;
        cmd.bend_angle.resize(5);
        for (int i = 0; i < 5; ++i) {
          cmd.bend_angle[i] = pos;
        }
        msg.header.stamp = ros::Time::now();
        msg.cmds.push_back(cmd);
      }
      pubSetTshandPos.publish(msg);
      rate.sleep();
    }
  }

  // 正弦曲线变速测试 - for the whole robot
  void case3() {
      ros::Rate rate(400);
      long count = 0;
      double spd_base = 3;
      double spd;
      double cur = 2.0; // A
      SinTime st;
      st.init(spd_base);
      while (1) {
        // 正弦变速
        spd = st.update(spd_base);

        bodyctrl_msgs::CmdSetMotorSpeed msg;

        // LEG-LEFT
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_5;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_6;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_1;
          cmd.spd = spd * 0.5;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_2;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_3;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_4;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        
        // LEG-RIGHT
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_5;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_1;
          cmd.spd = -spd * 0.5;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_2;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_3;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_4;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        
        pubSetMotorSpeed.publish(msg);
        rate.sleep();
      }
  }

  // 正弦曲线变速测试
  void case2() {
      ros::Rate rate(400);
      long count = 0;
      float spd = 5.0;
      float sd = 1.0;
      double Duration, Runtime = 0;
      auto start = std::chrono::system_clock::now();
      auto now = std::chrono::system_clock::now();
      while (1) {
        // 正弦变速
        now = std::chrono::system_clock::now();
        Duration = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - start).count() / 1000 / 1000 - Runtime;
        Runtime = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - start).count() / 1000 / 1000;
        spd = 40 * sin(Runtime);

        bodyctrl_msgs::CmdSetMotorSpeed msg;

        // LEG-LEFT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_1; i <= bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_3; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = 5;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        // LEG-RIGHT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_1; i <= bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = 5;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        // ARM-LEFT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_1; i <= bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_3; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = 5;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        // ARM-RIGHT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_1; i <= bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_3; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = 5;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        pubSetMotorSpeed.publish(msg);
        rate.sleep();
      }
  }

  // 基本测试，缓慢的阶梯式变速，正反转循环
  void case1() {
      ros::Rate rate(400);

      long count = 0;
      float spd = 5.0;
      float sd = 1.0;
      float cur = 10.0;
      while (1) {
        // 阶梯变速
        ++count;
        if (count > 500) {
          count = 0;
          spd += sd;
          if (spd > 40 || spd < -40) {
            sd = -sd;
          }
        }
        bodyctrl_msgs::CmdSetMotorSpeed msg;

        // LEG-LEFT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_1; i <= bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_6; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        // LEG-RIGHT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_1; i <= bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        // ARM-LEFT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_1; i <= bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_3; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        // ARM-RIGHT
        for (int i = bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_1; i <= bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_3; i++) {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = i;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }

        pubSetMotorSpeed.publish(msg);
        // std_msgs::Float64 amsg;
        // amsg.data = msg.cmds[0].spd;
        // pubForPlotSetMotorSpeed.publish(amsg);
        rate.sleep();
      }
  }

  ros::Subscriber subNodeState;
  ros::Subscriber subHandStatus;
  ros::Publisher pubSetMotorSpeed, pubSetMotorPos;
  ros::Publisher pubSetTshandPos;
  ros::Publisher pubSetTshandCtrl;
  ros::Publisher pubForPlotSetMotorSpeed;
  ros::Publisher pubForPlotMotorStatus;
  std::unordered_map<int, float> poss;
  bodyctrl_msgs::TsHandStatusMsg::ConstPtr lastStatus;
  long count = 0;
};
}

PLUGINLIB_EXPORT_CLASS(body_control::TestPlugin, nodelet::Nodelet);
