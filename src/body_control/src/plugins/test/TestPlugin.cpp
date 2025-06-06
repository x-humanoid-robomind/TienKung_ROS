#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <glog/logging.h>
#include <bodyctrl_msgs/CmdSetMotorSpeed.h>
#include <bodyctrl_msgs/MotorName.h>
#include <bodyctrl_msgs/NodeState.h>
#include <bodyctrl_msgs/MotorStatusMsg.h>
#include <bodyctrl_msgs/CmdSetMotorPosition.h>

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
    pubSetHeadPos = nh.advertise<bodyctrl_msgs::CmdSetMotorPosition>("/BodyControl/ey/set_pos", 1000);
    pubSetWaistPos = nh.advertise<bodyctrl_msgs::CmdSetMotorPosition>("/BodyControl/ze/set_pos", 1000);

    subNodeState = nh.subscribe("/BodyControl/node_state", 1000, &TestPlugin::OnNodeState, this);
    subLegStatus = nh.subscribe("/BodyControl/motor_state", 1000, &TestPlugin::OnLegStatus, this);
    subHeadStatus = nh.subscribe("/BodyControl/ey/status", 1000, &TestPlugin::OnHeadStatus, this);
    subWaistStatus = nh.subscribe("/BodyControl/ze/status", 1000, &TestPlugin::OnWaistStatus, this);
  }

  void OnNodeState(const bodyctrl_msgs::NodeState::ConstPtr& msg)
  {
    if (msg->state == bodyctrl_msgs::NodeState::NODE_STATE_RUNNING) {
      std::thread([this]() {
        bool ready = true;
        for (auto i = 0; i < 12; ++i) {
          ready = ready && inited_leg[i];
        }
        if (ready) {
          if (!module_started[0]) {
            module_started[0] = true;
            TestLegs();
          }
        }
      }).detach();

      std::thread([this]() {
        bool ready = true;
        for (auto i = 0; i < 3; ++i) {
          ready = ready && inited_head[i];
        }
        if (ready) {
          if (!module_started[1]) {
            module_started[1] = true;
            TestHead();
          }
        }
      }).detach();

      std::thread([this]() {
        if (inited_waist) {
          if (!module_started[2]) {
            module_started[2] = true;
            TestWaist();
          }
        }
        
      }).detach();
    }
  }

  void OnLegStatus(const bodyctrl_msgs::MotorStatusMsg::ConstPtr& msg)
  {
    for (auto& data : msg->status) {
      auto i = data.name - 1;
      if (!inited_leg[i]) {
        start_pos_leg[i] = data.pos;
        inited_leg[i] = true;
      }
    }
  }

  void OnHeadStatus(const bodyctrl_msgs::MotorStatusMsg::ConstPtr& msg)
  {
    for (auto& data : msg->status) {
      auto i = data.name - bodyctrl_msgs::MotorName::MOTOR_HEAD_TOP;
      if (!inited_head[i]) {
        start_pos_head[i] = data.pos;
        inited_head[i] = true;
      }
    }
  }

  void OnWaistStatus(const bodyctrl_msgs::MotorStatusMsg::ConstPtr& msg)
  {
    for (auto& data : msg->status) {
      if (!inited_waist) {
        start_pos_waist = data.pos;
        inited_waist = true;
        LOG(INFO) << "start_pos_waist:" << start_pos_waist;
      }
    }
  }

  void TestLegs() {
      ros::Rate rate(400);
      long count = 0;
      double spd_base = 1.5;
      double spd;
      double cur = 2.0; // A
      SinTime st;
      st.init(spd_base);
      while (ros::ok()) {
        // 正弦变速
        spd = st.update(spd_base);

        bodyctrl_msgs::CmdSetMotorSpeed msg;

        // LEG-LEFT
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
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_5;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_6;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        
        // LEG-RIGHT
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
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_5;
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        {
          bodyctrl_msgs::SetMotorSpeed cmd;
          cmd.name = bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6;
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        
        pubSetMotorSpeed.publish(msg);
        rate.sleep();
      }
  }

  // for 13715 motor
  void TestLegs2() {
      ros::Rate rate(400);
      long count = 0;
      double spd_base = 1.5;
      double spd;
      double cur = 2.0; // A
      SinTime st;
      st.init(spd_base);
      while (ros::ok()) {
        // 正弦变速
        spd = st.update(spd_base);

        bodyctrl_msgs::CmdSetMotorSpeed msg;

        // LEG-LEFT
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
          cmd.spd = -spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        
        // LEG-RIGHT
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
          cmd.spd = spd;
          cmd.cur = cur;
          msg.header.stamp = ros::Time::now();
          msg.cmds.push_back(cmd);
        }
        
        pubSetMotorSpeed.publish(msg);
        rate.sleep();
      }
  }

  void TestHead() {
    ros::Rate rate(0.1);
    float pos = 0.15;
    float spd = 0.5;
    while (ros::ok()) {
      bodyctrl_msgs::CmdSetMotorPosition msg;

      bodyctrl_msgs::SetMotorPosition cmd1;
      cmd1.name = bodyctrl_msgs::MotorName::MOTOR_HEAD_TOP;
      cmd1.pos = start_pos_head[0] + pos;
      cmd1.spd = spd;
      cmd1.cur = 1.0;

      bodyctrl_msgs::SetMotorPosition cmd2;
      cmd2.name = bodyctrl_msgs::MotorName::MOTOR_HEAD_LEFT;
      cmd2.pos = start_pos_head[1] + pos;
      cmd2.spd = spd;
      cmd2.cur = 1.0;

      bodyctrl_msgs::SetMotorPosition cmd3;
      cmd3.name = bodyctrl_msgs::MotorName::MOTOR_HEAD_RIGHT;
      cmd3.pos = start_pos_head[2] + pos;
      cmd3.spd = spd;
      cmd3.cur = 1.0;

      msg.cmds.push_back(cmd1);
      msg.cmds.push_back(cmd2);
      msg.cmds.push_back(cmd3);

      pubSetHeadPos.publish(msg);
      rate.sleep();

      pos = -pos;
    }
  }

  void TestWaist() {
    ros::Rate rate(0.2);
    float pos = 0.3;
    float spd = 0.3;
    while (ros::ok()) {
      bodyctrl_msgs::CmdSetMotorPosition msg;

      bodyctrl_msgs::SetMotorPosition cmd1;
      cmd1.name = bodyctrl_msgs::MotorName::MOTOR_WAIST;
      cmd1.pos = start_pos_waist + pos;
      cmd1.spd = spd;
      cmd1.cur = 1.0;

      msg.cmds.push_back(cmd1);

      pubSetWaistPos.publish(msg);
      rate.sleep();

      pos = -pos;
    }
  }

  bool inited_waist = false;
  bool inited_head[3] = {0};
  bool inited_leg[12] = {0};
  float start_pos_waist;
  float start_pos_head[3];
  float start_pos_leg[12];

  bool module_started[3] = {0};

  ros::Subscriber subLegStatus, subNodeState, subHeadStatus, subWaistStatus;
  ros::Publisher pubSetMotorSpeed, pubSetHeadPos, pubSetWaistPos;
};
}

PLUGINLIB_EXPORT_CLASS(body_control::TestPlugin, nodelet::Nodelet);
