#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <bodyctrl_msgs/MotorStatusMsg.h>
#include <bodyctrl_msgs/Imu.h>
#include <bodyctrl_msgs/CmdSetMotorSpeed.h>
#include <bodyctrl_msgs/CmdMotorCtrl.h>
#include <bodyctrl_msgs/MotorName.h>
#include <thread>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <time.h>
#include <fstream>
// #include <fast_ros/fast_ros.h>
#include "broccoli/core/Time.hpp"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "util/LockFreeQueue.h"
#include "../x_humanoid_rl_sdk/include/robot_interface/RobotInterface.h"
#include "../x_humanoid_rl_sdk/include/robot_FSM/RobotFSM.h"
#include "Joystick.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define DATALOG_MAIN

using namespace broccoli::core;

namespace rl_control_new  // The usage of the namespace is a good practice but not mandatory
{

class RLControlNewPlugin : public nodelet::Nodelet {
 public:
  RLControlNewPlugin() {}

 private:
  bool LoadConfig(const std::string& config_file) {
    std::ifstream configFile(config_file.c_str());
    if (!configFile.is_open()) {
      std::cout << "Unable to open config file: " << config_file << std::endl;
      return false;
    }
    std::string line;
    while (std::getline(configFile, line)) {
      size_t pos = line.find('=');
      if (pos != std::string::npos) {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        if (!key.empty()) {
          _config_map[key] = value;
        }
      }
    }
    configFile.close();
    return true;
  }

  double GetConfig(const std::string& key, double default_value) {
    auto it = _config_map.find(key);
    std::string val;
    if (it != _config_map.end()) {
      val = it->second;
    }
    return val.empty() ? default_value : ::atof(val.c_str());
  }

  virtual void onInit() {
    if (!LoadConfig(_config_file)) {
      std::cout << "load config file error: " << _config_file << std::endl;
    }
    auto &nh = getPrivateNodeHandle();
    pi = 3.14159265358979;
    rpm2rps = pi / 30.0;
    motor_num = 20;
    Q_a = Eigen::VectorXd::Zero(motor_num);
    Qdot_a = Eigen::VectorXd::Zero(motor_num);
    Tor_a = Eigen::VectorXd::Zero(motor_num);
    Q_d = Eigen::VectorXd::Zero(motor_num);
    Qdot_d = Eigen::VectorXd::Zero(motor_num);
    Tor_d = Eigen::VectorXd::Zero(motor_num);

    q_a = Eigen::VectorXd::Zero(motor_num);
    qdot_a = Eigen::VectorXd::Zero(motor_num);
    tor_a = Eigen::VectorXd::Zero(motor_num);
    q_d = Eigen::VectorXd::Zero(motor_num);
    qdot_d = Eigen::VectorXd::Zero(motor_num);
    tor_d = Eigen::VectorXd::Zero(motor_num);

    Q_a_last = Eigen::VectorXd::Zero(motor_num);
    Qdot_a_last = Eigen::VectorXd::Zero(motor_num);
    Tor_a_last = Eigen::VectorXd::Zero(motor_num);
    ct_scale = Eigen::VectorXd::Ones(motor_num);
    ct_scale << 2.5, 2.1, 2.5, 2.5, 1.4, 1.4, 
                2.5, 2.1, 2.5, 2.5, 1.4, 1.4, 
                1.4, 1.4, 1.4, 1.4, 
                1.4, 1.4, 1.4, 1.4;
    zero_pos = Eigen::VectorXd::Zero(motor_num);
    std::stringstream ss;
    for (int32_t i = 0; i < motor_num; ++i) {
      zero_pos[i] = GetConfig("zero_pos_" + std::to_string(i), 0.0);
      ss << zero_pos[i] << "  ";
    }
    std::cout << "zero pos: " << ss.str() << std::endl;
    std::cout << "xsense_data_roll: " << GetConfig("xsense_data_roll", 0.0) << std::endl;
    // zero_pos << -2.46139,  -2.54387, -0.649462,  -1.58331,  -2.15858,  0.610552, 
    //             -0.708972,  -2.18147,  0.752079,   2.65526,  2.31079, 3.07412,  
    //             3.2198,   -1.367,   1.5909,  0.8356,
    //             1.70386,  2.5492,   -0.9168, -2.3630; 

    init_pos = Eigen::VectorXd::Zero(motor_num);
    motor_dir = Eigen::VectorXd::Ones(motor_num);
    motor_dir << 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 
                1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 
                1.0, -1.0, -1.0, -1.0, 
                -1.0, -1.0, -1.0, 1.0;
    zero_cnt = Eigen::VectorXd::Zero(motor_num);
    zero_offset = Eigen::VectorXd::Ones(motor_num);
    zero_offset << 0.0, 0.0, -pi/3.0, 2.0*pi/3.0, -1.047, -1.047, 
                  0.0, 0.0, -pi/3.0, 2.0*pi/3.0, -1.047, -1.047, 
                  0.0, 0.2618, 0.0, 0.0, 
                  0.0, -0.2618, 0.0, 0.0;
    kp = Eigen::VectorXd::Ones(motor_num) * 50.0;
    kd = Eigen::VectorXd::Ones(motor_num) * 5.0;
    imu_data = Eigen::VectorXd::Zero(9);
    imu_raw_data = Eigen::VectorXd::Zero(9);
    xsense_data = Eigen::VectorXd::Zero(13);
    data = Eigen::VectorXd::Zero(350);

    motor_name.insert({0, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_1});
    motor_name.insert({1, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_2});
    motor_name.insert({2, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_3});
    motor_name.insert({3, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_4});
    motor_name.insert({4, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_5});
    motor_name.insert({5, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_6});
    motor_name.insert({6, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_1});
    motor_name.insert({7, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_2});
    motor_name.insert({8, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_3});
    motor_name.insert({9, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_4});
    motor_name.insert({10, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_5});
    motor_name.insert({11, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6});
    motor_name.insert({12, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_1});
    motor_name.insert({13, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_2});
    motor_name.insert({14, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_3});
    motor_name.insert({15, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_4});
    motor_name.insert({16, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_1});
    motor_name.insert({17, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_2});
    motor_name.insert({18, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_3});
    motor_name.insert({19, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_4});

    for (int i = 0; i < motor_num; i++) {
      motor_id.insert({motor_name[i], i});
    }

    // auto fnh = fast_ros::NodeHandle(nh);

    pubSetMotorCmd = nh.advertise<bodyctrl_msgs::CmdMotorCtrl>("/BodyControl/motor_ctrl", 1000);
    subState = nh.subscribe("/BodyControl/motor_state", 1000, &RLControlNewPlugin::OnMotorStatusMsg, this);
    // subImu = fnh.subscribe("/BodyControl/imu", 1000, &RLControlNewPlugin::OnImuStatusMsg, this, fast_ros::ConnectionType::NATIVE_ROS);
    // subImuXsens = fnh.subscribe("/XSensImu/imu", 1000, &RLControlNewPlugin::OnXsensImuStatusMsg, this, fast_ros::ConnectionType::NATIVE_ROS);
    subImuXsens = nh.subscribe("/BodyControl/imu", 1000, &RLControlNewPlugin::OnXsensImuStatusMsg, this);
    subJoyCmd = nh.subscribe<sensor_msgs::Joy>("/sbus_data", 1000, &RLControlNewPlugin::xbox_map_read, this);
    subCmdVel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &RLControlNewPlugin::OnCmdVelMsg, this);

    sleep(1);
    std::thread([this]() {
      rlControl();
    }).detach();
  }

  // 混合模式测试
  void rlControl() {
    // ros::Rate rate(1000);

    // set cpu-affinity
    // int cpus = 0;
    // cpu_set_t mask;

    // cpus = sysconf(_SC_NPROCESSORS_CONF);
    // printf("cpus: %d\n", cpus);

    // CPU_ZERO(&mask);          // init mask
    // CPU_SET(cpus - 1, &mask); // add last cup core to cpu set
    // // CPU_SET(1, &mask); // add first cup core to cpu set


    // if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
    //   printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
    // }
    // usleep(1000);
    // printf("set CPU affinity success\n");
    //set cpu-affinity

    //set sched-strategy
    struct sched_param sched;
    int max_priority;

    max_priority = sched_get_priority_max(SCHED_RR);
    sched.sched_priority = max_priority;

    if (sched_setscheduler(gettid(), SCHED_RR, &sched) == -1) {
      printf("Set Scheduler Param, ERROR:%s\n", strerror(errno));
    }
    usleep(1000);
    printf("set scheduler success\n");
    //set sched-strategy

    // joystick init
    Joystick_humanoid joystick_humanoid;
    joystick_humanoid.init();

    // robot FSM init
    RobotFSM *robot_fsm = get_robot_FSM(robot_data);

    // robot_interface init
    RobotInterface *robot_interface = get_robot_interface();
    robot_interface->Init();

    long count = 0;

//      kp << 1000., 1000., 1200., Eigen::VectorXd::Ones(3)*1000.0, 1000., 1000., 1200., Eigen::VectorXd::Ones(3)*1000.0, Eigen::VectorXd::Zero(6);
//      kd << 20., 20., 20., Eigen::VectorXd::Ones(3)*20.0, 20., 20., 20., Eigen::VectorXd::Ones(3)*20.0, Eigen::VectorXd::Zero(6);

    kp << 700.0, 500.0, 700.0, 700.0, 15.0, 15.0,
        700.0, 500.0, 700.0, 700.0, 15.0, 15.0,
        20.0, 10.0, 10.0, 10.0,
        20.0, 10.0, 10.0, 10.0;
    kd << 10.0, 5.0, 10.0, 10.0, 1.25, 1.25,
        10.0, 5.0, 10.0, 10.0, 1.25, 1.25,
        1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0;

    // kp(0) *= 1.1;
    // kd(0) *= 1.1;
    // kp(6) *= 1.1;
    // kd(6) *= 1.1;
    // kp *= 0.5;
    // kd *= 0.5;

    // kp << 150.0, 150.0, 150.0, 150.0, 75.0, 75.0,
    //     150.0, 150.0, 150.0, 150.0, 75.0, 75.0,
    //     20.0, 10.0, 10.0,
    //     20.0, 10.0, 10.0;
    // kd <<1.0, 1.0, 1.0, 1.0, 0.5, 0.5,
    //     1.0, 1.0, 1.0, 1.0,  0.5, 0.5,
    //     1.0, 1.0, 1.0,
    //     1.0, 1.0, 1.0;
    // kp= Eigen::VectorXd::Ones(18)*100.0;
      
    // kd= Eigen::VectorXd::Ones(18)*1.0;
    double t_now = 0;
    double dt = 0.0025;

    Time start_time;
    Time period(0, 2500000);
    Time sleep2Time;
    Time timer;
    timespec sleep2Time_spec;
    double timeFSM = 0.0;
    Time timer1, timer2, timer3, total_time;

#ifdef DATALOG_MAIN
    std::ostringstream oss;
    spdlog::init_thread_pool(8190, 1);
    time_t currentTime = time(nullptr);
    char chCurrentTime[256];
    strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d_%H%M%S", localtime(&currentTime));
    std::string stCurrentTime = chCurrentTime;
    std::string filename = stCurrentTime + "log_RL.txt";
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        filename, 1024 * 1024 * 100, 3);
    rotating_sink->set_pattern("%v");
    std::vector<spdlog::sink_ptr> sinks{rotating_sink};
    auto logger = std::make_shared<spdlog::async_logger>(
        "loggername", sinks.begin(), sinks.end(), spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
#endif


    while (!queueMotorState.empty()) {
      auto msg = queueMotorState.pop();
      // set motor buf
      int id = 0;
      for (auto& one : msg->status) {
        id = motor_id[one.name];
        Q_a(id) = one.pos;
        Qdot_a(id) = one.speed;
        Tor_a(id) = one.current*ct_scale(id);
      }
    }


    init_pos = Q_a;
    Q_a_last = Q_a;
    Qdot_a_last = Qdot_a;
    Tor_a_last = Tor_a;
    
    for (int i = 0; i < motor_num; i++) {
      q_a(i) = (Q_a(i) - zero_pos(i)) * motor_dir(i) + zero_offset(i);
      zero_cnt(i) = (q_a(i) > pi) ? -1.0 : zero_cnt(i);
      zero_cnt(i) = (q_a(i) < -pi) ? 1.0 : zero_cnt(i);
      q_a(i) += zero_cnt(i) * 2.0 * pi;
    }
    std::cout << "current Q_A pos: " << Q_a.transpose() << std::endl;
    std::cout << "current pos: " << q_a.transpose() << std::endl;
    std::cout << "enter 1: " << std::endl;
    double a;
    std::cin >> a;

    while (1) {
      total_time = timer.currentTime() - start_time;
      start_time = timer.currentTime();

      while (!queueMotorState.empty()) {
        auto msg = queueMotorState.pop();
        // set motor buf
        int id = 0;
        for (auto& one : msg->status) {
          id = motor_id[one.name];
          Q_a(id) = one.pos;
          Qdot_a(id) = one.speed;
          Tor_a(id) = one.current*ct_scale(id);
        }
      }

      while (!queueImuRm.empty()) {
        auto msg = queueImuRm.pop();
        // set rm imu buf
        imu_raw_data(0) = msg->euler.yaw;
        imu_raw_data(1) = msg->euler.pitch;
        imu_raw_data(2) = msg->euler.roll;
        imu_raw_data(3) = msg->angular_velocity.x;
        imu_raw_data(4) = -msg->angular_velocity.y;
        imu_raw_data(5) = -msg->angular_velocity.z;
        imu_raw_data(6) = msg->linear_acceleration.x;
        imu_raw_data(7) = -msg->linear_acceleration.y;
        imu_raw_data(8) = -msg->linear_acceleration.z;
      }

      while (!queueImuXsens.empty()) {
        auto msg = queueImuXsens.pop();
        // set xsens imu buf
        xsense_data(0) = msg->euler.yaw;
        xsense_data(1) = msg->euler.pitch;
        xsense_data(2) = msg->euler.roll;
        xsense_data(3) = msg->angular_velocity.x;
        xsense_data(4) = msg->angular_velocity.y;
        xsense_data(5) = msg->angular_velocity.z;
        xsense_data(6) = msg->linear_acceleration.x;
        xsense_data(7) = msg->linear_acceleration.y;
        xsense_data(8) = msg->linear_acceleration.z;
      }

      #ifdef USE_ROS_JOY
        while (!queueJoyCmd.empty()) {
          auto msg = queueJoyCmd.pop();
          // set joy cmd buf
          xbox_map.a = msg->axes[8];
          xbox_map.b = msg->axes[9];
          xbox_map.c = msg->axes[10];
          xbox_map.d = msg->axes[11];
          xbox_map.e = msg->axes[4];
          xbox_map.f = msg->axes[7];
          xbox_map.g = msg->axes[5];
          xbox_map.h = msg->axes[6];
          xbox_map.x1 = msg->axes[3];
          xbox_map.x2 = msg->axes[0];
          xbox_map.y1 = msg->axes[2];
          xbox_map.y2 = msg->axes[1];
        }
      #endif

      // calculate Command
      t_now = count * dt;

      for (int i = 0; i < motor_num; i++) {
        if (fabs(Q_a(i) - Q_a_last(i)) > pi) { 
          std::cout << i << "   joint error" << std::endl;
          std::cout << "Q_a_last: " << Q_a_last(i) << std::endl;
          std::cout << "Q_a: " << Q_a(i) << " Qdot_a: " << Qdot_a(i) << " Tor_a: " << Tor_a(i) << std::endl;
          Q_a(i) = Q_a_last(i);
          Qdot_a(i) = Qdot_a_last(i);
          Tor_a(i) = Tor_a_last(i);
        }
      }
      Q_a_last = Q_a;
      Qdot_a_last = Qdot_a;
      Tor_a_last = Tor_a;

     //std::cout << Q_a.transpose() << std::endl;


      //real feedback
      for (int i = 0; i < motor_num; i++) {
        q_a(i) = (Q_a(i) - zero_pos(i)) * motor_dir(i) + zero_offset(i);
        q_a(i) += zero_cnt(i) * 2.0 * pi;
        qdot_a(i) = Qdot_a(i) * motor_dir(i);
        tor_a(i) = Tor_a(i) * motor_dir(i);
      }

      imu_data = imu_raw_data;
      imu_data(2) = imu_data(2) > 0.0 ? imu_data(2) - pi : imu_data(2) + pi;

      // // virtual feedback
      // q_a = q_d;
      // qdot_a = qdot_d;
      // tor_a = tor_d;

      // std::cout << q_a.transpose() << std::endl;


      // std::cout << "ypr:" << xsense_data.head(3).transpose()/M_PI*180.0 << std::endl;
      // std::cout << "acc:" << xsense_data.segment(6,3).transpose() << std::endl;

      // xsense_data[1] -= 0.01;

      // get state
      // robot_data.q_a_.tail(motor_num) = q_a;
      // robot_data.q_dot_a_.tail(motor_num) = qdot_a;
      // robot_data.tau_a_.tail(motor_num) = tor_a;
      // robot_data.joint_kp_p_ = kp;
      // robot_data.joint_kd_p_ = kd;

      Eigen::VectorXd q_a_18 = Eigen::VectorXd::Zero(18);
      Eigen::VectorXd q_dot_a_18 = Eigen::VectorXd::Zero(18);
      Eigen::VectorXd tor_a_18 = Eigen::VectorXd::Zero(18);
      Eigen::VectorXd kp_18 = Eigen::VectorXd::Zero(18);
      Eigen::VectorXd kd_18 = Eigen::VectorXd::Zero(18);
      q_a_18 << q_a.head(14), q_a(15), q_a.segment(16, 2), q_a(19);
      q_dot_a_18 << qdot_a.head(14), qdot_a(15), qdot_a.segment(16, 2), qdot_a(19);
      tor_a_18 << tor_a.head(14), tor_a(15), tor_a.segment(16, 2), tor_a(19);
      kp_18 << kp.head(14), kp(15), kp.segment(16, 2), kp(19);
      kd_18 << kd.head(14), kd(15), kd.segment(16, 2), kd(19);
      robot_data.q_a_.tail(18) = q_a_18;
      robot_data.q_dot_a_.tail(18) = q_dot_a_18;
      robot_data.tau_a_.tail(18) = tor_a_18;
      robot_data.joint_kp_p_ = kp_18;
      robot_data.joint_kd_p_ = kd_18;

      // roll offset
      // xsense_data(2) -= 0.005;
      double offset = GetConfig("xsense_data_roll", 0.0);
      xsense_data(2) += offset;

      // robot_interface->imu_data_ = imu_data; // dji imu
      robot_interface->imu_data_ = xsense_data.head(9); // xsense imu
      

      // get state
      robot_interface->GetState(timeFSM, robot_data);

      #ifdef USE_ROS_JOY
        joystick_humanoid.xbox_flag_update(xbox_map);
      #endif
      xbox_flag flag_ = joystick_humanoid.get_xbox_flag();


      if (xbox_map.f == -1.0) {
        while (!queueCmdVel.empty()) {
          auto msg = queueCmdVel.pop();
          x_speed_command   = msg->linear.x;
          y_speed_command   = msg->linear.y;
          yaw_speed_command = msg->angular.z;
        }
        flag_.x_speed_command   = fmin(fmax(x_speed_command, -0.5), 1.0);
        flag_.y_speed_command   = fmin(fmax(y_speed_command, -0.1), 0.1);
        flag_.yaw_speed_command = fmin(fmax(yaw_speed_command, -0.2), 0.2);
      }

      timer1 = timer.currentTime() - start_time;

      //rl fsm
      robot_fsm->RunFSM(flag_);

      timer2 = timer.currentTime() - start_time - timer1;

      // set command
      robot_interface->SetCommand(robot_data);

      timeFSM += dt;

      q_d << robot_data.q_d_.segment(6, 14), 0.0, robot_data.q_d_(6 + 14), robot_data.q_d_.segment(6 + 15, 2), 0.0, robot_data.q_d_(6 + 17);
      qdot_d << robot_data.q_dot_d_.segment(6, 14), 0.0, robot_data.q_dot_d_(6 + 14), robot_data.q_dot_d_.segment(6 + 15, 2), 0.0, robot_data.q_dot_d_(6 + 17);
      tor_d << robot_data.tau_d_.segment(6, 14), 0.0, robot_data.tau_d_(6 + 14), robot_data.tau_d_.segment(6 + 15, 2), 0.0, robot_data.tau_d_(6 + 17);

      // q_d = robot_data.q_d_.tail(motor_num);
      // qdot_d = robot_data.q_dot_d_.tail(motor_num);
      // tor_d = robot_data.tau_d_.tail(motor_num);

      for (int i = 0; i < motor_num; i++) {
        Q_d(i) = (q_d(i) - zero_offset(i) - zero_cnt(i) * 2.0 * pi) * motor_dir(i) + zero_pos(i);
        Qdot_d(i) = qdot_d(i) * motor_dir(i);
        Tor_d(i) = tor_d(i) * motor_dir(i);
      }

      //std::cout << (q_d.segment(2,4)).transpose() << std::endl;
      // std::cout << (q_d.segment(2,4)-q_a_p.head(4)).transpose() << std::endl;
      // std::cout << (Q_d-Q_a).transpose() << std::endl;
      // //test plan
      // float T = 4.0;
      // float mag = 0.1;
      // float t_init = 1.0;

      // for(int i=0; i<motor_num; i++){
      //   Q_d(i) = init_pos(i) + motor_dir(i)*mag*0.5*(1.0 - std::cos(2.0*pi/T*t_now));
      //   Qdot_d(i) = motor_dir(i)*mag*pi/T*std::sin(2.0*pi/T*t_now);
      //   Tor_d(i) = 0.0;
      // }

      if (robot_fsm->disable_joints_) {
        kp.setZero();
        kd.setZero();
        Tor_d.setZero();
      }

      // Send Command motorctrl mode
      bodyctrl_msgs::CmdMotorCtrl msg;

      for (int i = 0; i < motor_num; i++) {
        // for (int i=0; i< 12; i++){
        bodyctrl_msgs::MotorCtrl cmd;
        cmd.name = motor_name[i];
        cmd.kp =  kp(i);
        cmd.kd =  kd(i);
        cmd.pos = Q_d(i);
        cmd.spd = Qdot_d(i);
        cmd.tor = Tor_d(i);
        msg.header.stamp = ros::Time::now();
        msg.cmds.push_back(cmd);
      }

      pubSetMotorCmd.publish(msg);
      // rate.sleep();

      timer3 = timer.currentTime() - start_time - timer1 - timer2;

#ifdef DATALOG_MAIN

      data.head(300) = robot_data.data_log_;
      data(300) = (timer.currentTime() - start_time).m_nanoSeconds * 1e-6;
      data(301) = timer1.m_nanoSeconds * 1e-6;
      data(302) = timer2.m_nanoSeconds * 1e-6;
      data(303) = timer3.m_nanoSeconds * 1e-6;
      data(304) = total_time.m_nanoSeconds * 1e-6;
      data.segment(305, 9) = imu_data;
      data.segment(314, 9) = xsense_data.head(9);
      data(323) = robot_fsm->getCurrentState();
      data(324) = t_now;

      for (const auto &i : data) {
        oss << i << " ";
      }
      logger->info(oss.str());
      oss.str("");
#endif

      sleep2Time = start_time + period;
      sleep2Time_spec = sleep2Time.toTimeSpec();
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(sleep2Time_spec), NULL);
      count++;
    }
#ifdef DATALOG_MAIN
    logger->flush();
#endif
  }

  void OnMotorStatusMsg(const bodyctrl_msgs::MotorStatusMsg::ConstPtr &msg) {
    auto wrapper = msg;
    queueMotorState.push(wrapper);
  }

  void OnImuStatusMsg(const bodyctrl_msgs::Imu::ConstPtr &msg) {
    auto wrapper = msg;
    queueImuRm.push(wrapper);
  }

  void OnXsensImuStatusMsg(const bodyctrl_msgs::Imu::ConstPtr& msg)
  {
    auto wrapper = msg;
    queueImuXsens.push(wrapper);
  }

  void xbox_map_read(const sensor_msgs::Joy::ConstPtr &msg)
  {
    auto wrapper = msg;
    queueJoyCmd.push(wrapper);
  }

  void OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr& msg){
    auto wrapper = msg;
    queueCmdVel.push(wrapper);
  }

  ros::Subscriber subState;
  // fast_ros::Subscriber subImu, subImuXsens;
  ros::Subscriber subImu, subImuXsens;
  ros::Publisher pubSetMotorCmd;
  ros::Subscriber subJoyCmd;
  ros::Subscriber subCmdVel;
  Eigen::VectorXd q_a;
  Eigen::VectorXd qdot_a;
  Eigen::VectorXd tor_a;
  Eigen::VectorXd q_d;
  Eigen::VectorXd qdot_d;
  Eigen::VectorXd tor_d;
  Eigen::VectorXd Q_a;
  Eigen::VectorXd Qdot_a;
  Eigen::VectorXd Tor_a;
  Eigen::VectorXd Q_a_last;
  Eigen::VectorXd Qdot_a_last;
  Eigen::VectorXd Tor_a_last;
  Eigen::VectorXd Q_d;
  Eigen::VectorXd Qdot_d;
  Eigen::VectorXd Tor_d;
  Eigen::VectorXd ct_scale;
  Eigen::VectorXd data;
  Eigen::VectorXd zero_pos;
  Eigen::VectorXd zero_offset;
  Eigen::VectorXd init_pos;
  Eigen::VectorXd motor_dir;
  Eigen::VectorXd zero_cnt;
  Eigen::VectorXd imu_raw_data;
  Eigen::VectorXd imu_data;
  Eigen::VectorXd xsense_data;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  double x_speed_command = 0.;  
  double y_speed_command = 0.;  
  double yaw_speed_command = 0.;
  std::string _config_file = "/home/ubuntu/data/param/rl_control_new.txt";
  std::unordered_map<std::string, std::string> _config_map;

  #ifdef USE_ROS_JOY
    xbox_map_t xbox_map;
  #endif

  int motor_num;
  std::map<int, int> motor_id, motor_name;
  float rpm2rps;
  float pi;

  LockFreeQueue<bodyctrl_msgs::MotorStatusMsg::ConstPtr> queueMotorState;
  LockFreeQueue<bodyctrl_msgs::Imu::ConstPtr> queueImuRm;
  LockFreeQueue<bodyctrl_msgs::Imu::ConstPtr> queueImuXsens;
  LockFreeQueue<sensor_msgs::Joy::ConstPtr> queueJoyCmd;
  LockFreeQueue<geometry_msgs::Twist::ConstPtr> queueCmdVel;

  // robot_data
  RobotData robot_data;
};
}

PLUGINLIB_EXPORT_CLASS(rl_control_new::RLControlNewPlugin, nodelet::Nodelet
);
