#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <bodyctrl_msgs/Imu.h>
#include <bodyctrl_msgs/XSensImuInit.h>

#include <math.h> //fabs

#include <thread>
#include <mutex>
#include <cmath>

#include <atomic>
#include <fast_ros/fast_ros.h>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "xsens/include/xsense/xsense.h"
#include "xsens/include/broccoli/core/Time.hpp"

using namespace broccoli::core;
using std::cout;

namespace xsens_imu  // The usage of the namespace is a good practice but not mandatory
{

class XSensImuPlugin : public nodelet::Nodelet
{
public:
  XSensImuPlugin()
  {}

private:
  virtual void onInit()
  {
    auto& nh = getPrivateNodeHandle();
    bool realtime = false;

    nh.getParam ("realtime", realtime);

    LOG(INFO) << "realtime: " << (realtime ? "true" : "false");

    auto fnh = fast_ros::NodeHandle(getPrivateNodeHandle());
    pubImu = fnh.advertise<bodyctrl_msgs::Imu>("imu", 1000);
    // ssInit = nh.advertiseService<bodyctrl_msgs::XSensImuInit>("init", &XSensImuPlugin::onInitSrv, this);
    InitSoem(realtime);
  }

  // bool onInitSrv(bodyctrl_msgs::XSensImuInit::Request& req, bodyctrl_msgs::XSensImuInit::Response& resp) {
  //   static bool targetState = false;

  //   auto rlt = InitSoem(req.nameOfNet);
  //   resp.ret = iSlaveCount;
    
  //   return rlt;
  // }

  bool InitSoem(bool realtime) {
    static bool targetState = false;

    if (inited.compare_exchange_strong(targetState, true)) {

      // std::thread(&XSensImuPlugin::Run, this).detach();
      xsense_init([this](XsensImuData data) {
        auto msg = boost::make_shared<bodyctrl_msgs::Imu>();
        msg->header.stamp = ros::Time::now();
        msg->euler.yaw = data.euler.yaw() / 180.0 * M_PI;
        msg->euler.pitch = data.euler.pitch() / 180.0 * M_PI;
        msg->euler.roll = data.euler.roll() / 180.0 * M_PI;
        msg->angular_velocity.x = data.gyr[0];
        msg->angular_velocity.y = data.gyr[1];
        msg->angular_velocity.z = data.gyr[2];
        msg->linear_acceleration.x = data.acc[0];
        msg->linear_acceleration.y = data.acc[1];
        msg->linear_acceleration.z = data.acc[2];
        msg->orientation.w = data.quaternion.w();
        msg->orientation.x = data.quaternion.x();
        msg->orientation.y = data.quaternion.y();
        msg->orientation.z = data.quaternion.z();
        pubImu.publish(msg);
      }, realtime);
	    xsense_start();
      return true;
    }

    return true;
  }

  // void Run() {
  //   {
  //     // set cpu-affinity
  //     int cpus = 0;
  //     cpu_set_t mask;

  //     cpus = sysconf(_SC_NPROCESSORS_CONF);
  //     printf("cpus: %d\n", cpus);

  //     CPU_ZERO(&mask);          // init mask
  //     CPU_SET(cpus - 1, &mask); // add last cup core to cpu set
  //     // CPU_SET(1, &mask); // add first cup core to cpu set


  //     if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
  //     printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
  //     }
  //     usleep(1000);
  //     printf("set CPU affinity success\n");
  //       //   set cpu-affinity


  //     //set sched-strategy
  //     struct sched_param sched;
  //     int max_priority;

  //     max_priority = sched_get_priority_max(SCHED_RR);
  //     sched.sched_priority = max_priority;

  //     if (sched_setscheduler(getpid(), SCHED_RR, &sched) == -1) {
  //     printf("Set Scheduler Param, ERROR:%s\n", strerror(errno));
  //     }
  //   }

  //   Eigen::Matrix3d R_xense;
  //   Eigen::Vector3d euler_w, euler_w2, euler_b;

  //   Eigen::VectorXd imuData=Eigen::VectorXd::Zero(13);
	
  //   xsense_init(&imuData);
	//   xsense_start();

  //   Time start_time;
  //   Time period(0,2500000);
  //   Time sleep2Time;
  //   Time timer;
	//   Time time1;
  //   timespec sleep2Time_spec;

  //   int simCnt = 0;
  //   double timeSim = 0.0;
  //   double timeStep = 0.005;

  //   // std::ofstream foutData;
  //   // foutData.open("datacollection.txt",std::ios::out);
  //   Eigen::VectorXd dataL = Eigen::VectorXd::Zero(30);
  //   while (ros::ok())
  //   {   
	// 	    start_time = timer.currentTime();
        
  //       auto msg = boost::make_shared<bodyctrl_msgs::Imu>();
  //       msg->header.stamp = ros::Time::now();
  //       msg->euler.yaw = imuData[0] ;
  //       msg->euler.pitch = imuData[1];
  //       msg->euler.roll = imuData[2];
  //       msg->angular_velocity.x = imuData[3];
  //       msg->angular_velocity.y = imuData[4];
  //       msg->angular_velocity.z = imuData[5];
  //       msg->linear_acceleration.x = imuData[6];
  //       msg->linear_acceleration.y = imuData[7];
  //       msg->linear_acceleration.z = imuData[8];
  //       msg->orientation.w = imuData[9];
  //       msg->orientation.x = imuData[10];
  //       msg->orientation.y = imuData[11];
  //       msg->orientation.z = imuData[12];
  //       pubImu.publish(msg);
		
		
  //       time1 = timer.currentTime()-start_time;
  //       simCnt += 1;
  //       timeSim =  simCnt*timeStep;
  //       //
  //       dataL[0] = timeSim;
  //       dataL[1] = start_time.m_nanoSeconds;
  //       dataL[2] = time1.m_nanoSeconds;
  //       dataL.segment(3,9) = imuData.head(9);
	
  //       sleep2Time = start_time + period;
  //       sleep2Time_spec = sleep2Time.toTimeSpec();
  //       clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&(sleep2Time_spec),NULL);
  //   }
    
  //   // foutData.close();
  // }

  fast_ros::Publisher pubImu;
  ros::ServiceServer ssInit;
  std::atomic_bool inited = false;
};
}

PLUGINLIB_EXPORT_CLASS(xsens_imu::XSensImuPlugin, nodelet::Nodelet);
