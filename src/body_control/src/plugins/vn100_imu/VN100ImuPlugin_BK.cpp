#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <thread>
#include <atomic>
#include <bodyctrl_msgs/Imu.h>
#include <fast_ros/fast_ros.h>
#include <glog/logging.h>

// Include this header file to get access to the EzAsyncData class.
#include "vn/ezasyncdata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

namespace vn100  
{

class VN100ImuPlugin : public nodelet::Nodelet
{
public:
    VN100ImuPlugin()
    {}

private:
    virtual void onInit()
    {
        auto& nh = getPrivateNodeHandle();
        auto fnh = fast_ros::NodeHandle(getPrivateNodeHandle());
        pubImu = fnh.advertise<bodyctrl_msgs::Imu>("imu", 1000);
        InitSoem();
    }


    bool InitSoem() {
        static bool targetState = false;

        if (inited.compare_exchange_strong(targetState, true)) {
            std::thread(std::bind(&VN100ImuPlugin::Run, this)).detach();
            return true;
        }

        return true;
    }

    void Run() {
        // This example walks through using the EzAsyncData class to easily access
        // asynchronous data from a VectorNav sensor at a slight performance hit which is
        // acceptable for many applications, especially simple data logging.

        // First determine which COM port your sensor is attached to and update the
        // constant below. Also, if you have changed your sensor from the factory
        // default baudrate of 115200, you will need to update the baudrate
        // constant below as well.
        // const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
        // const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
        const std::string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
        // const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
        // const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
        // const uint32_t SensorBaudrate = 115200;
        const uint32_t SensorBaudrate = 921600;

        // We create and connect to a sensor by the call below.
        EzAsyncData* ez = EzAsyncData::connect(SensorPort, SensorBaudrate);

        // Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
        while (true)
        {
            Thread::sleepMs(1);

            // This reads the latest data that has been processed by the EzAsyncData class.
            CompositeData cd = ez->currentData();

            // Make sure that we have some yaw, pitch, roll data.
            // LOG(INFO) << cd.hasYawPitchRoll() << "," 
            //     << cd.hasAnyAttitude() << ","
            //     << cd.hasAnyAcceleration() << ","
            //     << cd.hasAnyAngularRate();
            if (cd.hasYawPitchRoll() 
                    && cd.hasAnyAttitude()
                    && cd.hasAnyAcceleration()) {
                auto quat = cd.anyAttitude().quat();
                auto ypr = cd.yawPitchRoll();
                auto vel = cd.anyAngularRate();
                auto acc = cd.anyAcceleration();

                auto msg = boost::make_shared<bodyctrl_msgs::Imu>();
                msg->header.stamp = ros::Time::now();
                msg->euler.yaw = ypr[0] / 180.0 * M_PI;
                msg->euler.pitch = ypr[1] / 180.0 * M_PI;
                msg->euler.roll = ypr[2] / 180.0 * M_PI;
                msg->angular_velocity.x = vel[0];
                msg->angular_velocity.y = vel[1];
                msg->angular_velocity.z = vel[2];
                msg->linear_acceleration.x = acc[0];
                msg->linear_acceleration.y = acc[1];
                msg->linear_acceleration.z = acc[2];
                msg->orientation.w = quat[3];
                msg->orientation.x = quat[0];
                msg->orientation.y = quat[1];
                msg->orientation.z = quat[2];
                pubImu.publish(msg);
            }
        }
    }

    fast_ros::Publisher pubImu;
    ros::ServiceServer ssInit;
    std::atomic_bool inited = false;
};
}

PLUGINLIB_EXPORT_CLASS(vn100::VN100ImuPlugin, nodelet::Nodelet);
