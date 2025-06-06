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

    virtual ~VN100ImuPlugin() {
        vs.unregisterAsyncPacketReceivedHandler();
        vs.writeAsyncDataOutputFrequency(40);
        vs.disconnect();
    }

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
        // First determine which COM port your sensor is attached to and update the
        // constant below. Also, if you have changed your sensor from the factory
        // default baudrate of 115200, you will need to update the baudrate
        // constant below as well.
        // const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
        // const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
        const std::string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
        // const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
        // const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
        // const uint32_t SensorBaudrate = 921600;//921600;//115200;
        uint32_t SensorBaudrate = 115200;

        // Now let's create a VnSensor object and use it to connect to our sensor.
        
        vs.connect(SensorPort, SensorBaudrate);

        // read test and Change the baudrate
        try {
            uint32_t oldBaud = vs.baudrate();
            vs.changeBaudRate(921600);
            uint32_t newBaud = vs.baudrate();
        } catch (vn::timeout& e) {
            vs.disconnect();
            SensorBaudrate = 921600;
            vs.connect(SensorPort, SensorBaudrate);
        }

        uint32_t newBaud = vs.baudrate();

        // turn off auto-report
        uint32_t oldHz = vs.readAsyncDataOutputFrequency();
        vs.writeAsyncDataOutputFrequency(0);
        uint32_t newHz = vs.readAsyncDataOutputFrequency();

        // For the registers that have more complex configuration options, it is
        // convenient to read the current existing register configuration, change
        // only the values of interest, and then write the configuration to the
        // register. This allows preserving the current settings for the register's
        // other fields. Below, we change the heading mode used by the sensor.
        VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
        vpeReg.headingMode = HEADINGMODE_ABSOLUTE;
        vs.writeVpeBasicControl(vpeReg);
        vpeReg = vs.readVpeBasicControl();

        BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            1,
            // Note use of binary OR to configure flags.
            COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,	
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_NONE,
            INSGROUP_NONE,
            GPSGROUP_NONE);

        vs.writeBinaryOutput1(bor);

        vs.registerAsyncPacketReceivedHandler(this, &VN100ImuPlugin::asciiOrBinaryAsyncMessageReceived);
    }

    static void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
    {
        if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
        {
            // vec3f ypr;
            // p.parseVNYPR(&ypr);
            // cout << "ASCII Async YPR: " << ypr << endl;
            return;
        }

        if (p.type() == Packet::TYPE_BINARY)
        {
            // First make sure we have a binary packet type we expect since there
            // are many types of binary output types that can be configured.
            if (!p.isCompatible(
                COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
                TIMEGROUP_NONE,
                IMUGROUP_NONE,
                GPSGROUP_NONE,
                ATTITUDEGROUP_NONE,
                INSGROUP_NONE,
                GPSGROUP_NONE))
                // Not the type of binary packet we are expecting.
                return;

            // Ok, we have our expected binary output packet. Since there are many
            // ways to configure the binary data output, the burden is on the user
            // to correctly parse the binary packet. However, we can make use of
            // the parsing convenience methods provided by the Packet structure.
            // When using these convenience methods, you have to extract them in
            // the order they are organized in the binary packet per the User Manual.
            struct timeval t;
            gettimeofday(&t, NULL);
            auto ypr = p.extractVec3f();
            auto quat = p.extractVec4f();
            auto gyro = p.extractVec3f();
            auto acc = p.extractVec3f();

            auto msg = boost::make_shared<bodyctrl_msgs::Imu>();
            msg->header.stamp = ros::Time::now();
            msg->euler.yaw = ypr[0] / 180.0 * M_PI;
            msg->euler.pitch = ypr[1] / 180.0 * M_PI;
            msg->euler.roll = ypr[2] / 180.0 * M_PI;
            msg->angular_velocity.x = gyro[0];
            msg->angular_velocity.y = gyro[1];
            msg->angular_velocity.z = gyro[2];
            msg->linear_acceleration.x = acc[0];
            msg->linear_acceleration.y = acc[1];
            msg->linear_acceleration.z = acc[2];
            msg->orientation.x = quat[0];
            msg->orientation.y = quat[1];
            msg->orientation.z = quat[2];
            msg->orientation.w = quat[3];
            auto obj = (VN100ImuPlugin*)userData;
            obj->pubImu.publish(msg);
        }
    }

    ros::ServiceServer ssInit;
    std::atomic_bool inited = false;
    VnSensor vs;

public:
    fast_ros::Publisher pubImu;
};


}

PLUGINLIB_EXPORT_CLASS(vn100::VN100ImuPlugin, nodelet::Nodelet);
