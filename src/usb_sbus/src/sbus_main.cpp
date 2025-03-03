#include "ros/ros.h"
#include <stdio.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>
#include <mutex>
#include <math.h>
#include "usbSbus.h"

#define SBUS_USB_SERIAL_DEV "/dev/ttyUSB0"

#define DF_HC_MAX_LINE_SPEED (2.0 )          // m/s
#define DF_HC_MAX_ANGLE_SPEED (0.5)          // rad/s

#define JOY_MIN_VAL     282
#define JOY_MAX_VAL     1722
#define JOY_NORAM_VAL   1002


const float EPSINON = 1e-6;
#define Equ(a,b) ((fabs((a) - (b)) < (EPSINON)))

usbSbus sbusHandle_;
ros::Publisher joy_publisher;
bool bRunning=true;

std::mutex data_locker;
float fAxes_0,fAxes_1;
double axes_time;
unsigned int joy_seq=0;

int joy_now_ = 0;

bool bJoyEnable_ = false;

float get_axes_val(uint16_t nVal)
{
    float fData=0.0;
    if(nVal < JOY_NORAM_VAL)
        fData = (JOY_NORAM_VAL - nVal) * 1.0 / (JOY_NORAM_VAL - JOY_MIN_VAL);
    else if(nVal > JOY_NORAM_VAL)
        fData = (JOY_NORAM_VAL - nVal) * 1.0 / (JOY_MAX_VAL - JOY_NORAM_VAL);

    return (fData * (-1.0));
}

int ch5_button_check(uint16_t nVal)
{
    int nResult = JOY_NORAM_VAL - nVal;
    if(nResult>280)
    {
        ROS_INFO("joy control switch on!!!");
        bJoyEnable_ = true;
    }
    else
    {
        bJoyEnable_ = false;
    }
}

void recvThreadFunc(void)
{
    uint16_t channels_in[16];
    float fAxes;
    ROS_INFO("sbus read thread begin.");

    bool bEqu=false;

    uint16_t in_change[16];

    while (bRunning)
    {
        int nRet = sbusHandle_.sbus_read(channels_in);
        if(nRet!=16)
        {
            //ROS_ERROR("sbus_read error.\n");
            usleep(10*1000);
            continue;
        }
    #if 0
        data_locker.lock();
        
        // ch5_button_check(channels_in[4]);


        fAxes_0 = get_axes_val(channels_in[0]);
        bEqu = Equ(fAxes_0, 0.0);

        if(bEqu == false && bJoyEnable_ == true)
            axes_time = ros::Time::now().toSec();

        fAxes_1 = get_axes_val(channels_in[1]);
        bEqu = Equ(fAxes_1, 0.0);
        if(bEqu==false && bJoyEnable_==true)
            axes_time = ros::Time::now().toSec();
        data_locker.unlock();
    #else
        sensor_msgs::Joy joy;
        //uint16_t in_change[16];
        bool bHasPress = false;
        joy.header.seq = joy_seq++;
        joy.header.stamp = ros::Time::now();
        
        joy.axes.resize(12);
        joy.buttons.resize(0);
        for(int i=0;i < 12;i++)
        {
            joy.axes[i] = get_axes_val(channels_in[i]);
            if(in_change[i] != channels_in[i])
                bHasPress = true;
            in_change[i] = channels_in[i];
        }
        // fAxes = get_axes_val(channels_in[0]);
        // joy.axes[0] = fAxes * DF_HC_MAX_ANGLE_SPEED;//z方向，旋转，对应通道0
        // fAxes = get_axes_val(channels_in[1]);
        // joy.axes[1] = fAxes * DF_HC_MAX_LINE_SPEED;//x方向，直线，对应通道1
        if(bHasPress == true)
        {
            // ROS_INFO("joy : 0[%f] 1[%f] 2[%f] 3[%f] 4[%f] 5[%f] 6[%f] 7[%f] 8[%f] 9[%f] 10[%f] 11[%f]", presskey[0],presskey[1],presskey[2],presskey[3],presskey[4],presskey[5],presskey[6],presskey[7],presskey[8],presskey[9],presskey[10],presskey[11]);
            ROS_INFO("get sbus packet:");
            for (int ch=0; ch < 16; ch++) 
            {
                ROS_INFO("channel[%d] = %d    failsafe = %d", ch, channels_in[ch], sbusHandle_.sbus_failsafe());
            }
            ROS_INFO("---------------\n");
        }
        joy_publisher.publish(joy);
    #endif
        /*
        处理sbus数据
        */
    #if 0
        ROS_INFO("get sbus packet:");
        for (int ch=0; ch < 16; ch++) 
        {
            ROS_INFO("channel[%d] = %d    failsafe = %d", ch, channels_in[ch], sbusHandle_.sbus_failsafe());
        }
        ROS_INFO("---------------\n");
    #endif
    }
}

void spin()
{
       ros::MultiThreadedSpinner spinner;
       spinner.spin();
}

#if 1
int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_sbus_node");
    
    ros::NodeHandle node;
    //ros::Publisher 
    joy_publisher = node.advertise<sensor_msgs::Joy>("/sbus_data", 10);

    bool bRet = sbusHandle_.init(SBUS_USB_SERIAL_DEV);
    if(bRet == false)
        return -1;

    //boost::thread * spinnerThread_ = new boost::thread(boost::bind( &spin, this));
    boost::thread thrd(&spin); 

    boost::thread sbus_thrd(&recvThreadFunc); 

    ros::Rate r_10HZ(10); // 10 hz

    while (ros::ok())
    {
        // sensor_msgs::Joy joy;
        // joy.axes.resize(2);
        // joy.header.seq = joy_seq++;
        // joy.header.stamp = ros::Time::now();
        // joy.buttons.resize(0);

        // data_locker.lock();
        // double now = ros::Time::now().toSec();
        
        // long t_interval = (now - axes_time) * 1000;//ms
        // if(t_interval > 1000)
        // {
        //     data_locker.unlock();
        //     continue;//more than 1 seconds
        // }
        // else if(t_interval > 100)
        // {
        //     joy.axes[0] = 0.0;
        //     joy.axes[1] = 0.0;
        //     joy_publisher.publish(joy);
        //     data_locker.unlock();
        // }
        // else
        // {
        //     joy.axes[0] = fAxes_0;
        //     joy.axes[1] = fAxes_1;
        //     joy_publisher.publish(joy);
        //     data_locker.unlock();
        // }
        
        r_10HZ.sleep();
    }

    bRunning = false;

    sbus_thrd.join();
    thrd.join();

    return 1;
}
#else
int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_sbus_node");
    
    ros::NodeHandle node;
    ros::Publisher joy_publisher = node.advertise<sensor_msgs::Joy>("joy", 10);

    bool bRet = sbusHandle_.init(SBUS_USB_SERIAL_DEV);
    if(bRet == false)
        return -1;

    //boost::thread * spinnerThread_ = new boost::thread(boost::bind( &spin, this));
    boost::thread thrd(&spin); 

    boost::thread sbus_thrd(&recvThreadFunc); 

    ros::Rate r_10HZ(10); // 10 hz

    while (ros::ok())
    {
        sensor_msgs::Joy joy;
        joy.axes.resize(2);
        joy.header.seq = joy_seq++;
        joy.header.stamp = ros::Time::now();
        joy.buttons.resize(0);

        data_locker.lock();
        double now = ros::Time::now().toSec();
        
        long t_interval = (now - axes_time) * 1000;//ms
        if(t_interval > 1000)
        {
            data_locker.unlock();
            continue;//more than 1 seconds
        }
        else if(t_interval > 100)
        {
            joy.axes[0] = 0.0;
            joy.axes[1] = 0.0;
            joy_publisher.publish(joy);
            data_locker.unlock();
        }
        else
        {
            joy.axes[0] = fAxes_0;
            joy.axes[1] = fAxes_1;
            joy_publisher.publish(joy);
            data_locker.unlock();
        }
        
        r_10HZ.sleep();
    }

    bRunning = false;

    sbus_thrd.join();
    thrd.join();

    return 1;
}
#endif
