#include <cmath>
#include <ros/ros.h>

namespace ubt_hw {  
  
class HighPassFilter {  
private:  
    double alpha_; // 滤波系数  
    double previous_input_ = 0; // 上一个输入值  
    double previous_output_ = 0; // 上一个输出值  
    double wc;

    ros::Time prev_time_;
    ros::Duration delta_t_;
  
public:  
    HighPassFilter(double cutoff_freq) {  
        // 计算滤波系数alpha_  
        wc = 2.0 * M_PI * cutoff_freq; // 角频率  
    }  
  
    void input(double in, ros::Time time) {  

        if (!prev_time_.isZero())  // Not first time through the program
        {
            delta_t_ = time - prev_time_;
            prev_time_ = time;

            if (0 == delta_t_.toSec()) {
            ROS_ERROR(
                "delta_t is 0, skipping this loop. Possible overloaded cpu "
                "at time: %f",
                time.toSec());
            return;
            }
        } else {
            prev_time_ = time;
            return;
        }

        auto dt = delta_t_.toSec();
        alpha_ = wc * dt / (1.0 + wc * dt); // 一阶高通滤波器的alpha  

        // 高通滤波器的差分方程  
        double output = alpha_ * (in - previous_input_) + (1 - alpha_) * previous_output_;  
        previous_input_ = in;  
        previous_output_ = output;  
    }  
  
    void input(double in) { input(in, ros::Time::now()); } // 如果时间戳不重要，可以简化为这个形式  
  
    double output() {  
        // 返回当前高通滤波器的输出值  
        return previous_output_;  
    }  
  
    void reset() {  
        previous_input_ = 0.0;  
        previous_output_ = 0.0;  
    }  
};  
  
} // namespace ubt_hw