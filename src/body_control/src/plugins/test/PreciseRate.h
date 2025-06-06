#include <chrono>  
#include <ctime>  
#include <iostream>  
#include <thread>  
  
class PreciseRate {  
public:  
    PreciseRate(double hz) : frequency_(hz), period_ns_(static_cast<long long>(1e9 / hz)) {  
        if (period_ns_ <= 0) {  
            throw std::invalid_argument("Frequency is too high, period would be zero or negative.");  
        }  
    }  
  
    void sleep() {  
        timespec ts;  
  
        // 获取当前时间  
        clock_gettime(CLOCK_REALTIME, &ts);  
  
        // 计算下一个周期的时间点  
        auto next_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec + period_ns_;  
        ts.tv_sec = next_ns / 1000000000;  
        ts.tv_nsec = next_ns % 1000000000;  
  
        // 等待直到下一个周期的时间点  
        while (true) {  
            int result = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, nullptr);  
            if (result == 0) {  
                break; // 睡眠完成，退出循环  
            } else if (errno != EINTR) {  
                // 如果不是EINTR（被中断），则打印错误并退出  
                std::cerr << "clock_nanosleep error: " << strerror(errno) << std::endl;  
                std::exit(EXIT_FAILURE);  
            }  
  
            // 如果是EINTR，重新获取当前时间并计算下一个周期的时间点  
            clock_gettime(CLOCK_REALTIME, &ts);  
  
            // 重新计算下一个周期的时间点  
            next_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec + period_ns_;  
            ts.tv_sec = next_ns / 1000000000;  
            ts.tv_nsec = next_ns % 1000000000;  
        }  
    }  
  
private:  
    double frequency_;  
    long long period_ns_; // 纳秒为单位的周期时间  
};  