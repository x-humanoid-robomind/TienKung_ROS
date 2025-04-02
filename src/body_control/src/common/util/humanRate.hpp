#ifndef BODY_HUMAN_RATE_CONTROL_HPP_
#define BODY_HUMAN_RATE_CONTROL_HPP_
#include <iostream>
#include <iostream>
#include <chrono>
#include <thread>

namespace body_human_rate_control 
{

typedef enum e_sleep_unit {
    SLEEP_UNIT_MS,
    SLEEP_UNIT_US,
    SLEEP_UNIT_NS
} eSleepUnit;

class humanRateControl 
{
public:
    humanRateControl(int nCtrlRate) : frequency(nCtrlRate), start_time(std::chrono::steady_clock::now()) 
    {
        if (frequency <= 0) {
        throw std::invalid_argument("Frequency must be positive.");
        }
        else if(frequency <= 1000)
        {
            period = 1000 / frequency; // 周期为1000ms/频率
            sleep_unit = SLEEP_UNIT_MS;
        }
        else if(frequency <= 1000000)
        {
            period = 1000000 / frequency; // 周期为1000000us/频率
            sleep_unit = SLEEP_UNIT_US;
        }
        else if(frequency <= 1000000000)
        {
            period = 1000000000 / frequency; // 周期为1000000000ns/频率
            sleep_unit = SLEEP_UNIT_NS;
        }
        else
            throw std::invalid_argument("Frequency must be less than or equal to 1000000000.");

        std::cout << "hz: " << frequency << " sleep unit: " << sleep_unit << " period: " << period  << std::endl;
    }

    void sleep(void)
    {
        if(sleep_unit == SLEEP_UNIT_MS)
        {
            sleep_ms(period);
        }
        else if(sleep_unit == SLEEP_UNIT_US)
        {
            sleep_us(period);
        }
        else
        {
            sleep_ns(period);
        }
    }

    void sleep_ms(int period)
    {
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        // int period = 1000 / frequency;
        int sleep_time = period - elapsed_time;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
        start_time = std::chrono::steady_clock::now();
    }

    void sleep_us(int period)
    {
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        // int period = 1000000 / frequency;
        int sleep_time = period - elapsed_time;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        }
        start_time = std::chrono::steady_clock::now();
    }

    void sleep_ns(int period)
    {
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        // int period = 1000000000 / frequency;
        int sleep_time = period - elapsed_time;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
        }
        start_time = std::chrono::steady_clock::now();
    }
    

private:
    int frequency;//频率
    int period; // 周期
    std::chrono::steady_clock::time_point start_time;
private:
    eSleepUnit sleep_unit = SLEEP_UNIT_MS;
};

}

#endif //BODY_HUMAN_RATE_CONTROL_HPP_
