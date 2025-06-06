#include <chrono>  
#include <thread>  
#include <atomic>  
#include <iostream>  
  
class HighPrecisionRate {  
public:  
    HighPrecisionRate(double hz)  
        : target_hz_(hz),  
          period_us_(static_cast<long long>(1e6 / hz)) // Period in microseconds  
    {  
        if (period_us_ <= 0) {  
            throw std::invalid_argument("Frequency is too high, period would be zero or negative.");  
        }  
    }  
  
    void sleep() {  
        auto start = std::chrono::steady_clock::now();  
        auto end = start + std::chrono::microseconds(period_us_);  
  
        // Busy-wait with high-resolution clock checks  
        while (std::chrono::steady_clock::now() < end) {  
            // Optionally, you can add a small delay here to reduce CPU usage  
            // std::this_thread::yield(); // Hint to the scheduler to give up the timeslice  
        }  
  
        // Account for any oversleep  
        auto oversleep = std::chrono::steady_clock::now() - end;  
        if (oversleep > std::chrono::microseconds(period_us_ / 10)) {  
            // If we overslept by more than 10%, adjust the next sleep period  
            // This is a heuristic to try to correct for large scheduling delays  
            std::cerr << "Warning: Overslept by " << std::chrono::duration_cast<std::chrono::microseconds>(oversleep).count() << " us" << std::endl;  
            // You might consider adjusting the period_us_ here to compensate  
        }  
    }  
  
private:  
    double target_hz_;  
    long long period_us_; // Period in microseconds  
};  
  
int main() {  
    HighPrecisionRate rate(30000); // Attempt to create a 30000Hz rate object  
  
    for (int i = 0; i < 100000; ++i) {  
        rate.sleep();  
        // Perform your task here  
        // std::cout << "Tick" << std::endl; // Uncomment to see ticks, but this will affect performance  
    }  
  
    return 0;  
}