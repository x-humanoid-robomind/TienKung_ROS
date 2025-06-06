#pragma once

#include <cmath>
#include <chrono>

class SinTime {
public:
    void init(double base = 1.0, double mul = 1.0) {
        this->base = base;
        this->mul = mul;
        start = std::chrono::system_clock::now();
    }

    double update(double data) {
        auto now = std::chrono::system_clock::now();
        Duration = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - start).count() / 1000 / 1000 - Runtime;
        Runtime = (double)std::chrono::duration_cast<std::chrono::microseconds>(now - start).count() / 1000 / 1000;
        return data * sin(Runtime * mul);
    }

    double update() {
        return update(base);
    }

private:
    double multiplier;
    std::chrono::_V2::system_clock::time_point start;
    double Duration, Runtime = 0;
    double mul;
    double base;
};