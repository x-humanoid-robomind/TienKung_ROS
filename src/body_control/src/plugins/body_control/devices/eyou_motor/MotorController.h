#pragma once

#include <iostream>  
#include <vector>  
#include <cmath>  
#include <algorithm>  
  
class MotorController {  
public:  
    double currentPosition;  
    double currentVelocity;  
    double current; // 假设这是从电机获取的当前电流值  
    double maxVelocity;  
    double targetPosition;  
  
public:  
    MotorController() : currentPosition(0), currentVelocity(0), current(0), maxVelocity(0), targetPosition(0) {}  
  
    // 设置目标位置和最大速度  
    void setTargetPositionAndMaxVelocity(double position, double velocity) {  
        targetPosition = position;  
        maxVelocity = std::abs(velocity); // 取绝对值，因为速度是矢量  
    }  
  
    // 更新电机状态，并返回规划的速度  
    double update(double currentPositionInput, double currentVelocityInput, double currentInput) {  
        currentPosition = currentPositionInput;  
        currentVelocity = currentVelocityInput;  
        current = currentInput;  
  
        // 简单的位置和速度规划逻辑  
        double error = targetPosition - currentPosition;  
        double plannedVelocity = 0;  
  
        // 使用简单的P控制器来确定速度，可以根据需要添加PI或PID控制  
        if (error > 0) {  
            plannedVelocity = std::min(maxVelocity, error * 0.1); // 假设使用P控制，Kp=0.1  
        } else if (error < 0) {  
            plannedVelocity = std::max(-maxVelocity, error * 0.1);  
        }  
  
        // 可以在此处添加更复杂的控制逻辑，例如考虑加速度限制、电流限制等  
  
        return plannedVelocity;  
    }  
};  