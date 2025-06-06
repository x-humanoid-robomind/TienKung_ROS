#pragma once

struct ImuData {
  double orientation[4]; // x, y, z, w
  double angular_vel[3]; // x, y, z
  double linear_accel[3]; // x, y, z
  double rpy[3]; // roll, pitch, yaw
};