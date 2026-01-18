#pragma once

#include <array>

#include "RobotApp/Domain/types.hpp"

namespace robotapp::algo {

// BMI088 滤波接口占位：后续可替换为真正的卡尔曼实现。
struct ImuKalmanFilter {
  std::array<float, 3> angle_rad{0.0f, 0.0f, 0.0f};
  std::array<float, 3> bias_dps{0.0f, 0.0f, 0.0f};

  void reset()
  {
    angle_rad = {0.0f, 0.0f, 0.0f};
    bias_dps = {0.0f, 0.0f, 0.0f};
  }

  // 占位：dt 秒，输入 IMU 测量，输出估计的姿态角（rad）和去偏后的陀螺。
  void step(const domain::ImuState& meas, float dt,
            std::array<float, 3>& out_angle_rad,
            std::array<float, 3>& out_gyro_dps_unbiased)
  {
    // 简单的积分占位，后续替换为卡尔曼滤波。
    for (int i = 0; i < 3; ++i)
    {
      const float gyro = meas.gyro_dps[i] - bias_dps[i];
      angle_rad[i] += gyro * dt * 0.01745329252f;  // deg/s -> rad
      out_angle_rad[i] = angle_rad[i];
      out_gyro_dps_unbiased[i] = gyro;
    }
    // TODO: 融合 accel_mps2 观测，更新 bias/协方差。
  }
};

}  // namespace robotapp::algo
