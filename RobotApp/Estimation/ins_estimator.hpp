#pragma once

#include <cstdint>

#include "RobotApp/Domain/types.hpp"

namespace robotapp::estimation {

class InsEstimator {
 public:
  struct Gains {
    // Mahony-style attitude correction using accelerometer (gravity direction).
    float kp_acc = 2.0f;
    float ki_acc = 0.05f;
    // Yaw correction using magnetometer (tilt-compensated heading).
    float kp_mag_yaw = 1.0f;
    // Wheel-odometry velocity correction gain (0 disables odom correction).
    float odom_vel_gain = 0.4f;
    // Optional yaw-rate correction from wheel odom (0 disables).
    float odom_yaw_gain = 0.0f;
  };

  void reset(uint64_t ts_us = 0);

  void set_gains(const Gains& g) { gains_ = g; }
  const Gains& gains() const { return gains_; }

  void set_wheel_odom_config(const domain::WheelOdomConfig& c) { wheel_odom_cfg_ = c; }
  const domain::WheelOdomConfig& wheel_odom_config() const { return wheel_odom_cfg_; }

  // Run one estimator step at time ts_us (typically ControlTick timebase).
  // - imu is required; mag can be nullptr or stale (no yaw correction).
  // - motor can be nullptr or stale (no wheel-odometry correction).
  // - Output validity is gated by sensor staleness and dt sanity checks.
  domain::StateEstimate step(const domain::ImuState& imu, const domain::MagState* mag,
                             const domain::MotorFeedback* motor, uint64_t ts_us);

 private:
  Gains gains_{};
  domain::WheelOdomConfig wheel_odom_cfg_{};

  uint64_t last_ts_us_ = 0;

  float q_wxyz_[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // world-from-body
  float vel_mps_[3] = {0.0f, 0.0f, 0.0f};
  float pos_m_[3] = {0.0f, 0.0f, 0.0f};

  float gyro_bias_rps_[3] = {0.0f, 0.0f, 0.0f};
  float accel_bias_mps2_[3] = {0.0f, 0.0f, 0.0f};

  // Integral of attitude error (unitless); used to estimate gyro bias.
  float err_int_[3] = {0.0f, 0.0f, 0.0f};

  bool mount_offset_valid_ = false;
  float mount_offset_rpy_[3] = {0.0f, 0.0f, 0.0f};
};

}  // namespace robotapp::estimation
