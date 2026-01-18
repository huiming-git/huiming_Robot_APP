#pragma once

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Util/double_buffer.hpp"

namespace robotapp::drivers {

// IMU adapter placeholder; will wrap BMI088 reading/filtering.
class ImuAdapter {
 public:
  ImuAdapter() = default;

  void set_calibration(const domain::ImuCalibration& c) { calib_ = c; }

  void set_raw(const domain::ImuSample& sample) { raw_.push(sample); }

  domain::ImuState state() const
  {
    const auto raw = raw_.latest();
    domain::ImuState s{};
    s.ts_us = raw.ts_us;
    // BMI088 config in RobotApp/Drivers/bmi088.cpp: accel 6g, gyro 2000 dps.
    // Convert raw int16 counts into physical units, then apply runtime calibration.
    static constexpr float kG = 9.80665f;
    static constexpr float kAccelLsbPerG_6g = 5460.0f;  // BMI088 accel @ ±6g
    static constexpr float kGyroLsbPerDps_2000 = 16.4f; // BMI088 gyro @ ±2000 dps
    static constexpr float kAccelMps2PerLsb = kG / kAccelLsbPerG_6g;
    static constexpr float kGyroDpsPerLsb = 1.0f / kGyroLsbPerDps_2000;

    const float ax = static_cast<float>(raw.accel_x) * kAccelMps2PerLsb;
    const float ay = static_cast<float>(raw.accel_y) * kAccelMps2PerLsb;
    const float az = static_cast<float>(raw.accel_z) * kAccelMps2PerLsb;
    const float gx = static_cast<float>(raw.gyro_x) * kGyroDpsPerLsb;
    const float gy = static_cast<float>(raw.gyro_y) * kGyroDpsPerLsb;
    const float gz = static_cast<float>(raw.gyro_z) * kGyroDpsPerLsb;

    s.accel_mps2[0] = (ax * calib_.accel_scale[0]) + calib_.accel_bias[0];
    s.accel_mps2[1] = (ay * calib_.accel_scale[1]) + calib_.accel_bias[1];
    s.accel_mps2[2] = (az * calib_.accel_scale[2]) + calib_.accel_bias[2];
    s.gyro_dps[0] = (gx * calib_.gyro_scale[0]) + calib_.gyro_bias[0];
    s.gyro_dps[1] = (gy * calib_.gyro_scale[1]) + calib_.gyro_bias[1];
    s.gyro_dps[2] = (gz * calib_.gyro_scale[2]) + calib_.gyro_bias[2];
    return s;
  }

 private:
  util::DoubleBuffer<domain::ImuSample> raw_{};
  domain::ImuCalibration calib_{};
};

}  // namespace robotapp::drivers
