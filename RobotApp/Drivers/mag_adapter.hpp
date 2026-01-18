#pragma once

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Util/double_buffer.hpp"

namespace robotapp::drivers {

// Magnetometer adapter placeholder; will wrap IST8310 reading/calibration.
class MagAdapter {
 public:
  MagAdapter() = default;

  void set_calibration(const domain::MagCalibration& c) { calib_ = c; }

  void set_raw(const domain::MagSample& sample) { raw_.push(sample); }

  domain::MagState state() const
  {
    const auto raw = raw_.latest();
    domain::MagState s{};
    s.ts_us = raw.ts_us;
    // Convert raw int16 to meaningful units later; keep raw for now.
    const float mx = static_cast<float>(raw.mx);
    const float my = static_cast<float>(raw.my);
    const float mz = static_cast<float>(raw.mz);
    s.mag_uT[0] = (mx * calib_.scale[0]) + calib_.bias[0];
    s.mag_uT[1] = (my * calib_.scale[1]) + calib_.bias[1];
    s.mag_uT[2] = (mz * calib_.scale[2]) + calib_.bias[2];
    return s;
  }

 private:
  util::DoubleBuffer<domain::MagSample> raw_{};
  domain::MagCalibration calib_{};
};

}  // namespace robotapp::drivers
