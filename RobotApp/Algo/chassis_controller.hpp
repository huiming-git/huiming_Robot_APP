#pragma once

#include <cstdint>
#include <cmath>

#include "RobotApp/Domain/config.hpp"
#include "RobotApp/Domain/types.hpp"

namespace robotapp::algo {

namespace detail {
inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline int16_t clamp_i16(int32_t v)
{
  if (v < static_cast<int32_t>(INT16_MIN)) return INT16_MIN;
  if (v > static_cast<int32_t>(INT16_MAX)) return INT16_MAX;
  return static_cast<int16_t>(v);
}

inline float age_s(uint64_t now_us, uint64_t src_us)
{
  if (now_us == 0U || src_us == 0U) return 1.0e9f;
  if (now_us < src_us) return 0.0f;
  return static_cast<float>(now_us - src_us) * 1.0e-6f;
}

inline float motor_rpm_to_wheel_mps(int16_t motor_rpm, float gear_ratio, float wheel_radius_m,
                                   int8_t sign)
{
  if (!(gear_ratio > 0.0f) || !(wheel_radius_m > 0.0f)) return 0.0f;
  const float rpm = static_cast<float>(motor_rpm) * static_cast<float>(sign);
  const float wheel_rps = (rpm / 60.0f) / gear_ratio;  // rev/s at wheel
  const float wheel_radps = wheel_rps * 2.0f * 3.14159265358979323846f;
  return wheel_radps * wheel_radius_m;
}
}  // namespace detail

struct ChassisControllerInput {
  uint64_t ts_us = 0;
  uint32_t dt_us = 0;
  const domain::SensorsSnapshot* sensors = nullptr;
  const domain::StateEstimate* estimate = nullptr;
  const domain::WheelOdomConfig* wheel_odom_cfg = nullptr;
  const domain::WheelControllerConfig* wheel_ctrl_cfg = nullptr;
  const domain::OperatorState* operator_state = nullptr;
  const domain::HighLevelCommand* hl_cmd = nullptr;
};

struct ChassisControllerOutput {
  domain::ChassisCommand cmd{};
  bool cmd_valid = false;
};

using ChassisControllerFn = ChassisControllerOutput (*)(const ChassisControllerInput&);

// Controller interface stub:
// - Designed to be replaced by real kinematics/control computation later.
// - Current fallback: minimal closed-loop diff-drive velocity control using 3508 wheel RPM feedback.
inline ChassisControllerOutput chassis_controller_step(const ChassisControllerInput& in)
{
  ChassisControllerOutput out{};
  if (in.operator_state == nullptr || in.hl_cmd == nullptr) return out;
  if (!in.operator_state->enabled || in.operator_state->e_stop) return out;

  const float vx_u = detail::clampf(in.hl_cmd->vx, -1.0f, 1.0f);
  const float wz_u = detail::clampf(in.hl_cmd->wz, -1.0f, 1.0f);
  const domain::WheelControllerConfig ctrl =
      (in.wheel_ctrl_cfg != nullptr) ? *in.wheel_ctrl_cfg : domain::WheelControllerConfig{};
  const domain::WheelOdomConfig odom =
      (in.wheel_odom_cfg != nullptr) ? *in.wheel_odom_cfg : domain::WheelOdomConfig{};

  const float vx_cmd = vx_u * ctrl.max_vx_mps;
  const float wz_cmd = wz_u * ctrl.max_wz_rps;

  const float track = odom.wheel_track_m;
  const float v_l_des = vx_cmd - wz_cmd * (track * 0.5f);
  const float v_r_des = vx_cmd + wz_cmd * (track * 0.5f);

  bool odom_ok = false;
  float v_l_meas = 0.0f;
  float v_r_meas = 0.0f;
  if (in.sensors != nullptr)
  {
    const float age = detail::age_s(in.ts_us, in.sensors->motor.ts_us);
    odom_ok = std::isfinite(age) && (age * 1.0e6f <= static_cast<float>(domain::config::kOdomTimeoutUs));
    v_l_meas = detail::motor_rpm_to_wheel_mps(in.sensors->motor.wheels[0].rpm,
                                             odom.wheel_gear_ratio,
                                             odom.wheel_radius_m,
                                             odom.rpm_sign_left);
    v_r_meas = detail::motor_rpm_to_wheel_mps(in.sensors->motor.wheels[1].rpm,
                                             odom.wheel_gear_ratio,
                                             odom.wheel_radius_m,
                                             odom.rpm_sign_right);
    if (!std::isfinite(v_l_meas) || !std::isfinite(v_r_meas)) odom_ok = false;
  }

  // If wheel odom is not available yet, fall back to open-loop current mapping (normalized).
  if (!odom_ok)
  {
    const float scale = domain::config::kWheelCurrentScale_mA;
    const int16_t left = static_cast<int16_t>((vx_u - wz_u) * scale);
    const int16_t right = static_cast<int16_t>((vx_u + wz_u) * scale);
    if (!out.cmd.wheels.empty()) out.cmd.wheels[0].current = left;
    if (out.cmd.wheels.size() > 1) out.cmd.wheels[1].current = right;
    out.cmd_valid = true;
    return out;
  }

  const float kp = ctrl.wheel_vel_kp_mA_per_mps;
  const float e_l = v_l_des - v_l_meas;
  const float e_r = v_r_des - v_r_meas;
  int32_t i_l = static_cast<int32_t>(e_l * kp);
  int32_t i_r = static_cast<int32_t>(e_r * kp);

  const int32_t lim = static_cast<int32_t>(ctrl.wheel_current_limit_mA);
  if (i_l > lim) i_l = lim;
  if (i_l < -lim) i_l = -lim;
  if (i_r > lim) i_r = lim;
  if (i_r < -lim) i_r = -lim;

  if (!out.cmd.wheels.empty()) out.cmd.wheels[0].current = detail::clamp_i16(i_l);
  if (out.cmd.wheels.size() > 1) out.cmd.wheels[1].current = detail::clamp_i16(i_r);

  out.cmd_valid = true;
  return out;
}

}  // namespace robotapp::algo
