#include "RobotApp/Bridge/controller_hook.hpp"

#include <cmath>

#include "RobotApp/Domain/config.hpp"

namespace {

constexpr float kMotorKt_NmPerA = 0.0156f;  // bare M3508 motor, no gearbox

inline int16_t clamp_i16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

inline int16_t clamp_current_mA(float v_mA, int16_t limit_mA)
{
  if (v_mA > static_cast<float>(limit_mA)) v_mA = static_cast<float>(limit_mA);
  if (v_mA < -static_cast<float>(limit_mA)) v_mA = -static_cast<float>(limit_mA);
  const int32_t rounded = (v_mA >= 0.0f) ? static_cast<int32_t>(v_mA + 0.5f)
                                         : static_cast<int32_t>(v_mA - 0.5f);
  return clamp_i16(rounded);
}

}  // namespace

namespace robotapp::algo {

ChassisControllerOutput user_controller_step(const ChassisControllerInput& in)
{
  ChassisControllerOutput out{};
  static float int_err = 0.0f;
  static uint64_t last_ts_us = 0;
  if (in.operator_state == nullptr || in.lqr == nullptr)
  {
    int_err = 0.0f;
    last_ts_us = 0;
    return out;
  }
  if (!in.operator_state->enabled || in.operator_state->e_stop)
  {
    int_err = 0.0f;
    last_ts_us = 0;
    return out;
  }

  const auto& s = *in.lqr;
  if (!s.valid)
  {
    int_err = 0.0f;
    last_ts_us = 0;
    return out;
  }
  if (!(kMotorKt_NmPerA > 0.0f)) return out;

  if (!std::isfinite(s.theta_rad) || !std::isfinite(s.theta_dot_rps))
  {
    return out;
  }

  const uint64_t now_us = in.ts_us;
  float dt_s = 0.0f;
  if (last_ts_us != 0U && now_us >= last_ts_us)
  {
    dt_s = static_cast<float>(now_us - last_ts_us) * 1.0e-6f;
  }
  if (!(dt_s > 0.0f) || dt_s > 0.1f)
  {
    int_err = 0.0f;
    dt_s = 0.0f;
  }
  last_ts_us = now_us;

  const float err = -s.theta_rad;  // target pitch = 0
  if (dt_s > 0.0f)
  {
    int_err += err * dt_s;
    const float lim = domain::config::kBalanceIntLimit;
    if (int_err > lim) int_err = lim;
    if (int_err < -lim) int_err = -lim;
  }

  // PID: torque command (N*m).
  const float u = domain::config::kBalanceKp * err +
                  domain::config::kBalanceKi * int_err +
                  domain::config::kBalanceKd * (-s.theta_dot_rps);

  // Split total torque equally across two wheels.
  const float torque_per_wheel = 0.5f * u;
  const float current_A = torque_per_wheel / kMotorKt_NmPerA;
  const float current_mA = current_A * 1000.0f;

  const int16_t limit = (in.wheel_ctrl_cfg != nullptr)
                            ? in.wheel_ctrl_cfg->wheel_current_limit_mA
                            : domain::config::kWheelCurrentLimit_mA;
  const int16_t cmd_mA = clamp_current_mA(current_mA, limit);

  if (!out.cmd.wheels.empty()) out.cmd.wheels[0].current = cmd_mA;
  if (out.cmd.wheels.size() > 1) out.cmd.wheels[1].current = cmd_mA;
  out.cmd_valid = true;
  return out;
}

}  // namespace robotapp::algo

namespace robotapp {

algo::ChassisControllerFn ChassisControllerOverride() noexcept
{
  // Return nullptr to use built-in chassis_controller_step directly.
  return &algo::user_controller_step;
}

}  // namespace robotapp
