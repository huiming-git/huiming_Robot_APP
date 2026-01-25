#include "RobotApp/Bridge/controller_hook.hpp"

#include <cmath>

#include "RobotApp/Domain/config.hpp"

namespace {

constexpr float kLqrK[4] = {77.27f, 16.50f, -3.16f, -12.22f};
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
  if (in.operator_state == nullptr || in.lqr == nullptr) return out;
  if (!in.operator_state->enabled || in.operator_state->e_stop) return out;

  const auto& s = *in.lqr;
  if (!s.valid) return out;
  if (!(kMotorKt_NmPerA > 0.0f)) return out;

  const float x0 = s.theta_rad;
  const float x1 = s.theta_dot_rps;
  const float x2 = s.x_m;
  const float x3 = s.x_dot_mps;

  if (!std::isfinite(x0) || !std::isfinite(x1) || !std::isfinite(x2) || !std::isfinite(x3))
  {
    return out;
  }

  // LQR: u = -Kx, u is total wheel torque (N*m).
  const float u = -(kLqrK[0] * x0 + kLqrK[1] * x1 + kLqrK[2] * x2 + kLqrK[3] * x3);

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
