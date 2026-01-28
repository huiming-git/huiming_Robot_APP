#pragma once

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Domain/config.hpp"

namespace robotapp::logic {

struct LogicInput {
  uint64_t ts_us = 0;
  domain::RemoteState remote{};
  domain::OperatorState operator_state{};
  domain::SystemHealth health{};
  domain::ImuState imu{};
  domain::MagState mag{};
  domain::MotorFeedback motor{};
};

struct LogicOutput {
  domain::HighLevelCommand hl_cmd{};
  domain::ChassisCommand cmd{};
  bool hl_valid = false;
  bool cmd_valid = false;
};

// Pure function placeholder for logic layer (100 Hz) to compute high-level commands.
inline LogicOutput logic_step(const LogicInput& in)
{
  LogicOutput out{};
  // Placeholder behavior:
  // - If remote is stale/failsafe -> output zero command and mark invalid.
  // - If operator is not enabled or in e-stop -> output invalid.
  // - Else -> output high-level command only (controller will decide actuator outputs).
  const bool remote_stale =
      (in.remote.ts_us == 0U) ||
      ((in.ts_us != 0U) && (in.ts_us - in.remote.ts_us > domain::config::kRemoteTimeoutUs));
  const bool remote_failsafe = remote_stale || in.remote.failsafe;
  if (remote_failsafe)
  {
    out.cmd_valid = false;
    return out;
  }
  if (!in.operator_state.enabled || in.operator_state.e_stop)
  {
    out.cmd_valid = false;
    return out;
  }

  // Channel mapping:
  // - ch0 (SBUS ch1): clockwise rotation
  // - ch1 (SBUS ch2): forward/back
  float vx = in.remote.ch[1];
  float wz = -in.remote.ch[0] * 2.0f;  // boost turn torque
  if (wz > 1.0f) wz = 1.0f;
  if (wz < -1.0f) wz = -1.0f;
  if (vx > 1.0f) vx = 1.0f;
  if (vx < -1.0f) vx = -1.0f;
  if (wz > 1.0f) wz = 1.0f;
  if (wz < -1.0f) wz = -1.0f;
  if (vx > -domain::config::kRcDeadband && vx < domain::config::kRcDeadband) vx = 0.0f;
  if (wz > -domain::config::kRcDeadband && wz < domain::config::kRcDeadband) wz = 0.0f;

  out.hl_cmd.vx = vx;
  out.hl_cmd.wz = wz;
  out.hl_valid = true;
  return out;
}

}  // namespace robotapp::logic
