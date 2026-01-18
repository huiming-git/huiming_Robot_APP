#include "RobotApp/Tasks/control_loop.hpp"

#include "RobotApp/Domain/config.hpp"

#include <span>

#include "RobotApp/Algo/chassis_controller.hpp"
#include "RobotApp/Drivers/motors.hpp"

extern "C" {
#include "app_bridge.h"
#include "app_can.h"
#include "app_i2c.h"
#include "app_sbus.h"
#include "app_spi.h"
}

extern "C" {
volatile uint32_t g_robotapp_can1_rx_count = 0;
volatile uint32_t g_robotapp_can2_rx_count = 0;
volatile uint32_t g_robotapp_can1_tx_ok = 0;
volatile uint32_t g_robotapp_can2_tx_ok = 0;
volatile uint32_t g_robotapp_can1_tx_fail = 0;
volatile uint32_t g_robotapp_can2_tx_fail = 0;
volatile uint32_t g_robotapp_can1_tx_start_fail = 0;
volatile uint32_t g_robotapp_can2_tx_start_fail = 0;
volatile uint32_t g_robotapp_can1_tx_cplt_fail = 0;
volatile uint32_t g_robotapp_can2_tx_cplt_fail = 0;
volatile uint16_t g_robotapp_can1_last_std_id = 0;
volatile uint16_t g_robotapp_can2_last_std_id = 0;
volatile uint8_t g_robotapp_dm_last_motor_id = 0;
volatile float g_robotapp_dm_pos[4] = {0};
volatile float g_robotapp_dm_vel[4] = {0};
volatile float g_robotapp_dm_tor[4] = {0};
volatile uint32_t g_robotapp_safety_flags = 0;
}

namespace robotapp::tasks {

void ControlLoop::init()
{
  sensors_ = {};
  telemetry_ = {};
  last_ts_us_ = 0;
  jitter_samples_ = 0;
  last_cmd_ = {};
  last_cmd_valid_ = false;
  last_cmd_ts_us_ = 0;
  last_wheels_fb_ts_us_ = 0;

  // Motors & manager wiring.
  motor_mgr_.clear();

  dji_group_.configure(&can_port_, 1U);
  (void)motor_mgr_.add(dji_group_);

  for (std::size_t i = 0; i < dm_motors_.size(); ++i)
  {
    dm_motors_[i].configure(&can_port_, 2U, static_cast<uint16_t>(i + 1U));
    (void)motor_mgr_.add(dm_motors_[i]);
  }
}

void ControlLoop::tick(uint64_t ts_us)
{
  const uint32_t dt_us =
      (last_ts_us_ == 0) ? 0U : static_cast<uint32_t>(ts_us - last_ts_us_);
  last_ts_us_ = ts_us;

  telemetry_.ts_us = ts_us;
  telemetry_.tick_count++;
  telemetry_.last_dt_us = dt_us;
  if (dt_us != 0U)
  {
    const int32_t jitter = static_cast<int32_t>(dt_us) -
                           static_cast<int32_t>(domain::config::kControlPeriodUs);
    telemetry_.last_jitter_us = jitter;
    if (jitter_samples_ == 0U)
    {
      telemetry_.jitter_min_us = jitter;
      telemetry_.jitter_max_us = jitter;
      telemetry_.abs_jitter_max_us = static_cast<uint32_t>(jitter < 0 ? -jitter : jitter);
    }
    else
    {
      if (jitter < telemetry_.jitter_min_us) telemetry_.jitter_min_us = jitter;
      if (jitter > telemetry_.jitter_max_us) telemetry_.jitter_max_us = jitter;
      const uint32_t absj = static_cast<uint32_t>(jitter < 0 ? -jitter : jitter);
      if (absj > telemetry_.abs_jitter_max_us) telemetry_.abs_jitter_max_us = absj;
    }
    jitter_samples_ += 1U;
  }

  const auto sensor_snap = RobotApp_SensorsLatest();
  const auto age_us = [ts_us](uint64_t src_ts_us) -> uint32_t {
    if (src_ts_us == 0U || ts_us == 0U) return 0xFFFFFFFFU;
    if (ts_us < src_ts_us) return 0U;
    const uint64_t d = ts_us - src_ts_us;
    return (d > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : static_cast<uint32_t>(d);
  };
  telemetry_.imu_age_us = age_us(sensor_snap.imu.ts_us);
  telemetry_.mag_age_us = age_us(sensor_snap.mag.ts_us);

  platform::CanFrame frame{};
  bool wheels_rx_seen = false;
  while (can_port_.rx_pop(frame))
  {
    // 先更新收包计数/最后 ID，再把帧分发到对应电机对象。
    if (frame.bus == 1U)
    {
      g_robotapp_can1_rx_count += 1U;
      g_robotapp_can1_last_std_id = frame.std_id;
    }
    else if (frame.bus == 2U)
    {
      g_robotapp_can2_rx_count += 1U;
      g_robotapp_can2_last_std_id = frame.std_id;
    }
    if (frame.bus == domain::config::kCanBusWheels) wheels_rx_seen = true;

    // Online tuning (CAN debug commands):
    // - 0x7A0: SET wheel odom config (8 bytes)
    //   data[0]=0x10,
    //   data[1..2]=wheel_radius_mm (uint16, little endian),
    //   data[3..4]=wheel_track_mm,
    //   data[5..6]=gear_ratio_x100 (uint16),
    //   data[7] bits: bit0 left invert, bit1 right invert
    // - 0x7A1: GET wheel odom config (data[0]=0x11), reply on 0x7A2 with same packing.
    if (frame.std_id == 0x7A0U && frame.dlc == 8U && frame.data[0] == 0x10U)
    {
      const auto op = RobotApp_OperatorLatest();
      if (!op.enabled)
      {
        const uint16_t radius_mm = static_cast<uint16_t>(frame.data[1] | (frame.data[2] << 8));
        const uint16_t track_mm = static_cast<uint16_t>(frame.data[3] | (frame.data[4] << 8));
        const uint16_t gear_x100 = static_cast<uint16_t>(frame.data[5] | (frame.data[6] << 8));
        const uint8_t sign_bits = frame.data[7];

        App_WheelOdomConfig cfg = RobotApp_WheelOdomConfigLatest();
        cfg.wheel_radius_m = static_cast<float>(radius_mm) / 1000.0f;
        cfg.wheel_track_m = static_cast<float>(track_mm) / 1000.0f;
        cfg.wheel_gear_ratio = static_cast<float>(gear_x100) / 100.0f;
        cfg.rpm_sign_left = (sign_bits & 0x01U) ? -1 : 1;
        cfg.rpm_sign_right = (sign_bits & 0x02U) ? -1 : 1;
        RobotApp_SetWheelOdomConfig(&cfg);

        uint8_t ack[8] = {0};
        ack[0] = 0x10U;
        ack[1] = frame.data[1];
        ack[2] = frame.data[2];
        ack[3] = frame.data[3];
        ack[4] = frame.data[4];
        ack[5] = frame.data[5];
        ack[6] = frame.data[6];
        ack[7] = frame.data[7];
        (void)can_port_.tx(frame.bus, 0x7A2U, std::span<const uint8_t>(ack, sizeof(ack)),
                          platform::OpNone());
      }
      continue;
    }
    if (frame.std_id == 0x7A1U && frame.dlc >= 1U && frame.data[0] == 0x11U)
    {
      const auto cfg = RobotApp_WheelOdomConfigLatest();
      const uint16_t radius_mm = static_cast<uint16_t>(cfg.wheel_radius_m * 1000.0f);
      const uint16_t track_mm = static_cast<uint16_t>(cfg.wheel_track_m * 1000.0f);
      const uint16_t gear_x100 = static_cast<uint16_t>(cfg.wheel_gear_ratio * 100.0f);
      const uint8_t sign_bits =
          (cfg.rpm_sign_left < 0 ? 0x01U : 0x00U) | (cfg.rpm_sign_right < 0 ? 0x02U : 0x00U);

      uint8_t resp[8] = {0};
      resp[0] = 0x11U;
      resp[1] = static_cast<uint8_t>(radius_mm & 0xFFU);
      resp[2] = static_cast<uint8_t>((radius_mm >> 8) & 0xFFU);
      resp[3] = static_cast<uint8_t>(track_mm & 0xFFU);
      resp[4] = static_cast<uint8_t>((track_mm >> 8) & 0xFFU);
      resp[5] = static_cast<uint8_t>(gear_x100 & 0xFFU);
      resp[6] = static_cast<uint8_t>((gear_x100 >> 8) & 0xFFU);
      resp[7] = sign_bits;
      (void)can_port_.tx(frame.bus, 0x7A2U, std::span<const uint8_t>(resp, sizeof(resp)),
                        platform::OpNone());
      continue;
    }
    motor_mgr_.route_frame(frame);
  }
  if (wheels_rx_seen) last_wheels_fb_ts_us_ = ts_us;

  // Mirror feedback into sensors for the algorithm.
  sensors_.wheels = dji_group_.state();
  for (std::size_t i = 0; i < dm_motors_.size() && i < sensors_.joints.size(); ++i)
  {
    const auto &st = dm_motors_[i].state();
    sensors_.joints[i] = st;
    g_robotapp_dm_pos[i] = st.pos;
    g_robotapp_dm_vel[i] = st.vel;
    g_robotapp_dm_tor[i] = st.torque;
  }

  // 将当前反馈同步给逻辑层（100 Hz 将读取）。
  domain::MotorFeedback fb{};
  fb.wheels = sensors_.wheels;
  fb.joints = sensors_.joints;
  // Only mark wheel feedback timestamp as "fresh" when we've actually received CAN frames
  // for the wheel bus; otherwise odometry/velocity correction should be considered stale.
  fb.ts_us = last_wheels_fb_ts_us_;
  RobotApp_UpdateMotorFeedback(&fb);

  // Consume command snapshot from logic layer:
  // - Prefer actuator command (cmd_valid).
  // - Else use high-level command (hl_valid) and run controller stub to get actuator outputs.
  // - If stale/invalid -> output safe zeros.
  domain::ChassisCommand cmd{};
  bool cmd_ok = false;
  uint64_t cmd_ts_us = 0;
  domain::HighLevelCommand hl{};
  bool hl_ok = false;
  bool used_direct_cmd = false;
  bool used_hl = false;
  if (logic_bus_ != nullptr)
  {
    const auto snap = logic_bus_->latest();
    cmd = snap.cmd;
    cmd_ok = snap.cmd_valid;
    cmd_ts_us = snap.ts_us;
    hl = snap.hl_cmd;
    hl_ok = snap.hl_valid;
    if (cmd_ts_us == 0U || (ts_us - cmd_ts_us > domain::config::kLogicCmdTimeoutUs))
    {
      if (cmd_ok || hl_ok) telemetry_.logic_cmd_timeout_count += 1U;
      cmd_ok = false;
      hl_ok = false;
    }
    if (cmd_ok) used_direct_cmd = true;
  }
  telemetry_.logic_cmd_age_us = age_us(cmd_ts_us);

  if (RobotApp_FactoryResetInProgress() != 0U)
  {
    cmd_ok = false;
    hl_ok = false;
  }

  // Safety: evaluate against the freshest possible sources (avoid 1-tick lag through SystemHealth).
  auto health_for_safety = RobotApp_SystemHealthLatest();
  health_for_safety.operator_state = RobotApp_OperatorLatest();
  const auto remote_state = RobotApp_RemoteLatest();
  const auto remote_diag = RobotApp_RemoteDiagLatest();
  health_for_safety.remote_frames = remote_diag.frames;
  health_for_safety.remote_bad_frames = remote_diag.bad_frame;
  health_for_safety.remote_frame_lost = remote_state.frame_lost;
  health_for_safety.remote_failsafe = remote_state.failsafe;
  health_for_safety.remote_last_ts_us = remote_state.ts_us;
  telemetry_.remote_age_us = age_us(remote_state.ts_us);
  health_for_safety.sbus_pipe_dropped = static_cast<uint32_t>(g_app_sbus_pipe_dropped);
  health_for_safety.can1_rx_dropped = static_cast<uint32_t>(g_app_can1_rx_dropped);
  health_for_safety.can2_rx_dropped = static_cast<uint32_t>(g_app_can2_rx_dropped);
  health_for_safety.can1_tx_mailbox_full = static_cast<uint32_t>(g_app_can1_tx_mailbox_full);
  health_for_safety.can2_tx_mailbox_full = static_cast<uint32_t>(g_app_can2_tx_mailbox_full);
  health_for_safety.can1_tx_add_fail = static_cast<uint32_t>(g_app_can1_tx_add_fail);
  health_for_safety.can2_tx_add_fail = static_cast<uint32_t>(g_app_can2_tx_add_fail);
  health_for_safety.can1_tx_ok = g_robotapp_can1_tx_ok;
  health_for_safety.can2_tx_ok = g_robotapp_can2_tx_ok;
  health_for_safety.can1_tx_fail = g_robotapp_can1_tx_fail;
  health_for_safety.can2_tx_fail = g_robotapp_can2_tx_fail;
  health_for_safety.can1_tx_start_fail = g_robotapp_can1_tx_start_fail;
  health_for_safety.can2_tx_start_fail = g_robotapp_can2_tx_start_fail;
  health_for_safety.can1_tx_cplt_fail = g_robotapp_can1_tx_cplt_fail;
  health_for_safety.can2_tx_cplt_fail = g_robotapp_can2_tx_cplt_fail;
  health_for_safety.spi1_bmi088_txrx_fail = static_cast<uint32_t>(g_app_spi1_bmi088_txrx_fail);
  health_for_safety.i2c3_fail = static_cast<uint32_t>(g_app_i2c3_fail);
  health_for_safety.i2c3_abort = static_cast<uint32_t>(g_app_i2c3_abort);

  const auto safety_cfg = RobotApp_SafetyConfigLatest();
  const auto decision = safety_.evaluate(health_for_safety, safety_cfg, ts_us);
  g_robotapp_safety_flags = decision.flags;
  if (decision.global_stop(safety_cfg))
  {
    telemetry_.safety_global_stop_count += 1U;
    cmd_ok = false;
    hl_ok = false;
  }

  if (!cmd_ok && hl_ok)
  {
    const auto op = RobotApp_OperatorLatest();
    const auto estimate = RobotApp_StateEstimateLatest();
    const auto wheel_odom_cfg = RobotApp_WheelOdomConfigLatest();
    const auto wheel_ctrl_cfg = RobotApp_WheelControllerConfigLatest();
    const algo::ChassisControllerInput cin{
        .ts_us = ts_us,
        .dt_us = dt_us,
        .sensors = &sensor_snap,
        .estimate = &estimate,
        .wheel_odom_cfg = &wheel_odom_cfg,
        .wheel_ctrl_cfg = &wheel_ctrl_cfg,
        .operator_state = &op,
        .hl_cmd = &hl,
    };
    const auto controller = controller_.load(std::memory_order_relaxed);
    const auto cout = controller != nullptr ? controller(cin) : algo::chassis_controller_step(cin);
    if (cout.cmd_valid)
    {
      cmd = cout.cmd;
      cmd_ok = true;
      used_hl = true;
    }
    else
    {
      telemetry_.controller_invalid_count += 1U;
      hl_ok = false;
    }
  }

  if (!cmd_ok && !hl_ok)
  {
    cmd = {};  // safe zero outputs
    telemetry_.zero_output_count += 1U;
  }
  else
  {
    // If a CAN bus is dead, zero only the actuators on that bus.
    if (decision.wheels_stop(safety_cfg))
    {
      telemetry_.wheels_stop_count += 1U;
      for (auto& w : cmd.wheels) w.current = 0;
    }
    if (decision.joints_stop(safety_cfg))
    {
      telemetry_.joints_stop_count += 1U;
      for (auto& j : cmd.joints) j = {};
    }
  }

  if (cmd_ok)
  {
    if (used_hl)
    {
      telemetry_.hl_used_count += 1U;
    }
    else if (used_direct_cmd)
    {
      telemetry_.cmd_used_count += 1U;
    }
  }
  telemetry_.last_cmd_source = (!cmd_ok)        ? 0U
                             : (used_hl)        ? 2U
                             : (used_direct_cmd) ? 1U
                                                : 0U;

  for (std::size_t i = 0; i < telemetry_.wheel_current_error_mA.size() && i < cmd.wheels.size() &&
                          i < sensors_.wheels.size();
       ++i)
  {
    telemetry_.wheel_current_error_mA[i] =
        static_cast<int32_t>(cmd.wheels[i].current) - static_cast<int32_t>(sensors_.wheels[i].current);
  }

  last_cmd_ts_us_ = ts_us;
  last_cmd_valid_ = cmd_ok;
  last_cmd_ = cmd;

  for (std::size_t i = 0; i < sensors_.wheels.size() && i < cmd.wheels.size(); ++i)
  {
    dji_group_.set_current(i, cmd.wheels[i].current);
  }
  for (std::size_t i = 0; i < dm_motors_.size() && i < cmd.joints.size(); ++i)
  {
    dm_motors_[i].set_command(cmd.joints[i]);
  }

  motor_mgr_.tick_all(ts_us);
}

}  // namespace robotapp::tasks
