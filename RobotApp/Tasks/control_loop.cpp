#include "RobotApp/Tasks/control_loop.hpp"

#include "RobotApp/Domain/config.hpp"
#include "RobotApp/Domain/remote_config.hpp"
#include "RobotApp/Estimation/kalman_1d.hpp"

#include <span>
#include <cmath>

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
volatile uint16_t g_robotapp_dm_tx_last_std_id = 0;
volatile uint32_t g_robotapp_dm_tx_attempt[4] = {0, 0, 0, 0};
volatile uint32_t g_robotapp_dm_tx_started[4] = {0, 0, 0, 0};
volatile uint16_t g_robotapp_dm_enable_tx_last_std_id = 0;
volatile uint32_t g_robotapp_dm_enable_tx_attempt[4] = {0, 0, 0, 0};
volatile uint32_t g_robotapp_dm_enable_tx_started[4] = {0, 0, 0, 0};
volatile uint8_t g_robotapp_dm_bringup_index = 0;
volatile uint8_t g_robotapp_dm_bringup_done = 0;
volatile uint8_t g_robotapp_dm_bringup_step[4] = {0};
volatile uint32_t g_robotapp_safety_flags = 0;
}

namespace robotapp::tasks {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr int32_t kDjiEncMod = 8192;
constexpr int32_t kDjiEncHalf = kDjiEncMod / 2;
constexpr std::size_t kDmTxPerTick = 2;

struct EstimateFilter {
  std::array<estimation::Kalman1D, 3> rpy{};
  std::array<estimation::Kalman1D, 3> vel{};
  std::array<estimation::Kalman1D, 3> pos{};
  uint64_t last_est_ts_us = 0;
  uint64_t last_tx_ts_us = 0;

  void reset()
  {
    for (auto& k : rpy) k.reset();
    for (auto& k : vel) k.reset();
    for (auto& k : pos) k.reset();
    last_est_ts_us = 0;
    last_tx_ts_us = 0;

    // Slightly different tunings per signal group.
    for (auto& k : rpy) k.set_tunings(0.001f, 0.05f);
    for (auto& k : vel) k.set_tunings(0.01f, 0.10f);
    for (auto& k : pos) k.set_tunings(0.001f, 0.50f);
  }
};

EstimateFilter g_est_filter{};

inline int16_t clamp_i16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

inline int16_t float_to_i16(float v, float scale)
{
  const float scaled = v * scale;
  const int32_t rounded = (scaled >= 0.0f) ? static_cast<int32_t>(scaled + 0.5f)
                                           : static_cast<int32_t>(scaled - 0.5f);
  return clamp_i16(rounded);
}

inline void pack_i16(int16_t v, uint8_t* out)
{
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
}

static domain::StateEstimate filter_estimate(const domain::StateEstimate& est)
{
  domain::StateEstimate out = est;
  if (est.ts_us == 0U) return out;

  float dt_s = 0.0f;
  if (g_est_filter.last_est_ts_us != 0U && est.ts_us >= g_est_filter.last_est_ts_us)
  {
    dt_s = static_cast<float>(est.ts_us - g_est_filter.last_est_ts_us) * 1.0e-6f;
  }
  g_est_filter.last_est_ts_us = est.ts_us;

  if (est.valid != 0U && dt_s > 0.0f && dt_s < 0.1f)
  {
    for (int i = 0; i < 3; ++i)
    {
      out.rpy_rad[i] = g_est_filter.rpy[i].update(est.rpy_rad[i], dt_s);
      out.vel_mps[i] = g_est_filter.vel[i].update(est.vel_mps[i], dt_s);
      out.pos_m[i] = g_est_filter.pos[i].update(est.pos_m[i], dt_s);
    }
  }
  else if (est.valid == 0U)
  {
    // Reset if the upstream estimator becomes invalid.
    g_est_filter.reset();
  }
  return out;
}

static void send_estimate_telemetry(platform::CanPort& can_port,
                                    uint64_t now_us,
                                    const domain::StateEstimate& est)
{
  if (now_us == 0U) return;
  if (g_est_filter.last_tx_ts_us != 0U &&
      (now_us - g_est_filter.last_tx_ts_us < domain::config::kEstimateTxPeriodUs))
  {
    return;
  }
  if (est.ts_us == 0U) return;

  uint8_t payload[8] = {0};

  // RPY (rad): roll, pitch, yaw
  pack_i16(float_to_i16(est.rpy_rad[0], domain::config::kEstimateRpyScale), &payload[0]);
  pack_i16(float_to_i16(est.rpy_rad[1], domain::config::kEstimateRpyScale), &payload[2]);
  pack_i16(float_to_i16(est.rpy_rad[2], domain::config::kEstimateRpyScale), &payload[4]);
  payload[6] = static_cast<uint8_t>(est.valid ? 1U : 0U);
  payload[7] = 0U;
  (void)can_port.tx(domain::config::kEstimateTxBus, domain::config::kEstimateTxRpyId,
                    std::span<const uint8_t>(payload, sizeof(payload)), platform::OpNone());

  // Velocity (m/s)
  pack_i16(float_to_i16(est.vel_mps[0], domain::config::kEstimateVelScale), &payload[0]);
  pack_i16(float_to_i16(est.vel_mps[1], domain::config::kEstimateVelScale), &payload[2]);
  pack_i16(float_to_i16(est.vel_mps[2], domain::config::kEstimateVelScale), &payload[4]);
  payload[6] = static_cast<uint8_t>(est.valid ? 1U : 0U);
  payload[7] = 0U;
  (void)can_port.tx(domain::config::kEstimateTxBus, domain::config::kEstimateTxVelId,
                    std::span<const uint8_t>(payload, sizeof(payload)), platform::OpNone());

  // Position (m)
  pack_i16(float_to_i16(est.pos_m[0], domain::config::kEstimatePosScale), &payload[0]);
  pack_i16(float_to_i16(est.pos_m[1], domain::config::kEstimatePosScale), &payload[2]);
  pack_i16(float_to_i16(est.pos_m[2], domain::config::kEstimatePosScale), &payload[4]);
  payload[6] = static_cast<uint8_t>(est.valid ? 1U : 0U);
  payload[7] = 0U;
  (void)can_port.tx(domain::config::kEstimateTxBus, domain::config::kEstimateTxPosId,
                    std::span<const uint8_t>(payload, sizeof(payload)), platform::OpNone());

  g_est_filter.last_tx_ts_us = now_us;
}

}  // namespace

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
  last_est_filtered_ = {};
  lqr_state_ = {};
  wheel_odom_ = {};
  dm_bringup_index_ = 0;
  dm_tx_index_ = 0;
  dm_bringup_done_ = false;
  g_est_filter.reset();

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
  const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;
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
    g_robotapp_dm_bringup_step[i] = dm_motors_[i].bringup_step();
  }

  const auto wheel_odom_cfg = RobotApp_WheelOdomConfigLatest();
  const float ticks_to_m =
      (wheel_odom_cfg.wheel_radius_m > 0.0f && wheel_odom_cfg.wheel_gear_ratio > 0.0f)
          ? (2.0f * kPi * wheel_odom_cfg.wheel_radius_m) /
                (wheel_odom_cfg.wheel_gear_ratio * kDjiEncMod)
          : 0.0f;
  float wheel_pos_m[2] = {0.0f, 0.0f};
  float wheel_vel_mps[2] = {wheel_odom_[0].vel_mps, wheel_odom_[1].vel_mps};

  if (wheels_rx_seen && ticks_to_m > 0.0f)
  {
    for (std::size_t i = 0; i < sensors_.wheels.size() && i < wheel_odom_.size(); ++i)
    {
      auto& ws = wheel_odom_[i];
      const uint16_t enc = static_cast<uint16_t>(sensors_.wheels[i].encoder);
      if (!ws.valid)
      {
        ws.last_enc = enc;
        ws.valid = true;
      }
      else
      {
        int32_t diff = static_cast<int32_t>(enc) - static_cast<int32_t>(ws.last_enc);
        if (diff > kDjiEncHalf) diff -= kDjiEncMod;
        if (diff < -kDjiEncHalf) diff += kDjiEncMod;
        ws.ticks += diff;
        ws.last_enc = enc;
      }

      const float sign = (i == 0) ? static_cast<float>(wheel_odom_cfg.rpm_sign_left)
                                  : static_cast<float>(wheel_odom_cfg.rpm_sign_right);
      wheel_pos_m[i] = sign * static_cast<float>(ws.ticks) * ticks_to_m;

      const float rpm = static_cast<float>(sensors_.wheels[i].rpm) * sign;
      const float vel_meas = (wheel_odom_cfg.wheel_radius_m > 0.0f &&
                              wheel_odom_cfg.wheel_gear_ratio > 0.0f)
                                 ? (rpm * 2.0f * kPi / 60.0f) *
                                       (wheel_odom_cfg.wheel_radius_m /
                                        wheel_odom_cfg.wheel_gear_ratio)
                                 : 0.0f;
      if (dt_s > 0.0f && dt_s < 0.5f)
      {
        const float tau = 0.05f;
        const float alpha = dt_s / (tau + dt_s);
        ws.vel_mps = ws.vel_mps + alpha * (vel_meas - ws.vel_mps);
      }
      else
      {
        ws.vel_mps = vel_meas;
      }
      wheel_vel_mps[i] = ws.vel_mps;
    }
  }
  else
  {
    for (std::size_t i = 0; i < wheel_odom_.size(); ++i)
    {
      const float sign = (i == 0) ? static_cast<float>(wheel_odom_cfg.rpm_sign_left)
                                  : static_cast<float>(wheel_odom_cfg.rpm_sign_right);
      wheel_pos_m[i] = sign * static_cast<float>(wheel_odom_[i].ticks) * ticks_to_m;
      wheel_vel_mps[i] = wheel_odom_[i].vel_mps;
    }
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
  const auto op_state = RobotApp_OperatorLatest();
  auto health_for_safety = RobotApp_SystemHealthLatest();
  health_for_safety.operator_state = op_state;
  const auto remote_state = RobotApp_RemoteLatest();
  const auto remote_diag = RobotApp_RemoteDiagLatest();
  const bool bypass_safety =
      domain::config::kBypassSafetyPolicy ||
      (remote_state.sw[domain::remote::kDebugBypassSafetySwitchIdx] ==
       domain::remote::kDebugBypassSafetySwitchOn);
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
  if (!bypass_safety)
  {
    if (decision.global_stop(safety_cfg))
    {
      telemetry_.safety_global_stop_count += 1U;
      cmd_ok = false;
      hl_ok = false;
    }
  }

  const auto raw_est = RobotApp_StateEstimateLatest();
  const auto filtered_est = filter_estimate(raw_est);
  last_est_filtered_ = filtered_est;

  const bool imu_fresh =
      (sensor_snap.imu.ts_us != 0U) &&
      (ts_us - sensor_snap.imu.ts_us <= domain::config::kImuTimeoutUs);
  const bool wheel_fresh =
      (last_wheels_fb_ts_us_ != 0U) &&
      (ts_us - last_wheels_fb_ts_us_ <= domain::config::kOdomTimeoutUs);
  lqr_state_.theta_rad = filtered_est.rpy_rad[1];
  lqr_state_.theta_dot_rps = sensor_snap.imu.gyro_dps[1] * kDegToRad;
  lqr_state_.x_m = 0.5f * (wheel_pos_m[0] + wheel_pos_m[1]);
  lqr_state_.x_dot_mps = 0.5f * (wheel_vel_mps[0] + wheel_vel_mps[1]);
  lqr_state_.ts_us = ts_us;
  lqr_state_.valid = (filtered_est.valid != 0U && imu_fresh && wheel_fresh) ? 1U : 0U;

  const bool lqr_enabled = (op_state.mode == domain::OperatorMode::Auto);

  if (!cmd_ok && hl_ok)
  {
    const auto wheel_ctrl_cfg = RobotApp_WheelControllerConfigLatest();
    const algo::ChassisControllerInput cin{
        .ts_us = ts_us,
        .dt_us = dt_us,
        .sensors = &sensor_snap,
        .estimate = &filtered_est,
        .lqr = &lqr_state_,
        .wheel_odom_cfg = &wheel_odom_cfg,
        .wheel_ctrl_cfg = &wheel_ctrl_cfg,
        .operator_state = &op_state,
        .hl_cmd = &hl,
    };
    const auto controller =
        lqr_enabled ? controller_.load(std::memory_order_relaxed) : nullptr;
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
    if (!bypass_safety)
    {
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

  const uint8_t joint_sw = remote_state.sw[domain::remote::kJointHoldSwitchIdx];
  const bool joint_hold =
      op_state.enabled && !op_state.e_stop &&
      (joint_sw == domain::remote::kJointHoldSwitchOn) &&
      (bypass_safety || !decision.joints_stop(safety_cfg));
  if (joint_hold)
  {
    if (!joint_hold_active_)
    {
      joint_hold_active_ = true;
      joint_hold_start_us_ = ts_us;
    }
  }
  else
  {
    joint_hold_active_ = false;
    joint_hold_start_us_ = 0;
  }
  if (joint_hold)
  {
    const bool in_soft_start =
        (joint_hold_start_us_ != 0U) && (ts_us - joint_hold_start_us_ < 2'000'000U);
    const float kp =
        in_soft_start ? (domain::config::kJointHoldKp * 0.5f) : domain::config::kJointHoldKp;
    for (std::size_t i = 0; i < cmd.joints.size() && i < sensors_.joints.size(); ++i)
    {
      cmd.joints[i].mode = domain::JointMode::Position;
      cmd.joints[i].kp = kp;
      cmd.joints[i].kd = domain::config::kJointHoldKd;
      cmd.joints[i].vel = 0.0f;
      cmd.joints[i].tau = 0.0f;
      float target = (i < 4U) ? g_robotapp_joint_hold_pos[i] : 0.0f;
      if (!std::isfinite(target)) target = 0.0f;
      cmd.joints[i].pos = target;
    }
  }

  // Apply wheel direction mapping so positive command means "forward".
  if (!cmd.wheels.empty())
  {
    const int8_t left_sign = (wheel_odom_cfg.rpm_sign_left < 0) ? -1 : 1;
    const int8_t right_sign = (wheel_odom_cfg.rpm_sign_right < 0) ? -1 : 1;
    cmd.wheels[0].current =
        clamp_i16(static_cast<int32_t>(cmd.wheels[0].current) * static_cast<int32_t>(left_sign));
    if (cmd.wheels.size() > 1U)
    {
      cmd.wheels[1].current =
          clamp_i16(static_cast<int32_t>(cmd.wheels[1].current) * static_cast<int32_t>(right_sign));
    }
  }

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

  // DM4310 bringup: enable motors one-by-one to reduce bus contention.
  if (!dm_bringup_done_)
  {
    while (dm_bringup_index_ < dm_motors_.size() &&
           dm_motors_[dm_bringup_index_].bringup_done())
    {
      dm_bringup_index_++;
    }
    if (dm_bringup_index_ >= dm_motors_.size())
    {
      dm_bringup_done_ = true;
    }
  }

  dji_group_.tick(ts_us);

  // Stagger DM4310 CAN commands: send only one motor per control tick.
  if (!dm_motors_.empty())
  {
    if (dm_bringup_done_)
    {
      for (std::size_t n = 0; n < kDmTxPerTick; ++n)
      {
        if (dm_tx_index_ >= dm_motors_.size()) dm_tx_index_ = 0;
        dm_motors_[dm_tx_index_].tick(ts_us);
        dm_tx_index_++;
      }
      if (dm_tx_index_ >= dm_motors_.size()) dm_tx_index_ = 0;
    }
    else if (dm_bringup_index_ < dm_motors_.size())
    {
      dm_motors_[dm_bringup_index_].tick(ts_us);
      if (dm_motors_[dm_bringup_index_].bringup_done())
      {
        dm_bringup_index_++;
        if (dm_bringup_index_ >= dm_motors_.size()) dm_bringup_done_ = true;
      }
    }
  }

  g_robotapp_dm_bringup_index =
      (dm_bringup_index_ < dm_motors_.size()) ? static_cast<uint8_t>(dm_bringup_index_) : 0xFFU;
  g_robotapp_dm_bringup_done = dm_bringup_done_ ? 1U : 0U;

  // Send estimator telemetry (Kalman-smoothed) on CAN.
  send_estimate_telemetry(can_port_, ts_us, filtered_est);
}

}  // namespace robotapp::tasks
