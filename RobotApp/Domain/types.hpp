#pragma once

#include <array>
#include <cstdint>

namespace robotapp::domain {

struct WheelCommand {
  int16_t current = 0;  // mA to C620
};

enum class JointMode : uint8_t { Torque = 0, Velocity = 1, Position = 2, Impedance = 3 };

struct JointCommand {
  JointMode mode = JointMode::Torque;
  float pos = 0.0f;
  float vel = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;
  float tau = 0.0f;  // desired torque
};

struct ImuSample {
  int16_t accel_x = 0;
  int16_t accel_y = 0;
  int16_t accel_z = 0;
  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;
  uint64_t ts_us = 0;
};

struct MagSample {
  int16_t mx = 0;
  int16_t my = 0;
  int16_t mz = 0;
  uint64_t ts_us = 0;
};

struct WheelState {
  int32_t encoder = 0;
  int16_t rpm = 0;
  int16_t current = 0;
  uint8_t temperature = 0;
};

struct JointState {
  float pos = 0.0f;
  float vel = 0.0f;
  float torque = 0.0f;
};

struct RemoteState {
  float ch[8] = {0};      // normalized -1..1
  uint8_t sw[4] = {0};    // switch states
  bool frame_lost = false;
  bool failsafe = false;  // signal lost
  uint64_t ts_us = 0;
};

struct RemoteDiagnostics {
  uint32_t frames = 0;
  uint32_t frame_lost = 0;
  uint32_t failsafe = 0;
  uint32_t bad_frame = 0;
  uint32_t sbus_pipe_dropped = 0;
  uint64_t last_ts_us = 0;
};

struct SafetyConfig {
  uint8_t stop_on_sbus_pipe_overflow = 0;
  uint8_t stop_on_can_rx_overflow = 0;
  uint8_t stop_on_can_tx_congestion = 0;
  uint8_t stop_on_spi_comm_error = 0;
  uint8_t stop_on_i2c_comm_error = 0;
  uint8_t stop_on_remote_bad_frame_burst = 0;
  uint8_t reserved0 = 0;
  uint8_t reserved1 = 0;

  uint32_t remote_quality_hold_us = 0;
  uint32_t comm_error_hold_us = 0;
  uint32_t can_tx_ok_timeout_us = 0;

  uint32_t remote_bad_frame_burst_window_us = 0;
  uint32_t remote_bad_frame_burst_threshold = 0;
};

enum class OperatorMode : uint8_t {
  Safe = 0,
  Manual = 1,
  Auto = 2,
};

struct OperatorState {
  bool enabled = false;         // effective master enable/arm (after latch logic)
  bool e_stop = false;          // effective emergency stop (latched)
  bool enable_request = false;  // raw request from switch mapping (debounced)
  bool e_stop_request = false;  // raw request from switch mapping (debounced)
  bool e_stop_latched = false;
  OperatorMode mode = OperatorMode::Safe;
  uint8_t source_sw[2] = {0, 0};  // raw stable switch values used for mapping (debug)
  uint64_t ts_us = 0;
};

struct SystemHealth {
  uint64_t ts_us = 0;

  uint32_t sbus_pipe_dropped = 0;
  uint32_t remote_frames = 0;
  uint32_t remote_bad_frames = 0;
  bool remote_frame_lost = false;
  bool remote_failsafe = false;

  OperatorState operator_state{};

  uint32_t can1_rx_dropped = 0;
  uint32_t can2_rx_dropped = 0;
  uint32_t can1_tx_mailbox_full = 0;
  uint32_t can2_tx_mailbox_full = 0;
  uint32_t can1_tx_add_fail = 0;
  uint32_t can2_tx_add_fail = 0;
  uint32_t can1_tx_ok = 0;
  uint32_t can2_tx_ok = 0;
  uint32_t can1_tx_fail = 0;
  uint32_t can2_tx_fail = 0;
  uint32_t can1_tx_start_fail = 0;
  uint32_t can2_tx_start_fail = 0;
  uint32_t can1_tx_cplt_fail = 0;
  uint32_t can2_tx_cplt_fail = 0;

  uint32_t spi1_bmi088_txrx_fail = 0;
  uint32_t i2c3_fail = 0;
  uint32_t i2c3_abort = 0;

  // SafetyPolicy flags (RobotApp/Safety/safety_policy.hpp):
  // bit0=remote stale, bit1=imu stale, bit2=mag stale, bit3=can1 dead, bit4=can2 dead,
  // bit5=operator disabled, bit6=e-stop, bit7=remote frame lost, bit8=remote failsafe, ...
  uint32_t safety_flags = 0;

  uint64_t remote_last_ts_us = 0;
  uint64_t imu_last_ts_us = 0;
  uint64_t mag_last_ts_us = 0;
  uint64_t logic_cmd_last_ts_us = 0;
};

struct ImuState {
  float accel_mps2[3] = {0};
  float gyro_dps[3] = {0};
  uint64_t ts_us = 0;
};

struct ImuCalibration {
  float accel_scale[3] = {1.0f, 1.0f, 1.0f};
  float accel_bias[3] = {0.0f, 0.0f, 0.0f};
  float gyro_scale[3] = {1.0f, 1.0f, 1.0f};
  float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
};

struct MagState {
  float mag_uT[3] = {0};
  uint64_t ts_us = 0;
};

struct MagCalibration {
  float scale[3] = {1.0f, 1.0f, 1.0f};
  float bias[3] = {0.0f, 0.0f, 0.0f};
};

// Runtime-tunable chassis parameters (measured on the real robot).
// These are intentionally plain data (POD) so they can be persisted and live-tuned.
struct WheelOdomConfig {
  float wheel_radius_m = 0.06f;
  float wheel_track_m = 0.30f;
  float wheel_gear_ratio = 1.0f;
  int8_t rpm_sign_left = 1;
  int8_t rpm_sign_right = 1;
  uint16_t reserved0 = 0;
};

struct WheelControllerConfig {
  float max_vx_mps = 1.5f;                 // hl_cmd.vx in [-1,1] -> m/s
  float max_wz_rps = 3.0f;                 // hl_cmd.wz in [-1,1] -> rad/s
  float wheel_vel_kp_mA_per_mps = 3000.0f; // velocity error -> current
  int16_t wheel_current_limit_mA = 12000;
  int16_t reserved0 = 0;
};

// State estimate produced by the fusion/filter layer (orientation + dead-reckoned velocity/position).
// NOTE: With only IMU(+mag), velocity/position will drift without external references (odometry/vision/etc.).
struct StateEstimate {
  // World-from-body quaternion (w, x, y, z).
  float q_wxyz[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  // Euler angles in radians (roll, pitch, yaw) derived from q (for convenience/debug).
  float rpy_rad[3] = {0.0f, 0.0f, 0.0f};

  float pos_m[3] = {0.0f, 0.0f, 0.0f};
  float vel_mps[3] = {0.0f, 0.0f, 0.0f};

  float gyro_bias_rps[3] = {0.0f, 0.0f, 0.0f};
  float accel_bias_mps2[3] = {0.0f, 0.0f, 0.0f};

  uint64_t ts_us = 0;
  uint8_t valid = 0;
  uint8_t reserved0 = 0;
  uint16_t reserved1 = 0;
};

// LQR-friendly balancing state (derived from IMU + wheel encoder).
struct LqrState {
  float theta_rad = 0.0f;      // body pitch angle (rad)
  float theta_dot_rps = 0.0f;  // body pitch rate (rad/s)
  float x_m = 0.0f;            // wheel position along ground (m)
  float x_dot_mps = 0.0f;      // wheel velocity along ground (m/s)
  uint64_t ts_us = 0;
  uint8_t valid = 0;
  uint8_t reserved0 = 0;
  uint16_t reserved1 = 0;
};

struct SensorDiagnostics {
  uint32_t ist_i2c_started = 0;
  uint32_t ist_i2c_ok = 0;
  uint32_t ist_i2c_start_fail = 0;
  uint32_t ist_i2c_cplt_fail = 0;
  uint32_t ist_i2c_backoff_entered = 0;
  uint32_t ist_i2c_backoff_skipped = 0;
};

struct ParamDiagnostics {
  uint32_t load_ok = 0;
  uint32_t load_fail = 0;
  uint32_t save_ok = 0;
  uint32_t save_fail = 0;
  uint32_t erase_ok = 0;
  uint32_t erase_fail = 0;
  uint32_t region_invalid = 0;
  uint32_t factory_reset_requested = 0;
  uint32_t factory_reset_ok = 0;
  uint32_t factory_reset_fail = 0;
  uint8_t loaded_from_flash = 0;
  uint8_t reserved0 = 0;
  uint16_t reserved1 = 0;
};

struct MotorFeedback {
  std::array<WheelState, 2> wheels{};
  std::array<JointState, 4> joints{};
  uint64_t ts_us = 0;
};

struct SensorsSnapshot {
  ImuState imu{};
  MagState mag{};
  MotorFeedback motor{};
  uint64_t ts_us = 0;
};

struct HighLevelCommand {
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
};

struct ChassisSensors {
  ImuSample imu{};
  std::array<WheelState, 2> wheels{};
  std::array<JointState, 4> joints{};
};

struct ChassisCommand {
  std::array<WheelCommand, 2> wheels{};
  std::array<JointCommand, 4> joints{};
};

struct ControlTelemetry {
  uint64_t ts_us = 0;
  uint32_t tick_count = 0;
  uint32_t last_dt_us = 0;
  int32_t last_jitter_us = 0;
  int32_t jitter_min_us = 0;
  int32_t jitter_max_us = 0;
  uint32_t abs_jitter_max_us = 0;
  uint32_t logic_cmd_timeout_count = 0;
  uint32_t cmd_used_count = 0;
  uint32_t hl_used_count = 0;
  uint32_t controller_invalid_count = 0;
  uint32_t safety_global_stop_count = 0;
  uint32_t wheels_stop_count = 0;
  uint32_t joints_stop_count = 0;
  uint32_t zero_output_count = 0;
  std::array<int32_t, 2> wheel_current_error_mA{};
  uint32_t logic_cmd_age_us = 0;
  uint32_t remote_age_us = 0;
  uint32_t imu_age_us = 0;
  uint32_t mag_age_us = 0;
  uint8_t last_cmd_source = 0;  // 0=none, 1=cmd_valid, 2=hl_valid(controller)
  uint8_t reserved0 = 0;
  uint16_t reserved1 = 0;
};

struct DiagnosticsSnapshot {
  uint64_t ts_us = 0;
  OperatorState operator_state{};
  RemoteState remote{};
  RemoteDiagnostics remote_diag{};
  SensorsSnapshot sensors{};
  StateEstimate estimate{};
  LqrState lqr{};

  ChassisCommand actuator_cmd{};
  uint8_t actuator_cmd_valid = 0;
  uint8_t reserved0 = 0;
  uint16_t reserved1 = 0;
  uint64_t actuator_cmd_ts_us = 0;

  SafetyConfig safety_cfg{};
  ImuCalibration imu_calib{};
  MagCalibration mag_calib{};
  WheelOdomConfig wheel_odom_cfg{};
  WheelControllerConfig wheel_ctrl_cfg{};
  SensorDiagnostics sensor_diag{};
  ParamDiagnostics params{};
  SystemHealth health{};
  ControlTelemetry control{};
};

}  // namespace robotapp::domain
