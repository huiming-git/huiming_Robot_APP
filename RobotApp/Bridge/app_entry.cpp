#include "app_bridge.h"

#include <atomic>
#include <cmath>

#include "RobotApp/Tasks/control_loop.hpp"
#include "RobotApp/Logic/logic_bus.hpp"
#include "RobotApp/Logic/logic_context.hpp"
#include "RobotApp/Inputs/remote_adapter.hpp"
#include "RobotApp/Inputs/sbus.hpp"
#include "RobotApp/Drivers/imu_adapter.hpp"
#include "RobotApp/Drivers/mag_adapter.hpp"
#include "RobotApp/Drivers/ist8310.hpp"
#include "RobotApp/Estimation/ins_estimator.hpp"
#include "RobotApp/Platform/heap_guard.hpp"
#include "RobotApp/Util/double_buffer.hpp"

extern "C" {
#include "app_sbus.h"
#include "app_can.h"
#include "app_spi.h"
#include "app_i2c.h"
#include "app_params.h"
}

namespace robotapp {

static tasks::ControlLoop g_control_loop;
static logic::LogicBus g_logic_bus;
static logic::LogicContext g_logic_ctx{};
static inputs::RemoteAdapter g_remote{};
static drivers::ImuAdapter g_imu{};
static drivers::MagAdapter g_mag{};
static estimation::InsEstimator g_estimator{};
static inputs::SbusParser g_sbus_parser{};
static util::DoubleBuffer<domain::MotorFeedback> g_logic_motor_fb{};
static util::DoubleBuffer<App_RemoteState> g_remote_state{};
static util::DoubleBuffer<domain::OperatorState> g_operator_state{};
static util::DoubleBuffer<domain::RemoteDiagnostics> g_remote_diag{};
static util::DoubleBuffer<domain::SystemHealth> g_sys_health{};
static util::DoubleBuffer<domain::SensorsSnapshot> g_sensors{};
static util::DoubleBuffer<domain::StateEstimate> g_estimate{};
static util::DoubleBuffer<domain::DiagnosticsSnapshot> g_diag{};
static util::DoubleBuffer<domain::WheelOdomConfig> g_wheel_odom_cfg{};
static util::DoubleBuffer<domain::WheelControllerConfig> g_wheel_ctrl_cfg{};

static std::atomic<bool> g_params_suppress_save{false};
static std::atomic<uint32_t> g_factory_reset_requested{0};
static std::atomic<bool> g_factory_reset_in_progress{false};

static constexpr bool kLogicDebugInject = false;
static uint32_t g_logic_debug_tick = 0;

static void set_motor_feedback(const domain::MotorFeedback& fb) { g_logic_motor_fb.push(fb); }
static domain::MotorFeedback get_motor_feedback() { return g_logic_motor_fb.latest(); }
static void set_remote_state(const App_RemoteState& st) { g_remote_state.push(st); }
static App_RemoteState get_remote_state() { return g_remote_state.latest(); }
static void set_operator_state(const domain::OperatorState& st) { g_operator_state.push(st); }
static domain::OperatorState get_operator_state() { return g_operator_state.latest(); }
static void set_remote_diag(const domain::RemoteDiagnostics& d) { g_remote_diag.push(d); }
static domain::RemoteDiagnostics get_remote_diag() { return g_remote_diag.latest(); }
static void set_sys_health(const domain::SystemHealth& h) { g_sys_health.push(h); }
static domain::SystemHealth get_sys_health() { return g_sys_health.latest(); }
static void set_sensors(const domain::SensorsSnapshot& s) { g_sensors.push(s); }
static domain::SensorsSnapshot get_sensors() { return g_sensors.latest(); }
static void set_estimate(const domain::StateEstimate& e) { g_estimate.push(e); }
static domain::StateEstimate get_estimate() { return g_estimate.latest(); }
static void set_diag(const domain::DiagnosticsSnapshot& d) { g_diag.push(d); }
static domain::DiagnosticsSnapshot get_diag() { return g_diag.latest(); }
static void set_wheel_odom_cfg(const domain::WheelOdomConfig& c) { g_wheel_odom_cfg.push(c); }
static domain::WheelOdomConfig get_wheel_odom_cfg() { return g_wheel_odom_cfg.latest(); }
static void set_wheel_ctrl_cfg(const domain::WheelControllerConfig& c) { g_wheel_ctrl_cfg.push(c); }
static domain::WheelControllerConfig get_wheel_ctrl_cfg() { return g_wheel_ctrl_cfg.latest(); }

static void maybe_factory_reset_gesture(const domain::RemoteState& st,
                                        const domain::OperatorState& op,
                                        uint64_t ts_us)
{
  static uint64_t hold_since_us = 0;
  static bool triggered = false;

  if (ts_us == 0U) return;

  const bool armed =
      (st.sw[domain::remote::kFactoryResetArmSwitchIdx] == domain::remote::kFactoryResetArmSwitchOn);
  const bool axes_extreme = (std::abs(st.ch[0]) >= domain::remote::kFactoryResetAxisThreshold) &&
                            (std::abs(st.ch[1]) >= domain::remote::kFactoryResetAxisThreshold) &&
                            (std::abs(st.ch[2]) >= domain::remote::kFactoryResetAxisThreshold) &&
                            (std::abs(st.ch[3]) >= domain::remote::kFactoryResetAxisThreshold);

  const bool gesture = armed && axes_extreme && (!st.failsafe) && (!st.frame_lost) &&
                       (op.mode == domain::OperatorMode::Safe) && op.e_stop_latched &&
                       (!op.enabled);

  if (!gesture)
  {
    hold_since_us = 0;
    triggered = false;
    return;
  }

  if (hold_since_us == 0U) hold_since_us = ts_us;
  if (!triggered && (ts_us - hold_since_us >= domain::remote::kFactoryResetHoldUs))
  {
    g_factory_reset_requested.fetch_add(1U, std::memory_order_relaxed);
    RobotApp_FactoryResetParams();
    triggered = true;
  }
}

static void set_default_runtime_params(void)
{
  // Defaults for runtime-tunable safety policy (can be overridden via persisted params or RobotApp_SetSafetyConfig).
  ::g_robotapp_safety_cfg.stop_on_sbus_pipe_overflow =
      robotapp::domain::config::kStopOnSbusPipeOverflow ? 1U : 0U;
  ::g_robotapp_safety_cfg.stop_on_can_rx_overflow =
      robotapp::domain::config::kStopOnCanRxOverflow ? 1U : 0U;
  ::g_robotapp_safety_cfg.stop_on_can_tx_congestion =
      robotapp::domain::config::kStopOnCanTxCongestion ? 1U : 0U;
  ::g_robotapp_safety_cfg.stop_on_spi_comm_error =
      robotapp::domain::config::kStopOnSpiCommError ? 1U : 0U;
  ::g_robotapp_safety_cfg.stop_on_i2c_comm_error =
      robotapp::domain::config::kStopOnI2cCommError ? 1U : 0U;
  ::g_robotapp_safety_cfg.stop_on_remote_bad_frame_burst =
      robotapp::domain::config::kStopOnRemoteBadFrameBurst ? 1U : 0U;
  ::g_robotapp_safety_cfg.remote_quality_hold_us = robotapp::domain::config::kRemoteQualityHoldUs;
  ::g_robotapp_safety_cfg.comm_error_hold_us = robotapp::domain::config::kCommErrorHoldUs;
  ::g_robotapp_safety_cfg.can_tx_ok_timeout_us = robotapp::domain::config::kCanTxOkTimeoutUs;
  ::g_robotapp_safety_cfg.remote_bad_frame_burst_window_us =
      robotapp::domain::config::kRemoteBadFrameBurstWindowUs;
  ::g_robotapp_safety_cfg.remote_bad_frame_burst_threshold =
      robotapp::domain::config::kRemoteBadFrameBurstThreshold;

  ::g_robotapp_imu_calib = robotapp::domain::ImuCalibration{};
  ::g_robotapp_mag_calib = robotapp::domain::MagCalibration{};
  ::g_robotapp_wheel_odom_cfg = robotapp::domain::WheelOdomConfig{};
  ::g_robotapp_wheel_ctrl_cfg = robotapp::domain::WheelControllerConfig{};
  robotapp::set_wheel_odom_cfg(::g_robotapp_wheel_odom_cfg);
  robotapp::set_wheel_ctrl_cfg(::g_robotapp_wheel_ctrl_cfg);
}

static AppParamsSafetyConfig to_core_safety(const domain::SafetyConfig& in)
{
  AppParamsSafetyConfig out{};
  out.stop_on_sbus_pipe_overflow = in.stop_on_sbus_pipe_overflow;
  out.stop_on_can_rx_overflow = in.stop_on_can_rx_overflow;
  out.stop_on_can_tx_congestion = in.stop_on_can_tx_congestion;
  out.stop_on_spi_comm_error = in.stop_on_spi_comm_error;
  out.stop_on_i2c_comm_error = in.stop_on_i2c_comm_error;
  out.stop_on_remote_bad_frame_burst = in.stop_on_remote_bad_frame_burst;
  out.remote_quality_hold_us = in.remote_quality_hold_us;
  out.comm_error_hold_us = in.comm_error_hold_us;
  out.can_tx_ok_timeout_us = in.can_tx_ok_timeout_us;
  out.remote_bad_frame_burst_window_us = in.remote_bad_frame_burst_window_us;
  out.remote_bad_frame_burst_threshold = in.remote_bad_frame_burst_threshold;
  return out;
}

static domain::SafetyConfig from_core_safety(const AppParamsSafetyConfig& in)
{
  domain::SafetyConfig out{};
  out.stop_on_sbus_pipe_overflow = in.stop_on_sbus_pipe_overflow;
  out.stop_on_can_rx_overflow = in.stop_on_can_rx_overflow;
  out.stop_on_can_tx_congestion = in.stop_on_can_tx_congestion;
  out.stop_on_spi_comm_error = in.stop_on_spi_comm_error;
  out.stop_on_i2c_comm_error = in.stop_on_i2c_comm_error;
  out.stop_on_remote_bad_frame_burst = in.stop_on_remote_bad_frame_burst;
  out.remote_quality_hold_us = in.remote_quality_hold_us;
  out.comm_error_hold_us = in.comm_error_hold_us;
  out.can_tx_ok_timeout_us = in.can_tx_ok_timeout_us;
  out.remote_bad_frame_burst_window_us = in.remote_bad_frame_burst_window_us;
  out.remote_bad_frame_burst_threshold = in.remote_bad_frame_burst_threshold;
  return out;
}

static AppParamsImuCalibration to_core_imu_calib(const domain::ImuCalibration& in)
{
  AppParamsImuCalibration out{};
  for (int i = 0; i < 3; ++i)
  {
    out.accel_scale[i] = in.accel_scale[i];
    out.accel_bias[i] = in.accel_bias[i];
    out.gyro_scale[i] = in.gyro_scale[i];
    out.gyro_bias[i] = in.gyro_bias[i];
  }
  return out;
}

static domain::ImuCalibration from_core_imu_calib(const AppParamsImuCalibration& in)
{
  domain::ImuCalibration out{};
  for (int i = 0; i < 3; ++i)
  {
    out.accel_scale[i] = in.accel_scale[i];
    out.accel_bias[i] = in.accel_bias[i];
    out.gyro_scale[i] = in.gyro_scale[i];
    out.gyro_bias[i] = in.gyro_bias[i];
  }
  return out;
}

static AppParamsMagCalibration to_core_mag_calib(const domain::MagCalibration& in)
{
  AppParamsMagCalibration out{};
  for (int i = 0; i < 3; ++i)
  {
    out.scale[i] = in.scale[i];
    out.bias[i] = in.bias[i];
  }
  return out;
}

static domain::MagCalibration from_core_mag_calib(const AppParamsMagCalibration& in)
{
  domain::MagCalibration out{};
  for (int i = 0; i < 3; ++i)
  {
    out.scale[i] = in.scale[i];
    out.bias[i] = in.bias[i];
  }
  return out;
}

void InstallChassisController(algo::ChassisControllerFn fn) noexcept
{
  g_control_loop.set_chassis_controller(fn);
}

#if defined(__GNUC__)
__attribute__((weak))
#endif
algo::ChassisControllerFn ChassisControllerOverride() noexcept
{
  return nullptr;
}

static AppParamsWheelOdomConfig to_core_wheel_odom(const domain::WheelOdomConfig& in)
{
  AppParamsWheelOdomConfig out{};
  out.wheel_radius_m = in.wheel_radius_m;
  out.wheel_track_m = in.wheel_track_m;
  out.wheel_gear_ratio = in.wheel_gear_ratio;
  out.rpm_sign_left = in.rpm_sign_left;
  out.rpm_sign_right = in.rpm_sign_right;
  return out;
}

static domain::WheelOdomConfig from_core_wheel_odom(const AppParamsWheelOdomConfig& in)
{
  domain::WheelOdomConfig out{};
  out.wheel_radius_m = in.wheel_radius_m;
  out.wheel_track_m = in.wheel_track_m;
  out.wheel_gear_ratio = in.wheel_gear_ratio;
  out.rpm_sign_left = in.rpm_sign_left;
  out.rpm_sign_right = in.rpm_sign_right;
  return out;
}

static AppParamsWheelControllerConfig to_core_wheel_ctrl(const domain::WheelControllerConfig& in)
{
  AppParamsWheelControllerConfig out{};
  out.max_vx_mps = in.max_vx_mps;
  out.max_wz_rps = in.max_wz_rps;
  out.wheel_vel_kp_mA_per_mps = in.wheel_vel_kp_mA_per_mps;
  out.wheel_current_limit_mA = in.wheel_current_limit_mA;
  return out;
}

static domain::WheelControllerConfig from_core_wheel_ctrl(const AppParamsWheelControllerConfig& in)
{
  domain::WheelControllerConfig out{};
  out.max_vx_mps = in.max_vx_mps;
  out.max_wz_rps = in.max_wz_rps;
  out.wheel_vel_kp_mA_per_mps = in.wheel_vel_kp_mA_per_mps;
  out.wheel_current_limit_mA = in.wheel_current_limit_mA;
  return out;
}

}  // namespace robotapp

// Debug：暴露遥控状态供监视（C 链接）。
extern "C" {
App_RemoteState g_robotapp_rc_state = {};
App_OperatorState g_robotapp_operator_state = {};
App_RemoteDiagnostics g_robotapp_rc_diag = {};
App_SensorsSnapshot g_robotapp_sensors = {};
App_StateEstimate g_robotapp_estimate = {};
App_SystemHealth g_robotapp_sys_health = {};
App_DiagnosticsSnapshot g_robotapp_diag = {};
App_ImuCalibration g_robotapp_imu_calib = {};
App_MagCalibration g_robotapp_mag_calib = {};
App_SafetyConfig g_robotapp_safety_cfg = {};
App_WheelOdomConfig g_robotapp_wheel_odom_cfg = {};
App_WheelControllerConfig g_robotapp_wheel_ctrl_cfg = {};
}

extern "C" void RobotApp_Init(void)
{
  robotapp::g_params_suppress_save.store(true, std::memory_order_relaxed);
  robotapp::g_factory_reset_in_progress.store(false, std::memory_order_relaxed);

  robotapp::set_default_runtime_params();

  // Load persisted params (if present) and apply.
  App_Params_Init();
  App_Params_SetAutoResetOnFactoryReset(1U);
  if (App_Params_LoadedFromFlash() != 0U)
  {
    AppParamsPayloadV2 persisted{};
    App_Params_Get(&persisted);
    g_robotapp_safety_cfg = robotapp::from_core_safety(persisted.safety_cfg);
    g_robotapp_imu_calib = robotapp::from_core_imu_calib(persisted.imu_calib);
    g_robotapp_mag_calib = robotapp::from_core_mag_calib(persisted.mag_calib);
    g_robotapp_wheel_odom_cfg = robotapp::from_core_wheel_odom(persisted.wheel_odom_cfg);
    g_robotapp_wheel_ctrl_cfg = robotapp::from_core_wheel_ctrl(persisted.wheel_ctrl_cfg);
  }
  robotapp::g_imu.set_calibration(g_robotapp_imu_calib);
  robotapp::g_mag.set_calibration(g_robotapp_mag_calib);
  robotapp::g_estimator.set_wheel_odom_config(g_robotapp_wheel_odom_cfg);
  robotapp::set_wheel_odom_cfg(g_robotapp_wheel_odom_cfg);
  robotapp::set_wheel_ctrl_cfg(g_robotapp_wheel_ctrl_cfg);
  robotapp::g_estimator.reset();

  robotapp::g_control_loop.set_logic_bus(&robotapp::g_logic_bus);
  robotapp::g_control_loop.init();
  if (const auto fn = robotapp::ChassisControllerOverride(); fn != nullptr)
  {
    robotapp::g_control_loop.set_chassis_controller(fn);
  }

  robotapp::g_params_suppress_save.store(false, std::memory_order_relaxed);
}

extern "C" void RobotApp_ControlTick(uint64_t ts_us)
{
  robotapp::platform::heap_guard_mark_started();

  const auto imu_state = robotapp::g_imu.state();
  const auto mag_state = robotapp::g_mag.state();
  const auto motor_fb = robotapp::get_motor_feedback();
  const auto est = robotapp::g_estimator.step(imu_state, &mag_state, &motor_fb, ts_us);
  robotapp::set_estimate(est);
  g_robotapp_estimate = est;  // debug mirror only

  robotapp::g_control_loop.tick(ts_us);

  robotapp::domain::SensorsSnapshot s{};
  s.ts_us = ts_us;
  s.imu = imu_state;
  s.mag = mag_state;
  s.motor = robotapp::get_motor_feedback();
  robotapp::set_sensors(s);
  g_robotapp_sensors = s;  // debug mirror only

  robotapp::domain::SystemHealth h{};
  h.ts_us = ts_us;
  h.sbus_pipe_dropped = static_cast<uint32_t>(g_app_sbus_pipe_dropped);
  h.operator_state = robotapp::get_operator_state();
  h.can1_rx_dropped = static_cast<uint32_t>(g_app_can1_rx_dropped);
  h.can2_rx_dropped = static_cast<uint32_t>(g_app_can2_rx_dropped);
  h.can1_tx_mailbox_full = static_cast<uint32_t>(g_app_can1_tx_mailbox_full);
  h.can2_tx_mailbox_full = static_cast<uint32_t>(g_app_can2_tx_mailbox_full);
  h.can1_tx_add_fail = static_cast<uint32_t>(g_app_can1_tx_add_fail);
  h.can2_tx_add_fail = static_cast<uint32_t>(g_app_can2_tx_add_fail);
  h.can1_tx_ok = static_cast<uint32_t>(g_robotapp_can1_tx_ok);
  h.can2_tx_ok = static_cast<uint32_t>(g_robotapp_can2_tx_ok);
  h.can1_tx_fail = static_cast<uint32_t>(g_robotapp_can1_tx_fail);
  h.can2_tx_fail = static_cast<uint32_t>(g_robotapp_can2_tx_fail);
  h.can1_tx_start_fail = static_cast<uint32_t>(g_robotapp_can1_tx_start_fail);
  h.can2_tx_start_fail = static_cast<uint32_t>(g_robotapp_can2_tx_start_fail);
  h.can1_tx_cplt_fail = static_cast<uint32_t>(g_robotapp_can1_tx_cplt_fail);
  h.can2_tx_cplt_fail = static_cast<uint32_t>(g_robotapp_can2_tx_cplt_fail);
  h.spi1_bmi088_txrx_fail = static_cast<uint32_t>(g_app_spi1_bmi088_txrx_fail);
  h.i2c3_fail = static_cast<uint32_t>(g_app_i2c3_fail);
  h.i2c3_abort = static_cast<uint32_t>(g_app_i2c3_abort);
  h.safety_flags = static_cast<uint32_t>(g_robotapp_safety_flags);
  const auto remote_state = robotapp::get_remote_state();
  const auto remote_diag = robotapp::get_remote_diag();
  h.remote_frames = remote_diag.frames;
  h.remote_bad_frames = remote_diag.bad_frame;
  h.remote_frame_lost = remote_state.frame_lost;
  h.remote_failsafe = remote_state.failsafe;
  h.remote_last_ts_us = remote_state.ts_us;
  h.imu_last_ts_us = robotapp::g_imu.state().ts_us;
  h.mag_last_ts_us = robotapp::g_mag.state().ts_us;
  h.logic_cmd_last_ts_us = robotapp::g_logic_bus.latest().ts_us;
  robotapp::set_sys_health(h);
  g_robotapp_sys_health = h;  // debug mirror only

  robotapp::domain::DiagnosticsSnapshot d{};
  d.ts_us = ts_us;
  d.operator_state = robotapp::get_operator_state();
  d.remote = robotapp::get_remote_state();
  d.remote_diag = robotapp::get_remote_diag();
  d.sensors = s;
  d.estimate = est;
  d.actuator_cmd = robotapp::g_control_loop.last_command();
  d.actuator_cmd_valid = robotapp::g_control_loop.last_command_valid() ? 1U : 0U;
  d.actuator_cmd_ts_us = robotapp::g_control_loop.last_command_ts_us();
  d.safety_cfg = g_robotapp_safety_cfg;
  d.imu_calib = g_robotapp_imu_calib;
  d.mag_calib = g_robotapp_mag_calib;
  d.wheel_odom_cfg = robotapp::get_wheel_odom_cfg();
  d.wheel_ctrl_cfg = robotapp::get_wheel_ctrl_cfg();
  d.sensor_diag.ist_i2c_started = robotapp::drivers::ist8310::g_ist_i2c_started;
  d.sensor_diag.ist_i2c_ok = robotapp::drivers::ist8310::g_ist_i2c_ok;
  d.sensor_diag.ist_i2c_start_fail = robotapp::drivers::ist8310::g_ist_i2c_start_fail;
  d.sensor_diag.ist_i2c_cplt_fail = robotapp::drivers::ist8310::g_ist_i2c_cplt_fail;
  d.sensor_diag.ist_i2c_backoff_entered = robotapp::drivers::ist8310::g_ist_i2c_backoff_entered;
  d.sensor_diag.ist_i2c_backoff_skipped = robotapp::drivers::ist8310::g_ist_i2c_backoff_skipped;
  d.params.load_ok = g_app_params_load_ok;
  d.params.load_fail = g_app_params_load_fail;
  d.params.save_ok = g_app_params_save_ok;
  d.params.save_fail = g_app_params_save_fail;
  d.params.erase_ok = g_app_params_erase_ok;
  d.params.erase_fail = g_app_params_erase_fail;
  d.params.region_invalid = g_app_params_region_invalid;
  d.params.factory_reset_requested = robotapp::g_factory_reset_requested.load(std::memory_order_relaxed);
  d.params.factory_reset_ok = g_app_params_factory_reset_ok;
  d.params.factory_reset_fail = g_app_params_factory_reset_fail;
  d.params.loaded_from_flash = App_Params_LoadedFromFlash() != 0U ? 1U : 0U;
  d.health = h;
  d.control = robotapp::g_control_loop.telemetry();
  robotapp::set_diag(d);
  g_robotapp_diag = d;  // debug mirror only
}

extern "C" void RobotApp_UpdateMotorFeedback(const App_MotorFeedback* fb)
{
  robotapp::platform::heap_guard_mark_started();
  if (fb != NULL)
  {
    robotapp::set_motor_feedback(*fb);
  }
}

extern "C" void RobotApp_UpdateImuSample(const App_ImuSample* sample)
{
  robotapp::platform::heap_guard_mark_started();
  if (sample != NULL)
  {
    robotapp::g_imu.set_raw(*sample);
  }
}

extern "C" void RobotApp_UpdateMagSample(const App_MagSample* sample)
{
  robotapp::platform::heap_guard_mark_started();
  if (sample != NULL)
  {
    robotapp::g_mag.set_raw(*sample);
  }
}

extern "C" void RobotApp_SetImuCalibration(const App_ImuCalibration* calib)
{
  if (calib != NULL)
  {
    robotapp::g_imu.set_calibration(*calib);
    g_robotapp_imu_calib = *calib;  // debug mirror only
    const auto core = robotapp::to_core_imu_calib(*calib);
    App_Params_UpdateImuCalib(&core);
    if (!robotapp::g_params_suppress_save.load(std::memory_order_relaxed)) App_Params_RequestSave();
  }
}

extern "C" void RobotApp_SetMagCalibration(const App_MagCalibration* calib)
{
  if (calib != NULL)
  {
    robotapp::g_mag.set_calibration(*calib);
    g_robotapp_mag_calib = *calib;  // debug mirror only
    const auto core = robotapp::to_core_mag_calib(*calib);
    App_Params_UpdateMagCalib(&core);
    if (!robotapp::g_params_suppress_save.load(std::memory_order_relaxed)) App_Params_RequestSave();
  }
}

extern "C" void RobotApp_SetSafetyConfig(const App_SafetyConfig* cfg)
{
  if (cfg != NULL)
  {
    g_robotapp_safety_cfg = *cfg;  // debug mirror only (also serves as runtime config store)
    const auto core = robotapp::to_core_safety(*cfg);
    App_Params_UpdateSafetyCfg(&core);
    if (!robotapp::g_params_suppress_save.load(std::memory_order_relaxed)) App_Params_RequestSave();
  }
}

extern "C" void RobotApp_RequestSaveParams(void) { App_Params_RequestSave(); }
extern "C" uint32_t RobotApp_ParamsLoadedFromFlash(void) { return App_Params_LoadedFromFlash(); }
extern "C" void RobotApp_FactoryResetParams(void)
{
  robotapp::g_params_suppress_save.store(true, std::memory_order_relaxed);
  robotapp::g_factory_reset_in_progress.store(true, std::memory_order_relaxed);
  App_Params_RequestFactoryReset();
  robotapp::set_default_runtime_params();
  robotapp::g_imu.set_calibration(g_robotapp_imu_calib);
  robotapp::g_mag.set_calibration(g_robotapp_mag_calib);
  robotapp::g_params_suppress_save.store(false, std::memory_order_relaxed);
}
extern "C" uint32_t RobotApp_FactoryResetInProgress(void)
{
  return robotapp::g_factory_reset_in_progress.load(std::memory_order_relaxed) ? 1U : 0U;
}

extern "C" void RobotApp_LogicInit(void)
{
  robotapp::g_logic_ctx.bus = &robotapp::g_logic_bus;
  robotapp::g_logic_ctx.remote = &robotapp::g_remote;
}

extern "C" void RobotApp_LogicTick(uint64_t ts_us)
{
  robotapp::platform::heap_guard_mark_started();
  robotapp::g_logic_ctx.sensors = robotapp::get_sensors();
  robotapp::g_logic_ctx.health = robotapp::get_sys_health();

  if constexpr (robotapp::kLogicDebugInject)
  {
    const float vx = (robotapp::g_logic_debug_tick % 40U) < 20U ? 0.5f : -0.5f;
    const float wz = ((robotapp::g_logic_debug_tick % 50U) < 25U) ? 0.2f : -0.2f;
    robotapp::logic::LogicBus::Snapshot snap{};
    snap.hl_cmd.vx = vx;
    snap.hl_cmd.wz = wz;
    snap.hl_valid = true;
    snap.ts_us = ts_us;
    robotapp::g_logic_bus.publish(snap);
    robotapp::g_logic_debug_tick++;
    return;
  }

  robotapp::logic::logic_tick(robotapp::g_logic_ctx, ts_us);
}

extern "C" void RobotApp_SbusFeedBytes(const uint8_t* data, uint16_t len, uint64_t ts_us)
{
  robotapp::platform::heap_guard_mark_started();
  if (data == NULL) return;
  robotapp::inputs::SbusFrame frame{};
  uint32_t last_bad = robotapp::g_sbus_parser.bad_frames();
  for (uint16_t i = 0; i < len; ++i)
  {
    if (robotapp::g_sbus_parser.feed(data[i], frame))
    {
      robotapp::g_remote.apply_sbus(frame, ts_us);
      const auto st = robotapp::g_remote.state();
      robotapp::set_remote_state(st);
      g_robotapp_rc_state = st;  // debug mirror only

      const auto op = robotapp::g_remote.operator_state();
      robotapp::set_operator_state(op);
      g_robotapp_operator_state = op;  // debug mirror only

      robotapp::maybe_factory_reset_gesture(st, op, ts_us);

      auto diag = robotapp::g_remote.diagnostics();
      diag.sbus_pipe_dropped = static_cast<uint32_t>(g_app_sbus_pipe_dropped);
      robotapp::set_remote_diag(diag);
      g_robotapp_rc_diag = diag;  // debug mirror only
    }
    const uint32_t bad_now = robotapp::g_sbus_parser.bad_frames();
    if (bad_now != last_bad)
    {
      // Count parser-level invalid frames into diagnostics.
      const uint32_t diff = bad_now - last_bad;
      robotapp::g_remote.on_bad_frames(diff, ts_us);
      last_bad = bad_now;

      auto diag = robotapp::g_remote.diagnostics();
      diag.sbus_pipe_dropped = static_cast<uint32_t>(g_app_sbus_pipe_dropped);
      robotapp::set_remote_diag(diag);
      g_robotapp_rc_diag = diag;
    }
  }
}

extern "C" App_RemoteState RobotApp_RemoteLatest(void) { return robotapp::get_remote_state(); }
extern "C" App_OperatorState RobotApp_OperatorLatest(void) { return robotapp::get_operator_state(); }
extern "C" App_RemoteDiagnostics RobotApp_RemoteDiagLatest(void) { return robotapp::get_remote_diag(); }
extern "C" App_SensorsSnapshot RobotApp_SensorsLatest(void) { return robotapp::get_sensors(); }
extern "C" App_StateEstimate RobotApp_StateEstimateLatest(void) { return robotapp::get_estimate(); }
extern "C" App_SystemHealth RobotApp_SystemHealthLatest(void) { return robotapp::get_sys_health(); }
extern "C" App_DiagnosticsSnapshot RobotApp_DiagnosticsLatest(void) { return robotapp::get_diag(); }
extern "C" App_SafetyConfig RobotApp_SafetyConfigLatest(void) { return g_robotapp_safety_cfg; }
extern "C" App_WheelOdomConfig RobotApp_WheelOdomConfigLatest(void) { return robotapp::get_wheel_odom_cfg(); }
extern "C" App_WheelControllerConfig RobotApp_WheelControllerConfigLatest(void) { return robotapp::get_wheel_ctrl_cfg(); }

extern "C" void RobotApp_SetWheelOdomConfig(const App_WheelOdomConfig* cfg)
{
  robotapp::platform::heap_guard_mark_started();
  if (cfg == NULL) return;
  g_robotapp_wheel_odom_cfg = *cfg;  // debug mirror + runtime store
  robotapp::g_estimator.set_wheel_odom_config(g_robotapp_wheel_odom_cfg);
  robotapp::set_wheel_odom_cfg(g_robotapp_wheel_odom_cfg);
  const auto core = robotapp::to_core_wheel_odom(*cfg);
  App_Params_UpdateWheelOdomCfg(&core);
  if (!robotapp::g_params_suppress_save.load(std::memory_order_relaxed)) App_Params_RequestSave();
}

extern "C" void RobotApp_SetWheelControllerConfig(const App_WheelControllerConfig* cfg)
{
  robotapp::platform::heap_guard_mark_started();
  if (cfg == NULL) return;
  g_robotapp_wheel_ctrl_cfg = *cfg;  // debug mirror + runtime store
  robotapp::set_wheel_ctrl_cfg(g_robotapp_wheel_ctrl_cfg);
  const auto core = robotapp::to_core_wheel_ctrl(*cfg);
  App_Params_UpdateWheelCtrlCfg(&core);
  if (!robotapp::g_params_suppress_save.load(std::memory_order_relaxed)) App_Params_RequestSave();
}
