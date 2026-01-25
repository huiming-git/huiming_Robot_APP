#pragma once

#include <stdint.h>

#ifdef __cplusplus
namespace robotapp {
namespace domain {
struct DiagnosticsSnapshot;
struct ImuCalibration;
struct ImuSample;
struct MagCalibration;
struct MagSample;
struct MotorFeedback;
struct OperatorState;
struct SafetyConfig;
struct StateEstimate;
struct WheelControllerConfig;
struct WheelOdomConfig;
struct SensorsSnapshot;
struct RemoteDiagnostics;
struct RemoteState;
struct SystemHealth;
}
}  // namespace robotapp
using App_MotorFeedback = robotapp::domain::MotorFeedback;
using App_RemoteDiagnostics = robotapp::domain::RemoteDiagnostics;
using App_RemoteState = robotapp::domain::RemoteState;
using App_OperatorState = robotapp::domain::OperatorState;
using App_DiagnosticsSnapshot = robotapp::domain::DiagnosticsSnapshot;
using App_SafetyConfig = robotapp::domain::SafetyConfig;
using App_StateEstimate = robotapp::domain::StateEstimate;
using App_ImuCalibration = robotapp::domain::ImuCalibration;
using App_ImuSample = robotapp::domain::ImuSample;
using App_MagCalibration = robotapp::domain::MagCalibration;
using App_MagSample = robotapp::domain::MagSample;
using App_SensorsSnapshot = robotapp::domain::SensorsSnapshot;
using App_SystemHealth = robotapp::domain::SystemHealth;
using App_WheelOdomConfig = robotapp::domain::WheelOdomConfig;
using App_WheelControllerConfig = robotapp::domain::WheelControllerConfig;
extern "C" {
#else
typedef struct MotorFeedback App_MotorFeedback;
typedef struct RemoteDiagnostics App_RemoteDiagnostics;
typedef struct RemoteState App_RemoteState;
typedef struct OperatorState App_OperatorState;
typedef struct DiagnosticsSnapshot App_DiagnosticsSnapshot;
typedef struct SafetyConfig App_SafetyConfig;
typedef struct StateEstimate App_StateEstimate;
typedef struct ImuCalibration App_ImuCalibration;
typedef struct ImuSample App_ImuSample;
typedef struct MagCalibration App_MagCalibration;
typedef struct MagSample App_MagSample;
typedef struct SensorsSnapshot App_SensorsSnapshot;
typedef struct SystemHealth App_SystemHealth;
typedef struct WheelOdomConfig App_WheelOdomConfig;
typedef struct WheelControllerConfig App_WheelControllerConfig;
#endif

void RobotApp_Init(void);
void RobotApp_ControlTick(uint64_t ts_us);
void RobotApp_UpdateMotorFeedback(const App_MotorFeedback* fb);
void RobotApp_UpdateImuSample(const App_ImuSample* sample);
void RobotApp_UpdateMagSample(const App_MagSample* sample);
void RobotApp_SetImuCalibration(const App_ImuCalibration* calib);
void RobotApp_SetMagCalibration(const App_MagCalibration* calib);
void RobotApp_LogicInit(void);
void RobotApp_LogicTick(uint64_t ts_us);
void RobotApp_SbusFeedBytes(const uint8_t* data, uint16_t len, uint64_t ts_us);
App_RemoteState RobotApp_RemoteLatest(void);
App_OperatorState RobotApp_OperatorLatest(void);
App_RemoteDiagnostics RobotApp_RemoteDiagLatest(void);
App_SensorsSnapshot RobotApp_SensorsLatest(void);
App_StateEstimate RobotApp_StateEstimateLatest(void);
App_SystemHealth RobotApp_SystemHealthLatest(void);
App_DiagnosticsSnapshot RobotApp_DiagnosticsLatest(void);
App_SafetyConfig RobotApp_SafetyConfigLatest(void);
App_WheelOdomConfig RobotApp_WheelOdomConfigLatest(void);
App_WheelControllerConfig RobotApp_WheelControllerConfigLatest(void);
void RobotApp_SetSafetyConfig(const App_SafetyConfig* cfg);
void RobotApp_RequestSaveParams(void);
uint32_t RobotApp_ParamsLoadedFromFlash(void);
void RobotApp_FactoryResetParams(void);
uint32_t RobotApp_FactoryResetInProgress(void);
void RobotApp_SetWheelOdomConfig(const App_WheelOdomConfig* cfg);
void RobotApp_SetWheelControllerConfig(const App_WheelControllerConfig* cfg);

/* Debug/telemetry variables (optional). */
extern volatile uint32_t g_robotapp_can1_rx_count;
extern volatile uint32_t g_robotapp_can2_rx_count;
extern volatile uint32_t g_robotapp_can1_tx_ok;
extern volatile uint32_t g_robotapp_can2_tx_ok;
extern volatile uint32_t g_robotapp_can1_tx_fail;
extern volatile uint32_t g_robotapp_can2_tx_fail;
extern volatile uint32_t g_robotapp_can1_tx_start_fail;
extern volatile uint32_t g_robotapp_can2_tx_start_fail;
extern volatile uint32_t g_robotapp_can1_tx_cplt_fail;
extern volatile uint32_t g_robotapp_can2_tx_cplt_fail;
extern volatile uint16_t g_robotapp_can1_last_std_id;
extern volatile uint16_t g_robotapp_can2_last_std_id;
extern volatile uint8_t g_robotapp_dm_last_motor_id;
extern volatile float g_robotapp_dm_pos[4];
extern volatile float g_robotapp_dm_vel[4];
extern volatile float g_robotapp_dm_tor[4];
extern volatile uint16_t g_robotapp_dm_tx_last_std_id;
extern volatile uint32_t g_robotapp_dm_tx_attempt[4];
extern volatile uint32_t g_robotapp_dm_tx_started[4];
extern volatile uint16_t g_robotapp_dm_enable_tx_last_std_id;
extern volatile uint32_t g_robotapp_dm_enable_tx_attempt[4];
extern volatile uint32_t g_robotapp_dm_enable_tx_started[4];
extern volatile uint32_t g_robotapp_safety_flags;
extern volatile uint32_t g_robotapp_heap_started;
extern volatile uint32_t g_robotapp_heap_malloc_calls;
extern volatile uint32_t g_robotapp_heap_free_calls;
extern volatile uint32_t g_robotapp_heap_calloc_calls;
extern volatile uint32_t g_robotapp_heap_realloc_calls;
extern volatile uint32_t g_robotapp_heap_calls_after_start;
extern volatile uint32_t g_robotapp_heap_alloc_fail;
extern App_RemoteState g_robotapp_rc_state;
extern App_OperatorState g_robotapp_operator_state;
extern App_RemoteDiagnostics g_robotapp_rc_diag;
extern App_SensorsSnapshot g_robotapp_sensors;
extern App_StateEstimate g_robotapp_estimate;
extern App_SystemHealth g_robotapp_sys_health;
extern App_DiagnosticsSnapshot g_robotapp_diag;
extern App_ImuCalibration g_robotapp_imu_calib;
extern App_MagCalibration g_robotapp_mag_calib;
extern App_SafetyConfig g_robotapp_safety_cfg;
extern App_WheelOdomConfig g_robotapp_wheel_odom_cfg;
extern App_WheelControllerConfig g_robotapp_wheel_ctrl_cfg;
extern volatile int16_t g_robotapp_wheel_cmd_mA[2];
extern volatile uint8_t g_app_can2_dbg_enable;
extern volatile uint16_t g_app_can2_dbg_std_id;
extern volatile uint16_t g_app_can2_dbg_ids[4];
extern volatile uint8_t g_app_can2_dbg_dlc;
extern volatile uint8_t g_app_can2_dbg_data[8];
extern volatile uint32_t g_app_can2_dbg_period_ms;
extern volatile uint32_t g_app_can2_dbg_sent;
extern volatile uint32_t g_app_can2_dbg_failed;

#ifdef __cplusplus
}
#endif
