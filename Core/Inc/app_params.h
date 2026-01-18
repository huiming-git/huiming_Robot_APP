/* Platform-neutral parameter persistence port header (no HAL types). */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t stop_on_sbus_pipe_overflow;
  uint8_t stop_on_can_rx_overflow;
  uint8_t stop_on_can_tx_congestion;
  uint8_t stop_on_spi_comm_error;
  uint8_t stop_on_i2c_comm_error;
  uint8_t stop_on_remote_bad_frame_burst;
  uint8_t reserved0;
  uint8_t reserved1;

  uint32_t remote_quality_hold_us;
  uint32_t comm_error_hold_us;
  uint32_t can_tx_ok_timeout_us;

  uint32_t remote_bad_frame_burst_window_us;
  uint32_t remote_bad_frame_burst_threshold;
} AppParamsSafetyConfig;

typedef struct {
  float accel_scale[3];
  float accel_bias[3];
  float gyro_scale[3];
  float gyro_bias[3];
} AppParamsImuCalibration;

typedef struct {
  float scale[3];
  float bias[3];
} AppParamsMagCalibration;

typedef struct {
  AppParamsSafetyConfig safety_cfg;
  AppParamsImuCalibration imu_calib;
  AppParamsMagCalibration mag_calib;
} AppParamsPayloadV1;

typedef struct {
  float wheel_radius_m;
  float wheel_track_m;
  float wheel_gear_ratio;
  int8_t rpm_sign_left;
  int8_t rpm_sign_right;
  uint16_t reserved0;
} AppParamsWheelOdomConfig;

typedef struct {
  float max_vx_mps;
  float max_wz_rps;
  float wheel_vel_kp_mA_per_mps;
  int16_t wheel_current_limit_mA;
  int16_t reserved0;
} AppParamsWheelControllerConfig;

typedef struct {
  AppParamsSafetyConfig safety_cfg;
  AppParamsImuCalibration imu_calib;
  AppParamsMagCalibration mag_calib;
  AppParamsWheelOdomConfig wheel_odom_cfg;
  AppParamsWheelControllerConfig wheel_ctrl_cfg;
} AppParamsPayloadV2;

void App_Params_Init(void);
uint32_t App_Params_LoadedFromFlash(void);
void App_Params_Get(AppParamsPayloadV2* out);

void App_Params_UpdateSafetyCfg(const AppParamsSafetyConfig* cfg);
void App_Params_UpdateImuCalib(const AppParamsImuCalibration* calib);
void App_Params_UpdateMagCalib(const AppParamsMagCalibration* calib);
void App_Params_UpdateWheelOdomCfg(const AppParamsWheelOdomConfig* cfg);
void App_Params_UpdateWheelCtrlCfg(const AppParamsWheelControllerConfig* cfg);

void App_Params_RequestSave(void);
void App_Params_Service(void);

void App_Params_RequestFactoryReset(void);
void App_Params_SetAutoResetOnFactoryReset(uint8_t enable);

/* Debug counters (LiveWatch friendly). */
extern volatile uint32_t g_app_params_load_ok;
extern volatile uint32_t g_app_params_load_fail;
extern volatile uint32_t g_app_params_save_ok;
extern volatile uint32_t g_app_params_save_fail;
extern volatile uint32_t g_app_params_save_backoff_ms;
extern volatile uint32_t g_app_params_save_fail_streak;
extern volatile uint32_t g_app_params_erase_ok;
extern volatile uint32_t g_app_params_erase_fail;
extern volatile uint32_t g_app_params_factory_reset_ok;
extern volatile uint32_t g_app_params_factory_reset_fail;
extern volatile uint32_t g_app_params_region_invalid;

#ifdef __cplusplus
}
#endif
