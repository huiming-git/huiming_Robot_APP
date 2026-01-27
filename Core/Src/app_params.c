// Parameter persistence backend (STM32F407 internal Flash, sector 11 log).
// NOTE: This file is platform-specific by design; keep RobotApp free of HAL types.

#include "app_params.h"

#include <string.h>

#include "main.h"
#include "stm32f4xx_hal.h"

// Parameter region is reserved by the linker script (STM32F407XX_FLASH.ld).
// The backend uses these symbols to avoid hard-coding flash addresses.
extern const uint32_t __app_params_flash_start__;
extern const uint32_t __app_params_flash_end__;

#define APP_PARAMS_MAGIC (0x52504150UL)  // 'R''P''A''P'
#define APP_PARAMS_VERSION_CURRENT (2U)
#define APP_PARAMS_RECORD_BYTES (256U)  // fixed slot size for forward compatibility

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t seq;
  uint32_t payload_size;
  uint32_t payload_crc32;
  uint32_t reserved0;
} AppParamsRecordHeader;

static uint32_t record_size_bytes(void) { return APP_PARAMS_RECORD_BYTES; }

static uint32_t crc32_sw(const void* data, uint32_t len)
{
  const uint8_t* p = (const uint8_t*)data;
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < len; ++i)
  {
    crc ^= (uint32_t)p[i];
    for (uint32_t b = 0; b < 8; ++b)
    {
      const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static uint32_t params_flash_base(void)
{
  return (uint32_t)(uintptr_t)&__app_params_flash_start__;
}

static uint32_t params_flash_end(void)
{
  return (uint32_t)(uintptr_t)&__app_params_flash_end__;
}

static uint32_t sector_from_address(uint32_t addr)
{
  // STM32F407 internal flash sector layout (1MB):
  // 0..3: 16KB, 4: 64KB, 5..11: 128KB.
  if (addr < 0x08004000UL) return FLASH_SECTOR_0;
  if (addr < 0x08008000UL) return FLASH_SECTOR_1;
  if (addr < 0x0800C000UL) return FLASH_SECTOR_2;
  if (addr < 0x08010000UL) return FLASH_SECTOR_3;
  if (addr < 0x08020000UL) return FLASH_SECTOR_4;
  if (addr < 0x08040000UL) return FLASH_SECTOR_5;
  if (addr < 0x08060000UL) return FLASH_SECTOR_6;
  if (addr < 0x08080000UL) return FLASH_SECTOR_7;
  if (addr < 0x080A0000UL) return FLASH_SECTOR_8;
  if (addr < 0x080C0000UL) return FLASH_SECTOR_9;
  if (addr < 0x080E0000UL) return FLASH_SECTOR_10;
  return FLASH_SECTOR_11;
}

static uint32_t sector_start_addr(uint32_t sector)
{
  switch (sector)
  {
    case FLASH_SECTOR_0: return 0x08000000UL;
    case FLASH_SECTOR_1: return 0x08004000UL;
    case FLASH_SECTOR_2: return 0x08008000UL;
    case FLASH_SECTOR_3: return 0x0800C000UL;
    case FLASH_SECTOR_4: return 0x08010000UL;
    case FLASH_SECTOR_5: return 0x08020000UL;
    case FLASH_SECTOR_6: return 0x08040000UL;
    case FLASH_SECTOR_7: return 0x08060000UL;
    case FLASH_SECTOR_8: return 0x08080000UL;
    case FLASH_SECTOR_9: return 0x080A0000UL;
    case FLASH_SECTOR_10: return 0x080C0000UL;
    case FLASH_SECTOR_11: return 0x080E0000UL;
    default: return 0;
  }
}

static uint32_t sector_end_addr(uint32_t sector)
{
  // inclusive end address of the sector.
  switch (sector)
  {
    case FLASH_SECTOR_0: return 0x08003FFFUL;
    case FLASH_SECTOR_1: return 0x08007FFFUL;
    case FLASH_SECTOR_2: return 0x0800BFFFUL;
    case FLASH_SECTOR_3: return 0x0800FFFFUL;
    case FLASH_SECTOR_4: return 0x0801FFFFUL;
    case FLASH_SECTOR_5: return 0x0803FFFFUL;
    case FLASH_SECTOR_6: return 0x0805FFFFUL;
    case FLASH_SECTOR_7: return 0x0807FFFFUL;
    case FLASH_SECTOR_8: return 0x0809FFFFUL;
    case FLASH_SECTOR_9: return 0x080BFFFFUL;
    case FLASH_SECTOR_10: return 0x080DFFFFUL;
    case FLASH_SECTOR_11: return 0x080FFFFFUL;
    default: return 0;
  }
}

static uint32_t validate_region(uint32_t base, uint32_t end)
{
  // Only support STM32F407 internal flash range.
  if (base < 0x08000000UL || end > 0x08100000UL) return 0;
  if (base >= end) return 0;
  if ((base & 0x3U) != 0U || (end & 0x3U) != 0U) return 0;

  const uint32_t first = sector_from_address(base);
  const uint32_t last = sector_from_address(end - 1U);
  const uint32_t first_start = sector_start_addr(first);
  const uint32_t last_end = sector_end_addr(last);
  if (first_start == 0 || last_end == 0) return 0;

  // Require full-sector reservation to prevent accidental erasing into application code.
  if (base != first_start) return 0;
  if (end != (last_end + 1U)) return 0;

  if ((end - base) < record_size_bytes()) return 0;
  return 1;
}

static void defaults_v1(AppParamsPayloadV1* out)
{
  (void)memset(out, 0, sizeof(*out));

  // Safety defaults match RobotApp/Domain/config.hpp and are applied in RobotApp_Init.
  // Keep these aligned with RobotApp/Domain/config.hpp to avoid "partial save" overwriting fields.
  out->safety_cfg.stop_on_sbus_pipe_overflow = 0;
  out->safety_cfg.stop_on_can_rx_overflow = 0;
  out->safety_cfg.stop_on_can_tx_congestion = 0;
  out->safety_cfg.stop_on_spi_comm_error = 0;
  out->safety_cfg.stop_on_i2c_comm_error = 0;
  out->safety_cfg.stop_on_remote_bad_frame_burst = 0;
  out->safety_cfg.remote_quality_hold_us = 200000U;
  out->safety_cfg.comm_error_hold_us = 200000U;
  out->safety_cfg.can_tx_ok_timeout_us = 300000U;
  out->safety_cfg.remote_bad_frame_burst_window_us = 200000U;
  out->safety_cfg.remote_bad_frame_burst_threshold = 5U;

  // Identity calibration.
  out->imu_calib.accel_scale[0] = 1.0f;
  out->imu_calib.accel_scale[1] = 1.0f;
  out->imu_calib.accel_scale[2] = 1.0f;
  out->imu_calib.gyro_scale[0] = 1.0f;
  out->imu_calib.gyro_scale[1] = 1.0f;
  out->imu_calib.gyro_scale[2] = 1.0f;
  out->mag_calib.scale[0] = 1.0f;
  out->mag_calib.scale[1] = 1.0f;
  out->mag_calib.scale[2] = 1.0f;
}

static void defaults_v2(AppParamsPayloadV2* out)
{
  (void)memset(out, 0, sizeof(*out));
  defaults_v1((AppParamsPayloadV1*)out);

  // Wheel odometry defaults (keep aligned with RobotApp/Domain/config.hpp defaults).
  out->wheel_odom_cfg.wheel_radius_m = 0.06f;
  out->wheel_odom_cfg.wheel_track_m = 0.30f;
  out->wheel_odom_cfg.wheel_gear_ratio = 19.2f;
  out->wheel_odom_cfg.rpm_sign_left = -1;
  out->wheel_odom_cfg.rpm_sign_right = 1;

  // Minimal controller defaults (keep aligned with RobotApp/Domain/config.hpp defaults).
  out->wheel_ctrl_cfg.max_vx_mps = 1.5f;
  out->wheel_ctrl_cfg.max_wz_rps = 3.0f;
  out->wheel_ctrl_cfg.wheel_vel_kp_mA_per_mps = 3000.0f;
  out->wheel_ctrl_cfg.wheel_current_limit_mA = 12000;
}

volatile uint32_t g_app_params_load_ok = 0;
volatile uint32_t g_app_params_load_fail = 0;
volatile uint32_t g_app_params_save_ok = 0;
volatile uint32_t g_app_params_save_fail = 0;
volatile uint32_t g_app_params_save_backoff_ms = 0;
volatile uint32_t g_app_params_save_fail_streak = 0;
volatile uint32_t g_app_params_erase_ok = 0;
volatile uint32_t g_app_params_erase_fail = 0;
volatile uint32_t g_app_params_factory_reset_ok = 0;
volatile uint32_t g_app_params_factory_reset_fail = 0;
volatile uint32_t g_app_params_region_invalid = 0;

static volatile uint8_t g_inited = 0;
static volatile uint8_t g_loaded_from_flash = 0;
static volatile uint8_t g_dirty = 0;
static volatile uint8_t g_save_requested = 0;
static volatile uint8_t g_factory_reset_requested = 0;
static uint32_t g_dirty_since_ms = 0;
static uint32_t g_last_seq = 0;
static AppParamsPayloadV2 g_payload;
static uint8_t g_auto_reset_on_factory_reset = 1;
static uint32_t g_next_save_allowed_ms = 0;
static uint32_t g_save_fail_streak = 0;
static uint32_t g_next_reset_allowed_ms = 0;
static uint32_t g_reset_fail_streak = 0;

static uint32_t flash_erase_region(uint32_t base, uint32_t end)
{
  if (base >= end) return 0;
  if (end <= base) return 0;
  const uint32_t first = sector_from_address(base);
  const uint32_t last = sector_from_address(end - 1U);
  if (last < first) return 0;

  FLASH_EraseInitTypeDef erase;
  (void)memset(&erase, 0, sizeof(erase));
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = first;
  erase.NbSectors = (last - first) + 1U;
  uint32_t sector_error = 0;
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) return 0;
  return 1;
}

static uint32_t flash_program_words(uint32_t addr, const uint32_t* words, uint32_t word_count)
{
  for (uint32_t i = 0; i < word_count; ++i)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(addr + i * 4U), words[i]) != HAL_OK)
    {
      return 0;
    }
  }
  return 1;
}

static uint32_t is_valid_record_at(uint32_t addr,
                                   uint32_t end,
                                   uint32_t* out_seq,
                                   uint32_t* out_payload_size,
                                   uint32_t* out_version)
{
  if (addr + (uint32_t)sizeof(AppParamsRecordHeader) > end) return 0;
  const AppParamsRecordHeader* hdr = (const AppParamsRecordHeader*)addr;
  if (hdr->magic != APP_PARAMS_MAGIC) return 0;
  if (hdr->version == 0U || hdr->version > APP_PARAMS_VERSION_CURRENT) return 0;
  if (hdr->payload_size == 0U || hdr->payload_size > (uint32_t)sizeof(AppParamsPayloadV2)) return 0;
  const uint32_t payload_addr = addr + (uint32_t)sizeof(AppParamsRecordHeader);
  if (payload_addr + hdr->payload_size > end) return 0;

  const uint32_t crc = crc32_sw((const void*)payload_addr, hdr->payload_size);
  if (crc != hdr->payload_crc32) return 0;

  if (out_seq != NULL) *out_seq = hdr->seq;
  if (out_payload_size != NULL) *out_payload_size = hdr->payload_size;
  if (out_version != NULL) *out_version = hdr->version;
  return 1;
}

static uint32_t find_latest_record_any(uint32_t base,
                                       uint32_t end,
                                       uint32_t* out_addr,
                                       uint32_t* out_seq,
                                       uint32_t* out_payload_size,
                                       uint32_t* out_version)
{
  uint32_t best_addr = 0;
  uint32_t best_seq = 0;
  uint32_t best_payload_size = 0;
  uint32_t best_version = 0;

  for (uint32_t addr = base; addr + (uint32_t)sizeof(AppParamsRecordHeader) <= end; addr += 4U)
  {
    uint32_t seq = 0;
    uint32_t payload_size = 0;
    uint32_t version = 0;
    if (!is_valid_record_at(addr, end, &seq, &payload_size, &version)) continue;
    if (best_addr == 0 || (uint32_t)(seq - best_seq) < 0x80000000UL)
    {
      best_addr = addr;
      best_seq = seq;
      best_payload_size = payload_size;
      best_version = version;
    }
  }

  if (best_addr == 0) return 0;
  if (out_addr != NULL) *out_addr = best_addr;
  if (out_seq != NULL) *out_seq = best_seq;
  if (out_payload_size != NULL) *out_payload_size = best_payload_size;
  if (out_version != NULL) *out_version = best_version;
  return 1;
}

static uint32_t find_next_free_addr(uint32_t base, uint32_t end)
{
  const uint32_t step = record_size_bytes();
  for (uint32_t addr = base; addr + step <= end; addr += step)
  {
    const uint32_t magic = *(const uint32_t*)addr;
    if (magic == 0xFFFFFFFFUL) return addr;
  }
  return 0;
}

void App_Params_Init(void)
{
  defaults_v2(&g_payload);
  g_loaded_from_flash = 0;
  g_last_seq = 0;
  g_save_fail_streak = 0;
  g_app_params_save_fail_streak = 0;
  g_app_params_save_backoff_ms = 0;
  g_next_save_allowed_ms = 0;
  g_reset_fail_streak = 0;
  g_next_reset_allowed_ms = 0;

  const uint32_t base = params_flash_base();
  const uint32_t end = params_flash_end();
  if (!validate_region(base, end))
  {
    g_app_params_region_invalid += 1U;
    g_inited = 0;
    g_loaded_from_flash = 0;
    g_dirty = 0;
    g_save_requested = 0;
    g_factory_reset_requested = 0;
    g_app_params_load_fail += 1U;
    return;
  }

  uint32_t rec_addr = 0;
  uint32_t seq = 0;
  uint32_t payload_size = 0;
  uint32_t version = 0;
  if (find_latest_record_any(base, end, &rec_addr, &seq, &payload_size, &version) != 0U)
  {
    if (version == 1U)
    {
      AppParamsPayloadV1 tmp;
      defaults_v1(&tmp);
      const uint32_t copy_n =
          (payload_size > (uint32_t)sizeof(tmp)) ? (uint32_t)sizeof(tmp) : payload_size;
      (void)memcpy(&tmp,
                   (const void*)(rec_addr + (uint32_t)sizeof(AppParamsRecordHeader)),
                   (size_t)copy_n);
      defaults_v2(&g_payload);
      (void)memcpy(&g_payload, &tmp, sizeof(tmp));
      g_loaded_from_flash = 1;
      g_last_seq = seq;
      g_app_params_load_ok += 1U;
    }
    else if (version == 2U)
    {
      AppParamsPayloadV2 tmp;
      defaults_v2(&tmp);
      const uint32_t copy_n =
          (payload_size > (uint32_t)sizeof(tmp)) ? (uint32_t)sizeof(tmp) : payload_size;
      (void)memcpy(&tmp,
                   (const void*)(rec_addr + (uint32_t)sizeof(AppParamsRecordHeader)),
                   (size_t)copy_n);
      g_payload = tmp;
      g_loaded_from_flash = 1;
      g_last_seq = seq;
      g_app_params_load_ok += 1U;
    }
    else
    {
      g_app_params_load_fail += 1U;
    }
  }
  else
  {
    g_app_params_load_fail += 1U;
  }

  g_dirty = 0;
  g_save_requested = 0;
  g_factory_reset_requested = 0;
  g_dirty_since_ms = HAL_GetTick();
  g_inited = 1;
}

uint32_t App_Params_LoadedFromFlash(void) { return g_loaded_from_flash ? 1U : 0U; }

void App_Params_Get(AppParamsPayloadV2* out)
{
  if (out == NULL) return;
  *out = g_payload;
}

static void mark_dirty(void)
{
  g_dirty = 1;
  g_dirty_since_ms = HAL_GetTick();
}

void App_Params_UpdateSafetyCfg(const AppParamsSafetyConfig* cfg)
{
  if (cfg == NULL) return;
  g_payload.safety_cfg = *cfg;
  mark_dirty();
}

void App_Params_UpdateImuCalib(const AppParamsImuCalibration* calib)
{
  if (calib == NULL) return;
  g_payload.imu_calib = *calib;
  mark_dirty();
}

void App_Params_UpdateMagCalib(const AppParamsMagCalibration* calib)
{
  if (calib == NULL) return;
  g_payload.mag_calib = *calib;
  mark_dirty();
}

void App_Params_UpdateWheelOdomCfg(const AppParamsWheelOdomConfig* cfg)
{
  if (cfg == NULL) return;
  g_payload.wheel_odom_cfg = *cfg;
  mark_dirty();
}

void App_Params_UpdateWheelCtrlCfg(const AppParamsWheelControllerConfig* cfg)
{
  if (cfg == NULL) return;
  g_payload.wheel_ctrl_cfg = *cfg;
  mark_dirty();
}

void App_Params_RequestSave(void) { g_save_requested = 1; }

void App_Params_RequestFactoryReset(void)
{
  g_factory_reset_requested = 1;
}

void App_Params_SetAutoResetOnFactoryReset(uint8_t enable)
{
  g_auto_reset_on_factory_reset = (enable != 0U) ? 1U : 0U;
}

static uint32_t backoff_ms_for_fail_streak(uint32_t streak)
{
  if (streak <= 1U) return 1000U;
  if (streak == 2U) return 10000U;
  if (streak == 3U) return 60000U;
  return 300000U;
}

void App_Params_Service(void)
{
  if (!g_inited) return;

  const uint32_t base = params_flash_base();
  const uint32_t end = params_flash_end();
  if (!validate_region(base, end))
  {
    g_app_params_region_invalid += 1U;
    return;
  }

  if (g_factory_reset_requested)
  {
    const uint32_t now_ms = HAL_GetTick();
    if (g_next_reset_allowed_ms != 0U && (uint32_t)(now_ms - g_next_reset_allowed_ms) < 0x80000000UL)
    {
      if (now_ms < g_next_reset_allowed_ms) return;
    }

    HAL_FLASH_Unlock();
    const uint32_t ok = flash_erase_region(base, end);
    HAL_FLASH_Lock();

    if (ok)
    {
      defaults_v2(&g_payload);
      g_loaded_from_flash = 0;
      g_last_seq = 0;
      g_dirty = 0;
      g_save_requested = 0;
      g_factory_reset_requested = 0;
      g_app_params_factory_reset_ok += 1U;
      g_reset_fail_streak = 0;
      g_next_reset_allowed_ms = 0;
      if (g_auto_reset_on_factory_reset) NVIC_SystemReset();
    }
    else
    {
      g_app_params_factory_reset_fail += 1U;
      g_reset_fail_streak += 1U;
      const uint32_t bo = backoff_ms_for_fail_streak(g_reset_fail_streak);
      g_next_reset_allowed_ms = HAL_GetTick() + bo;
    }
    return;
  }

  if (!g_save_requested) return;
  if (!g_dirty) return;

  // Debounce: wait until params have been stable for a while to avoid flash wear.
  const uint32_t now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - g_dirty_since_ms) < 500U) return;
  if (g_next_save_allowed_ms != 0U && now_ms < g_next_save_allowed_ms) return;

  uint32_t rec_words[APP_PARAMS_RECORD_BYTES / 4U];
  for (uint32_t i = 0; i < (APP_PARAMS_RECORD_BYTES / 4U); ++i) rec_words[i] = 0xFFFFFFFFUL;
  AppParamsRecordHeader* hdr = (AppParamsRecordHeader*)&rec_words[0];
  hdr->magic = APP_PARAMS_MAGIC;
  hdr->version = APP_PARAMS_VERSION_CURRENT;
  hdr->seq = g_last_seq + 1U;
  hdr->payload_size = (uint32_t)sizeof(AppParamsPayloadV2);
  hdr->reserved0 = 0xFFFFFFFFUL;
  const uint32_t payload_off = (uint32_t)sizeof(AppParamsRecordHeader);
  (void)memcpy(((uint8_t*)rec_words) + payload_off, &g_payload, sizeof(g_payload));
  hdr->payload_crc32 =
      crc32_sw(((const uint8_t*)rec_words) + payload_off, (uint32_t)sizeof(AppParamsPayloadV2));

  uint32_t addr = find_next_free_addr(base, end);
  uint32_t need_erase = 0;
  if (addr == 0) need_erase = 1;
  if (addr != 0 && addr + record_size_bytes() > end) need_erase = 1;

  HAL_FLASH_Unlock();
  uint32_t ok = 1;
  if (need_erase)
  {
    ok = flash_erase_region(base, end);
    if (ok)
    {
      g_app_params_erase_ok += 1U;
      addr = base;
    }
    else
    {
      g_app_params_erase_fail += 1U;
    }
  }

  if (ok)
  {
    const uint32_t word_count = record_size_bytes() / 4U;
    ok = flash_program_words(addr, (const uint32_t*)&rec_words[0], word_count);
  }
  HAL_FLASH_Lock();

  if (ok)
  {
    g_last_seq = hdr->seq;
    g_loaded_from_flash = 1;
    g_dirty = 0;
    g_save_requested = 0;
    g_app_params_save_ok += 1U;
    g_save_fail_streak = 0;
    g_app_params_save_fail_streak = 0;
    g_app_params_save_backoff_ms = 0;
    g_next_save_allowed_ms = 0;
  }
  else
  {
    g_app_params_save_fail += 1U;
    g_save_fail_streak += 1U;
    g_app_params_save_fail_streak = g_save_fail_streak;
    const uint32_t bo = backoff_ms_for_fail_streak(g_save_fail_streak);
    g_app_params_save_backoff_ms = bo;
    g_next_save_allowed_ms = HAL_GetTick() + bo;
  }
}
