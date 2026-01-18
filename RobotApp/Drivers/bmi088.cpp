#include "RobotApp/Drivers/bmi088.hpp"

extern "C" {
#include "app_time.h"
}

extern "C" {
#include "bmi08x.h"
#include "drv_bmi088.h"
}

namespace robotapp::drivers::bmi088 {

namespace {
Buffer g_buf{};
bool g_inited = false;
struct bmi08x_dev g_dev = {};
}  // namespace

extern "C" {
RawFrame g_bmi_debug_shadow{};
}

extern "C" {
BMI08X_INTF_RET_TYPE RobotApp_BMI088_SpiRead_DMA(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
BMI08X_INTF_RET_TYPE RobotApp_BMI088_SpiWrite_DMA(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
}

extern "C" bool BMI088_Init(void)
{
  if (g_inited) return true;

  if (bmi08x_interface_init(&g_dev, BMI08X_SPI_INTF, BMI088_VARIANT) != BMI08X_OK)
    return false;

  // Override transport with DMA-backed implementation (CubeMX DMA+NVIC already configured).
  g_dev.read = RobotApp_BMI088_SpiRead_DMA;
  g_dev.write = RobotApp_BMI088_SpiWrite_DMA;

  if (bmi08a_init(&g_dev) != BMI08X_OK) return false;
  if (bmi08g_init(&g_dev) != BMI08X_OK) return false;

  // Config close to previous setup: accel 6g @400Hz, gyro 2000dps @200Hz.
  g_dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
  g_dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
  g_dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
  g_dev.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
  if (bmi08a_set_power_mode(&g_dev) != BMI08X_OK) return false;
  if (bmi08a_set_meas_conf(&g_dev) != BMI08X_OK) return false;

  g_dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
  g_dev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
  g_dev.gyro_cfg.odr = BMI08X_GYRO_BW_64_ODR_200_HZ;
  if (bmi08g_set_power_mode(&g_dev) != BMI08X_OK) return false;
  if (bmi08g_set_meas_conf(&g_dev) != BMI08X_OK) return false;

  g_inited = true;
  return true;
}

extern "C" bool BMI088_PollOnce(void)
{
  if (!g_inited) return false;

  struct bmi08x_sensor_data acc = {};
  struct bmi08x_sensor_data gyr = {};

  if (bmi08a_get_data(&acc, &g_dev) != BMI08X_OK) return false;
  if (bmi08g_get_data(&gyr, &g_dev) != BMI08X_OK) return false;

  RawFrame f{};
  f.ts_us = App_Tim1_LastTimestampUs();
  f.ax = acc.x;
  f.ay = acc.y;
  f.az = acc.z;
  f.gx = gyr.x;
  f.gy = gyr.y;
  f.gz = gyr.z;

  g_buf.push(f);
  g_bmi_debug_shadow = f;
  return true;
}

extern "C" RawFrame BMI088_Latest(void) { return g_buf.latest(); }

}  // namespace robotapp::drivers::bmi088
