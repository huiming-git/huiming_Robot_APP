#include "RobotApp/Platform/rtos.hpp"

#include "RobotApp/Drivers/bmi088.hpp"
#include "RobotApp/Drivers/ist8310.hpp"
#include "RobotApp/Bridge/app_bridge.h"
#include "RobotApp/Domain/types.hpp"

extern "C" {
#include "app_i2c.h"
}

extern "C" void StartSensorTask(void *argument)
{
  (void)argument;
  using namespace std::chrono_literals;

  robotapp::drivers::bmi088::BMI088_Init();
  robotapp::drivers::ist8310::IST8310_Init();

  robotapp::platform::Tick last_wake = robotapp::platform::tick_now();
  uint32_t mag_div = 0;
  uint32_t last_mag_ts = 0;
  for (;;)
  {
    // 200 Hz 基准
    robotapp::platform::delay_until(last_wake, 5ms);

    // Enforce async I2C timeouts (IST8310 uses async I2C3 callback mode).
    App_I2c3_Service();

    (void)robotapp::drivers::bmi088::BMI088_PollOnce();
    const auto bmi = robotapp::drivers::bmi088::BMI088_Latest();
    robotapp::domain::ImuSample sample{};
    sample.accel_x = bmi.ax;
    sample.accel_y = bmi.ay;
    sample.accel_z = bmi.az;
    sample.gyro_x = bmi.gx;
    sample.gyro_y = bmi.gy;
    sample.gyro_z = bmi.gz;
    sample.ts_us = bmi.ts_us;
    RobotApp_UpdateImuSample(&sample);

    // 50 Hz 读取磁力计
    if ((mag_div++ % 4U) == 0U)
    {
      (void)robotapp::drivers::ist8310::IST8310_KickAsync();
    }

    const auto mag = robotapp::drivers::ist8310::IST8310_Latest();
    if (mag.ts_us != 0U && mag.ts_us != last_mag_ts)
    {
      last_mag_ts = mag.ts_us;
      robotapp::domain::MagSample ms{};
      ms.mx = mag.mx;
      ms.my = mag.my;
      ms.mz = mag.mz;
      ms.ts_us = static_cast<uint64_t>(mag.ts_us);
      RobotApp_UpdateMagSample(&ms);
    }
  }
}
