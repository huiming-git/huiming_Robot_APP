#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "RobotApp/Util/double_buffer.hpp"

namespace robotapp::drivers::ist8310 {

constexpr uint8_t kI2cAddr = 0x0E << 1;  // 7-bit address shifted for HAL

struct RawFrame {
  int16_t mx = 0;
  int16_t my = 0;
  int16_t mz = 0;
  uint32_t ts_us = 0;
};

using Buffer = robotapp::util::DoubleBuffer<RawFrame>;

// C-linkage helpers for C code paths (defined in ist8310.cpp)
extern "C" {
bool IST8310_Init(void);
bool IST8310_PollOnce(void);
bool IST8310_KickAsync(void);
RawFrame IST8310_Latest(void);
// Debug shadow: copy of latest frame for LiveWatch/GDB without调用函数
extern RawFrame g_ist_debug_shadow;

// Async diagnostics.
extern volatile uint32_t g_ist_i2c_start_fail;
extern volatile uint32_t g_ist_i2c_cplt_fail;
extern volatile uint32_t g_ist_i2c_started;
extern volatile uint32_t g_ist_i2c_backoff_entered;
extern volatile uint32_t g_ist_i2c_backoff_skipped;
extern volatile uint32_t g_ist_i2c_ok;
}

}  // namespace robotapp::drivers::ist8310
