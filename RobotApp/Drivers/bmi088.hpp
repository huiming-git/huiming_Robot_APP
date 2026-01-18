#pragma once

#include <cstdint>

#include "RobotApp/Util/double_buffer.hpp"

namespace robotapp::drivers::bmi088 {

struct RawFrame {
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  uint32_t ts_us = 0;
};

using Buffer = robotapp::util::DoubleBuffer<RawFrame>;

extern "C" {
bool BMI088_Init(void);
bool BMI088_PollOnce(void);
RawFrame BMI088_Latest(void);
extern RawFrame g_bmi_debug_shadow;  // For LiveWatch/GDB without function calls.
}

}  // namespace robotapp::drivers::bmi088
