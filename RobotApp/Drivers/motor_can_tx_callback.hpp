#pragma once

#include <cstdint>

namespace robotapp::drivers {

struct MotorCanTxCbCtx {
  uint8_t bus = 0;
};

}  // namespace robotapp::drivers

extern "C" void RobotApp_MotorCanTxCallback(uint8_t ok, uint8_t in_isr, void* user);

