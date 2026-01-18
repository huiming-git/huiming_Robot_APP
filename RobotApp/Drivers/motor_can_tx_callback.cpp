#include <cstdint>

extern "C" {
#include "RobotApp/Bridge/app_bridge.h"
}

#include "RobotApp/Drivers/motor_can_tx_callback.hpp"

extern "C" void RobotApp_MotorCanTxCallback(uint8_t ok, uint8_t /*in_isr*/, void* user)
{
  auto* ctx = reinterpret_cast<robotapp::drivers::MotorCanTxCbCtx*>(user);
  if (ctx == nullptr) return;

  if (ctx->bus == 1U)
  {
    if (ok)
      g_robotapp_can1_tx_ok += 1U;
    else
    {
      g_robotapp_can1_tx_fail += 1U;
      g_robotapp_can1_tx_cplt_fail += 1U;
    }
  }
  else if (ctx->bus == 2U)
  {
    if (ok)
      g_robotapp_can2_tx_ok += 1U;
    else
    {
      g_robotapp_can2_tx_fail += 1U;
      g_robotapp_can2_tx_cplt_fail += 1U;
    }
  }
}
