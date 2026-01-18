#pragma once

#include <chrono>
#include <cstdint>

extern "C" {
#include "app_rtos.h"
}

namespace robotapp::platform {

using Tick = uint32_t;

inline Tick tick_now() { return App_Rtos_TickNow(); }

inline void delay_for(std::chrono::milliseconds ms) { App_Rtos_DelayMs((uint32_t)ms.count()); }

inline void delay_until(Tick& last_wake, std::chrono::milliseconds period)
{
  App_Rtos_DelayUntilMs(&last_wake, (uint32_t)period.count());
}

}  // namespace robotapp::platform

