#include "RobotApp/Platform/rtos.hpp"
#include "RobotApp/Platform/time.hpp"

extern "C" {
#include "app_bridge.h"
}

extern "C" void StartLogicTask(void* argument)
{
  (void)argument;
  using namespace std::chrono_literals;

  RobotApp_LogicInit();

  robotapp::platform::Tick last_wake = robotapp::platform::tick_now();
  for (;;)
  {
    robotapp::platform::delay_until(last_wake, 10ms);
    RobotApp_LogicTick(robotapp::platform::now_us());
  }
}

