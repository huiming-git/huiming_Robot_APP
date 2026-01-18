#include "RobotApp/Platform/time.hpp"

extern "C" {
#include "RobotApp/Bridge/app_bridge.h"
#include "app_sbus.h"
}

extern "C" void StartSbusTask(void* argument)
{
  (void)argument;

  // Event-driven SBUS consume:
  // - ISR pushes bytes into the pipe and wakes this task.
  // - This task drains the pipe and feeds bytes into the parser.
  uint8_t buf[64];
  for (;;)
  {
    while (1)
    {
      const uint16_t n = App_SbusPipe_Read(buf, (uint16_t)sizeof(buf));
      if (n == 0U) break;
      RobotApp_SbusFeedBytes(buf, n, robotapp::platform::now_us());
    }
    App_SbusPipe_WaitRx();
  }
}

