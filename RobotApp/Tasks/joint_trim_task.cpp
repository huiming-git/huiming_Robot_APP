extern "C" {
#include "cmsis_os.h"
#include "app_bridge.h"
}

#include <cmath>

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Domain/remote_config.hpp"

extern "C" void StartJointTrimTask(void *argument)
{
  (void)argument;
  const uint32_t period_ms = 20U;
  const float dt_s = 0.001f * static_cast<float>(period_ms);
  const float step_per_s = 0.1f;
  for (;;)
  {
    const auto remote = RobotApp_RemoteLatest();
    const auto op = RobotApp_OperatorLatest();
    const bool trim_enabled =
        (op.enabled) &&
        (remote.sw[robotapp::domain::remote::kJointTrimEnableSwitchIdx] ==
         robotapp::domain::remote::kJointTrimSwitchOn) &&
        (!remote.failsafe) && (!remote.frame_lost);

    if (trim_enabled)
    {
      const bool select_b =
          (remote.sw[robotapp::domain::remote::kJointTrimSelectSwitchIdx] ==
           robotapp::domain::remote::kJointTrimSwitchOn);
      const float ch3 = remote.ch[2];
      const float ch4 = remote.ch[3];
      const float deadband = 0.05f;
      const float u3 = (std::abs(ch3) < deadband) ? 0.0f : ch3;
      const float u4 = (std::abs(ch4) < deadband) ? 0.0f : ch4;
      const float delta3 = u3 * step_per_s * dt_s;
      const float delta4 = u4 * step_per_s * dt_s;

      auto apply_delta = [](int idx, float delta) {
        float v = g_robotapp_joint_hold_pos[idx];
        if (!std::isfinite(v)) v = 0.0f;
        v += delta;
        g_robotapp_joint_hold_pos[idx] = v;
      };

      if (!select_b)
      {
        apply_delta(0, delta3);
        apply_delta(2, delta4);
      }
      else
      {
        apply_delta(1, delta3);
        apply_delta(3, delta4);
      }
    }

    osDelay(period_ms);
  }
}
