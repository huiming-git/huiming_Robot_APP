#include "RobotApp/Bridge/controller_hook.hpp"

namespace robotapp::algo {

ChassisControllerOutput user_controller_step(const ChassisControllerInput& in)
{
  // Default: delegate to built-in stub. Replace this body with your real control algorithm.
  return chassis_controller_step(in);
}

}  // namespace robotapp::algo

namespace robotapp {

algo::ChassisControllerFn ChassisControllerOverride() noexcept
{
  // Return nullptr to use built-in chassis_controller_step directly.
  return &algo::user_controller_step;
}

}  // namespace robotapp

