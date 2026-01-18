#pragma once

#include "RobotApp/Algo/chassis_controller.hpp"

namespace robotapp {

// Install a chassis controller implementation used when Logic publishes hl_cmd (hl_valid=true)
// and no actuator-level command is provided (cmd_valid=false).
//
// Note: This function is lock-free; installing a new controller while ControlTick is running is
// supported, but typically you should set it during initialization.
void InstallChassisController(algo::ChassisControllerFn fn) noexcept;

// Optional hook: provide an initial controller at startup without editing RobotApp internals.
// Define this function in your own C++ file to return a controller function pointer; if not
// provided, the default implementation returns nullptr and RobotApp uses its built-in stub.
algo::ChassisControllerFn ChassisControllerOverride() noexcept;

}  // namespace robotapp
