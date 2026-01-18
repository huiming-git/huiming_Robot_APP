#pragma once

#include "RobotApp/Domain/types.hpp"

namespace robotapp::algo {

struct ControlInput {
  uint64_t ts_us = 0;
  uint32_t dt_us = 0;
  const domain::ChassisSensors* sensors = nullptr;
};

struct ControlOutput {
  domain::ChassisCommand cmd{};
};

inline ControlOutput control_step(const ControlInput& in)
{
  (void)in;
  return {};
}

}  // namespace robotapp::algo

