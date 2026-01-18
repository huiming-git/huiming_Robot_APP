#pragma once

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Logic/logic_bus.hpp"
#include "RobotApp/Logic/logic_step.hpp"
#include "RobotApp/Inputs/remote_adapter.hpp"

namespace robotapp::logic {

struct LogicContext {
  LogicBus* bus = nullptr;
  inputs::RemoteAdapter* remote = nullptr;
  domain::SensorsSnapshot sensors{};
  domain::SystemHealth health{};
};

inline void logic_tick(LogicContext& ctx, uint64_t ts_us)
{
  LogicInput in{};
  in.ts_us = ts_us;
  if (ctx.remote)
  {
    in.remote = ctx.remote->state();
    in.operator_state = ctx.remote->operator_state();
  }
  in.imu = ctx.sensors.imu;
  in.mag = ctx.sensors.mag;
  in.motor = ctx.sensors.motor;
  in.health = ctx.health;
  auto out = logic_step(in);

  if (ctx.bus != nullptr)
  {
    LogicBus::Snapshot snap{};
    snap.hl_cmd = out.hl_cmd;
    snap.hl_valid = out.hl_valid;
    snap.cmd = out.cmd;
    snap.cmd_valid = out.cmd_valid;
    snap.ts_us = ts_us;
    ctx.bus->publish(snap);
  }
}

}  // namespace robotapp::logic
