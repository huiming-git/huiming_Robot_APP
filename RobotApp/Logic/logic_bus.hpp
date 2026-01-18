#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Util/double_buffer.hpp"

namespace robotapp::logic {

// Double-buffered handoff between 100 Hz logic task (writer) and 1 kHz control loop (reader).
class LogicBus {
 public:
  struct Snapshot {
    domain::HighLevelCommand hl_cmd{};
    bool hl_valid = false;
    domain::ChassisCommand cmd{};
    bool cmd_valid = false;
    uint64_t ts_us = 0;
  };

  void publish(const Snapshot& s)
  {
    buf_.push(s);
  }

  Snapshot latest() const
  {
    return buf_.latest();
  }

 private:
  robotapp::util::DoubleBuffer<Snapshot> buf_{};
};

}  // namespace robotapp::logic
