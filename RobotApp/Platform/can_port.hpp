#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

extern "C" {
#include "app_can.h"
}

#include "RobotApp/Platform/operation.hpp"

namespace robotapp::platform {

using CanFrame = App_CanFrame_t;

class CanPort {
 public:
  virtual ~CanPort() = default;
  virtual bool tx(uint8_t bus, uint16_t std_id, std::span<const uint8_t> payload, Operation op) = 0;
  virtual bool rx_pop(CanFrame& out) = 0;
};

// Default implementation backed by Core's App_Can_* functions.
class AppCanPort final : public CanPort {
 public:
  bool tx(uint8_t bus, uint16_t std_id, std::span<const uint8_t> payload, Operation op) override
  {
    if (payload.size() > 8U) return false;
    return App_Can_TxOp(bus, std_id, payload.data(), static_cast<uint8_t>(payload.size()), &op) !=
           0U;
  }

  bool rx_pop(CanFrame& out) override { return App_Can_RxPop(&out) != 0U; }
};

}  // namespace robotapp::platform
