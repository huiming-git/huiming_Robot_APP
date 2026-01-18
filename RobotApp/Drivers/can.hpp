#pragma once

#include <cstdint>

namespace robotapp::drivers {

struct CanFrame {
  uint32_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {};
  bool is_extended = false;
};

class ICanBus {
 public:
  virtual ~ICanBus() = default;
  virtual bool Tx(const CanFrame& frame) = 0;
  virtual bool Rx(CanFrame& frame) = 0;
};

}  // namespace robotapp::drivers

