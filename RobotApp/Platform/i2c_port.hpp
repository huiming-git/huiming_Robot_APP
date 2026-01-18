#pragma once

#include <cstdint>
#include <span>

extern "C" {
#include "app_i2c.h"
}

#include "RobotApp/Platform/operation.hpp"

namespace robotapp::platform {

class I2cPort {
 public:
  virtual ~I2cPort() = default;
  virtual bool write(uint8_t addr_8bit, std::span<const uint8_t> data, Operation op) = 0;
  virtual bool read(uint8_t addr_8bit, std::span<uint8_t> data, Operation op) = 0;
  virtual bool write_read(uint8_t addr_8bit, std::span<const uint8_t> tx, std::span<uint8_t> rx,
                          Operation op) = 0;
  virtual void delay_ms(uint32_t ms) = 0;
};

class AppI2c3Port final : public I2cPort {
 public:
  bool write(uint8_t addr_8bit, std::span<const uint8_t> data, Operation op) override
  {
    return App_I2c3_WriteOp(addr_8bit, data.data(), static_cast<uint16_t>(data.size()), &op) != 0U;
  }

  bool read(uint8_t addr_8bit, std::span<uint8_t> data, Operation op) override
  {
    return App_I2c3_ReadOp(addr_8bit, data.data(), static_cast<uint16_t>(data.size()), &op) != 0U;
  }

  bool write_read(uint8_t addr_8bit, std::span<const uint8_t> tx, std::span<uint8_t> rx,
                  Operation op) override
  {
    return App_I2c3_WriteReadOp(addr_8bit, tx.data(), static_cast<uint16_t>(tx.size()), rx.data(),
                                static_cast<uint16_t>(rx.size()), &op) != 0U;
  }

  void delay_ms(uint32_t ms) override { App_DelayMs(ms); }
};

}  // namespace robotapp::platform
