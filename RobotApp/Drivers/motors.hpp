#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Drivers/dji_3508_c620_protocol.hpp"
#include "RobotApp/Drivers/dm4310_mit_protocol.hpp"
#include "RobotApp/Drivers/motor_can_tx_callback.hpp"
#include "RobotApp/Platform/can_port.hpp"

extern "C" {
#include "app_bridge.h"
}

namespace robotapp::drivers {

struct MotorInterface {
  virtual ~MotorInterface() = default;
  virtual bool handle_frame(const platform::CanFrame& f) = 0;
  virtual void tick(uint64_t ts_us) = 0;
};

class Dji3508Group : public MotorInterface {
 public:
  Dji3508Group() = default;
  explicit Dji3508Group(uint8_t can_bus) : bus_(can_bus) {}

  void configure(platform::CanPort* can, uint8_t can_bus)
  {
    can_ = can;
    bus_ = can_bus;
    tx_cb_ctx_.bus = can_bus;
  }

  void set_current(std::size_t idx, int16_t mA)
  {
    if (idx < currents_.size())
      currents_[idx] = mA;
  }

  const std::array<domain::WheelState, 2>& state() const { return state_; }

  bool handle_frame(const platform::CanFrame& f) override
  {
    if (f.bus != bus_) return false;
    uint8_t idx = 0;
    dji3508::Feedback fb{};
    if (!dji3508::decode_feedback(f.std_id, f.data, idx, fb)) return false;
    if (idx < state_.size())
    {
      state_[idx].encoder = static_cast<int32_t>(fb.angle);
      state_[idx].rpm = fb.speed_rpm;
      state_[idx].current = fb.current;
      state_[idx].temperature = fb.temperature;
    }
    return true;
  }

  void tick(uint64_t /*ts_us*/) override
  {
    const auto data = dji3508::pack_currents(currents_);
    const bool started =
        (can_ != nullptr) &&
        can_->tx(bus_, dji3508::kTxStdId, std::span<const uint8_t>(data.data(), 8),
                 platform::OpCallback(RobotApp_MotorCanTxCallback, &tx_cb_ctx_));
    if (!started)
    {
      if (bus_ == 1U)
      {
        g_robotapp_can1_tx_fail += 1U;
        g_robotapp_can1_tx_start_fail += 1U;
      }
      else if (bus_ == 2U)
      {
        g_robotapp_can2_tx_fail += 1U;
        g_robotapp_can2_tx_start_fail += 1U;
      }
    }
  }

 private:
  platform::CanPort* can_ = nullptr;
  uint8_t bus_ = 1U;
  MotorCanTxCbCtx tx_cb_ctx_{.bus = 1U};
  std::array<int16_t, 4> currents_{0, 0, 0, 0};
  std::array<domain::WheelState, 2> state_{};
};

class Dm4310Motor : public MotorInterface {
 public:
  Dm4310Motor() = default;
  Dm4310Motor(uint8_t can_bus, uint16_t std_id) : bus_(can_bus), std_id_(std_id) {}

  void configure(platform::CanPort* can, uint8_t can_bus, uint16_t std_id)
  {
    can_ = can;
    bus_ = can_bus;
    std_id_ = std_id;
    tx_cb_ctx_.bus = can_bus;
  }

  void set_command(const domain::JointCommand& cmd) { cmd_ = cmd; }

  const domain::JointState& state() const { return state_; }

  bool handle_frame(const platform::CanFrame& f) override
  {
    if (f.bus != bus_) return false;
    dm4310::Feedback fb{};
    if (!dm4310::decode_feedback(f.data, fb)) return false;
    last_motor_id_ = fb.motor_id;
    state_.pos = fb.pos;
    state_.vel = fb.vel;
    state_.torque = fb.tor;
    return true;
  }

  void tick(uint64_t /*ts_us*/) override
  {
    auto data = build_payload();
    if (can_ != nullptr)
    {
      const bool started =
          can_->tx(bus_, std_id_, std::span<const uint8_t>(data.data(), 8),
                   platform::OpCallback(RobotApp_MotorCanTxCallback, &tx_cb_ctx_));
      if (!started)
      {
        if (bus_ == 1U)
        {
          g_robotapp_can1_tx_fail += 1U;
          g_robotapp_can1_tx_start_fail += 1U;
        }
        else if (bus_ == 2U)
        {
          g_robotapp_can2_tx_fail += 1U;
          g_robotapp_can2_tx_start_fail += 1U;
        }
      }
    }
  }

  uint8_t last_motor_id() const { return last_motor_id_; }

 protected:
  std::array<uint8_t, 8> build_payload() const
  {
    switch (cmd_.mode)
    {
      case domain::JointMode::Torque:
        return dm4310::pack_control(0.0f, 0.0f, 0.0f, 0.0f, clamp_torque(cmd_.tau));
      case domain::JointMode::Velocity:
        return dm4310::pack_control(0.0f, clamp_vel(cmd_.vel), 0.0f, clamp_kd(cmd_.kd),
                                    clamp_torque(cmd_.tau));
      case domain::JointMode::Position:
        return dm4310::pack_control(clamp_pos(cmd_.pos), 0.0f, clamp_kp(cmd_.kp),
                                    clamp_kd(cmd_.kd), clamp_torque(cmd_.tau));
      case domain::JointMode::Impedance:
        return dm4310::pack_control(clamp_pos(cmd_.pos), clamp_vel(cmd_.vel), clamp_kp(cmd_.kp),
                                    clamp_kd(cmd_.kd), clamp_torque(cmd_.tau));
    }
    return {};
  }

 private:
  static float clamp(float v, float lo, float hi)
  {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }
  static float clamp_torque(float t) { return clamp(t, -1.0f, 1.0f); }
  static float clamp_pos(float p) { return clamp(p, -50.0f, 50.0f); }
  static float clamp_vel(float v) { return clamp(v, -50.0f, 50.0f); }
  static float clamp_kp(float v) { return clamp(v, 0.0f, 50.0f); }
  static float clamp_kd(float v) { return clamp(v, 0.0f, 5.0f); }

 protected:
  platform::CanPort* can_ = nullptr;
  uint8_t bus_ = 0;
  uint16_t std_id_ = 0;
  MotorCanTxCbCtx tx_cb_ctx_{};

 private:
  domain::JointCommand cmd_{};
  domain::JointState state_{};
  uint8_t last_motor_id_ = 0;
};

class Dm4310MotorWithBringup final : public Dm4310Motor {
 public:
  Dm4310MotorWithBringup() = default;
  Dm4310MotorWithBringup(uint8_t can_bus, uint16_t std_id) : Dm4310Motor(can_bus, std_id) {}

  bool handle_frame(const platform::CanFrame& f) override
  {
    const bool handled = Dm4310Motor::handle_frame(f);
    if (handled) g_robotapp_dm_last_motor_id = this->last_motor_id();
    return handled;
  }

  void tick(uint64_t ts_us) override
  {
    (void)ts_us;
    if (bringup_step_ < 3U)
    {
      const uint8_t tail = (bringup_step_ == 0U)
                               ? dm4310::kCmdDisableTail
                               : (bringup_step_ == 1U) ? dm4310::kCmdEnableTail
                                                       : dm4310::kCmdZeroTail;
      const auto data = dm4310::pack_special(tail);
      (void)tx_with_stats(std::span<const uint8_t>(data.data(), 8));
      if (++bringup_tick_ >= 10U)
      {
        bringup_step_++;
        bringup_tick_ = 0U;
      }
      return;
    }

    const auto payload = this->build_payload();
    (void)tx_with_stats(std::span<const uint8_t>(payload.data(), 8));
  }

 private:
  bool tx_with_stats(std::span<const uint8_t> payload)
  {
    const bool started = (can_ != nullptr) &&
                         can_->tx(bus_, std_id_, payload,
                                  platform::OpCallback(RobotApp_MotorCanTxCallback, &tx_cb_ctx_));
    if (!started)
    {
      if (bus_ == 1U)
      {
        g_robotapp_can1_tx_fail += 1U;
        g_robotapp_can1_tx_start_fail += 1U;
      }
      else if (bus_ == 2U)
      {
        g_robotapp_can2_tx_fail += 1U;
        g_robotapp_can2_tx_start_fail += 1U;
      }
    }
    return started;
  }

  uint8_t bringup_step_ = 0U;
  uint8_t bringup_tick_ = 0U;
};

template <std::size_t MaxMotors>
class StaticMotorManager {
 public:
  void clear() { count_ = 0; }

  bool add(MotorInterface& m)
  {
    if (count_ >= motors_.size()) return false;
    motors_[count_++] = &m;
    return true;
  }

  void route_frame(const platform::CanFrame& f)
  {
    for (std::size_t i = 0; i < count_; ++i)
    {
      if (motors_[i]->handle_frame(f)) break;
    }
  }

  void tick_all(uint64_t ts_us)
  {
    for (std::size_t i = 0; i < count_; ++i)
    {
      motors_[i]->tick(ts_us);
    }
  }

 private:
  std::array<MotorInterface*, MaxMotors> motors_{};
  std::size_t count_ = 0;
};

}  // namespace robotapp::drivers
