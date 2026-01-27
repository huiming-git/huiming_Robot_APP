#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "RobotApp/Domain/types.hpp"
#include "RobotApp/Domain/remote_config.hpp"
#include "RobotApp/Inputs/sbus.hpp"

namespace robotapp::inputs {

// Adapts raw RC frames to normalized RemoteState (pure function + internal state if needed).
class RemoteAdapter {
 public:
  RemoteAdapter() = default;

  // In future, raw type could be SBUS/DBUS bytes. For now, just expose setter.
  void set_raw_channels(const float ch[8], const uint8_t sw[5], bool failsafe, uint64_t ts_us)
  {
    diag_.frames += 1U;
    diag_.last_ts_us = ts_us;
    state_.ts_us = ts_us;
    state_.failsafe = failsafe;
    state_.frame_lost = false;
    if (failsafe) diag_.failsafe += 1U;
    for (int i = 0; i < 8; ++i) state_.ch[i] = ch[i];
    for (int i = 0; i < 5; ++i) state_.sw[i] = sw[i];
  }

  void apply_sbus(const SbusFrame& f, uint64_t ts_us)
  {
    diag_.frames += 1U;
    diag_.last_ts_us = ts_us;
    if (f.frame_lost) diag_.frame_lost += 1U;
    if (f.failsafe) diag_.failsafe += 1U;

    state_.ts_us = ts_us;
    state_.failsafe = f.failsafe;
    state_.frame_lost = f.frame_lost;

    for (int i = 0; i < 8; ++i)
    {
      const uint8_t idx = domain::remote::kAxisMap[(std::size_t)i];
      const uint16_t raw = (idx < f.ch.size()) ? f.ch[idx] : 0U;
      state_.ch[i] = shape_axis(sbus_to_unit(raw), domain::remote::kAxisCfg[(std::size_t)i]);
    }

    // Switches: raw threshold -> debounce -> publish stable.
    for (int i = 0; i < 5; ++i)
    {
      const uint8_t idx = domain::remote::kSwitchMap[(std::size_t)i];
      const uint16_t v = (idx < f.ch.size()) ? f.ch[idx] : 0U;
      const uint8_t raw_sw = (v > domain::remote::kSwitchHi) ? 2
                            : (v > domain::remote::kSwitchMid) ? 1
                                                               : 0;
      state_.sw[i] = sw_[i].update(raw_sw, ts_us);
    }

    operator_.ts_us = ts_us;
    operator_.source_sw[0] = state_.sw[domain::remote::kEnableSwitchIdx];
    operator_.source_sw[1] = state_.sw[domain::remote::kEStopSwitchIdx];
    operator_.enable_request = (operator_.source_sw[0] == domain::remote::kEnableSwitchOn);
    operator_.e_stop_request = (operator_.source_sw[1] == domain::remote::kEStopSwitchOn);

    // E-stop latch with explicit reset condition.
    if (operator_.e_stop_request) e_stop_latched_ = true;

    if (e_stop_latched_)
    {
      const bool reset_condition = (!operator_.enable_request) && (!operator_.e_stop_request);
      if (reset_condition)
      {
        if (e_stop_reset_since_us_ == 0U) e_stop_reset_since_us_ = ts_us;
        if (ts_us != 0U && (ts_us - e_stop_reset_since_us_ >= domain::remote::kEStopResetHoldUs))
        {
          e_stop_latched_ = false;
          e_stop_reset_since_us_ = 0U;
        }
      }
      else
      {
        e_stop_reset_since_us_ = 0U;
      }
    }

    operator_.e_stop_latched = e_stop_latched_;
    operator_.e_stop = e_stop_latched_;
    operator_.enabled = operator_.enable_request && (!operator_.e_stop_latched);

    const uint8_t mode_sw = state_.sw[domain::remote::kModeSwitchIdx];
    operator_.mode = (mode_sw == 2U)   ? domain::OperatorMode::Auto
                     : (mode_sw == 1U) ? domain::OperatorMode::Manual
                                      : domain::OperatorMode::Safe;
  }

  void on_bad_frame(uint64_t ts_us)
  {
    diag_.bad_frame += 1U;
    diag_.last_ts_us = ts_us;
  }

  void on_bad_frames(uint32_t n, uint64_t ts_us)
  {
    if (n == 0U) return;
    diag_.bad_frame += n;
    diag_.last_ts_us = ts_us;
  }

  const domain::RemoteState& state() const { return state_; }
  const domain::OperatorState& operator_state() const { return operator_; }
  const domain::RemoteDiagnostics& diagnostics() const { return diag_; }

 private:
  static float sbus_to_unit(uint16_t v)
  {
    constexpr float in_min = static_cast<float>(domain::remote::kSbusMin);
    constexpr float in_max = static_cast<float>(domain::remote::kSbusMax);
    float x = (static_cast<float>(v) - in_min) / (in_max - in_min);
    float y = x * 2.0f - 1.0f;
    if (y < -1.0f) y = -1.0f;
    if (y > 1.0f) y = 1.0f;
    return y;
  }

  static float shape_axis(float x, const domain::remote::AxisConfig& cfg)
  {
    if (cfg.invert) x = -x;

    float db = cfg.deadband;
    if (db < 0.0f) db = 0.0f;
    if (db > 0.9f) db = 0.9f;
    if (x > -db && x < db) x = 0.0f;

    float e = cfg.expo;
    if (e < 0.0f) e = 0.0f;
    if (e > 1.0f) e = 1.0f;
    const float x3 = x * x * x;
    return (1.0f - e) * x + e * x3;
  }

  struct SwitchDebounce {
    uint8_t stable = 0;
    uint8_t last_raw = 0;
    uint64_t last_change_ts_us = 0;

    uint8_t update(uint8_t raw, uint64_t ts_us)
    {
      if (raw != last_raw)
      {
        last_raw = raw;
        last_change_ts_us = ts_us;
        return stable;
      }

      if (stable != raw && ts_us != 0U &&
          (ts_us - last_change_ts_us >= domain::remote::kSwitchDebounceUs))
      {
        stable = raw;
      }
      return stable;
    }
  };

  domain::RemoteState state_{};
  domain::OperatorState operator_{};
  domain::RemoteDiagnostics diag_{};
  std::array<SwitchDebounce, 5> sw_{};
  bool e_stop_latched_ = false;
  uint64_t e_stop_reset_since_us_ = 0;
};

}  // namespace robotapp::inputs
