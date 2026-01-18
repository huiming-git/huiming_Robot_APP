#pragma once

#include <array>
#include <cstdint>

namespace robotapp::domain::remote {

// SBUS raw range typically 172..1811.
inline constexpr uint16_t kSbusMin = 172U;
inline constexpr uint16_t kSbusMax = 1811U;

struct AxisConfig {
  bool invert = false;
  float deadband = 0.05f;  // normalized -1..1
  float expo = 0.0f;       // 0..1, cubic blend
};

// Map SBUS channels (0..15) to normalized RemoteState ch[0..7].
inline constexpr std::array<uint8_t, 8> kAxisMap = {0, 1, 2, 3, 4, 5, 6, 7};
inline constexpr std::array<AxisConfig, 8> kAxisCfg = {
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.2f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.2f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.2f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.2f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.0f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.0f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.0f},
    AxisConfig{.invert = false, .deadband = 0.05f, .expo = 0.0f},
};

// Map SBUS channels to 3-position switch states sw[0..3] (0/1/2).
inline constexpr std::array<uint8_t, 4> kSwitchMap = {8, 9, 10, 11};
inline constexpr uint16_t kSwitchMid = 1000U;
inline constexpr uint16_t kSwitchHi = 1500U;
inline constexpr uint32_t kSwitchDebounceUs = 20'000U;

// Operator-state mapping from debounced RemoteState.sw[] values.
// sw value: 0/1/2 (low/mid/high).
inline constexpr uint8_t kEnableSwitchIdx = 0;      // RemoteState.sw[k]
inline constexpr uint8_t kEnableSwitchOn = 2;       // position meaning "enabled"
inline constexpr uint8_t kEStopSwitchIdx = 1;       // RemoteState.sw[k]
inline constexpr uint8_t kEStopSwitchOn = 0;        // position meaning "E-Stop"
inline constexpr uint8_t kModeSwitchIdx = 2;        // RemoteState.sw[k]
// Mode mapping: 0->Safe, 1->Manual, 2->Auto.

// E-stop latch behavior:
// - Once E-stop is asserted, it stays latched until reset condition holds.
// - Reset condition: enable request OFF AND e-stop request OFF for this long.
inline constexpr uint32_t kEStopResetHoldUs = 300'000U;

// Factory reset gesture (very deliberate):
// - Must be in Safe mode + E-stop latched (not enabled)
// - Must hold sw[3] at HIGH, and push all 4 main axes near extremes
// - Keep the gesture for this long to trigger.
inline constexpr uint8_t kFactoryResetArmSwitchIdx = 3U;
inline constexpr uint8_t kFactoryResetArmSwitchOn = 2U;
inline constexpr float kFactoryResetAxisThreshold = 0.95f;
inline constexpr uint32_t kFactoryResetHoldUs = 2'000'000U;

}  // namespace robotapp::domain::remote
