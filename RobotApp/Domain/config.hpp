#pragma once

#include <cstdint>

namespace robotapp::domain::config {

// Remote safety.
inline constexpr uint32_t kRemoteTimeoutUs = 150'000U;  // consider lost if older than this

// Logic output safety: if command snapshot is older than this, control loop outputs zero.
inline constexpr uint32_t kLogicCmdTimeoutUs = 200'000U;

// Control loop scheduling.
inline constexpr uint32_t kControlPeriodUs = 1'000U;

// Sensor safety: once a sensor has produced data, consider it stale after this timeout.
inline constexpr uint32_t kImuTimeoutUs = 200'000U;
inline constexpr uint32_t kMagTimeoutUs = 500'000U;

// CAN safety: if no TX completes within this timeout, consider the bus dead.
inline constexpr uint32_t kCanTxOkTimeoutUs = 300'000U;

// Quality/overflow flags: keep these latched for a short time after an error is observed.
inline constexpr uint32_t kRemoteQualityHoldUs = 200'000U;
inline constexpr uint32_t kCommErrorHoldUs = 200'000U;

// Safety action policy:
// - "Quality" flags default to warning-only unless explicitly enabled below.
inline constexpr bool kStopOnSbusPipeOverflow = false;
inline constexpr bool kStopOnCanRxOverflow = false;
inline constexpr bool kStopOnCanTxCongestion = false;
inline constexpr bool kStopOnSpiCommError = false;
inline constexpr bool kStopOnI2cCommError = false;

// Remote bad-frame burst policy (optional).
inline constexpr bool kStopOnRemoteBadFrameBurst = false;
inline constexpr uint32_t kRemoteBadFrameBurstWindowUs = 200'000U;
inline constexpr uint32_t kRemoteBadFrameBurstThreshold = 5U;

// IST8310 async I2C recovery.
inline constexpr uint32_t kIstBackoffUs = 500'000U;
inline constexpr uint32_t kIstFailStreakThreshold = 3U;

// Manual fallback mapping (temporary until full control algorithm is implemented).
inline constexpr float kWheelCurrentScale_mA = 300.0f;
inline constexpr float kRcDeadband = 0.05f;

// Topology (current hard-coded robot setup).
inline constexpr uint8_t kCanBusWheels = 1U;
inline constexpr uint8_t kCanBusJoints = 2U;
inline constexpr uint16_t kDm4310IdBase = 1U;

// Simple wheel odometry (2x DJI 3508) used by estimator as a velocity observation.
// NOTE: rpm in WheelState is assumed to be motor/rotor rpm. Output wheel speed is rpm/gear_ratio.
// Tune these constants for your chassis.
inline constexpr float kWheelRadiusM = 0.06f;
inline constexpr float kWheelTrackM = 0.30f;      // wheel-to-wheel distance (m)
inline constexpr float kWheelGearRatio = 19.2f;   // gearbox reduction ratio
inline constexpr int8_t kWheelRpmSignLeft = 1;    // set to -1 if wiring direction is inverted
inline constexpr int8_t kWheelRpmSignRight = 1;   // set to -1 if wiring direction is inverted
inline constexpr uint32_t kOdomTimeoutUs = 200'000U;

// Minimal closed-loop controller defaults (tune on real robot).
inline constexpr float kMaxVxMps = 1.5f;          // hl_cmd.vx in [-1,1] -> m/s
inline constexpr float kMaxWzRps = 3.0f;          // hl_cmd.wz in [-1,1] -> rad/s
inline constexpr float kWheelVelKp_mA_per_mps = 3000.0f;
inline constexpr int16_t kWheelCurrentLimit_mA = 12000;

}  // namespace robotapp::domain::config
