#pragma once

#include <array>
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

// Debug-only: bypass SafetyPolicy enforcement (still records flags).
// Note: you can also toggle via remote switch (see domain::remote config).
inline constexpr bool kBypassSafetyPolicy = false;

// IST8310 async I2C recovery.
inline constexpr uint32_t kIstBackoffUs = 500'000U;
inline constexpr uint32_t kIstFailStreakThreshold = 3U;

// Manual fallback mapping (temporary until full control algorithm is implemented).
inline constexpr float kWheelCurrentScale_mA = 22000.0f;
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
inline constexpr float kWheelGearRatio = 1.0f;    // gearbox reduction ratio
inline constexpr int8_t kWheelRpmSignLeft = -1;   // forward: left wheel CW (rear->front view)
inline constexpr int8_t kWheelRpmSignRight = 1;   // forward: right wheel CCW (rear->front view)
inline constexpr uint32_t kOdomTimeoutUs = 200'000U;

// Minimal closed-loop controller defaults (tune on real robot).
inline constexpr float kMaxVxMps = 2.0f;          // hl_cmd.vx in [-1,1] -> m/s
inline constexpr float kMaxWzRps = 4.0f;          // hl_cmd.wz in [-1,1] -> rad/s
inline constexpr float kWheelVelKp_mA_per_mps = 8000.0f;
inline constexpr int16_t kWheelCurrentLimit_mA = 30000;

// Balance PID (torque in N*m).
inline constexpr float kBalanceKp = 18.0f;
inline constexpr float kBalanceKi = 0.0f;
inline constexpr float kBalanceKd = 1.0f;
inline constexpr float kBalanceIntLimit = 0.3f;

// RC torque mapping (mA): forward/back and turn mixing.
inline constexpr float kRcDriveCurrent_mA = 8000.0f;
inline constexpr float kRcTurnCurrent_mA = 6000.0f;

// IMU mount offset capture (seconds).
inline constexpr float kImuMountOffsetWindowS = 2.0f;

// Pitch-only estimator (fast response, minimal lag).
inline constexpr bool kPitchOnlyEstimator = true;
inline constexpr float kPitchFilterAlpha = 0.95f;
inline constexpr float kPitchOutputAlpha = 0.8f;
inline constexpr float kPitchKalmanQ = 0.01f;
inline constexpr float kPitchKalmanR = 0.02f;

// Joint hold/debug settings (DM4310).
inline constexpr float kJointHoldKp = 32.0f;
inline constexpr float kJointHoldKd = 1.0f;
inline constexpr std::array<float, 4> kJointStandPosRad = {
    -1.22371221f,
    1.07631862f,
    -0.572659492f,
    0.631205797f,
};
inline constexpr float kJointTestAmplitudeRad = 0.35f;
inline constexpr float kJointTestFreqHz = 0.5f;
inline constexpr uint32_t kJointTestSegmentUs = 2'000'000U;

// CAN telemetry (state estimate) output.
inline constexpr uint8_t kEstimateTxBus = 1U;
inline constexpr uint32_t kEstimateTxPeriodUs = 20'000U;  // 50 Hz
inline constexpr uint16_t kEstimateTxRpyId = 0x7B0U;
inline constexpr uint16_t kEstimateTxVelId = 0x7B1U;
inline constexpr uint16_t kEstimateTxPosId = 0x7B2U;
inline constexpr float kEstimateRpyScale = 10000.0f;  // rad -> 1e-4 rad
inline constexpr float kEstimateVelScale = 1000.0f;   // m/s -> mm/s
inline constexpr float kEstimatePosScale = 100.0f;    // m -> cm

}  // namespace robotapp::domain::config
