#pragma once

#include <cstdint>

#include "RobotApp/Domain/config.hpp"
#include "RobotApp/Domain/types.hpp"

namespace robotapp::safety {

// Keep these bits stable: they are exported via SystemHealth.safety_flags and g_robotapp_safety_flags.
inline constexpr uint32_t kRemoteStale = 1U << 0;
inline constexpr uint32_t kImuStale = 1U << 1;
inline constexpr uint32_t kMagStale = 1U << 2;
inline constexpr uint32_t kCan1Dead = 1U << 3;
inline constexpr uint32_t kCan2Dead = 1U << 4;
inline constexpr uint32_t kOperatorDisabled = 1U << 5;
inline constexpr uint32_t kEStop = 1U << 6;
inline constexpr uint32_t kRemoteFrameLost = 1U << 7;
inline constexpr uint32_t kRemoteFailsafe = 1U << 8;
inline constexpr uint32_t kSbusPipeOverflow = 1U << 9;
inline constexpr uint32_t kCan1RxOverflow = 1U << 10;
inline constexpr uint32_t kCan2RxOverflow = 1U << 11;
inline constexpr uint32_t kSpiCommError = 1U << 12;
inline constexpr uint32_t kI2cCommError = 1U << 13;
inline constexpr uint32_t kCan1TxCongestion = 1U << 14;
inline constexpr uint32_t kCan2TxCongestion = 1U << 15;
inline constexpr uint32_t kRemoteBadFrameBurst = 1U << 16;

struct Decision {
  uint32_t flags = 0;

  bool global_stop(const domain::SafetyConfig& cfg) const
  {
    uint32_t mask =
        (kRemoteStale | kImuStale | kMagStale | kOperatorDisabled | kEStop | kRemoteFrameLost |
         kRemoteFailsafe);
    if (cfg.stop_on_sbus_pipe_overflow) mask |= kSbusPipeOverflow;
    if (cfg.stop_on_can_rx_overflow) mask |= (kCan1RxOverflow | kCan2RxOverflow);
    if (cfg.stop_on_can_tx_congestion) mask |= (kCan1TxCongestion | kCan2TxCongestion);
    if (cfg.stop_on_spi_comm_error) mask |= kSpiCommError;
    if (cfg.stop_on_i2c_comm_error) mask |= kI2cCommError;
    if (cfg.stop_on_remote_bad_frame_burst) mask |= kRemoteBadFrameBurst;
    return (flags & mask) != 0U;
  }
  bool wheels_stop(const domain::SafetyConfig& cfg) const
  {
    uint32_t mask = kCan1Dead;
    if (cfg.stop_on_can_tx_congestion) mask |= kCan1TxCongestion;
    if (cfg.stop_on_can_rx_overflow) mask |= kCan1RxOverflow;
    return (flags & mask) != 0U;
  }
  bool joints_stop(const domain::SafetyConfig& cfg) const
  {
    uint32_t mask = kCan2Dead;
    if (cfg.stop_on_can_tx_congestion) mask |= kCan2TxCongestion;
    if (cfg.stop_on_can_rx_overflow) mask |= kCan2RxOverflow;
    return (flags & mask) != 0U;
  }
};

class SafetyPolicy {
 public:
  Decision evaluate(const domain::SystemHealth& health, const domain::SafetyConfig& cfg,
                    uint64_t now_us)
  {
    Decision d{};

    // Remote staleness: remote_ts_us==0 means "no signal yet" -> stop.
    const bool remote_stale =
        (health.remote_last_ts_us == 0U) ||
        ((now_us != 0U) && (now_us - health.remote_last_ts_us > domain::config::kRemoteTimeoutUs));
    if (remote_stale) d.flags |= kRemoteStale;
    if (health.remote_frame_lost) d.flags |= kRemoteFrameLost;
    if (health.remote_failsafe) d.flags |= kRemoteFailsafe;

    // IMU/Mag staleness: only enforce once we have seen at least one sample.
    const bool imu_stale =
        (health.imu_last_ts_us != 0U) &&
        ((now_us != 0U) && (now_us - health.imu_last_ts_us > domain::config::kImuTimeoutUs));
    if (imu_stale) d.flags |= kImuStale;

    const bool mag_stale =
        (health.mag_last_ts_us != 0U) &&
        ((now_us != 0U) && (now_us - health.mag_last_ts_us > domain::config::kMagTimeoutUs));
    if (mag_stale) d.flags |= kMagStale;

    // Operator gating.
    if (!health.operator_state.enabled) d.flags |= kOperatorDisabled;
    if (health.operator_state.e_stop) d.flags |= kEStop;

    // Quality/overflow indicators (latched for a short time).
    d.flags |= quality_flags(health, cfg, now_us);

    // CAN TX liveness: if no TX completes within timeout, consider that bus dead.
    // This is stateful: we track the last moment can*_tx_ok changes.
    if (last_can1_tx_ok_ts_us_ == 0U) last_can1_tx_ok_ts_us_ = now_us;
    if (last_can2_tx_ok_ts_us_ == 0U) last_can2_tx_ok_ts_us_ = now_us;

    if (health.can1_tx_ok != last_can1_tx_ok_)
    {
      last_can1_tx_ok_ = health.can1_tx_ok;
      last_can1_tx_ok_ts_us_ = now_us;
    }
    if (health.can2_tx_ok != last_can2_tx_ok_)
    {
      last_can2_tx_ok_ = health.can2_tx_ok;
      last_can2_tx_ok_ts_us_ = now_us;
    }

    const uint32_t can_tx_ok_to =
        (cfg.can_tx_ok_timeout_us != 0U) ? cfg.can_tx_ok_timeout_us : domain::config::kCanTxOkTimeoutUs;
    const bool can1_dead = (now_us != 0U) && (now_us - last_can1_tx_ok_ts_us_ > can_tx_ok_to);
    const bool can2_dead = (now_us != 0U) && (now_us - last_can2_tx_ok_ts_us_ > can_tx_ok_to);
    if (can1_dead) d.flags |= kCan1Dead;
    if (can2_dead) d.flags |= kCan2Dead;

    return d;
  }

 private:
  uint32_t quality_flags(const domain::SystemHealth& health, const domain::SafetyConfig& cfg,
                         uint64_t now_us)
  {
    uint32_t f = 0;
    const uint32_t remote_hold =
        (cfg.remote_quality_hold_us != 0U) ? cfg.remote_quality_hold_us : domain::config::kRemoteQualityHoldUs;
    const uint32_t comm_hold =
        (cfg.comm_error_hold_us != 0U) ? cfg.comm_error_hold_us : domain::config::kCommErrorHoldUs;
    const uint32_t bad_window =
        (cfg.remote_bad_frame_burst_window_us != 0U) ? cfg.remote_bad_frame_burst_window_us
                                                     : domain::config::kRemoteBadFrameBurstWindowUs;
    const uint32_t bad_thresh =
        (cfg.remote_bad_frame_burst_threshold != 0U) ? cfg.remote_bad_frame_burst_threshold
                                                     : domain::config::kRemoteBadFrameBurstThreshold;

    // Remote bad-frame burst (rate-based within a sliding window).
    if (health.remote_bad_frames != last_remote_bad_frames_)
    {
      const uint32_t diff = health.remote_bad_frames - last_remote_bad_frames_;
      last_remote_bad_frames_ = health.remote_bad_frames;
      if (remote_bad_window_start_us_ == 0U ||
          (now_us != 0U && (now_us - remote_bad_window_start_us_ > bad_window)))
      {
        remote_bad_window_start_us_ = now_us;
        remote_bad_in_window_ = 0U;
      }
      remote_bad_in_window_ += diff;
      remote_bad_since_us_ = now_us;
    }
    if (remote_bad_since_us_ != 0U &&
        (now_us != 0U) && (now_us - remote_bad_since_us_ < remote_hold) &&
        (remote_bad_in_window_ >= bad_thresh))
    {
      f |= kRemoteBadFrameBurst;
    }

    // SBUS pipe dropped.
    if (health.sbus_pipe_dropped != last_sbus_pipe_dropped_)
    {
      last_sbus_pipe_dropped_ = health.sbus_pipe_dropped;
      sbus_overflow_since_us_ = now_us;
    }
    if (sbus_overflow_since_us_ != 0U &&
        (now_us != 0U) && (now_us - sbus_overflow_since_us_ < remote_hold))
    {
      f |= kSbusPipeOverflow;
    }

    // CAN RX queue overflow.
    if (health.can1_rx_dropped != last_can1_rx_dropped_)
    {
      last_can1_rx_dropped_ = health.can1_rx_dropped;
      can1_rx_overflow_since_us_ = now_us;
    }
    if (health.can2_rx_dropped != last_can2_rx_dropped_)
    {
      last_can2_rx_dropped_ = health.can2_rx_dropped;
      can2_rx_overflow_since_us_ = now_us;
    }
    if (can1_rx_overflow_since_us_ != 0U &&
        (now_us != 0U) && (now_us - can1_rx_overflow_since_us_ < comm_hold))
      f |= kCan1RxOverflow;
    if (can2_rx_overflow_since_us_ != 0U &&
        (now_us != 0U) && (now_us - can2_rx_overflow_since_us_ < comm_hold))
      f |= kCan2RxOverflow;

    // CAN TX congestion (mailbox full / add fail / start/cplt failures, counter deltas).
    if (health.can1_tx_mailbox_full != last_can1_tx_mailbox_full_ ||
        health.can1_tx_add_fail != last_can1_tx_add_fail_ ||
        health.can1_tx_start_fail != last_can1_tx_start_fail_ ||
        health.can1_tx_cplt_fail != last_can1_tx_cplt_fail_)
    {
      last_can1_tx_mailbox_full_ = health.can1_tx_mailbox_full;
      last_can1_tx_add_fail_ = health.can1_tx_add_fail;
      last_can1_tx_start_fail_ = health.can1_tx_start_fail;
      last_can1_tx_cplt_fail_ = health.can1_tx_cplt_fail;
      can1_tx_congest_since_us_ = now_us;
    }
    if (health.can2_tx_mailbox_full != last_can2_tx_mailbox_full_ ||
        health.can2_tx_add_fail != last_can2_tx_add_fail_ ||
        health.can2_tx_start_fail != last_can2_tx_start_fail_ ||
        health.can2_tx_cplt_fail != last_can2_tx_cplt_fail_)
    {
      last_can2_tx_mailbox_full_ = health.can2_tx_mailbox_full;
      last_can2_tx_add_fail_ = health.can2_tx_add_fail;
      last_can2_tx_start_fail_ = health.can2_tx_start_fail;
      last_can2_tx_cplt_fail_ = health.can2_tx_cplt_fail;
      can2_tx_congest_since_us_ = now_us;
    }
    if (can1_tx_congest_since_us_ != 0U &&
        (now_us != 0U) && (now_us - can1_tx_congest_since_us_ < comm_hold))
      f |= kCan1TxCongestion;
    if (can2_tx_congest_since_us_ != 0U &&
        (now_us != 0U) && (now_us - can2_tx_congest_since_us_ < comm_hold))
      f |= kCan2TxCongestion;

    // SPI/I2C comm errors (counter deltas).
    if (health.spi1_bmi088_txrx_fail != last_spi_fail_)
    {
      last_spi_fail_ = health.spi1_bmi088_txrx_fail;
      spi_err_since_us_ = now_us;
    }
    if (health.i2c3_fail != last_i2c_fail_ || health.i2c3_abort != last_i2c_abort_)
    {
      last_i2c_fail_ = health.i2c3_fail;
      last_i2c_abort_ = health.i2c3_abort;
      i2c_err_since_us_ = now_us;
    }
    if (spi_err_since_us_ != 0U &&
        (now_us != 0U) && (now_us - spi_err_since_us_ < comm_hold))
      f |= kSpiCommError;
    if (i2c_err_since_us_ != 0U &&
        (now_us != 0U) && (now_us - i2c_err_since_us_ < comm_hold))
      f |= kI2cCommError;

    return f;
  }

  uint32_t last_can1_tx_ok_ = 0;
  uint32_t last_can2_tx_ok_ = 0;
  uint64_t last_can1_tx_ok_ts_us_ = 0;
  uint64_t last_can2_tx_ok_ts_us_ = 0;

  uint32_t last_sbus_pipe_dropped_ = 0;
  uint32_t last_can1_rx_dropped_ = 0;
  uint32_t last_can2_rx_dropped_ = 0;
  uint32_t last_can1_tx_mailbox_full_ = 0;
  uint32_t last_can2_tx_mailbox_full_ = 0;
  uint32_t last_can1_tx_add_fail_ = 0;
  uint32_t last_can2_tx_add_fail_ = 0;
  uint32_t last_can1_tx_start_fail_ = 0;
  uint32_t last_can2_tx_start_fail_ = 0;
  uint32_t last_can1_tx_cplt_fail_ = 0;
  uint32_t last_can2_tx_cplt_fail_ = 0;
  uint32_t last_spi_fail_ = 0;
  uint32_t last_i2c_fail_ = 0;
  uint32_t last_i2c_abort_ = 0;
  uint32_t last_remote_bad_frames_ = 0;

  uint64_t sbus_overflow_since_us_ = 0;
  uint64_t remote_bad_since_us_ = 0;
  uint64_t remote_bad_window_start_us_ = 0;
  uint32_t remote_bad_in_window_ = 0;
  uint64_t can1_rx_overflow_since_us_ = 0;
  uint64_t can2_rx_overflow_since_us_ = 0;
  uint64_t can1_tx_congest_since_us_ = 0;
  uint64_t can2_tx_congest_since_us_ = 0;
  uint64_t spi_err_since_us_ = 0;
  uint64_t i2c_err_since_us_ = 0;
};

}  // namespace robotapp::safety
