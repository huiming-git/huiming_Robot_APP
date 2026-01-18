#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace robotapp::drivers::dji3508 {

// DJI GM3508 + C620 ESC command/feedback (standard 11-bit IDs).
//
// Tx (control currents):
//   StdId = 0x200, DLC=8
//   data[0..1] current for motor1 (int16, big-endian)
//   data[2..3] current for motor2
//   data[4..5] current for motor3
//   data[6..7] current for motor4
//
// Rx (feedback, one frame per motor):
//   StdId = 0x201..0x204 (motor1..motor4), DLC=8
//   data[0..1] angle (uint16, big-endian, 0..8191)
//   data[2..3] speed_rpm (int16, big-endian)
//   data[4..5] current (int16, big-endian)
//   data[6]    temperature (uint8)
//   data[7]    reserved

inline constexpr uint16_t kTxStdId = 0x200;
inline constexpr uint16_t kRxStdIdBase = 0x201;
inline constexpr uint16_t kRxStdIdMax = 0x204;

struct Feedback {
  uint16_t angle = 0;
  int16_t speed_rpm = 0;
  int16_t current = 0;
  uint8_t temperature = 0;
};

inline constexpr uint16_t be16(const uint8_t hi, const uint8_t lo)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) |
                               static_cast<uint16_t>(lo));
}

inline constexpr int16_t be16s(const uint8_t hi, const uint8_t lo)
{
  return static_cast<int16_t>(be16(hi, lo));
}

inline std::array<uint8_t, 8> pack_currents(const std::array<int16_t, 4>& currents)
{
  std::array<uint8_t, 8> data{};
  for (std::size_t i = 0; i < 4; i++) {
    const uint16_t u = static_cast<uint16_t>(currents[i]);
    data[i * 2 + 0] = static_cast<uint8_t>(u >> 8);
    data[i * 2 + 1] = static_cast<uint8_t>(u & 0xFFU);
  }
  return data;
}

inline bool decode_feedback(uint16_t std_id, const uint8_t data[8], uint8_t& out_index_0based,
                            Feedback& out)
{
  if (std_id < kRxStdIdBase || std_id > kRxStdIdMax) return false;
  out_index_0based = static_cast<uint8_t>(std_id - kRxStdIdBase);

  out.angle = be16(data[0], data[1]);
  out.speed_rpm = be16s(data[2], data[3]);
  out.current = be16s(data[4], data[5]);
  out.temperature = data[6];
  return true;
}

}  // namespace robotapp::drivers::dji3508
