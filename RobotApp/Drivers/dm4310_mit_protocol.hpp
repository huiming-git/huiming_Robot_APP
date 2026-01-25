#pragma once

#include <array>
#include <cstdint>

namespace robotapp::drivers::dm4310 {

// DMH4310 "MIT" style CAN protocol (as used in prior library).
// Convention used there:
// - MCU -> motor: StdId = CANID (1..4 by default), DLC=8
// - Motor -> MCU: StdId = MasterID (0x11..0x14 by default) or sometimes CANID.
//
// Special commands:
//   enable:  [FF FF FF FF FF FF FF FC]
//   disable: [FF FF FF FF FF FF FF FD]
//   zero:    [FF FF FF FF FF FF FF FE]
//
// Control command (8 bytes packed):
//   pos 16b, vel 12b, kp 12b, kd 12b, tor 12b
//   data[0] = pos[15:8]
//   data[1] = pos[7:0]
//   data[2] = vel[11:4]
//   data[3] = (vel[3:0] << 4) | kp[11:8]
//   data[4] = kp[7:0]
//   data[5] = kd[11:4]
//   data[6] = (kd[3:0] << 4) | tor[11:8]
//   data[7] = tor[7:0]
//
// Feedback (8 bytes):
//   data[0] high nibble = state, low nibble = motor id (1..4)
//   data[1..2] pos 16b
//   data[3..4] vel 12b
//   data[4..5] tor 12b
//   data[6] mos temp (uint8)
//   data[7] coil temp (uint8)

inline constexpr uint8_t kCmdEnableTail = 0xFC;
inline constexpr uint8_t kCmdDisableTail = 0xFD;
inline constexpr uint8_t kCmdZeroTail = 0xFE;

inline constexpr uint16_t kCanIdBase = 0x01;
inline constexpr uint16_t kMasterIdBase = 0x11;

inline constexpr float kPMin = -12.5f;
inline constexpr float kPMax = 12.5f;
inline constexpr float kVMin = -30.0f;
inline constexpr float kVMax = 30.0f;
inline constexpr float kKpMin = 0.0f;
inline constexpr float kKpMax = 500.0f;
inline constexpr float kKdMin = 0.0f;
inline constexpr float kKdMax = 5.0f;
inline constexpr float kTMin = -10.0f;
inline constexpr float kTMax = 10.0f;

inline constexpr uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

inline uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  const float span = x_max - x_min;
  const float offset = x - x_min;
  const uint32_t levels = (1u << bits) - 1u;
  const float scaled = offset * static_cast<float>(levels) / span;
  return static_cast<uint16_t>(scaled + 0.5f);
}

inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const uint32_t levels = (1u << bits) - 1u;
  return (static_cast<float>(x_int) * span / static_cast<float>(levels)) + x_min;
}

struct Feedback {
  uint8_t motor_id = 0;  // 1..4
  uint8_t state = 0;
  float pos = 0.0f;
  float vel = 0.0f;
  float tor = 0.0f;
  uint8_t mos_temp = 0;
  uint8_t coil_temp = 0;
};

inline std::array<uint8_t, 8> pack_special(uint8_t tail)
{
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, tail};
}

inline std::array<uint8_t, 8> pack_control(float kp, float kd, float pos, float vel, float tor)
{
  const uint16_t p_int = float_to_uint(pos, kPMin, kPMax, 16);
  const uint16_t v_int = float_to_uint(vel, kVMin, kVMax, 12);
  const uint16_t kp_int = float_to_uint(kp, kKpMin, kKpMax, 12);
  const uint16_t kd_int = float_to_uint(kd, kKdMin, kKdMax, 12);
  const uint16_t t_int = float_to_uint(tor, kTMin, kTMax, 12);

  std::array<uint8_t, 8> data{};
  data[0] = static_cast<uint8_t>(p_int >> 8);
  data[1] = static_cast<uint8_t>(p_int & 0xFFU);
  data[2] = static_cast<uint8_t>(v_int >> 4);
  data[3] = static_cast<uint8_t>(((v_int & 0x0FU) << 4) | static_cast<uint8_t>(kp_int >> 8));
  data[4] = static_cast<uint8_t>(kp_int & 0xFFU);
  data[5] = static_cast<uint8_t>(kd_int >> 4);
  data[6] = static_cast<uint8_t>(((kd_int & 0x0FU) << 4) | static_cast<uint8_t>(t_int >> 8));
  data[7] = static_cast<uint8_t>(t_int & 0xFFU);
  return data;
}

inline bool decode_feedback(const uint8_t data[8], Feedback& out)
{
  const uint8_t motor_id = static_cast<uint8_t>(data[0] & 0x0FU);
  if (motor_id > 15U) return false;

  const int p_int = (static_cast<int>(data[1]) << 8) | static_cast<int>(data[2]);
  const int v_int = (static_cast<int>(data[3]) << 4) | (static_cast<int>(data[4]) >> 4);
  const int t_int = ((static_cast<int>(data[4]) & 0x0F) << 8) | static_cast<int>(data[5]);

  out.motor_id = motor_id;
  out.state = static_cast<uint8_t>(data[0] >> 4);
  out.pos = uint_to_float(p_int, kPMin, kPMax, 16);
  out.vel = uint_to_float(v_int, kVMin, kVMax, 12);
  out.tor = uint_to_float(t_int, kTMin, kTMax, 12);
  out.mos_temp = data[6];
  out.coil_temp = data[7];
  return true;
}

}  // namespace robotapp::drivers::dm4310
