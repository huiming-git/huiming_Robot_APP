#pragma once

#include <array>
#include <cstdint>

namespace robotapp::inputs {

// 标准 SBUS 帧解析：25 字节，起始 0x0F，末尾保留。
struct SbusFrame {
  std::array<uint16_t, 16> ch{};  // 0..2047
  bool ch17 = false;
  bool ch18 = false;
  bool frame_lost = false;
  bool failsafe = false;
};

class SbusParser {
 public:
  // 逐字节喂入，返回 true 表示 out_frame 已填好一帧。
  bool feed(uint8_t byte, SbusFrame& out_frame);

  uint32_t good_frames() const { return good_frames_; }
  uint32_t bad_frames() const { return bad_frames_; }

 private:
  static constexpr uint8_t kStart = 0x0F;
  static constexpr std::size_t kFrameLen = 25;
  std::array<uint8_t, kFrameLen> buf_{};
  std::size_t idx_ = 0;
  uint32_t good_frames_ = 0;
  uint32_t bad_frames_ = 0;
};

}  // namespace robotapp::inputs
