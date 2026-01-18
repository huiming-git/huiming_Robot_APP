#include "RobotApp/Inputs/sbus.hpp"

namespace robotapp::inputs {

bool SbusParser::feed(uint8_t byte, SbusFrame& out_frame)
{
  if (idx_ == 0 && byte != kStart)
  {
    // 等待帧头
    return false;
  }

  buf_[idx_++] = byte;
  if (idx_ < kFrameLen)
    return false;

  // 一帧到齐
  idx_ = 0;

  // Basic footer check (SBUS2 variants may use 0x04 as end byte).
  if (buf_[0] != kStart || (buf_[24] != 0x00 && buf_[24] != 0x04))
  {
    bad_frames_ += 1U;
    return false;
  }

  // 解码 16 路 11bit（标准 SBUS 帧：byte[1..22] 为 packed channel bits）
  const uint8_t* b = buf_.data();
  out_frame.ch[0] = (uint16_t)((b[1] | (b[2] << 8)) & 0x07FF);
  out_frame.ch[1] = (uint16_t)(((b[2] >> 3) | (b[3] << 5)) & 0x07FF);
  out_frame.ch[2] = (uint16_t)(((b[3] >> 6) | (b[4] << 2) | (b[5] << 10)) & 0x07FF);
  out_frame.ch[3] = (uint16_t)(((b[5] >> 1) | (b[6] << 7)) & 0x07FF);
  out_frame.ch[4] = (uint16_t)(((b[6] >> 4) | (b[7] << 4)) & 0x07FF);
  out_frame.ch[5] = (uint16_t)(((b[7] >> 7) | (b[8] << 1) | (b[9] << 9)) & 0x07FF);
  out_frame.ch[6] = (uint16_t)(((b[9] >> 2) | (b[10] << 6)) & 0x07FF);
  out_frame.ch[7] = (uint16_t)(((b[10] >> 5) | (b[11] << 3)) & 0x07FF);
  out_frame.ch[8] = (uint16_t)((b[12] | (b[13] << 8)) & 0x07FF);
  out_frame.ch[9] = (uint16_t)(((b[13] >> 3) | (b[14] << 5)) & 0x07FF);
  out_frame.ch[10] = (uint16_t)(((b[14] >> 6) | (b[15] << 2) | (b[16] << 10)) & 0x07FF);
  out_frame.ch[11] = (uint16_t)(((b[16] >> 1) | (b[17] << 7)) & 0x07FF);
  out_frame.ch[12] = (uint16_t)(((b[17] >> 4) | (b[18] << 4)) & 0x07FF);
  out_frame.ch[13] = (uint16_t)(((b[18] >> 7) | (b[19] << 1) | (b[20] << 9)) & 0x07FF);
  out_frame.ch[14] = (uint16_t)(((b[20] >> 2) | (b[21] << 6)) & 0x07FF);
  out_frame.ch[15] = (uint16_t)(((b[21] >> 5) | (b[22] << 3)) & 0x07FF);

  const uint8_t flags = b[23];
  out_frame.ch17 = (flags & (1U << 0)) != 0U;
  out_frame.ch18 = (flags & (1U << 1)) != 0U;
  out_frame.frame_lost = (flags & (1U << 2)) != 0U;
  out_frame.failsafe = (flags & (1U << 3)) != 0U;

  good_frames_ += 1U;
  return true;
}

}  // namespace robotapp::inputs
