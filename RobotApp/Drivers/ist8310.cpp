#include "RobotApp/Drivers/ist8310.hpp"
#include "RobotApp/Platform/i2c_port.hpp"
#include "RobotApp/Domain/config.hpp"

extern "C" {
#include "app_time.h"
}

namespace robotapp::drivers::ist8310 {

namespace {
class Device {
 public:
  explicit Device(platform::I2cPort* i2c) : i2c_(i2c) {}

  bool init();
  bool read_raw(RawFrame& out);
  bool kick_read_async(uint64_t ts_us);

  Buffer& buffer() { return buffer_; }

 private:
  platform::I2cPort* i2c_;
  Buffer buffer_{};
};

bool i2c_write_reg(platform::I2cPort* i2c, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  return (i2c != nullptr) &&
         i2c->write(kI2cAddr, std::span<const uint8_t>(buf, sizeof(buf)), platform::OpBlock(20));
}

bool i2c_read_regs(platform::I2cPort* i2c, uint8_t reg, uint8_t* dst, uint16_t len)
{
  if (i2c == nullptr) return false;
  return i2c->write_read(kI2cAddr, std::span<const uint8_t>(&reg, 1),
                         std::span<uint8_t>(dst, len), platform::OpBlock(20));
}
}  // namespace

bool Device::init()
{
  uint8_t who = 0;
  if (!i2c_read_regs(i2c_, 0x00, &who, 1)) return false;
  // WHO_AM_I should be 0x10
  if (who != 0x10) return false;
  // Soft reset
  i2c_write_reg(i2c_, 0x0B, 0x01);
  if (i2c_) i2c_->delay_ms(10);
  // Config: AVGCNTL (oversampling), PDNT bit
  i2c_write_reg(i2c_, 0x0A, 0x08);  // 200Hz
  i2c_write_reg(i2c_, 0x0B, 0x00);  // Normal mode
  return true;
}

bool Device::read_raw(RawFrame& out)
{
  uint8_t buf[6] = {0};
  if (!i2c_read_regs(i2c_, 0x03, buf, sizeof(buf))) return false;
  out.mx = static_cast<int16_t>((buf[1] << 8) | buf[0]);
  out.my = static_cast<int16_t>((buf[3] << 8) | buf[2]);
  out.mz = static_cast<int16_t>((buf[5] << 8) | buf[4]);
  return true;
}

struct AsyncCtx {
  Device* dev = nullptr;
  uint8_t reg = 0x03;
  uint8_t rx[6] = {0};
  uint64_t ts_us = 0;
  volatile uint8_t in_progress = 0;
  uint32_t fail_streak = 0;
  uint64_t backoff_until_us = 0;
};

static AsyncCtx g_async{};

volatile uint32_t g_ist_i2c_start_fail = 0;
volatile uint32_t g_ist_i2c_cplt_fail = 0;
volatile uint32_t g_ist_i2c_started = 0;
volatile uint32_t g_ist_i2c_backoff_entered = 0;
volatile uint32_t g_ist_i2c_backoff_skipped = 0;
volatile uint32_t g_ist_i2c_ok = 0;

static void on_i2c_done(uint8_t ok, uint8_t /*in_isr*/, void* user)
{
  auto* ctx = reinterpret_cast<AsyncCtx*>(user);
  if (ctx == nullptr) return;

  if (!ok)
  {
    g_ist_i2c_cplt_fail += 1U;
    ctx->fail_streak += 1U;
    if (ctx->fail_streak >= domain::config::kIstFailStreakThreshold)
    {
      ctx->backoff_until_us = ctx->ts_us + domain::config::kIstBackoffUs;
      g_ist_i2c_backoff_entered += 1U;
    }
    ctx->in_progress = 0U;
    return;
  }

  g_ist_i2c_ok += 1U;
  ctx->fail_streak = 0;
  ctx->backoff_until_us = 0;

  RawFrame f{};
  f.ts_us = static_cast<uint32_t>(ctx->ts_us);
  f.mx = static_cast<int16_t>((ctx->rx[1] << 8) | ctx->rx[0]);
  f.my = static_cast<int16_t>((ctx->rx[3] << 8) | ctx->rx[2]);
  f.mz = static_cast<int16_t>((ctx->rx[5] << 8) | ctx->rx[4]);
  if (ctx->dev != nullptr)
  {
    ctx->dev->buffer().push(f);
    g_ist_debug_shadow = f;
  }
  ctx->in_progress = 0U;
}

bool Device::kick_read_async(uint64_t ts_us)
{
  if (i2c_ == nullptr) return false;
  if (g_async.backoff_until_us != 0U && ts_us != 0U && ts_us < g_async.backoff_until_us)
  {
    g_ist_i2c_backoff_skipped += 1U;
    return false;
  }
  if (g_async.in_progress) return false;
  g_async.in_progress = 1U;
  g_async.dev = this;
  g_async.ts_us = ts_us;

  const auto op = platform::OpCallback(on_i2c_done, &g_async);
  const bool started =
      i2c_->write_read(kI2cAddr, std::span<const uint8_t>(&g_async.reg, 1),
                       std::span<uint8_t>(g_async.rx, sizeof(g_async.rx)), op);
  if (!started)
  {
    g_ist_i2c_start_fail += 1U;
    g_async.fail_streak += 1U;
    if (g_async.fail_streak >= domain::config::kIstFailStreakThreshold)
    {
      g_async.backoff_until_us = ts_us + domain::config::kIstBackoffUs;
      g_ist_i2c_backoff_entered += 1U;
    }
    g_async.in_progress = 0U;
    return false;
  }
  g_ist_i2c_started += 1U;
  return true;
}

// C wrappers
extern "C" {
static robotapp::platform::AppI2c3Port g_i2c3{};
static Device g_ist(&g_i2c3);
RawFrame g_ist_debug_shadow{};

bool IST8310_Init(void) { return g_ist.init(); }

bool IST8310_PollOnce(void)
{
  RawFrame f{};
  f.ts_us = App_Tim1_LastTimestampUs();
  if (!g_ist.read_raw(f)) return false;
  g_ist.buffer().push(f);
  g_ist_debug_shadow = f;  // For LiveWatch/GDB observation without函数调用
  return true;
}

bool IST8310_KickAsync(void)
{
  const uint64_t ts_us = App_Tim1_LastTimestampUs();
  return g_ist.kick_read_async(ts_us);
}

RawFrame IST8310_Latest(void) { return g_ist.buffer().latest(); }
}

}  // namespace robotapp::drivers::ist8310
