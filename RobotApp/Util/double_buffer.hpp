#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace robotapp::util {

template <class T>
class DoubleBuffer {
 public:
  constexpr DoubleBuffer() = default;

  void push(const T& value)
  {
    const uint32_t next = (index_.load(std::memory_order_relaxed) ^ 1U);
    buffers_[next] = value;
    index_.store(next, std::memory_order_release);
  }

  void push(T&& value)
  {
    const uint32_t next = (index_.load(std::memory_order_relaxed) ^ 1U);
    buffers_[next] = std::move(value);
    index_.store(next, std::memory_order_release);
  }

  T latest() const
  {
    const uint32_t idx = index_.load(std::memory_order_acquire);
    return buffers_[idx];
  }

 private:
  std::array<T, 2> buffers_{};
  std::atomic<uint32_t> index_{0};
};

}  // namespace robotapp::util

