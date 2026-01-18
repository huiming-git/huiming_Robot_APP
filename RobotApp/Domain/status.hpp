#pragma once

#include <cstdint>

namespace robotapp::domain {

enum class ErrorCode : uint8_t {
  kOk = 0,
  kNotReady,
  kBadParam,
  kIoError,
  kTimeout,
};

struct Status {
  ErrorCode code = ErrorCode::kOk;

  constexpr bool ok() const { return code == ErrorCode::kOk; }
  static constexpr Status Ok() { return {}; }
  static constexpr Status Err(ErrorCode c) { return Status{c}; }
};

}  // namespace robotapp::domain

