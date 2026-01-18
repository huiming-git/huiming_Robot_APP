#pragma once

#include <cstdint>

extern "C" {
#include "app_time.h"
}

namespace robotapp::platform {

inline uint64_t now_us() { return App_Tim1_LastTimestampUs(); }

}  // namespace robotapp::platform

