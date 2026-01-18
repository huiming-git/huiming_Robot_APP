#pragma once

#include <cstdint>

extern "C" {
#include "app_op.h"
}

namespace robotapp::platform {

using OpMode = App_OpMode;
using Operation = App_Operation;

constexpr Operation OpNone() { return App_OpNone(); }
constexpr Operation OpBlock(uint32_t timeout_ms) { return App_OpBlock(timeout_ms); }
constexpr Operation OpCallback(App_OpCallback cb, void* user) { return App_OpMakeCallback(cb, user); }

}  // namespace robotapp::platform
