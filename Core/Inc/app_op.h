/* Platform-neutral operation model for I/O completion behavior. */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_OP_NONE = 0,
  APP_OP_BLOCK = 1,
  APP_OP_CALLBACK = 2,
  APP_OP_POLLING = 3,
} App_OpMode;

typedef void (*App_OpCallback)(uint8_t ok, uint8_t in_isr, void* user);

typedef struct {
  App_OpMode mode;
  uint32_t timeout_ms;   /* used by BLOCK/POLLING */
  App_OpCallback cb;     /* used by CALLBACK */
  void* user;            /* callback user pointer */
} App_Operation;

static inline App_Operation App_OpNone(void)
{
  App_Operation op = {APP_OP_NONE, 0U, (App_OpCallback)0, (void*)0};
  return op;
}

static inline App_Operation App_OpBlock(uint32_t timeout_ms)
{
  App_Operation op = {APP_OP_BLOCK, timeout_ms, (App_OpCallback)0, (void*)0};
  return op;
}

static inline App_Operation App_OpMakeCallback(App_OpCallback cb, void* user)
{
  App_Operation op = {APP_OP_CALLBACK, 0U, cb, user};
  return op;
}

#ifdef __cplusplus
}
#endif
