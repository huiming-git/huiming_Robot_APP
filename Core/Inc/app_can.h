/* Platform-neutral CAN app port header (no HAL types). */
#pragma once

#include <stdint.h>

#include "app_op.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t bus; /* 1=CAN1, 2=CAN2 */
  uint16_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} App_CanFrame_t;

uint8_t App_Can_Tx(uint8_t bus, uint16_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t App_Can_RxPop(App_CanFrame_t *out_frame);
uint8_t App_Can_TxOp(uint8_t bus, uint16_t std_id, const uint8_t* data, uint8_t dlc,
                     const App_Operation* op);

// LiveWatch-friendly diagnostics.
extern volatile uint32_t g_app_can1_rx_dropped;
extern volatile uint32_t g_app_can2_rx_dropped;
extern volatile uint32_t g_app_can1_tx_mailbox_full;
extern volatile uint32_t g_app_can2_tx_mailbox_full;
extern volatile uint32_t g_app_can1_tx_add_fail;
extern volatile uint32_t g_app_can2_tx_add_fail;

#ifdef __cplusplus
}
#endif
