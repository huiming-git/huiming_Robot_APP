#include <stdint.h>

#include "imu/bmi08x_defs.h"

#include "app_spi.h"
#include "app_op.h"

// DMA-backed transport for Bosch BMI08x driver (bmi08a/bmi08g).
//
// The Bosch driver expects:
// - read(reg, buf, len): reg already contains SPI RD mask if needed.
// - For accel, caller may request (data_len + dev->dummy_byte) and then skip dummy in upper layer.
//   We therefore return exactly "len" bytes after the command byte (including dummy if requested).
//
// We keep ISR work minimal: raise CS, notify the waiting task.

typedef struct
{
    uint8_t tx[72];
    uint8_t rx[72];
} bmi088_dma_ctx_t;

static bmi088_dma_ctx_t g_bmi_dma = {0};

volatile uint32_t g_bmi_dma_txrx_ok = 0;
volatile uint32_t g_bmi_dma_txrx_fail = 0;

static uint8_t select_dev(void* intf_ptr)
{
    // intf_ptr points to acc_dev_add / gyro_dev_add set by drv_bmi088.c
    const uint8_t dev_addr = *(uint8_t*)intf_ptr;
    return (dev_addr == 0U) ? 0U : 1U;
}

static BMI08X_INTF_RET_TYPE start_txrx_dma(uint8_t dev, uint16_t total_len)
{
    if (total_len > sizeof(g_bmi_dma.tx)) return (BMI08X_INTF_RET_TYPE)-1;
    const App_Operation op = App_OpBlock(20U);
    if (App_Spi1_Bmi088_TxRxDmaOp(dev, g_bmi_dma.tx, g_bmi_dma.rx, total_len, &op) == 0U)
    {
        g_bmi_dma_txrx_fail++;
        return (BMI08X_INTF_RET_TYPE)-1;
    }
    g_bmi_dma_txrx_ok++;
    return BMI08X_INTF_RET_SUCCESS;
}

BMI08X_INTF_RET_TYPE RobotApp_BMI088_SpiRead_DMA(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    if (!reg_data || len == 0) return (BMI08X_INTF_RET_TYPE)-1;

    const uint8_t dev = select_dev(intf_ptr);

    // Transaction: [reg][dummy... len bytes]
    const uint16_t total = (uint16_t)(1U + len);
    g_bmi_dma.tx[0] = reg_addr;
    for (uint16_t i = 1; i < total; ++i) g_bmi_dma.tx[i] = 0x55;

    if (start_txrx_dma(dev, total) != BMI08X_INTF_RET_SUCCESS) return (BMI08X_INTF_RET_TYPE)-1;

    // Return exactly "len" bytes clocked after the command byte.
    for (uint32_t i = 0; i < len; ++i) reg_data[i] = g_bmi_dma.rx[i + 1U];
    return BMI08X_INTF_RET_SUCCESS;
}

BMI08X_INTF_RET_TYPE RobotApp_BMI088_SpiWrite_DMA(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    const uint8_t dev = select_dev(intf_ptr);

    const uint16_t total = (uint16_t)(1U + len);
    g_bmi_dma.tx[0] = reg_addr;
    for (uint32_t i = 0; i < len; ++i) g_bmi_dma.tx[i + 1U] = reg_data[i];

    // Full-duplex TXRX is used even for write to reuse the same completion callback.
    if (start_txrx_dma(dev, total) != BMI08X_INTF_RET_SUCCESS) return (BMI08X_INTF_RET_TYPE)-1;
    return BMI08X_INTF_RET_SUCCESS;
}
