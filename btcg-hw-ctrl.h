#ifndef BTCG_HW_CTRL_H
#define BTCG_HW_CTRL_H

#include "spi-context.h"

bool chip_selector_init(void);
bool chip_select(uint8_t id);

bool chip_reset(struct spi_ctx *ctx, unsigned clk_core);


/* Macros for checking chip status */
#define STATUS_W_ALLOW(status)  ((status) & 0x1)
#define STATUS_R_READY(status)  ((status) & 0x2)
#define STATUS_BUSY(status)     (((status) & 0x3) == 0)
#define STATUS_NONCE_GRP0_RDY(status)   (((status) >> 2) & 0x01)
#define STATUS_NONCE_GRP1_RDY(status)   (((status) >> 3) & 0x01)
#define STATUS_NONCE_GRP2_RDY(status)   (((status) >> 4) & 0x01)
#define STATUS_NONCE_GRP3_RDY(status)   (((status) >> 5) & 0x01)
#define STATUS_NONCE_NO_GRP_RDY(status) ((((status) >> 2) & 0x0f) == 0)

bool chip_status(struct spi_ctx *ctx, uint8_t *status);


bool chip_write_job(struct spi_ctx *ctx, const uint8_t *midstate, const uint8_t *wdata);
bool chip_read_nonce(struct spi_ctx *ctx, const unsigned int grp, uint32_t *nonce);
bool chip_clean(struct spi_ctx *ctx);

#endif  /* BTCG_HW_CTRL_H */
