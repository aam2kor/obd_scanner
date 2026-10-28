/**
 * @file isotp.h
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer with 3x 4,095B Multi-Frame RX Pool,
 *        per-CLL 8B Single-Frame Buffer, and 100 us STmin resolution.
 */

#ifndef ISOTP_H
#define ISOTP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "can_hal.h"
#include "vci_config.h"

#define ISOTP_MAX_PDU_LEN       VCI_ISOTP_MAX_PDU_LEN   /* 4095 bytes */

typedef enum {
    ISOTP_RX_IGNORED = 0,       /* CAN ID or DLC not matching */
    ISOTP_RX_IN_PROGRESS,       /* FF or intermediate CF processed; wait for next CF within CP_N_CR_MS */
    ISOTP_RX_COMPLETE,          /* Complete SF (in CLL 8B buf) or Multi-Frame (in 4095B pool) ready */
    ISOTP_RX_ERR_SEQ,           /* CF sequence number mismatch */
    ISOTP_RX_ERR_OVERFLOW,      /* FF length > 4095B or all 3 MF buffers busy */
    ISOTP_RX_ERR_FC_TX,         /* Failed to transmit Flow Control frame */
} isotp_rx_status_t;

/**
 * @brief Shared 4,095-byte Multi-Frame RX assembly buffer from the 3-buffer pool.
 */
typedef struct {
    bool     in_use;
    uint8_t  owner_cll_id;
    uint32_t rx_can_id;
    uint32_t fc_tx_id;
    uint16_t total_len;
    uint16_t offset;
    uint8_t  next_sn;
    uint8_t  bs_counter;
    uint8_t  result_hdr_pad[9]; /* 9B headroom for in-place 0x81 PDU_IT_RESULT header */
    uint8_t  data[ISOTP_MAX_PDU_LEN];
} isotp_mf_buf_t;

/**
 * @brief ISO-TP configuration populated from the active CLL ComParam table.
 */
typedef struct {
    uint8_t  cll_id;            /* Target CLL_ID (1..8) */
    uint32_t can_req_id;        /* Outgoing Tester CAN ID (from COP or CP_CAN_PHYS_REQ_ID) */
    uint32_t can_resp_id;       /* Expected ECU Response CAN ID (CP_CAN_RESP_USDT_ID) */
    uint8_t  addressing_type;   /* CP_ADDRESSING_TYPE (0=FUNCTIONAL, 1=PHYSICAL) */
    uint8_t  can_filler_byte;   /* CP_CAN_FILLER_BYTE */
    uint8_t  fc_bs;             /* CP_BLOCK_SIZE */
    uint8_t  fc_stmin;          /* CP_ST_MIN (0..127 ms or 0xF1..0xF9 = 100..900 us) */
    uint32_t n_bs_ms;           /* CP_N_BS_MS */
    uint32_t n_cr_ms;           /* CP_N_CR_MS */
} isotp_config_t;

/**
 * @brief Per-transaction ISO-TP RX state binding a CLL's 8B SF buffer and the 3x 4,095B MF pool.
 */
typedef struct {
    uint8_t        *cll_sf_buf;     /* Pointer to the CLL's dedicated 8-byte Single-Frame buffer */
    isotp_mf_buf_t *active_mf;      /* Active 4,095B pool slot if a Multi-Frame transfer is in progress */
    bool            in_progress;    /* True while waiting for Consecutive Frames */
    uint32_t        rx_can_id;      /* CAN ID of the completed or in-progress response */
    uint16_t        total_len;      /* Completed N-PDU length (1..7 for SF, 8..4095 for MF) */
    const uint8_t  *completed_pdu;  /* Points to cll_sf_buf (for SF) or active_mf->data (for MF) */
} isotp_rx_ctx_t;

/**
 * @brief Reset the entire 3x 4,095-byte Multi-Frame RX buffer pool (e.g. on boot or PDUDestruct).
 */
void isotp_mf_pool_reset_all(void);

/**
 * @brief Return a pointer to a 4,095-byte buffer from the pool for large multi-frame TX payloads (> 64B).
 */
uint8_t *isotp_mf_pool_get_large_tx_buf(void);

/**
 * @brief Release any Multi-Frame RX buffers owned by a specific CLL_ID.
 */
void isotp_mf_pool_release_cll(uint8_t cll_id);

/**
 * @brief Initialize an ISO-TP RX context bound to a CLL's 8-byte Single-Frame buffer.
 */
void isotp_rx_init(isotp_rx_ctx_t *ctx, uint8_t *cll_sf_buf);

/**
 * @brief Release the active Multi-Frame buffer (if any) held by an ISO-TP RX context.
 */
void isotp_rx_release(isotp_rx_ctx_t *ctx);

/**
 * @brief Convert an ISO 15765-2 STmin byte (0x00..0x7F or 0xF1..0xF9) to microseconds (100 us resolution).
 */
uint32_t isotp_stmin_to_us(uint8_t stmin);

/**
 * @brief Transmit a diagnostic PDU over ISO 15765-2 (SF for <= 7B, Multi-Frame FF/FC/CF for 8..4095B).
 */
esp_err_t isotp_send_pdu(
    const isotp_config_t *cfg,
    const uint8_t *pdu,
    uint16_t pdu_len
);

/**
 * @brief Process an incoming CAN frame:
 *        - Single Frames (<= 7B) use ctx->cll_sf_buf (8B per-CLL buffer).
 *        - Multi-Frames (8..4095B) allocate from the 3x 4,095B Multi-Frame pool.
 */
isotp_rx_status_t isotp_rx_feed_frame(
    isotp_rx_ctx_t *ctx,
    const vci_can_frame_t *frame,
    const isotp_config_t *cfg
);

#endif /* ISOTP_H */
