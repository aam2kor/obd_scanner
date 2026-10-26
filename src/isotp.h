/**
 * @file isotp.h
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer driven by D-PDU ComParams.
 */

#ifndef ISOTP_H
#define ISOTP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "can_hal.h"

#define ISOTP_MAX_PDU_LEN       512U

typedef enum {
    ISOTP_RX_IGNORED = 0,       /* CAN ID or DLC not matching */
    ISOTP_RX_IN_PROGRESS,       /* FF or intermediate CF processed; wait for next CF within CP_N_CR_MS */
    ISOTP_RX_COMPLETE,          /* Complete SF or reassembled Multi-Frame PDU ready */
    ISOTP_RX_ERR_SEQ,           /* CF sequence number mismatch */
    ISOTP_RX_ERR_OVERFLOW,      /* FF length exceeds ISOTP_MAX_PDU_LEN */
    ISOTP_RX_ERR_FC_TX,         /* Failed to transmit Flow Control frame */
} isotp_rx_status_t;

/**
 * @brief ISO-TP configuration populated directly from the active D-PDU ComParam table.
 */
typedef struct {
    uint32_t can_req_id;        /* CP_CAN_PHYS_REQ_ID */
    uint32_t can_resp_id;       /* CP_CAN_RESP_USDT_ID */
    uint8_t  addressing_type;   /* CP_ADDRESSING_TYPE (0=FUNCTIONAL, 1=PHYSICAL) */
    uint8_t  can_filler_byte;   /* CP_CAN_FILLER_BYTE */
    uint8_t  fc_bs;             /* CP_BLOCK_SIZE */
    uint8_t  fc_stmin;          /* CP_ST_MIN */
    uint32_t n_bs_ms;           /* CP_N_BS_MS */
    uint32_t n_cr_ms;           /* CP_N_CR_MS */
} isotp_config_t;

typedef struct {
    bool     in_progress;
    uint32_t rx_can_id;
    uint32_t fc_tx_id;
    uint16_t total_len;
    uint16_t offset;
    uint8_t  next_sn;
    uint8_t  bs_counter;
    uint8_t  pdu[ISOTP_MAX_PDU_LEN];
} isotp_rx_ctx_t;

/**
 * @brief Initialize/reset an ISO-TP RX reassembly context.
 */
void isotp_rx_init(isotp_rx_ctx_t *ctx);

/**
 * @brief Transmit a diagnostic PDU over ISO 15765-2 (SF for <= 7B, Multi-Frame FF/FC/CF for > 7B).
 */
esp_err_t isotp_send_pdu(
    const isotp_config_t *cfg,
    const uint8_t *pdu,
    uint16_t pdu_len
);

/**
 * @brief Process an incoming CAN frame for ISO-TP Single-Frame or Multi-Frame RX reassembly.
 *        Automatically transmits Flow Control (0x30) frames using cfg->fc_bs, cfg->fc_stmin,
 *        and cfg->can_filler_byte.
 */
isotp_rx_status_t isotp_rx_feed_frame(
    isotp_rx_ctx_t *ctx,
    const vci_can_frame_t *frame,
    const isotp_config_t *cfg
);

#endif /* ISOTP_H */
