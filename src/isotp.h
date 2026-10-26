/**
 * @file isotp.h
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer (Single-Frame & Multi-Frame FF/FC/CF).
 */

#ifndef ISOTP_H
#define ISOTP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "can_hal.h"

#define ISOTP_MAX_PDU_LEN       512U
#define ISOTP_N_BS_TIMEOUT_MS   1000U
#define ISOTP_N_CR_TIMEOUT_MS   1000U

typedef enum {
    ISOTP_RX_IGNORED = 0,       /* CAN ID or DLC not matching */
    ISOTP_RX_IN_PROGRESS,       /* FF or intermediate CF processed; wait for next CF within N_Cr */
    ISOTP_RX_COMPLETE,          /* Complete SF or reassembled Multi-Frame PDU ready */
    ISOTP_RX_ERR_SEQ,           /* CF sequence number mismatch */
    ISOTP_RX_ERR_OVERFLOW,      /* FF length exceeds ISOTP_MAX_PDU_LEN */
    ISOTP_RX_ERR_FC_TX,         /* Failed to transmit Flow Control frame */
} isotp_rx_status_t;

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
    uint32_t tx_can_id,
    uint32_t expected_fc_can_id,
    const uint8_t *pdu,
    uint16_t pdu_len
);

/**
 * @brief Process an incoming CAN frame for ISO-TP Single-Frame or Multi-Frame RX reassembly.
 *        Automatically transmits Flow Control (0x30) frames upon receiving FF or BS block end.
 */
isotp_rx_status_t isotp_rx_feed_frame(
    isotp_rx_ctx_t *ctx,
    const vci_can_frame_t *frame,
    uint32_t expected_resp_id,
    uint32_t req_id,
    uint8_t fc_bs,
    uint8_t fc_stmin_ms
);

#endif /* ISOTP_H */
