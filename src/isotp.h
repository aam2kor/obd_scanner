/**
 * @file isotp.h
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer.
 */

#ifndef ISOTP_H
#define ISOTP_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "can_hal.h"

#define ISOTP_MAX_PDU_LEN   512U

typedef struct {
    uint32_t rx_can_id;
    uint16_t pdu_len;
    uint8_t  pdu[ISOTP_MAX_PDU_LEN];
} isotp_rx_pdu_t;

/**
 * @brief Transmit a diagnostic PDU over ISO 15765-2 (Single-Frame in Step 2; Multi-Frame in Step 3).
 */
esp_err_t isotp_send_pdu(uint32_t tx_can_id, const uint8_t *pdu, uint16_t pdu_len);

/**
 * @brief Process an incoming CAN frame and extract a complete ISO-TP PDU if ready.
 * @return true if a complete PDU was reassembled into out_pdu, false otherwise.
 */
bool isotp_on_can_frame(
    const vci_can_frame_t *frame,
    uint32_t expected_resp_id,
    uint32_t req_id,
    isotp_rx_pdu_t *out_pdu
);

#endif /* ISOTP_H */

