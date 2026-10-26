/**
 * @file isotp.c
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer (Single-Frame Step 2).
 */

#include "isotp.h"
#include <string.h>

esp_err_t isotp_send_pdu(uint32_t tx_can_id, const uint8_t *pdu, uint16_t pdu_len)
{
    if (pdu == NULL || pdu_len == 0U || pdu_len > 7U) {
        return ESP_ERR_INVALID_ARG;
    }

    vci_can_frame_t frame = {
        .id      = tx_can_id,
        .is_extd = (tx_can_id > 0x7FFU),
        .dlc     = 8U,
    };
    memset(frame.data, 0xCCU, sizeof(frame.data));
    frame.data[0] = (uint8_t)(pdu_len & 0x0FU); /* PCI = 0x0N (Single Frame) */
    memcpy(&frame.data[1], pdu, pdu_len);

    return can_hal_send(&frame, 20);
}

static bool is_matching_resp_id(uint32_t rx_id, uint32_t expected_resp_id, uint32_t req_id)
{
    if (rx_id == expected_resp_id) {
        return true;
    }
    /* For OBD-II functional broadcast (0x7DF), accept any standard ECU response (0x7E8..0x7EF) */
    if (req_id == 0x7DFU && rx_id >= 0x7E8U && rx_id <= 0x7EFU) {
        return true;
    }
    return false;
}

bool isotp_on_can_frame(
    const vci_can_frame_t *frame,
    uint32_t expected_resp_id,
    uint32_t req_id,
    isotp_rx_pdu_t *out_pdu
)
{
    if (frame == NULL || out_pdu == NULL || frame->dlc < 2U) {
        return false;
    }
    if (!is_matching_resp_id(frame->id, expected_resp_id, req_id)) {
        return false;
    }

    uint8_t pci_type = (frame->data[0] >> 4) & 0x0FU;
    if (pci_type == 0x00U) {
        /* Single Frame (SF): PCI = 0x0N */
        uint8_t sf_len = frame->data[0] & 0x0FU;
        if (sf_len >= 1U && sf_len <= 7U && (1U + sf_len) <= frame->dlc) {
            out_pdu->rx_can_id = frame->id;
            out_pdu->pdu_len   = sf_len;
            memcpy(out_pdu->pdu, &frame->data[1], sf_len);
            return true;
        }
    }

    return false;
}

