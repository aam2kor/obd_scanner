/**
 * @file isotp.c
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer driven by D-PDU ComParams.
 */

#include "isotp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <string.h>

#define ISOTP_PCI_SF            0x00U
#define ISOTP_PCI_FF            0x01U
#define ISOTP_PCI_CF            0x02U
#define ISOTP_PCI_FC            0x03U

#define ISOTP_FC_FS_CTS         0x00U /* Continue To Send */
#define ISOTP_FC_FS_WAIT        0x01U /* Wait */
#define ISOTP_FC_FS_OVFLW       0x02U /* Overflow */

static bool is_matching_resp_id(uint32_t rx_id, const isotp_config_t *cfg)
{
    if (rx_id == cfg->can_resp_id) {
        return true;
    }

    /* When CP_ADDRESSING_TYPE == FUNCTIONAL (0), accept standard 11-bit and 29-bit OBD-II ECU responses */
    if (cfg->addressing_type == 0U) {
        /* 11-bit functional broadcast (0x7DF) -> ECU responses 0x7E8..0x7EF */
        if (cfg->can_req_id == 0x7DFU && rx_id >= 0x7E8U && rx_id <= 0x7EFU) {
            return true;
        }
        /* 29-bit functional broadcast (0x18DB33F1) -> ECU responses 0x18DAF100..0x18DAF1FF */
        if (cfg->can_req_id == 0x18DB33F1U && (rx_id & 0xFFFFFF00U) == 0x18DAF100U) {
            return true;
        }
    }
    return false;
}

static uint32_t derive_fc_tx_can_id(uint32_t rx_resp_id, const isotp_config_t *cfg)
{
    /* Per ISO 15765-4: when request was functional, Flow Control must be sent to the physical ECU ID */
    if (cfg->addressing_type == 0U) {
        /* 11-bit OBD-II response (0x7E8..0x7EF) -> physical tester-to-ECU ID is (rx_resp_id - 8) */
        if (rx_resp_id >= 0x7E8U && rx_resp_id <= 0x7EFU) {
            return rx_resp_id - 8U;
        }
        /* 29-bit OBD-II response (0x18DAF1xx) -> physical tester-to-ECU ID is 0x18DAxxF1 */
        if ((rx_resp_id & 0xFFFFFF00U) == 0x18DAF100U) {
            uint32_t ecu_sa = rx_resp_id & 0xFFU;
            return 0x18DA00F1U | (ecu_sa << 8);
        }
    }
    return cfg->can_req_id;
}

static esp_err_t send_fc_frame(
    uint32_t fc_tx_id,
    uint8_t fs,
    uint8_t bs,
    uint8_t stmin,
    uint8_t filler_byte
)
{
    vci_can_frame_t fc = {
        .id      = fc_tx_id,
        .is_extd = (fc_tx_id > 0x7FFU),
        .dlc     = 8U,
    };
    memset(fc.data, filler_byte, sizeof(fc.data));
    fc.data[0] = (uint8_t)((ISOTP_PCI_FC << 4) | (fs & 0x0FU));
    fc.data[1] = bs;
    fc.data[2] = stmin;
    return can_hal_send(&fc, 20);
}

void isotp_rx_init(isotp_rx_ctx_t *ctx)
{
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

esp_err_t isotp_send_pdu(
    const isotp_config_t *cfg,
    const uint8_t *pdu,
    uint16_t pdu_len
)
{
    if (cfg == NULL || pdu == NULL || pdu_len == 0U || pdu_len > ISOTP_MAX_PDU_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. Single-Frame TX (<= 7 bytes) */
    if (pdu_len <= 7U) {
        vci_can_frame_t sf = {
            .id      = cfg->can_req_id,
            .is_extd = (cfg->can_req_id > 0x7FFU),
            .dlc     = 8U,
        };
        memset(sf.data, cfg->can_filler_byte, sizeof(sf.data));
        sf.data[0] = (uint8_t)(pdu_len & 0x0FU);
        memcpy(&sf.data[1], pdu, pdu_len);
        return can_hal_send(&sf, 20);
    }

    /* 2. Multi-Frame TX (> 7 bytes): Send First Frame (FF) */
    vci_can_frame_t ff = {
        .id      = cfg->can_req_id,
        .is_extd = (cfg->can_req_id > 0x7FFU),
        .dlc     = 8U,
    };
    ff.data[0] = (uint8_t)((ISOTP_PCI_FF << 4) | ((pdu_len >> 8) & 0x0FU));
    ff.data[1] = (uint8_t)(pdu_len & 0xFFU);
    memcpy(&ff.data[2], pdu, 6U);

    esp_err_t err = can_hal_send(&ff, 20);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t offset = 6U;
    uint8_t  sn = 1U;
    uint8_t  bs_rem = 0U;
    uint8_t  stmin = 0U;
    bool     need_fc = true;
    uint32_t n_bs_ms = (cfg->n_bs_ms > 0U) ? cfg->n_bs_ms : 1000U;

    while (offset < pdu_len) {
        if (need_fc) {
            int64_t fc_deadline_us = esp_timer_get_time() + ((int64_t)n_bs_ms * 1000LL);
            bool got_cts = false;

            while (esp_timer_get_time() < fc_deadline_us) {
                vci_can_frame_t rx;
                if (can_hal_recv(&rx, 5) == ESP_OK) {
                    if (!is_matching_resp_id(rx.id, cfg) || rx.dlc < 3U) {
                        continue;
                    }
                    uint8_t pci_type = (rx.data[0] >> 4) & 0x0FU;
                    uint8_t fs       = rx.data[0] & 0x0FU;
                    if (pci_type != ISOTP_PCI_FC) {
                        continue;
                    }
                    if (fs == ISOTP_FC_FS_CTS) {
                        bs_rem  = rx.data[1];
                        stmin   = rx.data[2];
                        got_cts = true;
                        break;
                    } else if (fs == ISOTP_FC_FS_WAIT) {
                        fc_deadline_us = esp_timer_get_time() + ((int64_t)n_bs_ms * 1000LL);
                    } else {
                        return ESP_ERR_INVALID_SIZE; /* Overflow */
                    }
                }
            }
            if (!got_cts) {
                return ESP_ERR_TIMEOUT;
            }
            need_fc = false;
        }

        /* Respect STmin delay between Consecutive Frames */
        if (stmin > 0U) {
            if (stmin <= 0x7FU) {
                esp_rom_delay_us((uint32_t)stmin * 1000U);
            } else if (stmin >= 0xF1U && stmin <= 0xF9U) {
                esp_rom_delay_us((uint32_t)(stmin - 0xF0U) * 100U);
            }
        }

        vci_can_frame_t cf = {
            .id      = cfg->can_req_id,
            .is_extd = (cfg->can_req_id > 0x7FFU),
            .dlc     = 8U,
        };
        memset(cf.data, cfg->can_filler_byte, sizeof(cf.data));
        cf.data[0] = (uint8_t)((ISOTP_PCI_CF << 4) | (sn & 0x0FU));

        uint16_t chunk = pdu_len - offset;
        if (chunk > 7U) {
            chunk = 7U;
        }
        memcpy(&cf.data[1], &pdu[offset], chunk);

        err = can_hal_send(&cf, 20);
        if (err != ESP_OK) {
            return err;
        }

        offset += chunk;
        sn = (sn + 1U) & 0x0FU;

        if (bs_rem > 0U && offset < pdu_len) {
            bs_rem--;
            if (bs_rem == 0U) {
                need_fc = true;
            }
        }
    }

    return ESP_OK;
}

isotp_rx_status_t isotp_rx_feed_frame(
    isotp_rx_ctx_t *ctx,
    const vci_can_frame_t *frame,
    const isotp_config_t *cfg
)
{
    if (ctx == NULL || frame == NULL || cfg == NULL || frame->dlc < 2U) {
        return ISOTP_RX_IGNORED;
    }

    /* If multi-frame reassembly is already active, lock onto ctx->rx_can_id */
    if (ctx->in_progress) {
        if (frame->id != ctx->rx_can_id) {
            return ISOTP_RX_IGNORED;
        }
    } else {
        if (!is_matching_resp_id(frame->id, cfg)) {
            return ISOTP_RX_IGNORED;
        }
    }

    uint8_t pci_type = (frame->data[0] >> 4) & 0x0FU;

    switch (pci_type) {
        case ISOTP_PCI_SF: {
            /* Single Frame (0x0N) */
            uint8_t sf_len = frame->data[0] & 0x0FU;
            if (sf_len == 0U || sf_len > 7U || (1U + sf_len) > frame->dlc) {
                return ISOTP_RX_IGNORED;
            }
            ctx->in_progress = false;
            ctx->rx_can_id   = frame->id;
            ctx->total_len   = sf_len;
            ctx->offset      = sf_len;
            memcpy(ctx->pdu, &frame->data[1], sf_len);
            return ISOTP_RX_COMPLETE;
        }

        case ISOTP_PCI_FF: {
            /* First Frame (0x1N NN) */
            if (frame->dlc < 8U) {
                return ISOTP_RX_IGNORED;
            }
            uint16_t ff_len = (uint16_t)(((uint16_t)(frame->data[0] & 0x0FU) << 8) | frame->data[1]);
            uint32_t fc_id  = derive_fc_tx_can_id(frame->id, cfg);

            if (ff_len < 8U) {
                return ISOTP_RX_IGNORED;
            }
            if (ff_len > ISOTP_MAX_PDU_LEN) {
                /* Send Flow Control with Overflow (FS=0x02) per ISO 15765-2 */
                (void)send_fc_frame(fc_id, ISOTP_FC_FS_OVFLW, 0U, 0U, cfg->can_filler_byte);
                ctx->in_progress = false;
                return ISOTP_RX_ERR_OVERFLOW;
            }

            ctx->in_progress = true;
            ctx->rx_can_id   = frame->id;
            ctx->fc_tx_id    = fc_id;
            ctx->total_len   = ff_len;
            ctx->offset      = 6U;
            ctx->next_sn     = 1U;
            ctx->bs_counter  = cfg->fc_bs;
            memcpy(ctx->pdu, &frame->data[2], 6U);

            /* Transmit Flow Control (CTS = 0x30) back to the ECU */
            if (send_fc_frame(
                    ctx->fc_tx_id,
                    ISOTP_FC_FS_CTS,
                    cfg->fc_bs,
                    cfg->fc_stmin,
                    cfg->can_filler_byte
                ) != ESP_OK) {
                ctx->in_progress = false;
                return ISOTP_RX_ERR_FC_TX;
            }

            return ISOTP_RX_IN_PROGRESS;
        }

        case ISOTP_PCI_CF: {
            /* Consecutive Frame (0x2N) */
            if (!ctx->in_progress) {
                return ISOTP_RX_IGNORED;
            }
            uint8_t sn = frame->data[0] & 0x0FU;
            if (sn != ctx->next_sn) {
                ctx->in_progress = false;
                return ISOTP_RX_ERR_SEQ;
            }

            uint16_t rem = ctx->total_len - ctx->offset;
            uint16_t copy_len = (rem > 7U) ? 7U : rem;
            if ((1U + copy_len) > frame->dlc) {
                ctx->in_progress = false;
                return ISOTP_RX_ERR_SEQ;
            }

            memcpy(&ctx->pdu[ctx->offset], &frame->data[1], copy_len);
            ctx->offset  += copy_len;
            ctx->next_sn  = (ctx->next_sn + 1U) & 0x0FU;

            if (ctx->offset >= ctx->total_len) {
                ctx->in_progress = false;
                return ISOTP_RX_COMPLETE;
            }

            /* If BlockSize > 0 and block finished, transmit another Flow Control CTS */
            if (cfg->fc_bs > 0U && ctx->bs_counter > 0U) {
                ctx->bs_counter--;
                if (ctx->bs_counter == 0U) {
                    ctx->bs_counter = cfg->fc_bs;
                    if (send_fc_frame(
                            ctx->fc_tx_id,
                            ISOTP_FC_FS_CTS,
                            cfg->fc_bs,
                            cfg->fc_stmin,
                            cfg->can_filler_byte
                        ) != ESP_OK) {
                        ctx->in_progress = false;
                        return ISOTP_RX_ERR_FC_TX;
                    }
                }
            }

            return ISOTP_RX_IN_PROGRESS;
        }

        default:
            return ISOTP_RX_IGNORED;
    }
}
