/**
 * @file isotp.c
 * @brief Layer 3: ISO 15765-2 (ISO-TP) Transport Layer with 3x 4,095B Multi-Frame RX Pool,
 *        per-CLL 8B Single-Frame Buffer, and 100 us STmin resolution.
 */

#include "isotp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <stdlib.h>
#include <string.h>

#define ISOTP_PCI_SF            0x00U
#define ISOTP_PCI_FF            0x01U
#define ISOTP_PCI_CF            0x02U
#define ISOTP_PCI_FC            0x03U

#define ISOTP_FC_FS_CTS         0x00U /* Continue To Send */
#define ISOTP_FC_FS_WAIT        0x01U /* Wait */
#define ISOTP_FC_FS_OVFLW       0x02U /* Overflow */

/* Shared Pool of 3x 4,095-Byte Multi-Frame Assembly Buffers (12.3 KB total in .bss) */
static isotp_mf_buf_t s_mf_pool[VCI_MF_POOL_COUNT];

void isotp_mf_pool_reset_all(void)
{
    for (uint8_t i = 0U; i < VCI_MF_POOL_COUNT; i++) {
        s_mf_pool[i].in_use       = false;
        s_mf_pool[i].owner_cll_id = 0U;
        s_mf_pool[i].rx_can_id    = 0U;
        s_mf_pool[i].total_len    = 0U;
        s_mf_pool[i].offset       = 0U;
    }
}

uint8_t *isotp_mf_pool_get_large_tx_buf(void)
{
    /* Returns 4,104 contiguous bytes (result_hdr_pad[9] + data[4095]) */
    return &s_mf_pool[VCI_MF_POOL_COUNT - 1U].result_hdr_pad[0];
}

void isotp_mf_pool_release_cll(uint8_t cll_id)
{
    for (uint8_t i = 0U; i < VCI_MF_POOL_COUNT; i++) {
        if (s_mf_pool[i].in_use && s_mf_pool[i].owner_cll_id == cll_id) {
            s_mf_pool[i].in_use       = false;
            s_mf_pool[i].owner_cll_id = 0U;
        }
    }
}

static isotp_mf_buf_t *alloc_mf_buf(uint8_t cll_id, uint32_t rx_can_id)
{
    /* Reuse existing slot if the same ECU restarted a First Frame */
    for (uint8_t i = 0U; i < VCI_MF_POOL_COUNT; i++) {
        if (s_mf_pool[i].in_use && s_mf_pool[i].rx_can_id == rx_can_id) {
            s_mf_pool[i].owner_cll_id = cll_id;
            return &s_mf_pool[i];
        }
    }
    /* Otherwise allocate a free slot from the 3x 4,095B pool */
    for (uint8_t i = 0U; i < VCI_MF_POOL_COUNT; i++) {
        if (!s_mf_pool[i].in_use) {
            s_mf_pool[i].in_use       = true;
            s_mf_pool[i].owner_cll_id = cll_id;
            s_mf_pool[i].rx_can_id    = rx_can_id;
            return &s_mf_pool[i];
        }
    }
    return NULL;
}

static isotp_mf_buf_t *find_active_mf_buf(uint8_t cll_id, uint32_t rx_can_id)
{
    for (uint8_t i = 0U; i < VCI_MF_POOL_COUNT; i++) {
        if (s_mf_pool[i].in_use &&
            s_mf_pool[i].owner_cll_id == cll_id &&
            s_mf_pool[i].rx_can_id == rx_can_id) {
            return &s_mf_pool[i];
        }
    }
    return NULL;
}

uint32_t isotp_stmin_to_us(uint8_t stmin)
{
    if (stmin <= 0x7FU) {
        return (uint32_t)stmin * 1000UL;          /* 0x00..0x7F -> 0..127,000 us (1 ms steps) */
    }
    if (stmin >= 0xF1U && stmin <= 0xF9U) {
        return (uint32_t)(stmin - 0xF0U) * 100UL; /* 0xF1..0xF9 -> 100..900 us (100 us steps) */
    }
    return 127000UL; /* ISO 15765-2 fallback for reserved values */
}

static bool is_matching_resp_id(uint32_t rx_id, const isotp_config_t *cfg)
{
    if (rx_id == cfg->can_resp_id) {
        return true;
    }

    /* When CP_ADDRESSING_TYPE == FUNCTIONAL (0) or req_id is 0x7DF / 0x18DB33F1, accept OBD-II ECUs */
    if (cfg->addressing_type == 0U || cfg->can_req_id == 0x7DFU || cfg->can_req_id == 0x18DB33F1U) {
        if (cfg->can_req_id == 0x7DFU && rx_id >= 0x7E8U && rx_id <= 0x7EFU) {
            return true;
        }
        if (cfg->can_req_id == 0x18DB33F1U && (rx_id & 0xFFFFFF00U) == 0x18DAF100U) {
            return true;
        }
    }

    /* Also match standard physical OBD-II pair (0x7E0..0x7E7 -> 0x7E8..0x7EF) if req_id was overridden */
    if (cfg->can_req_id >= 0x7E0U && cfg->can_req_id <= 0x7E7U && rx_id == (cfg->can_req_id + 8U)) {
        return true;
    }
    if ((cfg->can_req_id & 0xFFFF00FFU) == 0x18DA00F1U) {
        uint32_t ecu_sa = (cfg->can_req_id >> 8) & 0xFFU;
        if (rx_id == (0x18DAF100U | ecu_sa)) {
            return true;
        }
    }
    return false;
}

static uint32_t derive_fc_tx_can_id(uint32_t rx_resp_id, const isotp_config_t *cfg)
{
    /* Per ISO 15765-4: Flow Control must always be sent to the physical ECU CAN ID */
    if (rx_resp_id >= 0x7E8U && rx_resp_id <= 0x7EFU) {
        return rx_resp_id - 8U;
    }
    if ((rx_resp_id & 0xFFFFFF00U) == 0x18DAF100U) {
        uint32_t ecu_sa = rx_resp_id & 0xFFU;
        return 0x18DA00F1U | (ecu_sa << 8);
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

void isotp_rx_init(isotp_rx_ctx_t *ctx, uint8_t *cll_sf_buf)
{
    if (ctx != NULL) {
        ctx->cll_sf_buf    = cll_sf_buf;
        ctx->active_mf     = NULL;
        ctx->in_progress   = false;
        ctx->rx_can_id     = 0U;
        ctx->total_len     = 0U;
        ctx->completed_pdu = NULL;
    }
}

void isotp_rx_release(isotp_rx_ctx_t *ctx)
{
    if (ctx != NULL) {
        if (ctx->active_mf != NULL) {
            ctx->active_mf->in_use       = false;
            ctx->active_mf->owner_cll_id = 0U;
            ctx->active_mf               = NULL;
        }
        ctx->in_progress   = false;
        ctx->completed_pdu = NULL;
        ctx->total_len     = 0U;
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

    /* 2. Multi-Frame TX (8..4095 bytes): Send First Frame (FF) */
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

        /* Respect STmin delay with 100 us hardware resolution */
        uint32_t delay_us = isotp_stmin_to_us(stmin);
        if (delay_us > 0U) {
            esp_rom_delay_us(delay_us);
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

    if (!is_matching_resp_id(frame->id, cfg)) {
        return ISOTP_RX_IGNORED;
    }

    uint8_t pci_type = (frame->data[0] >> 4) & 0x0FU;

    switch (pci_type) {
        case ISOTP_PCI_SF: {
            /* Single Frame (0x0N): store directly in the CLL's 8-byte SF buffer */
            uint8_t sf_len = frame->data[0] & 0x0FU;
            if (sf_len == 0U || sf_len > 7U || (1U + sf_len) > frame->dlc || ctx->cll_sf_buf == NULL) {
                return ISOTP_RX_IGNORED;
            }
            memcpy(ctx->cll_sf_buf, &frame->data[1], sf_len);
            ctx->rx_can_id     = frame->id;
            ctx->total_len     = sf_len;
            ctx->completed_pdu = ctx->cll_sf_buf;
            return ISOTP_RX_COMPLETE;
        }

        case ISOTP_PCI_FF: {
            /* First Frame (0x1N NN): allocate a 4,095B buffer from the 3-buffer pool */
            if (frame->dlc < 8U) {
                return ISOTP_RX_IGNORED;
            }
            uint16_t ff_len = (uint16_t)(((uint16_t)(frame->data[0] & 0x0FU) << 8) | frame->data[1]);
            uint32_t fc_id  = derive_fc_tx_can_id(frame->id, cfg);

            if (ff_len < 8U) {
                return ISOTP_RX_IGNORED;
            }
            if (ff_len > ISOTP_MAX_PDU_LEN) {
                (void)send_fc_frame(fc_id, ISOTP_FC_FS_OVFLW, 0U, 0U, cfg->can_filler_byte);
                return ISOTP_RX_ERR_OVERFLOW;
            }

            isotp_mf_buf_t *mf = alloc_mf_buf(cfg->cll_id, frame->id);
            if (mf == NULL) {
                /* All 3x 4,095B Multi-Frame buffers are already in use */
                (void)send_fc_frame(fc_id, ISOTP_FC_FS_OVFLW, 0U, 0U, cfg->can_filler_byte);
                return ISOTP_RX_ERR_OVERFLOW;
            }

            mf->fc_tx_id   = fc_id;
            mf->total_len  = ff_len;
            mf->offset     = 6U;
            mf->next_sn    = 1U;
            mf->bs_counter = cfg->fc_bs;
            memcpy(mf->data, &frame->data[2], 6U);

            ctx->active_mf   = mf;
            ctx->in_progress = true;
            ctx->rx_can_id   = frame->id;

            /* Transmit Flow Control (CTS = 0x30) back to the physical ECU ID */
            if (send_fc_frame(
                    mf->fc_tx_id,
                    ISOTP_FC_FS_CTS,
                    cfg->fc_bs,
                    cfg->fc_stmin,
                    cfg->can_filler_byte
                ) != ESP_OK) {
                mf->in_use       = false;
                mf->owner_cll_id = 0U;
                ctx->active_mf   = NULL;
                ctx->in_progress = false;
                return ISOTP_RX_ERR_FC_TX;
            }

            return ISOTP_RX_IN_PROGRESS;
        }

        case ISOTP_PCI_CF: {
            /* Consecutive Frame (0x2N): match to the active 4,095B pool buffer for (cll_id, frame->id) */
            isotp_mf_buf_t *mf = find_active_mf_buf(cfg->cll_id, frame->id);
            if (mf == NULL) {
                return ISOTP_RX_IGNORED;
            }

            uint8_t sn = frame->data[0] & 0x0FU;
            if (sn != mf->next_sn) {
                mf->in_use       = false;
                mf->owner_cll_id = 0U;
                if (ctx->active_mf == mf) {
                    ctx->active_mf   = NULL;
                    ctx->in_progress = false;
                }
                return ISOTP_RX_ERR_SEQ;
            }

            uint16_t rem = mf->total_len - mf->offset;
            uint16_t copy_len = (rem > 7U) ? 7U : rem;
            if ((1U + copy_len) > frame->dlc) {
                mf->in_use       = false;
                mf->owner_cll_id = 0U;
                if (ctx->active_mf == mf) {
                    ctx->active_mf   = NULL;
                    ctx->in_progress = false;
                }
                return ISOTP_RX_ERR_SEQ;
            }

            memcpy(&mf->data[mf->offset], &frame->data[1], copy_len);
            mf->offset += copy_len;
            mf->next_sn = (mf->next_sn + 1U) & 0x0FU;

            if (mf->offset >= mf->total_len) {
                /* Multi-Frame reassembly complete! Caller will emit PDU_IT_RESULT and release mf */
                ctx->active_mf     = mf;
                ctx->in_progress   = false;
                ctx->rx_can_id     = mf->rx_can_id;
                ctx->total_len     = mf->total_len;
                ctx->completed_pdu = mf->data;
                return ISOTP_RX_COMPLETE;
            }

            /* If BlockSize > 0, count blocks and send next Flow Control when block finishes */
            if (cfg->fc_bs > 0U) {
                if (mf->bs_counter > 0U) {
                    mf->bs_counter--;
                }
                if (mf->bs_counter == 0U) {
                    mf->bs_counter = cfg->fc_bs;
                    if (send_fc_frame(
                            mf->fc_tx_id,
                            ISOTP_FC_FS_CTS,
                            cfg->fc_bs,
                            cfg->fc_stmin,
                            cfg->can_filler_byte
                        ) != ESP_OK) {
                        mf->in_use       = false;
                        mf->owner_cll_id = 0U;
                        ctx->active_mf   = NULL;
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
