/**
 * @file dpdu_protocol.c
 * @brief ISO 22900-2:2022 App-Device IF Binary Wire Protocol, 8-CLL State, 4-Slot Queues,
 *        Round-Robin Arbitration, IOCTLs, and 100 us Hardware Timestamps.
 */

#include "dpdu_protocol.h"
#include "can_hal.h"
#include "isotp.h"
#include "led_status.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

/* Spinlock protecting CLL queue admission/cancellation between Core 0 and Core 1 */
static portMUX_TYPE s_vci_spinlock = portMUX_INITIALIZER_UNLOCKED;

uint32_t dpdu_timestamp_us(void)
{
    return (uint32_t)((uint64_t)esp_timer_get_time() & 0xFFFFFFFFULL);
}

uint16_t dpdu_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint16_t dpdu_crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void write_u16_le(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val & 0xFFU);
    dst[1] = (uint8_t)((val >> 8) & 0xFFU);
}

static uint16_t read_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static void write_u32_le(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val & 0xFFU);
    dst[1] = (uint8_t)((val >> 8) & 0xFFU);
    dst[2] = (uint8_t)((val >> 16) & 0xFFU);
    dst[3] = (uint8_t)((val >> 24) & 0xFFU);
}

static uint32_t read_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0]
         | ((uint32_t)src[1] << 8)
         | ((uint32_t)src[2] << 16)
         | ((uint32_t)src[3] << 24);
}

void dpdu_parser_init(dpdu_rx_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->state = DPDU_PARSE_STATE_SOF0;
}

dpdu_parse_result_t dpdu_parser_feed_byte(dpdu_rx_parser_t *parser, uint8_t byte)
{
    if (parser == NULL) {
        return DPDU_PARSE_ERR_HDR;
    }

    switch (parser->state) {
        case DPDU_PARSE_STATE_SOF0:
            if (byte == DPDU_SOF_BYTE_0) {
                parser->hdr_buf[0] = byte;
                parser->state      = DPDU_PARSE_STATE_SOF1;
            }
            return DPDU_PARSE_INCOMPLETE;

        case DPDU_PARSE_STATE_SOF1:
            if (byte == DPDU_SOF_BYTE_1) {
                parser->hdr_buf[1] = byte;
                parser->hdr_idx    = 2U;
                parser->state      = DPDU_PARSE_STATE_HDR;
            } else if (byte == DPDU_SOF_BYTE_0) {
                /* Stay in SOF1 if repeated 0x55 */
                parser->hdr_buf[0] = byte;
            } else {
                parser->state = DPDU_PARSE_STATE_SOF0;
            }
            return DPDU_PARSE_INCOMPLETE;

        case DPDU_PARSE_STATE_HDR:
            parser->hdr_buf[parser->hdr_idx++] = byte;
            if (parser->hdr_idx < DPDU_HEADER_SIZE) {
                return DPDU_PARSE_INCOMPLETE;
            }

            parser->frame.msg_type = parser->hdr_buf[2];
            parser->frame.cll_id   = parser->hdr_buf[3];
            parser->frame.cop_id   = parser->hdr_buf[4];
            parser->frame.status   = parser->hdr_buf[5];
            parser->frame.length   = read_u16_le(&parser->hdr_buf[6]);

            if (parser->frame.length > DPDU_MAX_PAYLOAD) {
                parser->state = DPDU_PARSE_STATE_SOF0;
                return DPDU_PARSE_ERR_HDR;
            }

            parser->frame.ext_payload = (parser->frame.length > DPDU_CMD_INLINE_MAX)
                                      ? isotp_mf_pool_get_large_tx_buf()
                                      : parser->frame.payload;

            parser->payload_idx = 0U;
            parser->crc_idx     = 0U;
            parser->state       = (parser->frame.length == 0U)
                                ? DPDU_PARSE_STATE_CRC
                                : DPDU_PARSE_STATE_PAYLOAD;
            return DPDU_PARSE_INCOMPLETE;

        case DPDU_PARSE_STATE_PAYLOAD: {
            uint8_t *dst = (parser->frame.ext_payload != NULL)
                         ? parser->frame.ext_payload
                         : parser->frame.payload;
            dst[parser->payload_idx++] = byte;
            if (parser->payload_idx >= parser->frame.length) {
                parser->crc_idx = 0U;
                parser->state   = DPDU_PARSE_STATE_CRC;
            }
            return DPDU_PARSE_INCOMPLETE;
        }

        case DPDU_PARSE_STATE_CRC: {
            parser->crc_buf[parser->crc_idx++] = byte;
            if (parser->crc_idx < DPDU_CRC_SIZE) {
                return DPDU_PARSE_INCOMPLETE;
            }

            uint16_t rx_crc = read_u16_le(parser->crc_buf);
            /* Requirement 6: CRC16-CCITT computed over Msg_Type (offset 2) through end of Payload */
            uint16_t calc_crc = dpdu_crc16_update(0xFFFFU, &parser->hdr_buf[2], DPDU_HEADER_SIZE - 2U);
            if (parser->frame.length > 0U) {
                const uint8_t *pld = (parser->frame.ext_payload != NULL)
                                   ? parser->frame.ext_payload
                                   : parser->frame.payload;
                calc_crc = dpdu_crc16_update(calc_crc, pld, parser->frame.length);
            }

            parser->state = DPDU_PARSE_STATE_SOF0;
            if (rx_crc != calc_crc) {
                return DPDU_PARSE_ERR_CRC;
            }
            return DPDU_PARSE_OK;
        }

        default:
            parser->state = DPDU_PARSE_STATE_SOF0;
            return DPDU_PARSE_ERR_HDR;
    }
}

size_t dpdu_encode_packet(
    uint8_t msg_type,
    uint8_t cll_id,
    uint8_t cop_id,
    uint8_t status,
    const uint8_t *payload,
    uint16_t payload_len,
    uint8_t *out_buf,
    size_t out_max
)
{
    if (out_buf == NULL || payload_len > DPDU_MAX_PAYLOAD) {
        return 0U;
    }
    size_t total_len = DPDU_HEADER_SIZE + (size_t)payload_len + DPDU_CRC_SIZE;
    if (out_max < total_len) {
        return 0U;
    }

    out_buf[0] = DPDU_SOF_BYTE_0;
    out_buf[1] = DPDU_SOF_BYTE_1;
    out_buf[2] = msg_type;
    out_buf[3] = cll_id;
    out_buf[4] = cop_id;
    out_buf[5] = status;
    write_u16_le(&out_buf[6], payload_len);

    if (payload_len > 0U && payload != NULL) {
        memcpy(&out_buf[DPDU_HEADER_SIZE], payload, payload_len);
    }

    /* CRC16-CCITT over Msg_Type (byte 2) through end of Payload */
    uint16_t crc = dpdu_crc16_ccitt(&out_buf[2], (DPDU_HEADER_SIZE - 2U) + (size_t)payload_len);
    write_u16_le(&out_buf[DPDU_HEADER_SIZE + payload_len], crc);
    return total_len;
}

static void emit_ack(
    dpdu_send_pkt_fn_t send_fn,
    uint8_t cll_id,
    uint8_t cop_id,
    uint8_t status,
    const uint8_t *payload,
    uint16_t payload_len
)
{
    if (send_fn != NULL) {
        send_fn(PDU_EVT_CMD_ACK, cll_id, cop_id, status, payload, payload_len);
    }
}

static void emit_it_status(
    dpdu_send_pkt_fn_t send_fn,
    uint8_t cll_id,
    uint8_t cop_id,
    uint16_t status_code
)
{
    if (send_fn == NULL) {
        return;
    }
    uint8_t payload[6];
    write_u32_le(&payload[0], dpdu_timestamp_us());
    write_u16_le(&payload[4], status_code);
    send_fn(PDU_IT_STATUS, cll_id, cop_id, 0U, payload, sizeof(payload));
}

static void emit_it_error(
    dpdu_send_pkt_fn_t send_fn,
    uint8_t cll_id,
    uint8_t cop_id,
    uint16_t err_code,
    uint16_t vendor_subcode
)
{
    if (send_fn == NULL) {
        return;
    }
    uint8_t payload[8];
    write_u32_le(&payload[0], dpdu_timestamp_us());
    write_u16_le(&payload[4], err_code);
    write_u16_le(&payload[6], vendor_subcode);
    send_fn(PDU_IT_ERROR, cll_id, cop_id, 0U, payload, sizeof(payload));
}

static void emit_it_result(
    dpdu_send_pkt_fn_t send_fn,
    uint8_t cll_id,
    uint8_t cop_id,
    uint32_t rx_timestamp_us,
    uint32_t rx_can_id,
    uint8_t extra_addr_info,
    const uint8_t *npdu,
    uint16_t npdu_len
)
{
    if (send_fn == NULL || npdu == NULL || npdu_len == 0U || npdu_len > VCI_ISOTP_MAX_PDU_LEN) {
        return;
    }

    if (npdu_len <= 256U) {
        static uint8_t s_small_res[9U + 256U];
        write_u32_le(&s_small_res[0], rx_timestamp_us);
        write_u32_le(&s_small_res[4], rx_can_id);
        s_small_res[8] = extra_addr_info;
        memcpy(&s_small_res[9], npdu, npdu_len);
        send_fn(PDU_IT_RESULT, cll_id, cop_id, 0U, s_small_res, (uint16_t)(9U + npdu_len));
    } else {
        /* Multi-Frame PDUs > 256B reside in isotp_mf_buf_t::data, preceded by result_hdr_pad[9] */
        uint8_t *in_place = ((uint8_t *)(uintptr_t)npdu) - 9U;
        write_u32_le(&in_place[0], rx_timestamp_us);
        write_u32_le(&in_place[4], rx_can_id);
        in_place[8] = extra_addr_info;
        send_fn(PDU_IT_RESULT, cll_id, cop_id, 0U, in_place, (uint16_t)(9U + npdu_len));
    }
}

static void reset_cll_defaults(dpdu_cll_t *cll, uint8_t cll_id)
{
    memset(cll, 0, sizeof(*cll));
    cll->cll_id             = cll_id;
    cll->online             = false;
    cll->baudrate_bps       = 500000UL;     /* CP_BAUDRATE */
    cll->can_req_id         = 0x7DFUL;      /* CP_CAN_PHYS_REQ_ID */
    cll->can_resp_id        = 0x7E8UL;      /* CP_CAN_RESP_USDT_ID */
    cll->addressing_type    = DPDU_ADDR_FUNCTIONAL;
    cll->can_filler_byte    = 0xCCUL;       /* CP_CAN_FILLER_BYTE */
    cll->isotp_bs           = 0UL;          /* CP_BLOCK_SIZE */
    cll->isotp_stmin        = 0UL;          /* CP_ST_MIN */
    cll->isotp_n_bs_ms      = 1000UL;       /* CP_N_BS_MS */
    cll->isotp_n_cr_ms      = 1000UL;       /* CP_N_CR_MS */
    cll->p2_max_ms          = 50UL;         /* CP_P2_MAX_MS */
    cll->p2_star_ms         = 5000UL;       /* CP_P2_STAR_MS */
    cll->p3_phys_ms         = 5UL;          /* CP_P3_PHYS_MS */
    cll->p3_func_ms         = 50UL;         /* CP_P3_FUNC_MS */
    cll->next_allowed_tx_us = 0LL;
}

void dpdu_engine_init(dpdu_vci_state_t *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    for (uint8_t i = 0U; i < VCI_MAX_CLLS; i++) {
        reset_cll_defaults(&state->clls[i], (uint8_t)(i + 1U));
    }
    isotp_mf_pool_reset_all();
    led_status_set_mode(VCI_LED_DISCONNECTED);
}

void dpdu_engine_destruct_all(dpdu_vci_state_t *state, dpdu_send_pkt_fn_t send_fn, bool emit_events)
{
    if (state == NULL) {
        return;
    }

    uint8_t offline_clls[VCI_MAX_CLLS];
    uint8_t offline_cnt = 0U;

    portENTER_CRITICAL(&s_vci_spinlock);
    for (uint8_t i = 0U; i < VCI_MAX_CLLS; i++) {
        dpdu_cll_t *cll = &state->clls[i];
        if (cll->online) {
            offline_clls[offline_cnt++] = cll->cll_id;
        }
        cll->online    = false;
        cll->cop_head  = 0U;
        cll->cop_tail  = 0U;
        cll->cop_count = 0U;
        memset(cll->cop_queue, 0, sizeof(cll->cop_queue));
    }
    state->online_cll_count    = 0U;
    state->locked_baudrate_bps = 0UL;
    state->large_tx_in_use     = false;
    portEXIT_CRITICAL(&s_vci_spinlock);

    isotp_mf_pool_reset_all();
    can_hal_stop();
    led_status_set_mode(VCI_LED_DISCONNECTED);

    if (emit_events && send_fn != NULL) {
        for (uint8_t i = 0U; i < offline_cnt; i++) {
            emit_it_status(send_fn, offline_clls[i], 0U, PDU_CLLST_OFFLINE);
        }
    }
}

static dpdu_cll_t *get_cll(dpdu_vci_state_t *state, uint8_t cll_id)
{
    if (state == NULL || cll_id < 1U || cll_id > VCI_MAX_CLLS) {
        return NULL;
    }
    return &state->clls[cll_id - 1U];
}

static bool is_supported_baudrate(uint32_t bps)
{
    return (bps == 125000UL || bps == 250000UL || bps == 500000UL || bps == 1000000UL);
}

static bool is_valid_stmin(uint32_t val)
{
    return (val <= 0x7FUL) || (val >= 0xF1UL && val <= 0xF9UL);
}

static void handle_mod_ctrl_ioctl(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_send_pkt_fn_t send_fn
)
{
    (void)state;
    if (cmd->cll_id != 0U || cmd->length < 2U) {
        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
        return;
    }

    uint16_t ioctl_id = read_u16_le(&cmd->payload[0]);
    switch (ioctl_id) {
        case PDU_IOCTL_READ_VBATT: {
            uint8_t rsp[4];
            write_u32_le(rsp, VCI_DEFAULT_VBATT_MV);
            emit_ack(send_fn, 0U, cmd->cop_id, PDU_STATUS_NOERROR, rsp, sizeof(rsp));
            break;
        }
        case PDU_IOCTL_GET_CABLE_ID: {
            uint8_t rsp[4];
            write_u32_le(rsp, VCI_SAE_J1962_CABLE_ID);
            emit_ack(send_fn, 0U, cmd->cop_id, PDU_STATUS_NOERROR, rsp, sizeof(rsp));
            break;
        }
        case PDU_IOCTL_GET_VERSION: {
            uint8_t rsp[16];
            uint8_t mac[6] = {0};
            (void)esp_efuse_mac_get_default(mac);
            write_u32_le(&rsp[0], VCI_FW_VERSION_U32);
            write_u32_le(&rsp[4], VCI_HW_VERSION_U32);
            memset(&rsp[8], 0, 8U);
            memcpy(&rsp[8], mac, 6U);
            emit_ack(send_fn, 0U, cmd->cop_id, PDU_STATUS_NOERROR, rsp, sizeof(rsp));
            break;
        }
        default:
            emit_ack(send_fn, 0U, cmd->cop_id, PDU_ERR_FCT_NOT_SUPPORTED, NULL, 0U);
            break;
    }
}

void dpdu_engine_handle_host_cmd(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_send_pkt_fn_t send_fn
)
{
    if (state == NULL || cmd == NULL) {
        return;
    }

    state->rx_cmd_count++;

    switch (cmd->msg_type) {
        case PDU_CMD_MOD_CTRL:
            handle_mod_ctrl_ioctl(state, cmd, send_fn);
            return;

        case PDU_CMD_CONNECT: {
            dpdu_cll_t *cll = get_cll(state, cmd->cll_id);
            if (cll == NULL) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }

            uint8_t req_bus_mode = (cmd->length >= 1U) ? cmd->payload[0] : VCI_BUS_MODE_SIM_LOOPBACK;
            if (cmd->length >= 3U) {
                /* Optional legacy [bus_mode:1B][baud_khz:2B] */
                uint32_t legacy_bps = (uint32_t)read_u16_le(&cmd->payload[1]) * 1000UL;
                if (is_supported_baudrate(legacy_bps)) {
                    cll->baudrate_bps = legacy_bps;
                }
            }

            /* Requirement 3: First CLL to enter ONLINE locks CP_Baudrate; reject conflicting rates */
            if (state->online_cll_count > 0U && cll->baudrate_bps != state->locked_baudrate_bps) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_ERR_BAUDRATE_CONFLICT, NULL, 0U);
                return;
            }

            if (state->online_cll_count == 0U) {
                uint16_t baud_khz = (uint16_t)(cll->baudrate_bps / 1000UL);
                if (can_hal_start(req_bus_mode, baud_khz) != ESP_OK) {
                    emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                    return;
                }
                state->locked_baudrate_bps = cll->baudrate_bps;
                state->bus_mode            = req_bus_mode;
            }

            portENTER_CRITICAL(&s_vci_spinlock);
            if (!cll->online) {
                cll->online = true;
                state->online_cll_count++;
            }
            cll->next_allowed_tx_us = 0LL;
            portEXIT_CRITICAL(&s_vci_spinlock);

            led_status_set_mode(VCI_LED_CONNECTED_IDLE);
            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOERROR, NULL, 0U);
            emit_it_status(send_fn, cmd->cll_id, 0U, PDU_CLLST_ONLINE);
            return;
        }

        case PDU_CMD_DISCONNECT: {
            if (cmd->cll_id == 0U) {
                /* Module-level disconnect -> full PDUDestruct equivalent */
                emit_ack(send_fn, 0U, cmd->cop_id, PDU_STATUS_NOERROR, NULL, 0U);
                dpdu_engine_destruct_all(state, send_fn, true);
                return;
            }

            dpdu_cll_t *cll = get_cll(state, cmd->cll_id);
            if (cll == NULL) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }

            bool stop_can = false;
            portENTER_CRITICAL(&s_vci_spinlock);
            if (cll->online) {
                cll->online = false;
                if (state->online_cll_count > 0U) {
                    state->online_cll_count--;
                }
            }
            for (uint8_t i = 0U; i < VCI_CLL_COP_QUEUE_DEPTH; i++) {
                if (cll->cop_queue[i].in_use && cll->cop_queue[i].uses_large_tx_buf) {
                    state->large_tx_in_use = false;
                }
            }
            cll->cop_head  = 0U;
            cll->cop_tail  = 0U;
            cll->cop_count = 0U;
            memset(cll->cop_queue, 0, sizeof(cll->cop_queue));
            if (state->online_cll_count == 0U) {
                state->locked_baudrate_bps = 0UL;
                stop_can = true;
            }
            portEXIT_CRITICAL(&s_vci_spinlock);

            isotp_mf_pool_release_cll(cmd->cll_id);
            if (stop_can) {
                can_hal_stop();
                led_status_set_mode(VCI_LED_DISCONNECTED);
            }
            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOERROR, NULL, 0U);
            emit_it_status(send_fn, cmd->cll_id, 0U, PDU_CLLST_OFFLINE);
            return;
        }

        case PDU_CMD_SET_COMPARAM: {
            dpdu_cll_t *cll = get_cll(state, cmd->cll_id);
            if (cll == NULL || cmd->length < 5U) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }

            uint16_t param_id;
            uint32_t param_val;
            if (cmd->length >= 6U) {
                /* Requirement 7: [Param_ID (2B LE) | Param_Value (4B LE)] */
                param_id  = read_u16_le(&cmd->payload[0]);
                param_val = read_u32_le(&cmd->payload[2]);
            } else {
                /* Legacy 5B: [Param_ID (1B) | Param_Value (4B LE)] */
                param_id  = cmd->payload[0];
                param_val = read_u32_le(&cmd->payload[1]);
            }

            switch (param_id) {
                case CP_BAUDRATE:
                    if (!is_supported_baudrate(param_val)) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    if (state->online_cll_count > 1U && param_val != state->locked_baudrate_bps) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_ERR_BAUDRATE_CONFLICT, NULL, 0U);
                        return;
                    }
                    cll->baudrate_bps = param_val;
                    if (cll->online && state->online_cll_count == 1U) {
                        state->locked_baudrate_bps = param_val;
                    }
                    break;
                case CP_CAN_PHYS_REQ_ID:
                    cll->can_req_id = param_val;
                    break;
                case CP_CAN_RESP_USDT_ID:
                    cll->can_resp_id = param_val;
                    break;
                case CP_ADDRESSING_TYPE:
                    if (param_val > DPDU_ADDR_PHYSICAL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->addressing_type = param_val;
                    break;
                case CP_CAN_FILLER_BYTE:
                    if (param_val > 0xFFUL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->can_filler_byte = param_val;
                    break;
                case CP_BLOCK_SIZE:
                    if (param_val > 0xFFUL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->isotp_bs = param_val;
                    break;
                case CP_ST_MIN:
                    if (!is_valid_stmin(param_val)) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->isotp_stmin = param_val;
                    break;
                case CP_N_BS_MS:
                    if (param_val == 0UL || param_val > 60000UL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->isotp_n_bs_ms = param_val;
                    break;
                case CP_N_CR_MS:
                    if (param_val == 0UL || param_val > 60000UL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->isotp_n_cr_ms = param_val;
                    break;
                case CP_P2_MAX_MS:
                    if (param_val == 0UL || param_val > 60000UL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->p2_max_ms = param_val;
                    break;
                case CP_P2_STAR_MS:
                    if (param_val == 0UL || param_val > 60000UL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->p2_star_ms = param_val;
                    break;
                case CP_P3_PHYS_MS:
                    if (param_val > 60000UL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->p3_phys_ms = param_val;
                    break;
                case CP_P3_FUNC_MS:
                    if (param_val > 60000UL) {
                        emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                        return;
                    }
                    cll->p3_func_ms = param_val;
                    break;
                default:
                    emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_ERR_FCT_NOT_SUPPORTED, NULL, 0U);
                    return;
            }

            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOERROR, NULL, 0U);
            return;
        }

        case PDU_CMD_GET_COMPARAM: {
            dpdu_cll_t *cll = get_cll(state, cmd->cll_id);
            if (cll == NULL || cmd->length < 1U) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }
            uint16_t param_id = (cmd->length >= 2U) ? read_u16_le(&cmd->payload[0]) : cmd->payload[0];
            uint32_t val = 0UL;
            switch (param_id) {
                case CP_BAUDRATE:          val = cll->baudrate_bps;    break;
                case CP_CAN_PHYS_REQ_ID:   val = cll->can_req_id;      break;
                case CP_CAN_RESP_USDT_ID:  val = cll->can_resp_id;     break;
                case CP_ADDRESSING_TYPE:   val = cll->addressing_type; break;
                case CP_CAN_FILLER_BYTE:   val = cll->can_filler_byte; break;
                case CP_BLOCK_SIZE:        val = cll->isotp_bs;        break;
                case CP_ST_MIN:            val = cll->isotp_stmin;     break;
                case CP_N_BS_MS:           val = cll->isotp_n_bs_ms;   break;
                case CP_N_CR_MS:           val = cll->isotp_n_cr_ms;   break;
                case CP_P2_MAX_MS:         val = cll->p2_max_ms;       break;
                case CP_P2_STAR_MS:        val = cll->p2_star_ms;      break;
                case CP_P3_PHYS_MS:        val = cll->p3_phys_ms;      break;
                case CP_P3_FUNC_MS:        val = cll->p3_func_ms;      break;
                default:
                    emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_ERR_FCT_NOT_SUPPORTED, NULL, 0U);
                    return;
            }
            uint8_t rsp[6];
            write_u16_le(&rsp[0], param_id);
            write_u32_le(&rsp[2], val);
            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOERROR, rsp, sizeof(rsp));
            return;
        }

        case PDU_CMD_START_COP: {
            /* Requirement 7 StartComPrimitive payload:
             * [ NumSendCycles (1B) | NumRecvCycles (1B) | CycleTime_ms (2B) | CAN_ID (4B) | N-PDU (1..4095B) ]
             */
            dpdu_cll_t *cll = get_cll(state, cmd->cll_id);
            if (cll == NULL || cmd->cop_id == 0U || cmd->length < 9U) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }
            if (!cll->online) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOT_CONN, NULL, 0U);
                return;
            }

            const uint8_t *cmd_pld       = (cmd->length > DPDU_CMD_INLINE_MAX && cmd->ext_payload != NULL)
                                         ? cmd->ext_payload
                                         : cmd->payload;
            uint8_t        num_send      = cmd_pld[0];
            uint8_t        num_recv      = cmd_pld[1];
            uint16_t       cycle_time_ms = read_u16_le(&cmd_pld[2]);
            uint32_t       req_can_id    = read_u32_le(&cmd_pld[4]);
            const uint8_t *npdu          = &cmd_pld[8];
            uint16_t       npdu_len      = (uint16_t)(cmd->length - 8U);

            if (num_send == 0U || npdu_len == 0U || npdu_len > VCI_ISOTP_MAX_PDU_LEN) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }

            uint8_t          status_code = PDU_STATUS_NOERROR;
            dpdu_cop_slot_t *admitted_slot = NULL;

            portENTER_CRITICAL(&s_vci_spinlock);
            if (cll->cop_count >= VCI_CLL_COP_QUEUE_DEPTH) {
                /* Requirement 3: Reject 5th queued ComPrimitive on this CLL with Overrun NACK */
                status_code = PDU_ERR_COMM_PC_TO_VCI_OVERRUN;
            } else if (npdu_len > DPDU_COP_INLINE_PDU_MAX && state->large_tx_in_use) {
                status_code = PDU_ERR_COMM_PC_TO_VCI_OVERRUN;
            } else {
                dpdu_cop_slot_t *slot   = &cll->cop_queue[cll->cop_tail];
                slot->in_use            = false; /* Enabled after synchronous 0x80 ACK is emitted */
                slot->started           = false;
                slot->cancel_requested  = false;
                slot->cll_id            = cmd->cll_id;
                slot->cop_id            = cmd->cop_id;
                slot->num_send_cycles   = num_send;
                slot->num_recv_cycles   = num_recv;
                slot->send_cycles_done  = 0U;
                slot->cycle_time_ms     = cycle_time_ms;
                slot->next_cycle_tx_us  = 0LL;
                slot->req_can_id        = (req_can_id != 0UL) ? req_can_id : cll->can_req_id;
                slot->pdu_len           = npdu_len;

                if (npdu_len <= DPDU_COP_INLINE_PDU_MAX) {
                    slot->uses_large_tx_buf = false;
                    memcpy(slot->pdu, npdu, npdu_len);
                } else {
                    slot->uses_large_tx_buf = true;
                    state->large_tx_in_use  = true;
                    memmove(isotp_mf_pool_get_large_tx_buf(), npdu, npdu_len);
                }

                cll->cop_tail = (uint8_t)((cll->cop_tail + 1U) % VCI_CLL_COP_QUEUE_DEPTH);
                admitted_slot = slot;
            }
            portEXIT_CRITICAL(&s_vci_spinlock);

            /* Emit synchronous 0x80 ACK/NACK before Core 1 starts executing the slot */
            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, status_code, NULL, 0U);

            if (admitted_slot != NULL) {
                portENTER_CRITICAL(&s_vci_spinlock);
                admitted_slot->in_use = true;
                cll->cop_count++;
                portEXIT_CRITICAL(&s_vci_spinlock);
            }
            return;
        }

        case PDU_CMD_CANCEL_COP: {
            /* Out-of-Band Priority CancelComPrimitive */
            if (cmd->cll_id > VCI_MAX_CLLS) {
                emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_INVALID_ARG, NULL, 0U);
                return;
            }

            portENTER_CRITICAL(&s_vci_spinlock);
            for (uint8_t c = 0U; c < VCI_MAX_CLLS; c++) {
                dpdu_cll_t *cll = &state->clls[c];
                if (cmd->cll_id != 0U && cll->cll_id != cmd->cll_id) {
                    continue;
                }
                for (uint8_t s = 0U; s < VCI_CLL_COP_QUEUE_DEPTH; s++) {
                    dpdu_cop_slot_t *slot = &cll->cop_queue[s];
                    if (slot->in_use && (cmd->cop_id == 0U || slot->cop_id == cmd->cop_id)) {
                        slot->cancel_requested = true;
                    }
                }
            }
            portEXIT_CRITICAL(&s_vci_spinlock);

            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOERROR, NULL, 0U);
            return;
        }

        case PDU_CMD_GET_STATUS: {
            uint8_t online_mask = 0U;
            for (uint8_t i = 0U; i < VCI_MAX_CLLS; i++) {
                if (state->clls[i].online) {
                    online_mask |= (uint8_t)(1U << i);
                }
            }
            dpdu_cll_t *target_cll = (cmd->cll_id >= 1U && cmd->cll_id <= VCI_MAX_CLLS)
                                   ? &state->clls[cmd->cll_id - 1U]
                                   : &state->clls[0];
            uint32_t baud_bps = (state->online_cll_count > 0U)
                              ? state->locked_baudrate_bps
                              : target_cll->baudrate_bps;

            uint8_t rsp[16];
            rsp[0] = online_mask;
            rsp[1] = state->bus_mode;
            write_u16_le(&rsp[2], (uint16_t)(baud_bps / 1000UL));
            write_u32_le(&rsp[4], state->rx_cmd_count);
            write_u16_le(&rsp[8], (uint16_t)(state->crc_err_count & 0xFFFFU));
            write_u32_le(&rsp[10], (uint32_t)(esp_timer_get_time() / 1000LL));
            write_u16_le(&rsp[14], (uint16_t)(target_cll->can_req_id & 0xFFFFU));
            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_STATUS_NOERROR, rsp, sizeof(rsp));
            return;
        }

        default:
            emit_ack(send_fn, cmd->cll_id, cmd->cop_id, PDU_ERR_FCT_NOT_SUPPORTED, NULL, 0U);
            return;
    }
}

static void pop_cll_head_cop(dpdu_vci_state_t *state, dpdu_cll_t *cll)
{
    portENTER_CRITICAL(&s_vci_spinlock);
    if (cll->cop_count > 0U) {
        dpdu_cop_slot_t *slot = &cll->cop_queue[cll->cop_head];
        if (slot->uses_large_tx_buf) {
            state->large_tx_in_use = false;
        }
        memset(slot, 0, sizeof(*slot));
        cll->cop_head = (uint8_t)((cll->cop_head + 1U) % VCI_CLL_COP_QUEUE_DEPTH);
        cll->cop_count--;
    }
    portEXIT_CRITICAL(&s_vci_spinlock);
}

static bool is_cop_cancelled(const dpdu_cll_t *cll, const dpdu_cop_slot_t *slot)
{
    return (!cll->online || !slot->in_use || slot->cancel_requested);
}

static bool is_matching_service_response(uint8_t req_sid, const uint8_t *rx_pdu, uint16_t rx_len)
{
    if (rx_pdu == NULL || rx_len == 0U) {
        return false;
    }
    if (rx_pdu[0] == (uint8_t)(req_sid + 0x40U)) {
        return true;
    }
    if (rx_len >= 3U && rx_pdu[0] == 0x7FU && rx_pdu[1] == req_sid) {
        return true;
    }
    return false;
}

static void update_cll_p3_deadline(dpdu_cll_t *cll, uint32_t req_can_id, int64_t ref_time_us)
{
    bool is_func = (cll->addressing_type == DPDU_ADDR_FUNCTIONAL ||
                    req_can_id == 0x7DFUL ||
                    req_can_id == 0x18DB33F1UL);
    uint32_t p3_ms = is_func ? cll->p3_func_ms : cll->p3_phys_ms;
    cll->next_allowed_tx_us = ref_time_us + ((int64_t)p3_ms * 1000LL);
}

static void execute_one_cop_cycle(
    dpdu_vci_state_t *state,
    dpdu_cll_t *cll,
    dpdu_cop_slot_t *slot,
    dpdu_send_pkt_fn_t send_fn
)
{
    uint8_t        cll_id   = cll->cll_id;
    uint8_t        cop_id   = slot->cop_id;
    uint32_t       req_id   = (slot->req_can_id != 0UL) ? slot->req_can_id : cll->can_req_id;
    const uint8_t *tx_pdu   = slot->uses_large_tx_buf ? isotp_mf_pool_get_large_tx_buf() : slot->pdu;
    uint16_t       pdu_len  = slot->pdu_len;
    uint8_t        req_sid  = tx_pdu[0];

    bool is_func = (cll->addressing_type == DPDU_ADDR_FUNCTIONAL ||
                    req_id == 0x7DFUL ||
                    req_id == 0x18DB33F1UL);

    isotp_config_t isotp_cfg = {
        .cll_id          = cll_id,
        .can_req_id      = req_id,
        .can_resp_id     = cll->can_resp_id,
        .addressing_type = is_func ? DPDU_ADDR_FUNCTIONAL : DPDU_ADDR_PHYSICAL,
        .can_filler_byte = (uint8_t)(cll->can_filler_byte & 0xFFU),
        .fc_bs           = (uint8_t)(cll->isotp_bs & 0xFFU),
        .fc_stmin        = (uint8_t)(cll->isotp_stmin & 0xFFU),
        .n_bs_ms         = cll->isotp_n_bs_ms,
        .n_cr_ms         = cll->isotp_n_cr_ms,
    };

    if (!slot->started) {
        slot->started = true;
        emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_EXECUTING);
    }

    /* Drain any stale CAN frames before starting TX */
    vci_can_frame_t stale;
    while (can_hal_recv(&stale, 0) == ESP_OK) {
    }
    led_status_pulse_activity();

    int64_t tx_start_us = esp_timer_get_time();
    esp_err_t tx_err = isotp_send_pdu(&isotp_cfg, tx_pdu, pdu_len);
    if (tx_err != ESP_OK) {
        uint16_t err_code = (tx_err == ESP_ERR_TIMEOUT) ? PDU_ERR_EVT_RX_TIMEOUT : PDU_ERR_EVT_TX_ERROR;
        uint16_t sub_code = (tx_err == ESP_ERR_TIMEOUT) ? PDU_ERR_ISOTP_TIMEOUT  : PDU_ERR_CAN_BUS_OFF;
        emit_it_error(send_fn, cll_id, cop_id, err_code, sub_code);
        pop_cll_head_cop(state, cll);
        emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_FINISHED);
        return;
    }

    /* SEND_ONLY primitive (NumRecvCycles == 0) */
    if (slot->num_recv_cycles == 0U) {
        update_cll_p3_deadline(cll, req_id, is_func ? tx_start_us : esp_timer_get_time());
        slot->send_cycles_done++;
        if (slot->num_send_cycles != 0xFFU && slot->send_cycles_done >= slot->num_send_cycles) {
            pop_cll_head_cop(state, cll);
            emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_FINISHED);
        } else {
            slot->next_cycle_tx_us = esp_timer_get_time() + ((int64_t)slot->cycle_time_ms * 1000LL);
        }
        return;
    }

    /* SENDRECV primitive: bind the CLL's 8B SF buffer and 3x 4,095B MF pool */
    isotp_rx_ctx_t rx_ctx;
    isotp_rx_init(&rx_ctx, cll->sf_rx_buf);

    uint32_t p2_ms      = (cll->p2_max_ms > 0UL)   ? cll->p2_max_ms       : 50UL;
    uint32_t p2_star_ms = (cll->p2_star_ms > 0UL)  ? cll->p2_star_ms      : 5000UL;
    uint32_t n_cr_ms    = (cll->isotp_n_cr_ms > 0UL) ? cll->isotp_n_cr_ms : 1000UL;

    int64_t deadline_us = tx_start_us + ((int64_t)p2_ms * 1000LL);
    uint8_t recv_count  = 0U;

    while (esp_timer_get_time() < deadline_us) {
        if (is_cop_cancelled(cll, slot)) {
            isotp_rx_release(&rx_ctx);
            pop_cll_head_cop(state, cll);
            emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_CANCELLED);
            return;
        }

        int64_t rem_us = deadline_us - esp_timer_get_time();
        uint32_t wait_ms = (uint32_t)((rem_us + 999LL) / 1000LL);
        if (wait_ms > 5U) {
            wait_ms = 5U;
        } else if (wait_ms == 0U) {
            wait_ms = 1U;
        }

        vci_can_frame_t rx_frame;
        if (can_hal_recv(&rx_frame, wait_ms) != ESP_OK) {
            continue;
        }

        isotp_rx_status_t status = isotp_rx_feed_frame(&rx_ctx, &rx_frame, &isotp_cfg);

        if (status == ISOTP_RX_IN_PROGRESS) {
            deadline_us = esp_timer_get_time() + ((int64_t)n_cr_ms * 1000LL);
            continue;
        }

        if (status == ISOTP_RX_ERR_SEQ || status == ISOTP_RX_ERR_OVERFLOW || status == ISOTP_RX_ERR_FC_TX) {
            isotp_rx_release(&rx_ctx);
            update_cll_p3_deadline(cll, req_id, esp_timer_get_time());
            emit_it_error(send_fn, cll_id, cop_id, PDU_ERR_EVT_PROT_ERR, PDU_ERR_MALFORMED_FRAME);
            pop_cll_head_cop(state, cll);
            emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_FINISHED);
            return;
        }

        if (status == ISOTP_RX_COMPLETE) {
            const uint8_t *pdu_ptr = rx_ctx.completed_pdu;
            uint16_t       rx_len  = rx_ctx.total_len;

            if (!is_matching_service_response(req_sid, pdu_ptr, rx_len)) {
                isotp_rx_release(&rx_ctx);
                continue;
            }

            /* Check for UDS/OBD Negative Response 0x7F [SID] 0x78 (ResponsePending) */
            if (rx_len >= 3U &&
                pdu_ptr[0] == 0x7FU &&
                pdu_ptr[1] == req_sid &&
                pdu_ptr[2] == 0x78U) {
                isotp_rx_release(&rx_ctx);
                deadline_us = esp_timer_get_time() + ((int64_t)p2_star_ms * 1000LL);
                continue;
            }

            /* Valid response assembled! Emit 0x81 PDU_IT_RESULT with 100 us hardware timestamp */
            uint32_t rx_ts_us = dpdu_timestamp_us();
            emit_it_result(send_fn, cll_id, cop_id, rx_ts_us, rx_ctx.rx_can_id, 0U, pdu_ptr, rx_len);
            isotp_rx_release(&rx_ctx);
            recv_count++;

            if (recv_count >= slot->num_recv_cycles) {
                update_cll_p3_deadline(cll, req_id, is_func ? tx_start_us : esp_timer_get_time());
                slot->send_cycles_done++;
                if (slot->num_send_cycles != 0xFFU && slot->send_cycles_done >= slot->num_send_cycles) {
                    pop_cll_head_cop(state, cll);
                    emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_FINISHED);
                } else {
                    slot->next_cycle_tx_us = esp_timer_get_time() + ((int64_t)slot->cycle_time_ms * 1000LL);
                }
                return;
            }
        }
    }

    /* Timeout reached: if we already received >= 1 functional response in a multi-ECU window, finish cleanly */
    if (recv_count > 0U && !rx_ctx.in_progress) {
        isotp_rx_release(&rx_ctx);
        update_cll_p3_deadline(cll, req_id, is_func ? tx_start_us : esp_timer_get_time());
        slot->send_cycles_done++;
        if (slot->num_send_cycles != 0xFFU && slot->send_cycles_done >= slot->num_send_cycles) {
            pop_cll_head_cop(state, cll);
            emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_FINISHED);
        } else {
            slot->next_cycle_tx_us = esp_timer_get_time() + ((int64_t)slot->cycle_time_ms * 1000LL);
        }
        return;
    }

    /* Otherwise emit 0x83 PDU_IT_ERROR (PDU_ERR_EVT_RX_TIMEOUT) with P2 or ISOTP subcode */
    bool was_mf_in_progress = rx_ctx.in_progress;
    isotp_rx_release(&rx_ctx);
    update_cll_p3_deadline(cll, req_id, is_func ? tx_start_us : esp_timer_get_time());
    emit_it_error(
        send_fn,
        cll_id,
        cop_id,
        PDU_ERR_EVT_RX_TIMEOUT,
        was_mf_in_progress ? PDU_ERR_ISOTP_TIMEOUT : PDU_ERR_P2_TIMEOUT
    );
    pop_cll_head_cop(state, cll);
    emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_FINISHED);
}

bool dpdu_engine_step_round_robin(
    dpdu_vci_state_t *state,
    dpdu_send_pkt_fn_t send_fn
)
{
    if (state == NULL) {
        return false;
    }

    /* First, immediately drain any cancelled ComPrimitives across all 8 CLLs */
    for (uint8_t c = 0U; c < VCI_MAX_CLLS; c++) {
        dpdu_cll_t *cll = &state->clls[c];
        while (cll->cop_count > 0U) {
            dpdu_cop_slot_t *head = &cll->cop_queue[cll->cop_head];
            if (!head->cancel_requested) {
                break;
            }
            uint8_t cll_id = cll->cll_id;
            uint8_t cop_id = head->cop_id;
            pop_cll_head_cop(state, cll);
            emit_it_status(send_fn, cll_id, cop_id, PDU_COPST_CANCELLED);
        }

        /* Also check non-head slots in the 4-slot queue if a specific COP_ID was cancelled */
        if (cll->cop_count > 1U) {
            portENTER_CRITICAL(&s_vci_spinlock);
            dpdu_cop_slot_t compacted[VCI_CLL_COP_QUEUE_DEPTH];
            uint8_t         compacted_cnt = 0U;
            uint8_t         cancelled_ids[VCI_CLL_COP_QUEUE_DEPTH];
            uint8_t         cancelled_cnt = 0U;

            for (uint8_t i = 0U; i < cll->cop_count; i++) {
                uint8_t idx = (uint8_t)((cll->cop_head + i) % VCI_CLL_COP_QUEUE_DEPTH);
                dpdu_cop_slot_t *slot = &cll->cop_queue[idx];
                if (slot->cancel_requested) {
                    if (slot->uses_large_tx_buf) {
                        state->large_tx_in_use = false;
                    }
                    cancelled_ids[cancelled_cnt++] = slot->cop_id;
                } else {
                    compacted[compacted_cnt++] = *slot;
                }
            }
            if (cancelled_cnt > 0U) {
                memset(cll->cop_queue, 0, sizeof(cll->cop_queue));
                for (uint8_t i = 0U; i < compacted_cnt; i++) {
                    cll->cop_queue[i] = compacted[i];
                }
                cll->cop_head  = 0U;
                cll->cop_tail  = (uint8_t)(compacted_cnt % VCI_CLL_COP_QUEUE_DEPTH);
                cll->cop_count = compacted_cnt;
            }
            portEXIT_CRITICAL(&s_vci_spinlock);

            for (uint8_t i = 0U; i < cancelled_cnt; i++) {
                emit_it_status(send_fn, cll->cll_id, cancelled_ids[i], PDU_COPST_CANCELLED);
            }
        }
    }

    /* Second, step Round-Robin across the 8 CLLs to pick the next ready ComPrimitive */
    int64_t now_us = esp_timer_get_time();
    for (uint8_t attempt = 0U; attempt < VCI_MAX_CLLS; attempt++) {
        uint8_t idx = state->rr_cll_idx;
        state->rr_cll_idx = (uint8_t)((state->rr_cll_idx + 1U) % VCI_MAX_CLLS);

        dpdu_cll_t *cll = &state->clls[idx];
        if (!cll->online || cll->cop_count == 0U) {
            continue;
        }

        dpdu_cop_slot_t *head = &cll->cop_queue[cll->cop_head];
        if (!head->in_use) {
            continue;
        }

        /* Respect both Cyclic period (next_cycle_tx_us) and P3Phys/P3Func (next_allowed_tx_us) */
        if (now_us < head->next_cycle_tx_us || now_us < cll->next_allowed_tx_us) {
            continue;
        }

        execute_one_cop_cycle(state, cll, head, send_fn);
        return true;
    }

    return false;
}
