/**
 * @file dpdu_protocol.c
 * @brief Lean D-PDU API (ISO 22900-2 subset) binary wire framing, CRC-16-CCITT,
 *        13-ComParam Table, and ComPrimitive / ISO-TP / P2 / P2* / P3 engine.
 */

#include "dpdu_protocol.h"
#include "can_hal.h"
#include "isotp.h"
#include "led_status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <string.h>

static dpdu_check_cancel_fn_t s_cancel_check_fn = NULL;

void dpdu_engine_set_cancel_checker(dpdu_check_cancel_fn_t cancel_fn)
{
    s_cancel_check_fn = cancel_fn;
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
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

uint16_t dpdu_crc16_ccitt(const uint8_t *data, size_t len)
{
    return crc16_update(0xFFFFU, data, len);
}

void dpdu_parser_init(dpdu_rx_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = DPDU_PARSE_STATE_SOF;
}

dpdu_parse_result_t dpdu_parser_feed_byte(dpdu_rx_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {
        case DPDU_PARSE_STATE_SOF:
            if (byte == DPDU_SOF_BYTE) {
                parser->hdr_buf[0] = byte;
                parser->hdr_idx = 1;
                parser->state = DPDU_PARSE_STATE_HDR;
            }
            return DPDU_PARSE_INCOMPLETE;

        case DPDU_PARSE_STATE_HDR:
            parser->hdr_buf[parser->hdr_idx++] = byte;
            if (parser->hdr_idx == DPDU_HEADER_SIZE) {
                uint8_t ver = parser->hdr_buf[1];
                uint16_t len = read_u16_le(&parser->hdr_buf[6]);
                if (ver != DPDU_PROTOCOL_VERSION || len > DPDU_MAX_PAYLOAD) {
                    parser->state = DPDU_PARSE_STATE_SOF;
                    return DPDU_PARSE_ERR_HDR;
                }
                parser->frame.opcode  = parser->hdr_buf[2];
                parser->frame.handle  = parser->hdr_buf[3];
                parser->frame.seq_tag = read_u16_le(&parser->hdr_buf[4]);
                parser->frame.length  = len;
                parser->payload_idx   = 0;
                parser->crc_idx       = 0;
                parser->state = (len > 0) ? DPDU_PARSE_STATE_PAYLOAD : DPDU_PARSE_STATE_CRC;
            }
            return DPDU_PARSE_INCOMPLETE;

        case DPDU_PARSE_STATE_PAYLOAD:
            parser->frame.payload[parser->payload_idx++] = byte;
            if (parser->payload_idx >= parser->frame.length) {
                parser->crc_idx = 0;
                parser->state = DPDU_PARSE_STATE_CRC;
            }
            return DPDU_PARSE_INCOMPLETE;

        case DPDU_PARSE_STATE_CRC:
            parser->crc_buf[parser->crc_idx++] = byte;
            if (parser->crc_idx >= DPDU_CRC_SIZE) {
                uint16_t rx_crc = read_u16_le(parser->crc_buf);
                uint16_t calc_crc = crc16_update(0xFFFFU, parser->hdr_buf, DPDU_HEADER_SIZE);
                if (parser->frame.length > 0) {
                    calc_crc = crc16_update(calc_crc, parser->frame.payload, parser->frame.length);
                }
                parser->state = DPDU_PARSE_STATE_SOF;
                if (rx_crc == calc_crc) {
                    return DPDU_PARSE_OK;
                }
                return DPDU_PARSE_ERR_CRC;
            }
            return DPDU_PARSE_INCOMPLETE;

        default:
            parser->state = DPDU_PARSE_STATE_SOF;
            return DPDU_PARSE_INCOMPLETE;
    }
}

size_t dpdu_encode_frame(const dpdu_frame_t *frame, uint8_t *out_buf, size_t out_max)
{
    size_t total_len = DPDU_HEADER_SIZE + frame->length + DPDU_CRC_SIZE;
    if (frame->length > DPDU_MAX_PAYLOAD || out_max < total_len) {
        return 0;
    }

    out_buf[0] = DPDU_SOF_BYTE;
    out_buf[1] = DPDU_PROTOCOL_VERSION;
    out_buf[2] = frame->opcode;
    out_buf[3] = frame->handle;
    write_u16_le(&out_buf[4], frame->seq_tag);
    write_u16_le(&out_buf[6], frame->length);

    if (frame->length > 0) {
        memcpy(&out_buf[DPDU_HEADER_SIZE], frame->payload, frame->length);
    }

    uint16_t crc = dpdu_crc16_ccitt(out_buf, DPDU_HEADER_SIZE + frame->length);
    write_u16_le(&out_buf[DPDU_HEADER_SIZE + frame->length], crc);

    return total_len;
}

void dpdu_engine_init(dpdu_vci_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->connected          = false;
    state->bus_mode           = VCI_BUS_MODE_SIM_LOOPBACK;

    /* Initialize all 13 ISO 22900-2 aligned ComParams */
    state->baudrate_bps       = 500000U;                /* CP_BAUDRATE */
    state->can_req_id         = 0x000007DFU;            /* CP_CAN_PHYS_REQ_ID */
    state->can_resp_id        = 0x000007E8U;            /* CP_CAN_RESP_USDT_ID */
    state->addressing_type    = DPDU_ADDR_FUNCTIONAL;   /* CP_ADDRESSING_TYPE */
    state->can_filler_byte    = 0xCCU;                  /* CP_CAN_FILLER_BYTE */
    state->isotp_bs           = 0U;                     /* CP_BLOCK_SIZE */
    state->isotp_stmin        = 0U;                     /* CP_ST_MIN */
    state->isotp_n_bs_ms      = 1000U;                  /* CP_N_BS_MS */
    state->isotp_n_cr_ms      = 1000U;                  /* CP_N_CR_MS */
    state->p2_max_ms          = 50U;                    /* CP_P2_MAX_MS */
    state->p2_star_ms         = 5000U;                  /* CP_P2_STAR_MS */
    state->p3_phys_ms         = 5U;                     /* CP_P3_PHYS_MS */
    state->p3_func_ms         = 50U;                    /* CP_P3_FUNC_MS */
    state->next_allowed_tx_us = 0LL;
}

void dpdu_build_ack_frame(dpdu_frame_t *out_ack, const dpdu_frame_t *cmd, uint8_t status)
{
    memset(out_ack, 0, sizeof(*out_ack));
    out_ack->opcode     = PDU_EVT_CMD_ACK;
    out_ack->handle     = cmd->handle;
    out_ack->seq_tag    = cmd->seq_tag;
    out_ack->length     = 2U;
    out_ack->payload[0] = cmd->opcode;
    out_ack->payload[1] = status;
}

void dpdu_build_error_frame(dpdu_frame_t *out_err, const dpdu_frame_t *cmd, uint8_t err_code, uint8_t detail)
{
    memset(out_err, 0, sizeof(*out_err));
    out_err->opcode     = PDU_EVT_ERROR;
    out_err->handle     = cmd->handle;
    out_err->seq_tag    = cmd->seq_tag;
    out_err->length     = 2U;
    out_err->payload[0] = err_code;
    out_err->payload[1] = detail;
}

uint8_t dpdu_validate_start_cop(const dpdu_vci_state_t *state, const dpdu_frame_t *cmd)
{
    if (!state->connected) {
        return PDU_STATUS_NOT_CONN;
    }
    if (cmd->length < 3U) {
        return PDU_STATUS_INVALID_ARG;
    }
    uint8_t cop_type = cmd->payload[0];
    if (cop_type != PDU_COPT_SENDRECV && cop_type != PDU_COPT_SEND_ONLY) {
        return PDU_STATUS_INVALID_ARG;
    }
    return PDU_STATUS_NOERROR;
}

static void emit_ack(
    dpdu_emit_evt_fn_t emit_fn,
    const dpdu_frame_t *cmd,
    uint8_t status,
    const uint8_t *extra,
    uint16_t extra_len
)
{
    static dpdu_frame_t rsp;
    dpdu_build_ack_frame(&rsp, cmd, status);
    if (extra != NULL && extra_len > 0 && (2U + extra_len) <= DPDU_MAX_PAYLOAD) {
        memcpy(&rsp.payload[2], extra, extra_len);
        rsp.length = 2U + extra_len;
    }
    emit_fn(&rsp);
}

static void emit_result(
    dpdu_emit_evt_fn_t emit_fn,
    const dpdu_frame_t *cmd,
    uint32_t rx_can_id,
    uint16_t rx_time_ms,
    const uint8_t *pdu,
    uint16_t pdu_len
)
{
    static dpdu_frame_t rsp;
    if ((6U + pdu_len) > DPDU_MAX_PAYLOAD) {
        return;
    }
    memset(&rsp, 0, sizeof(rsp));
    rsp.opcode  = PDU_EVT_RESULT;
    rsp.handle  = cmd->handle;
    rsp.seq_tag = cmd->seq_tag;
    write_u32_le(&rsp.payload[0], rx_can_id);
    write_u16_le(&rsp.payload[4], rx_time_ms);
    memcpy(&rsp.payload[6], pdu, pdu_len);
    rsp.length = 6U + pdu_len;
    emit_fn(&rsp);
}

static void emit_error(
    dpdu_emit_evt_fn_t emit_fn,
    const dpdu_frame_t *cmd,
    uint8_t err_code,
    uint8_t detail
)
{
    static dpdu_frame_t rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.opcode     = PDU_EVT_ERROR;
    rsp.handle     = cmd->handle;
    rsp.seq_tag    = cmd->seq_tag;
    rsp.length     = 2U;
    rsp.payload[0] = err_code;
    rsp.payload[1] = detail;
    emit_fn(&rsp);
}

static bool is_valid_baudrate_bps(uint32_t *inout_baud)
{
    /* Accept either bps (125000, 250000, 500000, 1000000) or kHz (125, 250, 500, 1000) */
    if (*inout_baud == 125U || *inout_baud == 250U || *inout_baud == 500U || *inout_baud == 1000U) {
        *inout_baud *= 1000U;
    }
    return (*inout_baud == 125000U ||
            *inout_baud == 250000U ||
            *inout_baud == 500000U ||
            *inout_baud == 1000000U);
}

static void enforce_p3_inter_request_delay(dpdu_vci_state_t *state)
{
    while (esp_timer_get_time() < state->next_allowed_tx_us) {
        if (s_cancel_check_fn != NULL && s_cancel_check_fn(NULL)) {
            return;
        }
        int64_t wait_us = state->next_allowed_tx_us - esp_timer_get_time();
        if (wait_us <= 0) {
            break;
        }
        if (wait_us >= 10000LL) {
            vTaskDelay(1);
        } else {
            esp_rom_delay_us((uint32_t)wait_us);
            break;
        }
    }
}

static void schedule_next_p3_window(dpdu_vci_state_t *state, int64_t t_req_start_us, bool send_only)
{
    int64_t now_us = esp_timer_get_time();
    if (state->addressing_type == DPDU_ADDR_FUNCTIONAL) {
        /* Functional requests must respect CP_P3_FUNC_MS from request start (and at least CP_P3_PHYS_MS after completion) */
        int64_t func_end_us = t_req_start_us + ((int64_t)state->p3_func_ms * 1000LL);
        int64_t min_gap_us  = now_us + ((int64_t)state->p3_phys_ms * 1000LL);
        state->next_allowed_tx_us = (func_end_us > min_gap_us) ? func_end_us : min_gap_us;
    } else {
        /* Physical requests enforce CP_P3_PHYS_MS after response (or CP_P2_MAX_MS if SEND_ONLY) */
        uint32_t gap_ms = send_only ? state->p2_max_ms : state->p3_phys_ms;
        state->next_allowed_tx_us = now_us + ((int64_t)gap_ms * 1000LL);
    }
}

static bool is_matching_diagnostic_sid(uint8_t req_sid, const uint8_t *rx_pdu, uint16_t rx_len)
{
    if (rx_len == 0U) {
        return false;
    }
    /* Positive Response: SID + 0x40 (e.g., 0x01 -> 0x41, 0x09 -> 0x49, 0x2E -> 0x6E) */
    if (rx_pdu[0] == (uint8_t)(req_sid + 0x40U)) {
        return true;
    }
    /* Negative Response: [0x7F, req_sid, NRC] */
    if (rx_len >= 3U && rx_pdu[0] == 0x7FU && rx_pdu[1] == req_sid) {
        return true;
    }
    return false;
}

static void execute_start_cop(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_emit_evt_fn_t emit_fn
)
{
    /* Validate state in case link was disconnected while command was queued */
    uint8_t val_status = dpdu_validate_start_cop(state, cmd);
    if (val_status != PDU_STATUS_NOERROR) {
        emit_error(emit_fn, cmd, PDU_ERR_CAN_BUS_OFF, val_status);
        return;
    }

    uint8_t cop_type       = cmd->payload[0];
    const uint8_t *pdu_req = &cmd->payload[2];
    uint16_t pdu_req_len   = cmd->length - 2U;
    uint8_t req_sid        = pdu_req[0];

    /* Check if cancel was requested before starting CAN transmission */
    if (s_cancel_check_fn != NULL && s_cancel_check_fn(NULL)) {
        emit_error(emit_fn, cmd, PDU_ERR_CANCELLED, req_sid);
        return;
    }

    /* 1. Enforce CP_P3_PHYS_MS / CP_P3_FUNC_MS inter-request timing before transmitting */
    enforce_p3_inter_request_delay(state);

    /* Re-check cancel after P3 delay */
    if (s_cancel_check_fn != NULL && s_cancel_check_fn(NULL)) {
        emit_error(emit_fn, cmd, PDU_ERR_CANCELLED, req_sid);
        return;
    }

    /* 2. Build ISO-TP Configuration from the 13-ComParam Table */
    isotp_config_t isotp_cfg = {
        .can_req_id      = state->can_req_id,
        .can_resp_id     = state->can_resp_id,
        .addressing_type = (uint8_t)state->addressing_type,
        .can_filler_byte = (uint8_t)state->can_filler_byte,
        .fc_bs           = (uint8_t)state->isotp_bs,
        .fc_stmin        = (uint8_t)state->isotp_stmin,
        .n_bs_ms         = state->isotp_n_bs_ms,
        .n_cr_ms         = state->isotp_n_cr_ms,
    };

    /* 3. Transmit PDU over ISO-TP / CAN HAL (PDU_EVT_CMD_ACK was already emitted on Core 0 admission) */
    int64_t t_start_us = esp_timer_get_time();
    esp_err_t tx_err = isotp_send_pdu(&isotp_cfg, pdu_req, pdu_req_len);
    if (tx_err != ESP_OK) {
        uint8_t err_code = (tx_err == ESP_ERR_TIMEOUT) ? PDU_ERR_ISOTP_TIMEOUT : PDU_ERR_CAN_BUS_OFF;
        emit_error(emit_fn, cmd, err_code, (uint8_t)tx_err);
        return;
    }

    if (cop_type == PDU_COPT_SEND_ONLY) {
        schedule_next_p3_window(state, t_start_us, true);
        return;
    }

    /* 4. Wait for ECU response within CP_P2_MAX_MS (or CP_P2_STAR_MS on NRC 0x78, or CP_N_CR_MS during Multi-Frame) */
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)state->p2_max_ms * 1000LL);
    vci_can_frame_t rx_frame;
    static isotp_rx_ctx_t rx_ctx;
    isotp_rx_init(&rx_ctx);

    while (esp_timer_get_time() < deadline_us) {
        /* Check if Host sent out-of-band PDU_CMD_CANCEL_COP while ComPrimitive is in flight */
        if (s_cancel_check_fn != NULL && s_cancel_check_fn(NULL)) {
            emit_error(emit_fn, cmd, PDU_ERR_CANCELLED, req_sid);
            return;
        }

        int64_t rem_us = deadline_us - esp_timer_get_time();
        uint32_t wait_ms = (uint32_t)((rem_us + 999LL) / 1000LL);
        if (wait_ms > 5U) {
            wait_ms = 5U;
        } else if (wait_ms == 0U) {
            wait_ms = 1U;
        }

        if (can_hal_recv(&rx_frame, wait_ms) == ESP_OK) {
            isotp_rx_status_t st = isotp_rx_feed_frame(&rx_ctx, &rx_frame, &isotp_cfg);

            if (st == ISOTP_RX_IN_PROGRESS) {
                /* Multi-Frame FF or intermediate CF received; reset deadline to CP_N_CR_MS */
                deadline_us = esp_timer_get_time() + ((int64_t)state->isotp_n_cr_ms * 1000LL);
            } else if (st == ISOTP_RX_COMPLETE) {
                /* Verify SID matches Positive Response (SID + 0x40) or Negative Response (0x7F + SID) */
                if (!is_matching_diagnostic_sid(req_sid, rx_ctx.pdu, rx_ctx.total_len)) {
                    isotp_rx_init(&rx_ctx);
                    continue;
                }

                /* Check for UDS/OBD Negative Response 0x7F [SID] 0x78 (ResponsePending) */
                if (rx_ctx.total_len >= 3U &&
                    rx_ctx.pdu[0] == 0x7FU &&
                    rx_ctx.pdu[1] == req_sid &&
                    rx_ctx.pdu[2] == 0x78U) {
                    /* Extend timeout to CP_P2_STAR_MS (5000ms default) and re-arm ISO-TP RX */
                    isotp_rx_init(&rx_ctx);
                    deadline_us = esp_timer_get_time() + ((int64_t)state->p2_star_ms * 1000LL);
                    continue;
                }

                uint16_t elapsed_ms = (uint16_t)((esp_timer_get_time() - t_start_us) / 1000LL);
                schedule_next_p3_window(state, t_start_us, false);
                led_status_pulse_activity();
                emit_result(emit_fn, cmd, rx_ctx.rx_can_id, elapsed_ms, rx_ctx.pdu, rx_ctx.total_len);
                return;
            } else if (st == ISOTP_RX_ERR_SEQ || st == ISOTP_RX_ERR_OVERFLOW || st == ISOTP_RX_ERR_FC_TX) {
                schedule_next_p3_window(state, t_start_us, false);
                emit_error(emit_fn, cmd, PDU_ERR_MALFORMED_FRAME, (uint8_t)st);
                return;
            }
        }
    }

    schedule_next_p3_window(state, t_start_us, false);

    /* Timeout expired: distinguish Multi-Frame CP_N_CR_MS timeout from initial CP_P2_MAX_MS timeout */
    if (rx_ctx.in_progress) {
        emit_error(emit_fn, cmd, PDU_ERR_ISOTP_TIMEOUT, req_sid);
    } else {
        emit_error(emit_fn, cmd, PDU_ERR_P2_TIMEOUT, req_sid);
    }
}

void dpdu_engine_process_cmd(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_emit_evt_fn_t emit_fn
)
{
    state->rx_cmd_count++;
    led_status_pulse_activity();

    switch (cmd->opcode) {
        case PDU_CMD_CONNECT: {
            if (cmd->length < 1U) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }
            uint8_t  bus_mode = cmd->payload[0];
            uint16_t baud_khz = (uint16_t)(state->baudrate_bps / 1000U);
            uint8_t  flags    = 0U;

            if (cmd->length >= 3U) {
                uint16_t req_baud_khz = read_u16_le(&cmd->payload[1]);
                if (req_baud_khz != 0U) {
                    baud_khz = req_baud_khz;
                }
            }
            if (cmd->length >= 4U) {
                flags = cmd->payload[3];
            }

            uint32_t baud_bps = (uint32_t)baud_khz * 1000U;
            if ((bus_mode != VCI_BUS_MODE_SIM_LOOPBACK && bus_mode != VCI_BUS_MODE_PHYSICAL_CAN) ||
                !is_valid_baudrate_bps(&baud_bps)) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }

            if (can_hal_start(bus_mode, (uint16_t)(baud_bps / 1000U)) != ESP_OK) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }

            state->connected          = true;
            state->bus_mode           = bus_mode;
            state->baudrate_bps       = baud_bps;
            state->connect_flags      = flags;
            state->next_allowed_tx_us = 0LL;
            led_status_set_mode(VCI_LED_CONNECTED_IDLE);

            uint8_t info[4];
            info[0] = state->bus_mode;
            write_u16_le(&info[1], (uint16_t)(state->baudrate_bps / 1000U));
            info[3] = state->connect_flags;
            emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, info, sizeof(info));
            return;
        }

        case PDU_CMD_DISCONNECT: {
            can_hal_stop();
            state->connected = false;
            led_status_set_mode(VCI_LED_DISCONNECTED);
            emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, NULL, 0);
            return;
        }

        case PDU_CMD_SET_COMPARAM: {
            if (cmd->length < 5U) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }
            uint8_t  param_id  = cmd->payload[0];
            uint32_t param_val = read_u32_le(&cmd->payload[1]);

            switch (param_id) {
                case CP_BAUDRATE:
                    if (!is_valid_baudrate_bps(&param_val)) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->baudrate_bps = param_val;
                    if (state->connected) {
                        (void)can_hal_start(state->bus_mode, (uint16_t)(state->baudrate_bps / 1000U));
                    }
                    break;
                case CP_CAN_PHYS_REQ_ID:
                    state->can_req_id = param_val;
                    break;
                case CP_CAN_RESP_USDT_ID:
                    state->can_resp_id = param_val;
                    break;
                case CP_ADDRESSING_TYPE:
                    if (param_val > DPDU_ADDR_PHYSICAL) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->addressing_type = param_val;
                    break;
                case CP_CAN_FILLER_BYTE:
                    if (param_val > 0xFFU) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->can_filler_byte = param_val;
                    break;
                case CP_BLOCK_SIZE:
                    if (param_val > 0xFFU) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->isotp_bs = param_val;
                    break;
                case CP_ST_MIN:
                    if (param_val > 0xFFU) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->isotp_stmin = param_val;
                    break;
                case CP_N_BS_MS:
                    if (param_val == 0U || param_val > 60000U) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->isotp_n_bs_ms = param_val;
                    break;
                case CP_N_CR_MS:
                    if (param_val == 0U || param_val > 60000U) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->isotp_n_cr_ms = param_val;
                    break;
                case CP_P2_MAX_MS:
                    if (param_val == 0U || param_val > 60000U) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->p2_max_ms = param_val;
                    break;
                case CP_P2_STAR_MS:
                    if (param_val == 0U || param_val > 60000U) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->p2_star_ms = param_val;
                    break;
                case CP_P3_PHYS_MS:
                    if (param_val > 60000U) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->p3_phys_ms = param_val;
                    break;
                case CP_P3_FUNC_MS:
                    if (param_val > 60000U) {
                        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                        return;
                    }
                    state->p3_func_ms = param_val;
                    break;
                default:
                    emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                    return;
            }

            uint8_t echo[5];
            echo[0] = param_id;
            write_u32_le(&echo[1], param_val);
            emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, echo, sizeof(echo));
            return;
        }

        case PDU_CMD_GET_COMPARAM: {
            if (cmd->length < 1U) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }
            uint8_t  param_id  = cmd->payload[0];
            uint32_t param_val = 0;

            switch (param_id) {
                case CP_BAUDRATE:         param_val = state->baudrate_bps;    break;
                case CP_CAN_PHYS_REQ_ID:  param_val = state->can_req_id;      break;
                case CP_CAN_RESP_USDT_ID: param_val = state->can_resp_id;     break;
                case CP_ADDRESSING_TYPE:  param_val = state->addressing_type; break;
                case CP_CAN_FILLER_BYTE:  param_val = state->can_filler_byte; break;
                case CP_BLOCK_SIZE:       param_val = state->isotp_bs;        break;
                case CP_ST_MIN:           param_val = state->isotp_stmin;     break;
                case CP_N_BS_MS:          param_val = state->isotp_n_bs_ms;   break;
                case CP_N_CR_MS:          param_val = state->isotp_n_cr_ms;   break;
                case CP_P2_MAX_MS:        param_val = state->p2_max_ms;       break;
                case CP_P2_STAR_MS:       param_val = state->p2_star_ms;      break;
                case CP_P3_PHYS_MS:       param_val = state->p3_phys_ms;      break;
                case CP_P3_FUNC_MS:       param_val = state->p3_func_ms;      break;
                default:
                    emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                    return;
            }

            uint8_t out[5];
            out[0] = param_id;
            write_u32_le(&out[1], param_val);
            emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, out, sizeof(out));
            return;
        }

        case PDU_CMD_GET_STATUS: {
            uint8_t stat[20];
            uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
            stat[0] = state->connected ? 1U : 0U;
            stat[1] = state->bus_mode;
            write_u16_le(&stat[2], (uint16_t)(state->baudrate_bps / 1000U));
            write_u32_le(&stat[4], state->rx_cmd_count);
            write_u32_le(&stat[8], state->crc_err_count);
            write_u32_le(&stat[12], uptime_ms);
            write_u32_le(&stat[16], state->can_req_id);
            emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, stat, sizeof(stat));
            return;
        }

        case PDU_CMD_START_COP: {
            execute_start_cop(state, cmd, emit_fn);
            return;
        }

        case PDU_CMD_CANCEL_COP: {
            emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, NULL, 0);
            return;
        }

        default:
            emit_ack(emit_fn, cmd, PDU_STATUS_UNSUPPORTED, NULL, 0);
            return;
    }
}
