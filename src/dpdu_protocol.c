/**
 * @file dpdu_protocol.c
 * @brief Lean D-PDU API binary wire framing, CRC-16-CCITT, and ComPrimitive/P2/P2* engine.
 */

#include "dpdu_protocol.h"
#include "can_hal.h"
#include "isotp.h"
#include "led_status.h"
#include "esp_timer.h"
#include <string.h>

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
    state->connected      = false;
    state->bus_mode       = VCI_BUS_MODE_SIM_LOOPBACK;
    state->baud_khz       = 500;
    state->can_req_id     = 0x000007DFU;
    state->can_resp_id    = 0x000007E8U;
    state->p2_max_ms      = 50U;
    state->p2_star_ms     = 5000U;
    state->isotp_bs       = 0U;
    state->isotp_stmin_ms = 0U;
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
    memset(&rsp, 0, sizeof(rsp));
    rsp.opcode  = PDU_EVT_CMD_ACK;
    rsp.handle  = cmd->handle;
    rsp.seq_tag = cmd->seq_tag;
    rsp.payload[0] = cmd->opcode;
    rsp.payload[1] = status;
    if (extra != NULL && extra_len > 0 && (2U + extra_len) <= DPDU_MAX_PAYLOAD) {
        memcpy(&rsp.payload[2], extra, extra_len);
        rsp.length = 2U + extra_len;
    } else {
        rsp.length = 2U;
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

static void execute_start_cop(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_emit_evt_fn_t emit_fn
)
{
    if (!state->connected) {
        emit_ack(emit_fn, cmd, PDU_STATUS_NOT_CONN, NULL, 0);
        return;
    }
    if (cmd->length < 3U) {
        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
        return;
    }

    uint8_t cop_type = cmd->payload[0];
    const uint8_t *pdu_req = &cmd->payload[2];
    uint16_t pdu_req_len   = cmd->length - 2U;

    if (cop_type != PDU_COPT_SENDRECV && cop_type != PDU_COPT_SEND_ONLY) {
        emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
        return;
    }

    /* 1. Acknowledge ComPrimitive acceptance */
    emit_ack(emit_fn, cmd, PDU_STATUS_NOERROR, NULL, 0);

    /* 2. Transmit PDU over ISO-TP / CAN HAL */
    int64_t t_start_us = esp_timer_get_time();
    esp_err_t tx_err = isotp_send_pdu(state->can_req_id, pdu_req, pdu_req_len);
    if (tx_err != ESP_OK) {
        emit_error(emit_fn, cmd, PDU_ERR_CAN_BUS_OFF, (uint8_t)tx_err);
        return;
    }

    if (cop_type == PDU_COPT_SEND_ONLY) {
        return;
    }

    /* 3. Wait for ECU response within P2_Max (or P2*_Max on NRC 0x78 ResponsePending) */
    uint8_t req_sid = pdu_req[0];
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)state->p2_max_ms * 1000LL);
    vci_can_frame_t rx_frame;
    static isotp_rx_pdu_t rx_pdu;

    while (esp_timer_get_time() < deadline_us) {
        int64_t rem_us = deadline_us - esp_timer_get_time();
        uint32_t wait_ms = (uint32_t)((rem_us + 999LL) / 1000LL);
        if (wait_ms > 5U) {
            wait_ms = 5U;
        } else if (wait_ms == 0U) {
            wait_ms = 1U;
        }

        if (can_hal_recv(&rx_frame, wait_ms) == ESP_OK) {
            if (isotp_on_can_frame(&rx_frame, state->can_resp_id, state->can_req_id, &rx_pdu)) {
                /* Check for UDS/OBD Negative Response 0x7F [SID] 0x78 (ResponsePending) */
                if (rx_pdu.pdu_len >= 3U &&
                    rx_pdu.pdu[0] == 0x7FU &&
                    rx_pdu.pdu[1] == req_sid &&
                    rx_pdu.pdu[2] == 0x78U) {
                    /* Extend timeout to P2*_Max (5000ms default) */
                    deadline_us = esp_timer_get_time() + ((int64_t)state->p2_star_ms * 1000LL);
                    continue;
                }

                uint16_t elapsed_ms = (uint16_t)((esp_timer_get_time() - t_start_us) / 1000LL);
                led_status_pulse_activity();
                emit_result(emit_fn, cmd, rx_pdu.rx_can_id, elapsed_ms, rx_pdu.pdu, rx_pdu.pdu_len);
                return;
            }
        }
    }

    /* P2 / P2* Timeout expired */
    emit_error(emit_fn, cmd, PDU_ERR_P2_TIMEOUT, req_sid);
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
            if (cmd->length < 4U) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }
            uint8_t  bus_mode = cmd->payload[0];
            uint16_t baud_khz = read_u16_le(&cmd->payload[1]);
            uint8_t  flags    = cmd->payload[3];

            if ((bus_mode != VCI_BUS_MODE_SIM_LOOPBACK && bus_mode != VCI_BUS_MODE_PHYSICAL_CAN) ||
                (baud_khz != 250U && baud_khz != 500U && baud_khz != 1000U)) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }

            if (can_hal_start(bus_mode, baud_khz) != ESP_OK) {
                emit_ack(emit_fn, cmd, PDU_STATUS_INVALID_ARG, NULL, 0);
                return;
            }

            state->connected     = true;
            state->bus_mode      = bus_mode;
            state->baud_khz      = baud_khz;
            state->connect_flags = flags;
            led_status_set_mode(VCI_LED_CONNECTED_IDLE);

            uint8_t info[4];
            info[0] = state->bus_mode;
            write_u16_le(&info[1], state->baud_khz);
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
                case CP_CAN_PHYS_REQ_ID:  state->can_req_id     = param_val; break;
                case CP_CAN_RESP_USDT_ID: state->can_resp_id    = param_val; break;
                case CP_P2_MAX_MS:        state->p2_max_ms      = param_val; break;
                case CP_P2_STAR_MS:       state->p2_star_ms     = param_val; break;
                case CP_ISOTP_BS:         state->isotp_bs       = param_val; break;
                case CP_ISOTP_STMIN_MS:   state->isotp_stmin_ms = param_val; break;
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
                case CP_CAN_PHYS_REQ_ID:  param_val = state->can_req_id;     break;
                case CP_CAN_RESP_USDT_ID: param_val = state->can_resp_id;    break;
                case CP_P2_MAX_MS:        param_val = state->p2_max_ms;      break;
                case CP_P2_STAR_MS:       param_val = state->p2_star_ms;     break;
                case CP_ISOTP_BS:         param_val = state->isotp_bs;       break;
                case CP_ISOTP_STMIN_MS:   param_val = state->isotp_stmin_ms; break;
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
            write_u16_le(&stat[2], state->baud_khz);
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
