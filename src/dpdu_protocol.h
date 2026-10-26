/**
 * @file dpdu_protocol.h
 * @brief Lean D-PDU API (ISO 22900-2 subset) binary wire protocol and engine definitions.
 */

#ifndef DPDU_PROTOCOL_H
#define DPDU_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Wire Framing Constants */
#define DPDU_SOF_BYTE               0xA5U
#define DPDU_PROTOCOL_VERSION       0x01U
#define DPDU_HEADER_SIZE            8U
#define DPDU_CRC_SIZE               2U
#define DPDU_MAX_PAYLOAD            512U

/* Host -> VCI Command Opcodes */
typedef enum {
    PDU_CMD_CONNECT         = 0x01, /* [bus_mode:1B] (or legacy 4B [bus_mode:1B][baud_khz:2B][flags:1B]) */
    PDU_CMD_DISCONNECT      = 0x02, /* Empty */
    PDU_CMD_SET_COMPARAM    = 0x03, /* [param_id:1B][param_val:4B (<I)] */
    PDU_CMD_START_COP       = 0x04, /* [cop_type:1B][flags:1B][pdu:N B] */
    PDU_CMD_CANCEL_COP      = 0x05, /* Empty */
    PDU_CMD_GET_STATUS      = 0x06, /* Empty */
    PDU_CMD_GET_COMPARAM    = 0x07, /* [param_id:1B] */
} dpdu_cmd_opcode_t;

/* VCI -> Host Event Opcodes */
typedef enum {
    PDU_EVT_CMD_ACK         = 0x80, /* [orig_opcode:1B][status:1B][optional_data...] */
    PDU_EVT_RESULT          = 0x81, /* [rx_can_id:4B (<I)][rx_time_ms:2B (<H)][pdu:N B] */
    PDU_EVT_ERROR           = 0x82, /* [err_code:1B][detail:1B] */
} dpdu_evt_opcode_t;

/* Status Codes (in PDU_EVT_CMD_ACK) */
typedef enum {
    PDU_STATUS_NOERROR      = 0x00,
    PDU_STATUS_INVALID_ARG  = 0x01,
    PDU_STATUS_NOT_CONN     = 0x02,
    PDU_STATUS_BUSY         = 0x03,
    PDU_STATUS_UNSUPPORTED  = 0x04,
} dpdu_status_t;

/* Error Codes (in PDU_EVT_ERROR) */
typedef enum {
    PDU_ERR_P2_TIMEOUT      = 0x01,
    PDU_ERR_ISOTP_TIMEOUT   = 0x02,
    PDU_ERR_CAN_BUS_OFF     = 0x03,
    PDU_ERR_CRC_MISMATCH    = 0x04,
    PDU_ERR_MALFORMED_FRAME = 0x05,
    PDU_ERR_CANCELLED       = 0x06,
} dpdu_err_code_t;

/* ISO 22900-2 Aligned ComParam IDs (for PDU_CMD_SET_COMPARAM / PDU_CMD_GET_COMPARAM) */
typedef enum {
    /* Physical Layer */
    CP_BAUDRATE             = 0x01, /* Default: 500000 bps (125000, 250000, 500000, 1000000) */

    /* CAN Addressing Layer */
    CP_CAN_PHYS_REQ_ID      = 0x02, /* Default: 0x000007DF (or 0x7E0, 0x18DB33F1, 0x18DAxxF1) */
    CP_CAN_RESP_USDT_ID     = 0x03, /* Default: 0x000007E8 (or 0x18DAF1xx) */
    CP_ADDRESSING_TYPE      = 0x04, /* Default: 0 (0 = FUNCTIONAL, 1 = PHYSICAL) */

    /* ISO 15765-2 (ISO-TP) Transport Layer */
    CP_CAN_FILLER_BYTE      = 0x05, /* Default: 0xCC (0x00..0xFF padding byte) */
    CP_BLOCK_SIZE           = 0x06, /* Default: 0 (Flow Control BS: 0 = continuous, 1..255) */
    CP_ST_MIN               = 0x07, /* Default: 0 (Flow Control STmin: 0..127 ms or 0xF1..0xF9) */
    CP_N_BS_MS              = 0x08, /* Default: 1000 ms (ISO-TP Sender wait for Flow Control) */
    CP_N_CR_MS              = 0x09, /* Default: 1000 ms (ISO-TP Receiver wait for Consecutive Frame) */

    /* ISO 15765-3 / ISO 14229-2 Application Timing Layer */
    CP_P2_MAX_MS            = 0x0A, /* Default: 50 ms (Max time from TX to initial ECU response) */
    CP_P2_STAR_MS           = 0x0B, /* Default: 5000 ms (Extended timeout on NRC 0x78 ResponsePending) */
    CP_P3_PHYS_MS           = 0x0C, /* Default: 5 ms (Min inter-request gap after Physical request) */
    CP_P3_FUNC_MS           = 0x0D, /* Default: 50 ms (Functional collection window & min inter-request gap) */
} dpdu_comparam_id_t;

/* Addressing Types (for CP_ADDRESSING_TYPE) */
typedef enum {
    DPDU_ADDR_FUNCTIONAL    = 0x00,
    DPDU_ADDR_PHYSICAL      = 0x01,
} dpdu_addr_type_t;

/* ComPrimitive Types */
typedef enum {
    PDU_COPT_SENDRECV       = 0x01,
    PDU_COPT_SEND_ONLY      = 0x02,
} dpdu_cop_type_t;

/* Bus Modes */
typedef enum {
    VCI_BUS_MODE_SIM_LOOPBACK = 0x00,
    VCI_BUS_MODE_PHYSICAL_CAN = 0x01,
} vci_bus_mode_t;

/* Parsed D-PDU Packet Structure (passed via FreeRTOS queues) */
typedef struct {
    uint8_t  opcode;
    uint8_t  handle;
    uint16_t seq_tag;
    uint16_t length;
    uint8_t  payload[DPDU_MAX_PAYLOAD];
} dpdu_frame_t;

/* Stream Parser State Machine */
typedef enum {
    DPDU_PARSE_STATE_SOF = 0,
    DPDU_PARSE_STATE_HDR,
    DPDU_PARSE_STATE_PAYLOAD,
    DPDU_PARSE_STATE_CRC,
} dpdu_parse_state_t;

typedef enum {
    DPDU_PARSE_INCOMPLETE = 0,
    DPDU_PARSE_OK,
    DPDU_PARSE_ERR_CRC,
    DPDU_PARSE_ERR_HDR,
} dpdu_parse_result_t;

typedef struct {
    dpdu_parse_state_t state;
    uint8_t            hdr_buf[DPDU_HEADER_SIZE];
    uint16_t           hdr_idx;
    uint16_t           payload_idx;
    uint8_t            crc_buf[DPDU_CRC_SIZE];
    uint16_t           crc_idx;
    dpdu_frame_t       frame;
} dpdu_rx_parser_t;

/* VCI Logical Link & ComParam State */
typedef struct {
    bool     connected;
    uint8_t  bus_mode;
    uint8_t  connect_flags;

    /* 13 Standard ComParams */
    uint32_t baudrate_bps;      /* CP_BAUDRATE */
    uint32_t can_req_id;        /* CP_CAN_PHYS_REQ_ID */
    uint32_t can_resp_id;       /* CP_CAN_RESP_USDT_ID */
    uint32_t addressing_type;   /* CP_ADDRESSING_TYPE (0=FUNC, 1=PHYS) */
    uint32_t can_filler_byte;   /* CP_CAN_FILLER_BYTE */
    uint32_t isotp_bs;          /* CP_BLOCK_SIZE */
    uint32_t isotp_stmin;       /* CP_ST_MIN */
    uint32_t isotp_n_bs_ms;     /* CP_N_BS_MS */
    uint32_t isotp_n_cr_ms;     /* CP_N_CR_MS */
    uint32_t p2_max_ms;         /* CP_P2_MAX_MS */
    uint32_t p2_star_ms;        /* CP_P2_STAR_MS */
    uint32_t p3_phys_ms;        /* CP_P3_PHYS_MS */
    uint32_t p3_func_ms;        /* CP_P3_FUNC_MS */

    /* P3 Inter-Request Timing State */
    int64_t  next_allowed_tx_us;

    /* Diagnostics Counters */
    uint32_t rx_cmd_count;
    uint32_t crc_err_count;
} dpdu_vci_state_t;

/**
 * @brief Optional callback to check if a PDU_CMD_CANCEL_COP arrived while a ComPrimitive is in flight.
 */
typedef bool (*dpdu_check_cancel_fn_t)(dpdu_frame_t *out_cancel_cmd);

/**
 * @brief Compute CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF).
 */
uint16_t dpdu_crc16_ccitt(const uint8_t *data, size_t len);

/**
 * @brief Initialize or reset a stream parser instance.
 */
void dpdu_parser_init(dpdu_rx_parser_t *parser);

/**
 * @brief Feed one byte into the stream parser.
 */
dpdu_parse_result_t dpdu_parser_feed_byte(dpdu_rx_parser_t *parser, uint8_t byte);

/**
 * @brief Serialize a dpdu_frame_t into wire bytes (8B header + payload + 2B CRC16).
 * @return Total wire bytes written to out_buf, or 0 on buffer overflow.
 */
size_t dpdu_encode_frame(const dpdu_frame_t *frame, uint8_t *out_buf, size_t out_max);

/**
 * @brief Callback type for emitting D-PDU events (ACK, RESULT, ERROR) to the host queue.
 */
typedef void (*dpdu_emit_evt_fn_t)(const dpdu_frame_t *evt);

/**
 * @brief Initialize the VCI D-PDU state and default OBD-II ComParams.
 */
void dpdu_engine_init(dpdu_vci_state_t *state);

/**
 * @brief Register a non-blocking cancel-check hook used during active ComPrimitives.
 */
void dpdu_engine_set_cancel_checker(dpdu_check_cancel_fn_t cancel_fn);

/**
 * @brief Process an incoming Host D-PDU command and emit response/event frames.
 */
void dpdu_engine_process_cmd(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_emit_evt_fn_t emit_fn
);

#endif /* DPDU_PROTOCOL_H */
