/**
 * @file dpdu_protocol.h
 * @brief ISO 22900-2:2022 App-Device IF Binary Wire Protocol, 8-CLL State, 4-Slot Queues,
 *        Round-Robin Arbitration, IOCTLs, and 100 us Hardware Timestamp Definitions.
 */

#ifndef DPDU_PROTOCOL_H
#define DPDU_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "vci_config.h"

/* Wire Framing Constants:
 * [ SOF (2B: 0xAA55) | Msg_Type (1B) | CLL_ID (1B) | COP_ID (1B) | Status (1B) | Payload_Len (2B) | Payload (0..4103B) | CRC16-CCITT (2B) ]
 */
#define DPDU_SOF_WORD                       0xAA55U
#define DPDU_SOF_BYTE_0                     0x55U   /* Little-endian LSB of 0xAA55 */
#define DPDU_SOF_BYTE_1                     0xAAU   /* Little-endian MSB of 0xAA55 */
#define DPDU_HEADER_SIZE                    8U
#define DPDU_CRC_SIZE                       2U
#define DPDU_MAX_PAYLOAD                    VCI_MAX_PACKET_PAYLOAD /* 4103 bytes */
#define DPDU_COP_INLINE_PDU_MAX             64U     /* Inline TX PDU storage per queued COP slot */
#define DPDU_CMD_INLINE_MAX                 (8U + DPDU_COP_INLINE_PDU_MAX) /* 72B inline parser buffer */

/* Host -> VCI Command Msg_Types (0x01..0x7F) */
typedef enum {
    PDU_CMD_CONNECT                         = 0x01, /* CLL_Connect (CLL_ID=1..8): [bus_mode:1B] */
    PDU_CMD_DISCONNECT                      = 0x02, /* CLL_Disconnect (CLL_ID=1..8, or 0x00=PDUDestruct) */
    PDU_CMD_SET_COMPARAM                    = 0x03, /* SetComParam (CLL_ID=1..8): [Param_ID(2B)|Param_Val(4B)] */
    PDU_CMD_START_COP                       = 0x04, /* StartComPrimitive: [NumSend(1B)|NumRecv(1B)|Cycle_ms(2B)|CAN_ID(4B)|N-PDU] */
    PDU_CMD_CANCEL_COP                      = 0x05, /* CancelComPrimitive (CLL_ID=1..8, COP_ID=0..255) */
    PDU_CMD_GET_STATUS                      = 0x06, /* Module/CLL Diagnostics Status Query */
    PDU_CMD_GET_COMPARAM                    = 0x07, /* GetComParam (CLL_ID=1..8): [Param_ID(2B)] */
    PDU_CMD_MOD_CTRL                        = 0x08, /* Module IOCTL / GetVersion (CLL_ID=0x00): [IOCTL_ID(2B)] */
} dpdu_cmd_opcode_t;

/* VCI -> Host Packet Msg_Types (0x80..0xFF) */
typedef enum {
    PDU_EVT_CMD_ACK                         = 0x80, /* Synchronous Command ACK/NACK (Status in Header Byte 5) */
    PDU_IT_RESULT                           = 0x81, /* [RX_Timestamp_us(4B)|CAN_ID(4B)|Extra_Addr(1B)|N-PDU(1..4095B)] */
    PDU_IT_STATUS                           = 0x82, /* [Event_Timestamp_us(4B)|D-PDU_Status_Code(2B)] */
    PDU_IT_ERROR                            = 0x83, /* [Error_Timestamp_us(4B)|D-PDU_Error_Code(2B)|Vendor_SubCode(2B)] */
} dpdu_evt_opcode_t;

/* Legacy alias names for compatibility */
#define PDU_EVT_RESULT                      PDU_IT_RESULT
#define PDU_EVT_STATUS                      PDU_IT_STATUS
#define PDU_EVT_ERROR                       PDU_IT_ERROR

/* Synchronous 0x80 ACK/NACK Status Codes (in Header Byte 5) */
typedef enum {
    PDU_STATUS_NOERROR                      = 0x00,
    PDU_STATUS_INVALID_ARG                  = 0x01,
    PDU_STATUS_NOT_CONN                     = 0x02,
    PDU_ERR_COMM_PC_TO_VCI_OVERRUN          = 0x03, /* CLL 4-slot queue is full */
    PDU_ERR_FCT_NOT_SUPPORTED               = 0x04, /* Unsupported protocol, Msg_Type, or IOCTL */
    PDU_ERR_BAUDRATE_CONFLICT               = 0x05, /* Conflicting CP_Baudrate while another CLL is online */
} dpdu_status_t;

#define PDU_STATUS_BUSY                     PDU_ERR_COMM_PC_TO_VCI_OVERRUN
#define PDU_STATUS_UNSUPPORTED              PDU_ERR_FCT_NOT_SUPPORTED

/* Module IOCTL IDs (for PDU_CMD_MOD_CTRL = 0x08 with CLL_ID = 0x00) */
typedef enum {
    PDU_IOCTL_READ_VBATT                    = 0x0001, /* Returns [VBatt_mV: 4B LE] */
    PDU_IOCTL_GET_CABLE_ID                  = 0x0002, /* Returns [Cable_ID: 4B LE] (SAE J1962 16-pin) */
    PDU_IOCTL_GET_VERSION                   = 0x0003, /* PDUGetVersion: Returns [FW(4B)|HW(4B)|Serial(8B)] */
} dpdu_ioctl_id_t;

/* D-PDU Status Codes (2B LE in 0x82 PDU_IT_STATUS) */
typedef enum {
    PDU_COPST_IDLE                          = 0x8001,
    PDU_COPST_EXECUTING                     = 0x8002,
    PDU_COPST_FINISHED                      = 0x8003,
    PDU_COPST_CANCELLED                     = 0x8004,
    PDU_COPST_WAITING                       = 0x8005,
    PDU_CLLST_OFFLINE                       = 0x8010,
    PDU_CLLST_ONLINE                        = 0x8011,
    PDU_CLLST_COMM_STARTED                  = 0x8012,
} dpdu_it_status_code_t;

/* D-PDU Error Codes (2B LE in 0x83 PDU_IT_ERROR) */
typedef enum {
    PDU_ERR_EVT_RX_TIMEOUT                  = 0x0001, /* P2 / P2* or ISO-TP N_Bs / N_Cr timeout */
    PDU_ERR_EVT_PROT_ERR                    = 0x0002, /* ISO-TP SN mismatch, overflow, or protocol error */
    PDU_ERR_EVT_TX_ERROR                    = 0x0003, /* CAN Bus-Off or TX failure */
    PDU_ERR_EVT_RX_ERROR                    = 0x0004, /* Host framing or CRC-16 error */
} dpdu_it_error_code_t;

/* Vendor Error SubCodes (2B LE in 0x83 PDU_IT_ERROR) */
typedef enum {
    PDU_ERR_P2_TIMEOUT                      = 0x0001,
    PDU_ERR_ISOTP_TIMEOUT                   = 0x0002,
    PDU_ERR_CAN_BUS_OFF                     = 0x0003,
    PDU_ERR_CRC_MISMATCH                    = 0x0004,
    PDU_ERR_MALFORMED_FRAME                 = 0x0005,
    PDU_ERR_CANCELLED                       = 0x0006,
} dpdu_vendor_err_subcode_t;

/* ISO 22900-2 Annex B ComParam IDs (2B LE in SetComParam / GetComParam) */
typedef enum {
    /* PDU_PC_BUSTYPE */
    CP_BAUDRATE                             = 0x0001, /* Default: 500000 bps (125k, 250k, 500k, 1M) */

    /* PDU_PC_UNIQUE_ID */
    CP_CAN_PHYS_REQ_ID                      = 0x0002, /* Default: 0x000007DF */
    CP_CAN_RESP_USDT_ID                     = 0x0003, /* Default: 0x000007E8 */
    CP_ADDRESSING_TYPE                      = 0x0004, /* Default: 0 (0 = FUNCTIONAL, 1 = PHYSICAL) */

    /* PDU_PC_BUSTYPE / ISO-TP Frame Padding */
    CP_CAN_FILLER_BYTE                      = 0x0005, /* Default: 0xCC (0x00..0xFF) */

    /* PDU_PC_TIMING (ISO 15765-2 & ISO 15765-3) */
    CP_BLOCK_SIZE                           = 0x0006, /* Default: 0 (0 = continuous, 1..255) */
    CP_ST_MIN                               = 0x0007, /* Default: 0 (0..127 ms or 0xF1..0xF9 = 100..900 us) */
    CP_N_BS_MS                              = 0x0008, /* Default: 1000 ms */
    CP_N_CR_MS                              = 0x0009, /* Default: 1000 ms */
    CP_P2_MAX_MS                            = 0x000A, /* Default: 50 ms */
    CP_P2_STAR_MS                           = 0x000B, /* Default: 5000 ms */
    CP_P3_PHYS_MS                           = 0x000C, /* Default: 5 ms */
    CP_P3_FUNC_MS                           = 0x000D, /* Default: 50 ms */
} dpdu_comparam_id_t;

/* Addressing Types (for CP_ADDRESSING_TYPE) */
typedef enum {
    DPDU_ADDR_FUNCTIONAL                    = 0x00,
    DPDU_ADDR_PHYSICAL                      = 0x01,
} dpdu_addr_type_t;

/* Bus Modes */
typedef enum {
    VCI_BUS_MODE_SIM_LOOPBACK               = 0x00,
    VCI_BUS_MODE_PHYSICAL_CAN               = 0x01,
} vci_bus_mode_t;

/**
 * @brief Parsed App-Device IF Wire Packet Structure:
 *        [ SOF (2B: 0xAA55) | Msg_Type (1B) | CLL_ID (1B) | COP_ID (1B) | Status (1B) | Payload_Len (2B) ]
 */
typedef struct {
    uint8_t  msg_type;
    uint8_t  cll_id;
    uint8_t  cop_id;
    uint8_t  status;
    uint16_t length;
    uint8_t  payload[DPDU_CMD_INLINE_MAX];
    uint8_t *ext_payload; /* Points to shared 4,104B pool buffer when length > DPDU_CMD_INLINE_MAX */
} dpdu_frame_t;

/* Stream Parser State Machine */
typedef enum {
    DPDU_PARSE_STATE_SOF0 = 0,  /* Waiting for 0x55 (LSB of 0xAA55) */
    DPDU_PARSE_STATE_SOF1,      /* Waiting for 0xAA (MSB of 0xAA55) */
    DPDU_PARSE_STATE_HDR,       /* Bytes 2..7 of 8-byte header */
    DPDU_PARSE_STATE_PAYLOAD,   /* 0..4103 payload bytes */
    DPDU_PARSE_STATE_CRC,       /* 2-byte CRC16-CCITT */
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

/**
 * @brief Queued ComPrimitive Slot (4 slots per CLL in FIFO order).
 */
typedef struct {
    bool     in_use;
    bool     started;               /* True once PDU_COPST_EXECUTING has been emitted */
    bool     cancel_requested;      /* Set out-of-band by CancelComPrimitive */
    bool     uses_large_tx_buf;     /* True if pdu_len > 64B (uses shared large TX buffer) */
    uint8_t  cll_id;                /* 1..8 */
    uint8_t  cop_id;                /* 1..255 */
    uint8_t  num_send_cycles;       /* 1..254, or 0xFF for continuous cyclic polling */
    uint8_t  num_recv_cycles;       /* 0 = SEND_ONLY, 1..254 = expected responses, 0xFF = functional window */
    uint8_t  send_cycles_done;
    uint16_t cycle_time_ms;         /* Delay between periodic send cycles */
    int64_t  next_cycle_tx_us;      /* Next scheduled microsecond timestamp for cyclic TX */
    uint32_t req_can_id;            /* Explicit CAN_ID from StartComPrimitive (or 0 to use CLL's) */
    uint16_t pdu_len;
    uint8_t  pdu[DPDU_COP_INLINE_PDU_MAX];
} dpdu_cop_slot_t;

/**
 * @brief ComLogicalLink (CLL) Context (8 concurrent CLLs supported in RAM).
 */
typedef struct {
    uint8_t         cll_id;             /* 1..8 */
    bool            online;             /* True when in PDU_CLLST_ONLINE / COMM_STARTED */

    /* 13 ISO 22900-2 Annex B ComParams */
    uint32_t        baudrate_bps;       /* CP_BAUDRATE */
    uint32_t        can_req_id;         /* CP_CAN_PHYS_REQ_ID */
    uint32_t        can_resp_id;        /* CP_CAN_RESP_USDT_ID */
    uint32_t        addressing_type;    /* CP_ADDRESSING_TYPE (0=FUNC, 1=PHYS) */
    uint32_t        can_filler_byte;    /* CP_CAN_FILLER_BYTE */
    uint32_t        isotp_bs;           /* CP_BLOCK_SIZE */
    uint32_t        isotp_stmin;        /* CP_ST_MIN (0..127 ms or 0xF1..0xF9 = 100..900 us) */
    uint32_t        isotp_n_bs_ms;      /* CP_N_BS_MS */
    uint32_t        isotp_n_cr_ms;      /* CP_N_CR_MS */
    uint32_t        p2_max_ms;          /* CP_P2_MAX_MS */
    uint32_t        p2_star_ms;         /* CP_P2_STAR_MS */
    uint32_t        p3_phys_ms;         /* CP_P3_PHYS_MS */
    uint32_t        p3_func_ms;         /* CP_P3_FUNC_MS */

    /* Per-CLL P3 Inter-Request Timing State */
    int64_t         next_allowed_tx_us;

    /* Dedicated 8-Byte Single-Frame (SF) RX Buffer for this CLL */
    uint8_t         sf_rx_buf[VCI_CLL_SF_BUF_SIZE];

    /* 4-Slot ComPrimitive FIFO Ring Queue */
    dpdu_cop_slot_t cop_queue[VCI_CLL_COP_QUEUE_DEPTH];
    uint8_t         cop_head;
    uint8_t         cop_tail;
    uint8_t         cop_count;
} dpdu_cll_t;

/**
 * @brief Complete VCI Engine State across all 8 CLLs.
 */
typedef struct {
    dpdu_cll_t      clls[VCI_MAX_CLLS];     /* CLL_ID 1..8 -> index 0..7 */
    uint8_t         online_cll_count;       /* Number of CLLs currently ONLINE */
    uint32_t        locked_baudrate_bps;    /* Locked by the first CLL to enter ONLINE */
    uint8_t         bus_mode;               /* VCI_BUS_MODE_SIM_LOOPBACK or PHYSICAL_CAN */
    uint8_t         rr_cll_idx;             /* Round-Robin scheduler index (0..7) */

    /* Shared Large TX Flag for rare Multi-Frame TX requests > 64B (uses s_mf_pool[2]) */
    bool            large_tx_in_use;

    /* Diagnostics Counters */
    uint32_t        rx_cmd_count;
    uint32_t        crc_err_count;
} dpdu_vci_state_t;

/**
 * @brief Callback type for transmitting a framed VCI -> Host packet (0x80, 0x81, 0x82, 0x83).
 */
typedef void (*dpdu_send_pkt_fn_t)(
    uint8_t msg_type,
    uint8_t cll_id,
    uint8_t cop_id,
    uint8_t status,
    const uint8_t *payload,
    uint16_t payload_len
);

/**
 * @brief Return the current hardware timestamp in microseconds (uint32_t, 1 us / 100 us resolution).
 */
uint32_t dpdu_timestamp_us(void);

/**
 * @brief Compute CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF) over a byte buffer.
 */
uint16_t dpdu_crc16_ccitt(const uint8_t *data, size_t len);

/**
 * @brief Initialize or reset a stream parser instance.
 */
void dpdu_parser_init(dpdu_rx_parser_t *parser);

/**
 * @brief Feed one byte into the 0xAA55 stream parser.
 */
dpdu_parse_result_t dpdu_parser_feed_byte(dpdu_rx_parser_t *parser, uint8_t byte);

/**
 * @brief Serialize a header + payload into wire bytes (8B header + payload + 2B CRC16 over Msg_Type..Payload).
 */
size_t dpdu_encode_packet(
    uint8_t msg_type,
    uint8_t cll_id,
    uint8_t cop_id,
    uint8_t status,
    const uint8_t *payload,
    uint16_t payload_len,
    uint8_t *out_buf,
    size_t out_max
);

/**
 * @brief Initialize all 8 CLLs, default ComParams, and Multi-Frame RX buffer pool.
 */
void dpdu_engine_init(dpdu_vci_state_t *state);

/**
 * @brief Autonomously execute internal PDUDestruct (e.g. on BLE GATT disconnect within 100 ms):
 *        halts all cyclic primitives, aborts in-flight ISO-TP, clears all 8 CLLs, and stops CAN.
 */
void dpdu_engine_destruct_all(dpdu_vci_state_t *state, dpdu_send_pkt_fn_t send_fn, bool emit_events);

/**
 * @brief Handle an incoming Host -> VCI command on Core 0 (synchronous 0x80 ACK/NACK within < 20 ms,
 *        4-slot CLL queue admission, Overrun NACK, Out-of-Band Cancel, and IOCTLs).
 */
void dpdu_engine_handle_host_cmd(
    dpdu_vci_state_t *state,
    const dpdu_frame_t *cmd,
    dpdu_send_pkt_fn_t send_fn
);

/**
 * @brief Execute one Round-Robin arbitration step across the 8 CLLs on Core 1.
 * @return True if a ComPrimitive cycle was executed during this step, false if all CLLs were idle/waiting.
 */
bool dpdu_engine_step_round_robin(
    dpdu_vci_state_t *state,
    dpdu_send_pkt_fn_t send_fn
);

#endif /* DPDU_PROTOCOL_H */
