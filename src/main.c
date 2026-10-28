/**
 * @file main.c
 * @brief ESP32 OBD-II VCI Entry Point — Dual-Core Architecture for ISO 22900-2:2022 App-Device IF:
 *        - Core 0 (PRO_CPU): BLE GATT NUS & UART0 Host I/O, 0xAA55 Stream Parser, Synchronous 0x80 ACK/NACK
 *        - Core 1 (APP_CPU): 8-CLL Round-Robin D-PDU Scheduler, 3x 4,095B ISO-TP Engine, CAN TWAI & Virtual ECU
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "vci_config.h"
#include "led_status.h"
#include "dpdu_protocol.h"
#include "can_hal.h"
#include "ecu_sim.h"
#include "ble_nus.h"

/* Shared VCI Engine State (8 CLLs, 4-slot queues, ComParams) */
static dpdu_vci_state_t  s_vci_state;

/* Mutex serializing outgoing VCI -> Host wire packets from Core 0 (0x80 ACK) and Core 1 (0x81/0x82/0x83) */
static SemaphoreHandle_t s_host_tx_mutex       = NULL;
static TaskHandle_t      s_vci_rt_task_hdl     = NULL;

/* Separate 0xAA55 stream parsers for BLE NUS and USB-UART0 (~96 bytes each in .bss) */
static dpdu_rx_parser_t  s_ble_parser;
static dpdu_rx_parser_t  s_uart_parser;

/* Flag set on BLE GATT disconnect to trigger autonomous PDUDestruct within < 100 ms */
static volatile bool     s_ble_destruct_pending = false;

static void host_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = VCI_HOST_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        VCI_HOST_UART_NUM,
        VCI_HOST_UART_RX_BUF,
        VCI_HOST_UART_TX_BUF,
        0,
        NULL,
        0
    ));
    ESP_ERROR_CHECK(uart_param_config(VCI_HOST_UART_NUM, &uart_cfg));
}

/**
 * @brief Thread-safe VCI -> Host packet transmitter (0x80 ACK/NACK, 0x81 RESULT, 0x82 STATUS, 0x83 ERROR).
 */
static void host_send_packet(
    uint8_t msg_type,
    uint8_t cll_id,
    uint8_t cop_id,
    uint8_t status,
    const uint8_t *payload,
    uint16_t payload_len
)
{
    static uint8_t small_wire[DPDU_HEADER_SIZE + 256U + DPDU_CRC_SIZE];

    if (payload_len > DPDU_MAX_PAYLOAD) {
        return;
    }

    if (s_host_tx_mutex != NULL) {
        xSemaphoreTake(s_host_tx_mutex, portMAX_DELAY);
    }

    if (payload_len <= 256U) {
        size_t wire_len = dpdu_encode_packet(
            msg_type,
            cll_id,
            cop_id,
            status,
            payload,
            payload_len,
            small_wire,
            sizeof(small_wire)
        );
        if (wire_len > 0U) {
            uart_write_bytes(VCI_HOST_UART_NUM, (const char *)small_wire, wire_len);
            if (ble_nus_is_connected()) {
                (void)ble_nus_send(small_wire, wire_len);
            }
        }
    } else if (payload != NULL) {
        /* Stream large packet (> 256B payload) without a 4.1 KB contiguous wire buffer */
        uint8_t hdr[DPDU_HEADER_SIZE];
        hdr[0] = DPDU_SOF_BYTE_0;
        hdr[1] = DPDU_SOF_BYTE_1;
        hdr[2] = msg_type;
        hdr[3] = cll_id;
        hdr[4] = cop_id;
        hdr[5] = status;
        hdr[6] = (uint8_t)(payload_len & 0xFFU);
        hdr[7] = (uint8_t)((payload_len >> 8) & 0xFFU);

        /* Compute CRC16-CCITT over Msg_Type..Payload */
        uint16_t crc = 0xFFFFU;
        for (size_t i = 2U; i < DPDU_HEADER_SIZE; i++) {
            crc ^= (uint16_t)hdr[i] << 8;
            for (int b = 0; b < 8; b++) {
                crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
            }
        }
        for (size_t i = 0U; i < payload_len; i++) {
            crc ^= (uint16_t)payload[i] << 8;
            for (int b = 0; b < 8; b++) {
                crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
            }
        }
        uint8_t crc_bytes[2] = {
            (uint8_t)(crc & 0xFFU),
            (uint8_t)((crc >> 8) & 0xFFU)
        };

        uart_write_bytes(VCI_HOST_UART_NUM, (const char *)hdr, sizeof(hdr));
        uart_write_bytes(VCI_HOST_UART_NUM, (const char *)payload, payload_len);
        uart_write_bytes(VCI_HOST_UART_NUM, (const char *)crc_bytes, sizeof(crc_bytes));

        if (ble_nus_is_connected()) {
            (void)ble_nus_send(hdr, sizeof(hdr));
            (void)ble_nus_send(payload, payload_len);
            (void)ble_nus_send(crc_bytes, sizeof(crc_bytes));
        }
    }

    if (s_host_tx_mutex != NULL) {
        xSemaphoreGive(s_host_tx_mutex);
    }
}

static void emit_rx_framing_error(uint8_t cll_id, uint8_t cop_id, uint16_t vendor_subcode)
{
    uint8_t payload[8];
    uint32_t ts = dpdu_timestamp_us();
    payload[0] = (uint8_t)(ts & 0xFFU);
    payload[1] = (uint8_t)((ts >> 8) & 0xFFU);
    payload[2] = (uint8_t)((ts >> 16) & 0xFFU);
    payload[3] = (uint8_t)((ts >> 24) & 0xFFU);
    payload[4] = (uint8_t)(PDU_ERR_EVT_RX_ERROR & 0xFFU);
    payload[5] = (uint8_t)((PDU_ERR_EVT_RX_ERROR >> 8) & 0xFFU);
    payload[6] = (uint8_t)(vendor_subcode & 0xFFU);
    payload[7] = (uint8_t)((vendor_subcode >> 8) & 0xFFU);
    host_send_packet(PDU_IT_ERROR, cll_id, cop_id, 0U, payload, sizeof(payload));
}

/**
 * @brief Callback invoked on Core 0 when BLE GATT connection drops (Requirement 5: PDUDestruct < 100 ms).
 */
static void on_ble_nus_disconnect(void)
{
    s_ble_destruct_pending = true;
    /* Mark all active ComPrimitives as cancelled immediately so Core 1 aborts within <= 5 ms */
    for (uint8_t c = 0U; c < VCI_MAX_CLLS; c++) {
        for (uint8_t s = 0U; s < VCI_CLL_COP_QUEUE_DEPTH; s++) {
            s_vci_state.clls[c].cop_queue[s].cancel_requested = true;
        }
    }
    if (s_vci_rt_task_hdl != NULL) {
        xTaskNotifyGive(s_vci_rt_task_hdl);
    }
}

/**
 * @brief Callback invoked on Core 0 when bytes arrive over BLE NUS RX Characteristic.
 */
static void on_ble_nus_rx(const uint8_t *data, size_t len)
{
    for (size_t i = 0U; i < len; i++) {
        dpdu_parse_result_t res = dpdu_parser_feed_byte(&s_ble_parser, data[i]);
        if (res == DPDU_PARSE_OK) {
            dpdu_engine_handle_host_cmd(&s_vci_state, &s_ble_parser.frame, host_send_packet);
            if (s_vci_rt_task_hdl != NULL) {
                xTaskNotifyGive(s_vci_rt_task_hdl);
            }
        } else if (res == DPDU_PARSE_ERR_CRC) {
            s_vci_state.crc_err_count++;
            emit_rx_framing_error(s_ble_parser.frame.cll_id, s_ble_parser.frame.cop_id, PDU_ERR_CRC_MISMATCH);
        } else if (res == DPDU_PARSE_ERR_HDR) {
            emit_rx_framing_error(0U, 0U, PDU_ERR_MALFORMED_FRAME);
        }
    }
}

/**
 * @brief Core 0 Task: Reads raw bytes from Host (UART0), runs the 0xAA55 + CRC-16
 *        stream parser, and dispatches synchronous ACK/NACK & queue admission.
 */
static void host_io_task(void *arg)
{
    (void)arg;
    static uint8_t rx_chunk[256];

    dpdu_parser_init(&s_uart_parser);

    while (1) {
        int rx_len = uart_read_bytes(
            VCI_HOST_UART_NUM,
            rx_chunk,
            sizeof(rx_chunk),
            pdMS_TO_TICKS(5)
        );

        for (int i = 0; i < rx_len; i++) {
            dpdu_parse_result_t res = dpdu_parser_feed_byte(&s_uart_parser, rx_chunk[i]);
            if (res == DPDU_PARSE_OK) {
                dpdu_engine_handle_host_cmd(&s_vci_state, &s_uart_parser.frame, host_send_packet);
                if (s_vci_rt_task_hdl != NULL) {
                    xTaskNotifyGive(s_vci_rt_task_hdl);
                }
            } else if (res == DPDU_PARSE_ERR_CRC) {
                s_vci_state.crc_err_count++;
                emit_rx_framing_error(s_uart_parser.frame.cll_id, s_uart_parser.frame.cop_id, PDU_ERR_CRC_MISMATCH);
            } else if (res == DPDU_PARSE_ERR_HDR) {
                emit_rx_framing_error(0U, 0U, PDU_ERR_MALFORMED_FRAME);
            }
        }
    }
}

/**
 * @brief Core 1 Task: Real-time VCI engine executing 8-CLL Round-Robin arbitration,
 *        ISO 15765-2 segmentation/reassembly, cyclic primitives, and P2/P2_Star/P3 timers.
 */
static void vci_rt_task(void *arg)
{
    (void)arg;

    while (1) {
        if (s_ble_destruct_pending) {
            s_ble_destruct_pending = false;
            dpdu_engine_destruct_all(&s_vci_state, host_send_packet, false);
        }

        bool did_work = dpdu_engine_step_round_robin(&s_vci_state, host_send_packet);
        if (!did_work) {
            /* Sleep up to 1 tick (10 ms) or until woken immediately by xTaskNotifyGive on Core 0 */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        }
    }
}

void app_main(void)
{
    /* Silence asynchronous ESP_LOG output on UART0 so binary 0xAA55 frames are never corrupted */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* 1. Initialize Status LED (Core 0) */
    led_status_init();

    /* 2. Initialize TX Mutex, CAN HAL, Virtual ECU (Core 1), and 8-CLL D-PDU Engine */
    s_host_tx_mutex = xSemaphoreCreateMutex();
    can_hal_init();
    ecu_sim_init();
    dpdu_engine_init(&s_vci_state);
    dpdu_parser_init(&s_ble_parser);

    /* 3. Launch Real-Time VCI Engine Task on Core 1 (APP_CPU) */
    BaseType_t rt_rc = xTaskCreatePinnedToCore(
        vci_rt_task,
        "vci_rt",
        4096,
        NULL,
        VCI_PRIO_VCI_RT_TASK,
        &s_vci_rt_task_hdl,
        VCI_CORE_REALTIME
    );
    ESP_ERROR_CHECK(rt_rc == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    /* 4. Initialize Host Transports: UART0 (Wired) & NimBLE GATT NUS (Wireless, Core 0) */
    host_uart_init();
    ble_nus_set_disconnect_cb(on_ble_nus_disconnect);
    ESP_ERROR_CHECK(ble_nus_init(on_ble_nus_rx));

    /* 5. Launch Host I/O Task on Core 0 (PRO_CPU) */
    BaseType_t io_rc = xTaskCreatePinnedToCore(
        host_io_task,
        "host_io",
        4096,
        NULL,
        VCI_PRIO_HOST_IO_TASK,
        NULL,
        VCI_CORE_HOST_IO
    );
    ESP_ERROR_CHECK(io_rc == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
