/**
 * @file main.c
 * @brief ESP32 OBD-II VCI — Step 4: Dual-Transport (BLE NimBLE NUS + USB-UART0),
 *        Lean D-PDU API, Multi-Frame ISO 15765-2 (FF/FC/CF), P2/P2* Timers & CAN HAL.
 *
 * Core 0 (PRO_CPU): BLE NimBLE Host/Controller + Host I/O Task (UART0 + BLE NUS) + Status LED
 * Core 1 (APP_CPU): Real-Time VCI Task (Lean D-PDU + ISO-TP + CAN HAL) + Virtual ECU Task
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "vci_config.h"
#include "led_status.h"
#include "dpdu_protocol.h"
#include "can_hal.h"
#include "ecu_sim.h"
#include "ble_nus.h"

static const char *TAG = "vci_main";

static QueueHandle_t s_q_dpdu_cmd = NULL;
static QueueHandle_t s_q_dpdu_evt = NULL;
static dpdu_vci_state_t s_vci_state;
static dpdu_rx_parser_t s_ble_parser;

/**
 * @brief Transmit a D-PDU frame over both Host USB-UART0 and BLE NUS (when connected).
 */
static void host_send_frame(const dpdu_frame_t *frame)
{
    static uint8_t tx_buf[DPDU_HEADER_SIZE + DPDU_MAX_PAYLOAD + DPDU_CRC_SIZE];
    size_t wire_len = dpdu_encode_frame(frame, tx_buf, sizeof(tx_buf));
    if (wire_len > 0) {
        uart_write_bytes(VCI_HOST_UART_NUM, (const char *)tx_buf, wire_len);
        if (ble_nus_is_connected()) {
            (void)ble_nus_send(tx_buf, wire_len);
        }
    }
}

static void vci_emit_evt(const dpdu_frame_t *evt)
{
    if (s_q_dpdu_evt != NULL && evt != NULL) {
        xQueueSend(s_q_dpdu_evt, evt, pdMS_TO_TICKS(100));
    }
}

static bool vci_check_cancel(dpdu_frame_t *out_cancel_cmd)
{
    dpdu_frame_t peek_frame;
    if (s_q_dpdu_cmd != NULL && xQueuePeek(s_q_dpdu_cmd, &peek_frame, 0) == pdTRUE) {
        if (peek_frame.opcode == PDU_CMD_CANCEL_COP) {
            xQueueReceive(s_q_dpdu_cmd, out_cancel_cmd, 0);
            s_vci_state.rx_cmd_count++;
            return true;
        }
    }
    return false;
}

/**
 * @brief Callback invoked on Core 0 when bytes arrive over BLE NUS RX Characteristic.
 */
static void on_ble_nus_rx(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        dpdu_parse_result_t res = dpdu_parser_feed_byte(&s_ble_parser, data[i]);
        if (res == DPDU_PARSE_OK) {
            xQueueSend(s_q_dpdu_cmd, &s_ble_parser.frame, pdMS_TO_TICKS(50));
        } else if (res == DPDU_PARSE_ERR_CRC) {
            s_vci_state.crc_err_count++;
            dpdu_frame_t err_evt = {
                .opcode  = PDU_EVT_ERROR,
                .handle  = s_ble_parser.frame.handle,
                .seq_tag = s_ble_parser.frame.seq_tag,
                .length  = 2U,
                .payload = { PDU_ERR_CRC_MISMATCH, s_ble_parser.frame.opcode },
            };
            vci_emit_evt(&err_evt);
        } else if (res == DPDU_PARSE_ERR_HDR) {
            dpdu_frame_t err_evt = {
                .opcode  = PDU_EVT_ERROR,
                .handle  = 0U,
                .seq_tag = 0U,
                .length  = 2U,
                .payload = { PDU_ERR_MALFORMED_FRAME, 0U },
            };
            vci_emit_evt(&err_evt);
        }
    }
}

/**
 * @brief Core 0 Task: Reads raw bytes from Host (UART0), runs the 0xA5 + CRC-16
 *        stream parser, forwards valid commands to Core 1 via s_q_dpdu_cmd, and
 *        transmits outgoing events from s_q_dpdu_evt to UART0 and BLE NUS.
 */
static void host_io_task(void *arg)
{
    (void)arg;
    static dpdu_rx_parser_t uart_parser;
    static uint8_t rx_chunk[128];
    static dpdu_frame_t evt_frame;

    dpdu_parser_init(&uart_parser);

    while (1) {
        int rx_len = uart_read_bytes(
            VCI_HOST_UART_NUM,
            rx_chunk,
            sizeof(rx_chunk),
            pdMS_TO_TICKS(5)
        );

        for (int i = 0; i < rx_len; i++) {
            dpdu_parse_result_t res = dpdu_parser_feed_byte(&uart_parser, rx_chunk[i]);
            if (res == DPDU_PARSE_OK) {
                xQueueSend(s_q_dpdu_cmd, &uart_parser.frame, pdMS_TO_TICKS(50));
            } else if (res == DPDU_PARSE_ERR_CRC) {
                s_vci_state.crc_err_count++;
                dpdu_frame_t err_evt = {
                    .opcode  = PDU_EVT_ERROR,
                    .handle  = uart_parser.frame.handle,
                    .seq_tag = uart_parser.frame.seq_tag,
                    .length  = 2U,
                    .payload = { PDU_ERR_CRC_MISMATCH, uart_parser.frame.opcode },
                };
                host_send_frame(&err_evt);
            } else if (res == DPDU_PARSE_ERR_HDR) {
                dpdu_frame_t err_evt = {
                    .opcode  = PDU_EVT_ERROR,
                    .handle  = 0U,
                    .seq_tag = 0U,
                    .length  = 2U,
                    .payload = { PDU_ERR_MALFORMED_FRAME, 0U },
                };
                host_send_frame(&err_evt);
            }
        }

        while (xQueueReceive(s_q_dpdu_evt, &evt_frame, 0) == pdTRUE) {
            host_send_frame(&evt_frame);
        }
    }
}

/**
 * @brief Core 1 Task: Real-time VCI engine processing D-PDU commands, ISO-TP, and P2/P2* timers.
 */
static void vci_rt_task(void *arg)
{
    (void)arg;
    static dpdu_frame_t cmd_frame;

    while (1) {
        if (xQueueReceive(s_q_dpdu_cmd, &cmd_frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            dpdu_engine_process_cmd(&s_vci_state, &cmd_frame, vci_emit_evt);
        }
    }
}

static void host_uart_init(void)
{
    const uart_config_t uart_config = {
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
    ESP_ERROR_CHECK(uart_param_config(VCI_HOST_UART_NUM, &uart_config));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 OBD-II VCI (Step 4: BLE NimBLE NUS + USB-UART0 + ISO-TP)...");

    dpdu_engine_init(&s_vci_state);
    dpdu_engine_set_cancel_checker(vci_check_cancel);
    dpdu_parser_init(&s_ble_parser);
    can_hal_init();
    ecu_sim_init();
    led_status_init();
    host_uart_init();

    s_q_dpdu_cmd = xQueueCreate(VCI_CMD_QUEUE_DEPTH, sizeof(dpdu_frame_t));
    s_q_dpdu_evt = xQueueCreate(VCI_EVT_QUEUE_DEPTH, sizeof(dpdu_frame_t));
    configASSERT(s_q_dpdu_cmd != NULL && s_q_dpdu_evt != NULL);

    ESP_ERROR_CHECK(ble_nus_init(on_ble_nus_rx));

    xTaskCreatePinnedToCore(
        vci_rt_task,
        "vci_rt",
        4096,
        NULL,
        VCI_PRIO_VCI_RT_TASK,
        NULL,
        VCI_CORE_REALTIME
    );

    xTaskCreatePinnedToCore(
        host_io_task,
        "host_io",
        4096,
        NULL,
        VCI_PRIO_HOST_IO_TASK,
        NULL,
        VCI_CORE_HOST_IO
    );

    ESP_LOGI(TAG, "VCI Ready on UART0 (%d baud) and BLE ('%s').", VCI_HOST_UART_BAUD, VCI_BLE_DEVICE_NAME);
}
