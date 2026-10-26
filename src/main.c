/**
 * @file main.c
 * @brief ESP32 OBD-II VCI — Step 1: Dual-Core FreeRTOS & Binary Lean D-PDU Engine.
 *
 * Core 0 (PRO_CPU): Host I/O Task (UART0 / BLE binary frame parser & transmitter) + Status LED
 * Core 1 (APP_CPU): Real-Time VCI Task (Lean D-PDU Command & ComParam Engine)
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

static const char *TAG = "vci_main";

static QueueHandle_t s_q_dpdu_cmd = NULL;
static QueueHandle_t s_q_dpdu_evt = NULL;
static dpdu_vci_state_t s_vci_state;

/**
 * @brief Transmit a D-PDU frame over the host UART interface.
 */
static void host_uart_send_frame(const dpdu_frame_t *frame)
{
    static uint8_t tx_buf[DPDU_HEADER_SIZE + DPDU_MAX_PAYLOAD + DPDU_CRC_SIZE];
    size_t wire_len = dpdu_encode_frame(frame, tx_buf, sizeof(tx_buf));
    if (wire_len > 0) {
        uart_write_bytes(VCI_HOST_UART_NUM, (const char *)tx_buf, wire_len);
    }
}

/**
 * @brief Core 0 Task: Reads raw bytes from Host (UART0), runs the 0xA5 + CRC-16
 *        stream parser, forwards valid commands to Core 1 via s_q_dpdu_cmd, and
 *        transmits outgoing events from s_q_dpdu_evt.
 */
static void host_io_task(void *arg)
{
    (void)arg;
    static dpdu_rx_parser_t parser;
    static uint8_t rx_chunk[128];
    static dpdu_frame_t evt_frame;

    dpdu_parser_init(&parser);

    while (1) {
        /* 1. Read incoming bytes from Host UART (up to 10ms wait) */
        int rx_len = uart_read_bytes(
            VCI_HOST_UART_NUM,
            rx_chunk,
            sizeof(rx_chunk),
            pdMS_TO_TICKS(10)
        );

        for (int i = 0; i < rx_len; i++) {
            dpdu_parse_result_t res = dpdu_parser_feed_byte(&parser, rx_chunk[i]);
            if (res == DPDU_PARSE_OK) {
                xQueueSend(s_q_dpdu_cmd, &parser.frame, pdMS_TO_TICKS(50));
            } else if (res == DPDU_PARSE_ERR_CRC) {
                s_vci_state.crc_err_count++;
                dpdu_frame_t err_evt = {
                    .opcode  = PDU_EVT_ERROR,
                    .handle  = parser.frame.handle,
                    .seq_tag = parser.frame.seq_tag,
                    .length  = 2U,
                    .payload = { PDU_ERR_CRC_MISMATCH, parser.frame.opcode },
                };
                host_uart_send_frame(&err_evt);
            } else if (res == DPDU_PARSE_ERR_HDR) {
                dpdu_frame_t err_evt = {
                    .opcode  = PDU_EVT_ERROR,
                    .handle  = 0U,
                    .seq_tag = 0U,
                    .length  = 2U,
                    .payload = { PDU_ERR_MALFORMED_FRAME, 0U },
                };
                host_uart_send_frame(&err_evt);
            }
        }

        /* 2. Drain any outgoing VCI events queued by Core 1 */
        while (xQueueReceive(s_q_dpdu_evt, &evt_frame, 0) == pdTRUE) {
            host_uart_send_frame(&evt_frame);
        }
    }
}

/**
 * @brief Core 1 Task: Real-time VCI engine processing D-PDU commands and ComParams.
 */
static void vci_rt_task(void *arg)
{
    (void)arg;
    static dpdu_frame_t cmd_frame;
    static dpdu_frame_t rsp_frame;

    while (1) {
        if (xQueueReceive(s_q_dpdu_cmd, &cmd_frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            memset(&rsp_frame, 0, sizeof(rsp_frame));
            dpdu_engine_process_cmd(&s_vci_state, &cmd_frame, &rsp_frame);
            xQueueSend(s_q_dpdu_evt, &rsp_frame, pdMS_TO_TICKS(50));
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
    ESP_LOGI(TAG, "Starting ESP32 OBD-II VCI (Step 1: Lean D-PDU Core)...");

    dpdu_engine_init(&s_vci_state);
    led_status_init();
    host_uart_init();

    s_q_dpdu_cmd = xQueueCreate(VCI_CMD_QUEUE_DEPTH, sizeof(dpdu_frame_t));
    s_q_dpdu_evt = xQueueCreate(VCI_EVT_QUEUE_DEPTH, sizeof(dpdu_frame_t));
    configASSERT(s_q_dpdu_cmd != NULL && s_q_dpdu_evt != NULL);

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

    ESP_LOGI(TAG, "VCI Ready on UART0 (%d baud) — Waiting for binary 0xA5 D-PDU frames.", VCI_HOST_UART_BAUD);
}
