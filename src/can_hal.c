/**
 * @file can_hal.c
 * @brief Layer 4: CAN Hardware Abstraction Layer (ESP-IDF TWAI + Virtual ECU Loopback).
 */

#include "can_hal.h"
#include "vci_config.h"
#include "dpdu_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "driver/twai.h"
#pragma GCC diagnostic pop
#include <string.h>

static QueueHandle_t s_q_tester_to_ecu = NULL;
static QueueHandle_t s_q_ecu_to_tester = NULL;
static bool          s_started         = false;
static uint8_t       s_bus_mode        = VCI_BUS_MODE_SIM_LOOPBACK;
static bool          s_twai_installed  = false;

void can_hal_init(void)
{
    if (s_q_tester_to_ecu == NULL) {
        s_q_tester_to_ecu = xQueueCreate(16, sizeof(vci_can_frame_t));
    }
    if (s_q_ecu_to_tester == NULL) {
        s_q_ecu_to_tester = xQueueCreate(16, sizeof(vci_can_frame_t));
    }
}

esp_err_t can_hal_start(uint8_t bus_mode, uint16_t baud_khz)
{
    can_hal_stop();

    if (bus_mode == VCI_BUS_MODE_SIM_LOOPBACK) {
        xQueueReset(s_q_tester_to_ecu);
        xQueueReset(s_q_ecu_to_tester);
        s_bus_mode = VCI_BUS_MODE_SIM_LOOPBACK;
        s_started  = true;
        return ESP_OK;
    }

    if (bus_mode == VCI_BUS_MODE_PHYSICAL_CAN) {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            VCI_CAN_TX_GPIO,
            VCI_CAN_RX_GPIO,
            TWAI_MODE_NORMAL
        );
        g_config.rx_queue_len = 32;
        g_config.tx_queue_len = 16;

        twai_timing_config_t t_config;
        if (baud_khz == 125U) {
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        } else if (baud_khz == 250U) {
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        } else if (baud_khz == 1000U) {
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        } else {
            t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        }

        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
        if (err != ESP_OK) {
            return err;
        }
        s_twai_installed = true;

        err = twai_start();
        if (err != ESP_OK) {
            twai_driver_uninstall();
            s_twai_installed = false;
            return err;
        }

        s_bus_mode = VCI_BUS_MODE_PHYSICAL_CAN;
        s_started  = true;
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

void can_hal_stop(void)
{
    if (s_twai_installed) {
        twai_stop();
        twai_driver_uninstall();
        s_twai_installed = false;
    }
    if (s_q_tester_to_ecu != NULL) {
        xQueueReset(s_q_tester_to_ecu);
    }
    if (s_q_ecu_to_tester != NULL) {
        xQueueReset(s_q_ecu_to_tester);
    }
    s_started = false;
}

static TickType_t ms_to_ticks(uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return 0;
    }
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    return (ticks > 0) ? ticks : 1;
}

esp_err_t can_hal_send(const vci_can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_started || frame == NULL || frame->dlc > 8U) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = ms_to_ticks(timeout_ms);

    if (s_bus_mode == VCI_BUS_MODE_SIM_LOOPBACK) {
        if (xQueueSend(s_q_tester_to_ecu, frame, ticks) == pdTRUE) {
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }

    twai_message_t msg = {0};
    msg.identifier       = frame->id;
    msg.extd             = frame->is_extd ? 1 : 0;
    msg.rtr              = 0;
    msg.data_length_code = frame->dlc;
    memcpy(msg.data, frame->data, frame->dlc);

    return twai_transmit(&msg, ticks);
}

esp_err_t can_hal_recv(vci_can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_started || frame == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = ms_to_ticks(timeout_ms);

    if (s_bus_mode == VCI_BUS_MODE_SIM_LOOPBACK) {
        if (xQueueReceive(s_q_ecu_to_tester, frame, ticks) == pdTRUE) {
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }

    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, ticks);
    if (err != ESP_OK) {
        return err;
    }

    frame->id      = msg.identifier;
    frame->is_extd = msg.extd ? true : false;
    frame->dlc     = (msg.data_length_code <= 8U) ? msg.data_length_code : 8U;
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, msg.data, frame->dlc);
    return ESP_OK;
}

esp_err_t can_hal_sim_ecu_recv(vci_can_frame_t *frame, uint32_t timeout_ms)
{
    if (s_q_tester_to_ecu == NULL || frame == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueReceive(s_q_tester_to_ecu, frame, ms_to_ticks(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t can_hal_sim_ecu_send(const vci_can_frame_t *frame, uint32_t timeout_ms)
{
    if (s_q_ecu_to_tester == NULL || frame == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_q_ecu_to_tester, frame, ms_to_ticks(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}
