/**
 * @file ecu_sim.c
 * @brief Virtual OBD-II / UDS ECU Task on Core 1 for loopback simulation testing.
 */

#include "ecu_sim.h"
#include "can_hal.h"
#include "vci_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include <string.h>

#define OBD_FUNC_REQ_ID     0x7DFU
#define OBD_ECU1_REQ_ID     0x7E0U
#define OBD_ECU1_RESP_ID    0x7E8U

static void send_sf_response(const uint8_t *pdu, uint8_t pdu_len)
{
    if (pdu_len == 0U || pdu_len > 7U) {
        return;
    }
    vci_can_frame_t rsp = {
        .id      = OBD_ECU1_RESP_ID,
        .is_extd = false,
        .dlc     = 8U,
    };
    memset(rsp.data, 0xCCU, sizeof(rsp.data));
    rsp.data[0] = pdu_len & 0x0FU; /* ISO-TP Single Frame PCI (0x0N) */
    memcpy(&rsp.data[1], pdu, pdu_len);
    can_hal_sim_ecu_send(&rsp, 20);
}

static void handle_mode_01(uint8_t pid)
{
    switch (pid) {
        case 0x00U: {
            /* Supported PIDs 01..20 bitmap */
            const uint8_t rsp[] = { 0x41U, 0x00U, 0xBEU, 0x3EU, 0xB8U, 0x13U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x05U: {
            /* Coolant Temp: A - 40 => 130 - 40 = 90 °C */
            const uint8_t rsp[] = { 0x41U, 0x05U, 130U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x0CU: {
            /* Engine RPM: ((A*256)+B)/4 => 0x1AF8 / 4 = 1726.0 RPM */
            const uint8_t rsp[] = { 0x41U, 0x0CU, 0x1AU, 0xF8U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x0DU: {
            /* Vehicle Speed: A = 85 km/h */
            const uint8_t rsp[] = { 0x41U, 0x0DU, 85U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x11U: {
            /* Throttle Position: A * 100 / 255 => 64 => ~25.1% */
            const uint8_t rsp[] = { 0x41U, 0x11U, 64U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0xF0U: {
            /* Simulated slow ECU response:
             * 1. Wait 10ms, send Negative Response 0x7F 0x01 0x78 (ResponsePending)
             * 2. Wait 120ms (> 50ms P2_Max!), send Positive Response 0x41 0xF0 0xAB 0xCD
             */
            esp_rom_delay_us(10000);
            const uint8_t nrc78[] = { 0x7FU, 0x01U, 0x78U };
            send_sf_response(nrc78, sizeof(nrc78));

            vTaskDelay(pdMS_TO_TICKS(120) > 0 ? pdMS_TO_TICKS(120) : 12);
            const uint8_t pos_rsp[] = { 0x41U, 0xF0U, 0xABU, 0xCDU };
            send_sf_response(pos_rsp, sizeof(pos_rsp));
            break;
        }
        default:
            /* Unsupported PID (e.g., 0xFF): stay silent to test P2_Max timeout */
            break;
    }
}

static void ecu_sim_task(void *arg)
{
    (void)arg;
    vci_can_frame_t rx;

    while (1) {
        if (can_hal_sim_ecu_recv(&rx, 50) != ESP_OK) {
            continue;
        }

        /* Only respond to OBD-II functional broadcast (0x7DF) or Engine ECU #1 (0x7E0) */
        if (rx.is_extd || (rx.id != OBD_FUNC_REQ_ID && rx.id != OBD_ECU1_REQ_ID) || rx.dlc < 2U) {
            continue;
        }

        uint8_t pci_type = (rx.data[0] >> 4) & 0x0FU;
        uint8_t sf_len   = rx.data[0] & 0x0FU;

        if (pci_type == 0x00U && sf_len >= 1U && sf_len <= 7U) {
            uint8_t mode = rx.data[1];
            if (mode == 0x01U && sf_len >= 2U) {
                esp_rom_delay_us(1500); /* Realistic 1.5ms ECU turnaround */
                handle_mode_01(rx.data[2]);
            }
        }
    }
}

void ecu_sim_init(void)
{
    xTaskCreatePinnedToCore(
        ecu_sim_task,
        "ecu_sim",
        4096,
        NULL,
        VCI_PRIO_ECU_SIM_TASK,
        NULL,
        VCI_CORE_REALTIME
    );
}

