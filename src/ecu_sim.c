/**
 * @file ecu_sim.c
 * @brief Virtual OBD-II / UDS ECU Task on Core 1 supporting 11-bit & 29-bit CAN,
 *        Single-Frame, and Multi-Frame ISO-TP.
 */

#include "ecu_sim.h"
#include "can_hal.h"
#include "vci_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <string.h>
#include <stdbool.h>

/* 11-bit Standard OBD-II IDs */
#define OBD_FUNC_REQ_ID_11      0x7DFU
#define OBD_ECU1_REQ_ID_11      0x7E0U
#define OBD_ECU1_RESP_ID_11     0x7E8U

/* 29-bit Extended OBD-II IDs (ISO 15765-4: Tester=0xF1, Engine ECU=0x10) */
#define OBD_FUNC_REQ_ID_29      0x18DB33F1U
#define OBD_ECU1_REQ_ID_29      0x18DA10F1U
#define OBD_ECU1_RESP_ID_29     0x18DAF110U

#define ECU_PAD_BYTE            0xCCU

/* Active response CAN ID & addressing mode for the current request */
static uint32_t s_active_resp_id    = OBD_ECU1_RESP_ID_11;
static bool     s_active_is_extd    = false;
static uint8_t  s_last_rx_pad_byte  = 0xCCU;

/* Simulated 17-character VIN (writable via UDS 0x2E F190, readable via OBD Mode 09 PID 02) */
static uint8_t s_ecu_vin[18] = "1FAFP40411F192837";

/* Simulated Stored DTCs (SAE J1979 2-byte encoding) */
static const uint8_t s_default_dtcs[] = {
    0x01U, 0x71U, /* P0171 */
    0x03U, 0x00U, /* P0300 */
    0x40U, 0x35U, /* C0035 */
    0xC1U, 0x00U, /* U0100 */
};
static bool s_dtcs_cleared = false;

static bool is_valid_tester_req_id(uint32_t id, bool is_extd)
{
    if (!is_extd) {
        return (id == OBD_FUNC_REQ_ID_11 || id == OBD_ECU1_REQ_ID_11);
    }
    return (id == OBD_FUNC_REQ_ID_29 || id == OBD_ECU1_REQ_ID_29);
}

static void send_sf_response(const uint8_t *pdu, uint8_t pdu_len)
{
    if (pdu_len == 0U || pdu_len > 7U) {
        return;
    }
    vci_can_frame_t rsp = {
        .id      = s_active_resp_id,
        .is_extd = s_active_is_extd,
        .dlc     = 8U,
    };
    memset(rsp.data, ECU_PAD_BYTE, sizeof(rsp.data));
    rsp.data[0] = pdu_len & 0x0FU; /* ISO-TP Single Frame PCI (0x0N) */
    memcpy(&rsp.data[1], pdu, pdu_len);
    can_hal_sim_ecu_send(&rsp, 20);
}

static bool wait_for_tester_fc(uint8_t *out_bs, uint8_t *out_stmin, uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
    vci_can_frame_t rx;

    while (esp_timer_get_time() < deadline_us) {
        if (can_hal_sim_ecu_recv(&rx, 5) == ESP_OK) {
            if (!is_valid_tester_req_id(rx.id, rx.is_extd) || rx.dlc < 3U) {
                continue;
            }
            uint8_t pci_type = (rx.data[0] >> 4) & 0x0FU;
            uint8_t fs       = rx.data[0] & 0x0FU;
            if (pci_type == 0x03U && fs == 0x00U) { /* Flow Control CTS */
                *out_bs    = rx.data[1];
                *out_stmin = rx.data[2];
                s_last_rx_pad_byte = rx.data[7];
                return true;
            }
        }
    }
    return false;
}

static void send_isotp_response(const uint8_t *pdu, uint16_t pdu_len)
{
    if (pdu == NULL || pdu_len == 0U) {
        return;
    }
    if (pdu_len <= 7U) {
        send_sf_response(pdu, (uint8_t)pdu_len);
        return;
    }

    /* Multi-Frame Response: 1. Transmit First Frame (FF: 0x1N NN + 6 bytes) */
    vci_can_frame_t ff = {
        .id      = s_active_resp_id,
        .is_extd = s_active_is_extd,
        .dlc     = 8U,
    };
    ff.data[0] = (uint8_t)(0x10U | ((pdu_len >> 8) & 0x0FU));
    ff.data[1] = (uint8_t)(pdu_len & 0xFFU);
    memcpy(&ff.data[2], pdu, 6U);
    if (can_hal_sim_ecu_send(&ff, 20) != ESP_OK) {
        return;
    }

    uint16_t offset  = 6U;
    uint8_t  sn      = 1U;
    uint8_t  bs      = 0U;
    uint8_t  stmin   = 0U;
    uint8_t  bs_rem  = 0U;
    bool     need_fc = true;

    while (offset < pdu_len) {
        if (need_fc) {
            if (!wait_for_tester_fc(&bs, &stmin, 1000U)) {
                return; /* N_Bs timeout */
            }
            bs_rem  = bs;
            need_fc = false;
        }

        if (stmin > 0U && stmin <= 0x7FU) {
            esp_rom_delay_us((uint32_t)stmin * 1000U);
        } else {
            esp_rom_delay_us(500); /* 0.5 ms inter-frame spacing */
        }

        vci_can_frame_t cf = {
            .id      = s_active_resp_id,
            .is_extd = s_active_is_extd,
            .dlc     = 8U,
        };
        memset(cf.data, ECU_PAD_BYTE, sizeof(cf.data));
        cf.data[0] = (uint8_t)(0x20U | (sn & 0x0FU));

        uint16_t chunk = pdu_len - offset;
        if (chunk > 7U) {
            chunk = 7U;
        }
        memcpy(&cf.data[1], &pdu[offset], chunk);
        if (can_hal_sim_ecu_send(&cf, 20) != ESP_OK) {
            return;
        }

        offset += chunk;
        sn = (sn + 1U) & 0x0FU;

        if (bs > 0U && offset < pdu_len) {
            bs_rem--;
            if (bs_rem == 0U) {
                need_fc = true;
            }
        }
    }
}

static void handle_mode_01(uint8_t pid)
{
    switch (pid) {
        case 0x00U: {
            const uint8_t rsp[] = { 0x41U, 0x00U, 0xBEU, 0x3EU, 0xB8U, 0x13U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x05U: {
            const uint8_t rsp[] = { 0x41U, 0x05U, 130U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x0CU: {
            const uint8_t rsp[] = { 0x41U, 0x0CU, 0x1AU, 0xF8U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x0DU: {
            const uint8_t rsp[] = { 0x41U, 0x0DU, 85U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x11U: {
            const uint8_t rsp[] = { 0x41U, 0x11U, 64U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0xF0U: {
            esp_rom_delay_us(10000);
            const uint8_t nrc78[] = { 0x7FU, 0x01U, 0x78U };
            send_sf_response(nrc78, sizeof(nrc78));

            vTaskDelay(pdMS_TO_TICKS(120) > 0 ? pdMS_TO_TICKS(120) : 12);
            const uint8_t pos_rsp[] = { 0x41U, 0xF0U, 0xABU, 0xCDU };
            send_sf_response(pos_rsp, sizeof(pos_rsp));
            break;
        }
        case 0xF3U: {
            /* Echo the tester's CAN filler byte (rx.data[7]) to verify CP_CAN_FILLER_BYTE */
            const uint8_t rsp[] = { 0x41U, 0xF3U, s_last_rx_pad_byte };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0xF4U: {
            /* Long-running P2* wait (for testing in-flight PDU_CMD_CANCEL_COP):
             * Sends NRC 0x78 (ResponsePending) and stays quiet so host can cancel during P2* window.
             */
            esp_rom_delay_us(5000);
            const uint8_t nrc78[] = { 0x7FU, 0x01U, 0x78U };
            send_sf_response(nrc78, sizeof(nrc78));
            break;
        }
        default:
            break;
    }
}

static void handle_mode_03(void)
{
    if (s_dtcs_cleared) {
        const uint8_t rsp[] = { 0x43U, 0x00U };
        send_sf_response(rsp, sizeof(rsp));
        return;
    }

    /* 4 stored DTCs => 2 + 8 = 10 bytes (Multi-Frame ISO-TP) */
    uint8_t rsp[2U + sizeof(s_default_dtcs)];
    rsp[0] = 0x43U;
    rsp[1] = (uint8_t)(sizeof(s_default_dtcs) / 2U);
    memcpy(&rsp[2], s_default_dtcs, sizeof(s_default_dtcs));
    send_isotp_response(rsp, sizeof(rsp));
}

static void handle_mode_04(void)
{
    s_dtcs_cleared = true;
    const uint8_t rsp[] = { 0x44U };
    send_sf_response(rsp, sizeof(rsp));
}

static void handle_mode_09(uint8_t pid)
{
    switch (pid) {
        case 0x00U: {
            const uint8_t rsp[] = { 0x49U, 0x00U, 0x54U, 0x00U, 0x00U, 0x00U };
            send_sf_response(rsp, sizeof(rsp));
            break;
        }
        case 0x02U: {
            uint8_t rsp[20];
            rsp[0] = 0x49U;
            rsp[1] = 0x02U;
            rsp[2] = 0x01U;
            memcpy(&rsp[3], s_ecu_vin, 17U);
            send_isotp_response(rsp, sizeof(rsp));
            break;
        }
        case 0x04U: {
            static const char cal_id[17] = "ESP32_VCI_CAL_01";
            uint8_t rsp[19];
            rsp[0] = 0x49U;
            rsp[1] = 0x04U;
            rsp[2] = 0x01U;
            memcpy(&rsp[3], cal_id, 16U);
            send_isotp_response(rsp, sizeof(rsp));
            break;
        }
        case 0xF1U: {
            /* Simulated broken Multi-Frame: send FF, wait for FC, then drop CF #1 to test CP_N_CR_MS timeout */
            vci_can_frame_t ff = {
                .id      = s_active_resp_id,
                .is_extd = s_active_is_extd,
                .dlc     = 8U,
                .data    = { 0x10U, 0x14U, 0x49U, 0xF1U, 0x01U, 'B', 'A', 'D' },
            };
            (void)can_hal_sim_ecu_send(&ff, 20);
            uint8_t bs = 0, stmin = 0;
            (void)wait_for_tester_fc(&bs, &stmin, 200U);
            break;
        }
        case 0xF2U: {
            /* Simulated sequence error: send FF, wait for FC, then send CF with wrong SN=5 instead of 1 */
            vci_can_frame_t ff = {
                .id      = s_active_resp_id,
                .is_extd = s_active_is_extd,
                .dlc     = 8U,
                .data    = { 0x10U, 0x14U, 0x49U, 0xF2U, 0x01U, 'S', 'E', 'Q' },
            };
            (void)can_hal_sim_ecu_send(&ff, 20);
            uint8_t bs = 0, stmin = 0;
            if (wait_for_tester_fc(&bs, &stmin, 200U)) {
                esp_rom_delay_us(500);
                vci_can_frame_t bad_cf = {
                    .id      = s_active_resp_id,
                    .is_extd = s_active_is_extd,
                    .dlc     = 8U,
                    .data    = { 0x25U, 'E', 'R', 'R', 'O', 'R', '!', 0xCCU },
                };
                (void)can_hal_sim_ecu_send(&bad_cf, 20);
            }
            break;
        }
        default:
            break;
    }
}

static void handle_uds_2e_write_did(const uint8_t *pdu, uint16_t pdu_len)
{
    if (pdu_len == 20U && pdu[1] == 0xF1U && pdu[2] == 0x90U) {
        memcpy(s_ecu_vin, &pdu[3], 17U);
        const uint8_t pos_rsp[] = { 0x6EU, 0xF1U, 0x90U };
        send_sf_response(pos_rsp, sizeof(pos_rsp));
    } else {
        const uint8_t nrc[] = { 0x7FU, 0x2EU, 0x31U }; /* requestOutOfRange */
        send_sf_response(nrc, sizeof(nrc));
    }
}

static void dispatch_diagnostic_pdu(const uint8_t *pdu, uint16_t pdu_len)
{
    if (pdu_len == 0U) {
        return;
    }
    esp_rom_delay_us(1500); /* Realistic 1.5ms ECU turnaround */

    uint8_t sid = pdu[0];
    switch (sid) {
        case 0x01U:
            if (pdu_len >= 2U) {
                handle_mode_01(pdu[1]);
            }
            break;
        case 0x03U:
            handle_mode_03();
            break;
        case 0x04U:
            handle_mode_04();
            break;
        case 0x09U:
            if (pdu_len >= 2U) {
                handle_mode_09(pdu[1]);
            }
            break;
        case 0x2EU:
            handle_uds_2e_write_did(pdu, pdu_len);
            break;
        default:
            break;
    }
}

static void ecu_sim_task(void *arg)
{
    (void)arg;
    vci_can_frame_t rx;

    static uint8_t  rx_buf[512];
    static uint16_t rx_total_len = 0;
    static uint16_t rx_offset    = 0;
    static uint8_t  rx_next_sn   = 1;
    static bool     rx_mf_active = false;

    while (1) {
        if (can_hal_sim_ecu_recv(&rx, 50) != ESP_OK) {
            continue;
        }

        if (!is_valid_tester_req_id(rx.id, rx.is_extd) || rx.dlc < 2U) {
            continue;
        }

        s_active_is_extd   = rx.is_extd;
        s_active_resp_id   = rx.is_extd ? OBD_ECU1_RESP_ID_29 : OBD_ECU1_RESP_ID_11;
        s_last_rx_pad_byte = rx.data[7];

        uint8_t pci_type = (rx.data[0] >> 4) & 0x0FU;

        if (pci_type == 0x00U) {
            uint8_t sf_len = rx.data[0] & 0x0FU;
            rx_mf_active = false;
            if (sf_len >= 1U && sf_len <= 7U && (1U + sf_len) <= rx.dlc) {
                dispatch_diagnostic_pdu(&rx.data[1], sf_len);
            }
        } else if (pci_type == 0x01U && rx.dlc == 8U) {
            rx_total_len = (uint16_t)(((uint16_t)(rx.data[0] & 0x0FU) << 8) | rx.data[1]);
            if (rx_total_len >= 8U && rx_total_len <= sizeof(rx_buf)) {
                memcpy(rx_buf, &rx.data[2], 6U);
                rx_offset    = 6U;
                rx_next_sn   = 1U;
                rx_mf_active = true;

                esp_rom_delay_us(500);
                vci_can_frame_t fc = {
                    .id      = s_active_resp_id,
                    .is_extd = s_active_is_extd,
                    .dlc     = 8U,
                    .data    = { 0x30U, 0x00U, 0x00U, ECU_PAD_BYTE, ECU_PAD_BYTE, ECU_PAD_BYTE, ECU_PAD_BYTE, ECU_PAD_BYTE },
                };
                (void)can_hal_sim_ecu_send(&fc, 20);
            }
        } else if (pci_type == 0x02U && rx_mf_active) {
            uint8_t sn = rx.data[0] & 0x0FU;
            if (sn != rx_next_sn) {
                rx_mf_active = false;
                continue;
            }
            uint16_t rem = rx_total_len - rx_offset;
            uint16_t chunk = (rem > 7U) ? 7U : rem;
            memcpy(&rx_buf[rx_offset], &rx.data[1], chunk);
            rx_offset += chunk;
            rx_next_sn = (rx_next_sn + 1U) & 0x0FU;

            if (rx_offset >= rx_total_len) {
                rx_mf_active = false;
                dispatch_diagnostic_pdu(rx_buf, rx_total_len);
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
