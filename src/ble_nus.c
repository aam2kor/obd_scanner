/**
 * @file ble_nus.c
 * @brief Layer 1: Bluetooth LE Nordic UART Service (NUS) Transport via ESP-IDF NimBLE (Core 0).
 */

#include "ble_nus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/dis/ble_svc_dis.h"
#include "esp_mac.h"
#include "vci_config.h"
#include <string.h>

void ble_store_config_init(void);
extern int esp_rom_printf(const char *fmt, ...);

static const char *TAG = "ble_nus";

static ble_nus_rx_cb_t         s_rx_cb              = NULL;
static ble_nus_disconnect_cb_t s_disconnect_cb      = NULL;
static uint16_t                s_conn_hdl           = BLE_HS_CONN_HANDLE_NONE;
static uint16_t                s_tx_val_hdl         = 0;
static uint8_t                 s_own_addr_type      = BLE_OWN_ADDR_PUBLIC;
static bool                    s_tx_notify_enabled  = false;
static volatile bool           s_conn_encrypted     = false;
static char                    s_dis_serial_str[16] = {0};
static char                    s_ble_device_name[32] = {0};

/* Link Negotiation & Dynamic Power States (Phase 2) */
typedef enum {
    VCI_BLE_LINK_STATE_ACTIVE = 0,
    VCI_BLE_LINK_STATE_IDLE   = 1,
} vci_ble_link_state_t;

static vci_ble_link_state_t    s_link_state         = VCI_BLE_LINK_STATE_ACTIVE;
static esp_timer_handle_t      s_conn_init_timer    = NULL;
static esp_timer_handle_t      s_inactivity_timer   = NULL;
static uint16_t                s_dle_max_tx_octets  = 27;
static uint16_t                s_dle_max_rx_octets  = 27;
static uint8_t                 s_tx_phy             = 1; /* 1 = 1M, 2 = 2M */
static uint8_t                 s_rx_phy             = 1;
static uint32_t                s_idle_entry_count   = 0;
static uint32_t                s_active_wake_count  = 0;

static void on_conn_init_timer(void *arg)
{
    (void)arg;
    if (s_conn_hdl == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    /* Active Diagnostic State: Conn_Interval = 7.5 - 15.0 ms, Latency = 0, Supervision_Timeout = 5000 ms */
    struct ble_gap_upd_params active_params = {
        .itvl_min = 6,              /* 7.5 ms (6 * 1.25 ms) */
        .itvl_max = 12,             /* 15.0 ms (12 * 1.25 ms) */
        .latency = 0,               /* 0 */
        .supervision_timeout = 500, /* 5000 ms (500 * 10 ms) */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(s_conn_hdl, &active_params);
    esp_rom_printf("[NIMBLE GAP] conn_update rc=%d\n", rc);
    ESP_LOGI(TAG, "Initiated Active Conn Params: itvl=[7.5, 15.0]ms, latency=0, timeout=5000ms (rc=%d)", rc);

    /* Arm 10-second inactivity timer */
    if (s_inactivity_timer != NULL) {
        (void)esp_timer_stop(s_inactivity_timer);
        (void)esp_timer_start_once(s_inactivity_timer, 10000000ULL);
    }
}

static void on_inactivity_timer(void *arg)
{
    (void)arg;
    if (s_conn_hdl == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (s_link_state == VCI_BLE_LINK_STATE_ACTIVE) {
        s_link_state = VCI_BLE_LINK_STATE_IDLE;
        s_idle_entry_count++;
        /* Idle State (>10s inactivity): Peripheral_Latency = 4, Timeout = 5000 ms */
        struct ble_gap_upd_params idle_params = {
            .itvl_min = 6,              /* 7.5 ms */
            .itvl_max = 12,             /* 15.0 ms */
            .latency = 4,               /* Peripheral_Latency = 4 */
            .supervision_timeout = 500, /* 5000 ms */
            .min_ce_len = 0,
            .max_ce_len = 0,
        };
        int rc = ble_gap_update_params(s_conn_hdl, &idle_params);
        ESP_LOGI(TAG, "Link Idle (>10s GATT/CAN inactivity): Requested Peripheral_Latency=4 (rc=%d)", rc);
    }
}

/**
 * @brief Retrieve or generate an 8-character alphanumeric serial number stored in NVS.
 * If unprovisioned, generates a deterministic 8-character uppercase hex string from
 * the hardware eFuse MAC address and commits it to NVS.
 */
static esp_err_t vci_get_or_create_nvs_serial(char *out_serial, size_t max_len)
{
    if (out_serial == NULL || max_len < 9) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_hdl;
    esp_err_t err = nvs_open("vci_cfg", NVS_READWRITE, &nvs_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(vci_cfg) failed: %d", err);
        return err;
    }

    size_t req_len = max_len;
    err = nvs_get_str(nvs_hdl, "serial_num", out_serial, &req_len);
    if (err == ESP_OK && strlen(out_serial) == 8) {
        ESP_LOGI(TAG, "Loaded serial from NVS: '%s'", out_serial);
        nvs_close(nvs_hdl);
        return ESP_OK;
    }

    /* Unprovisioned: derive 8-character uppercase hex serial from eFuse MAC */
    uint8_t mac[6] = {0};
    (void)esp_efuse_mac_get_default(mac);
    /* Use 4 bytes: mac[2..5] -> 8 alphanumeric uppercase hex chars */
    snprintf(out_serial, max_len, "%02X%02X%02X%02X", mac[2], mac[3], mac[4], mac[5]);

    err = nvs_set_str(nvs_hdl, "serial_num", out_serial);
    if (err == ESP_OK) {
        (void)nvs_commit(nvs_hdl);
        ESP_LOGI(TAG, "Provisioned new 8-char serial in NVS: '%s'", out_serial);
    } else {
        ESP_LOGW(TAG, "Failed to persist serial in NVS: %d", err);
    }

    nvs_close(nvs_hdl);
    return ESP_OK;
}

/* Nordic UART Service (NUS) 128-bit UUIDs (Little-Endian) */
/* 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
static const ble_uuid128_t s_nus_svc_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
);

/* RX Characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Host -> VCI Write) */
static const ble_uuid128_t s_nus_rx_chr_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
);

/* TX Characteristic: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (VCI -> Host Notify) */
static const ble_uuid128_t s_nus_tx_chr_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
);

static void ble_nus_start_advertising(void);

static int gatt_nus_chr_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    (void)attr_handle;
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /* Enforce LE Security Mode 1, Level 2: reject unencrypted GATT writes (including Write Without Response) */
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle, &desc) != 0 || !desc.sec_state.encrypted) {
            (void)ble_gap_security_initiate(conn_handle);
            return BLE_ATT_ERR_INSUFFICIENT_ENC;
        }
        s_conn_encrypted = true;
        ble_nus_notify_activity();

        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len > 0 && s_rx_cb != NULL) {
            uint8_t buf[256];
            const struct os_mbuf *cur = ctxt->om;
            while (cur != NULL) {
                uint16_t chunk_len = cur->om_len;
                if (chunk_len > sizeof(buf)) {
                    chunk_len = sizeof(buf);
                }
                memcpy(buf, cur->om_data, chunk_len);
                s_rx_cb(buf, chunk_len);
                cur = SLIST_NEXT(cur, om_next);
            }
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_gatt_nus_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* NUS RX Characteristic: Write / Write Without Response (Requires Mode 1 Level 2 Encryption) */
                .uuid       = &s_nus_rx_chr_uuid.u,
                .access_cb  = gatt_nus_chr_access,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
            },
            {
                /* NUS TX Characteristic: Notify */
                .uuid       = &s_nus_tx_chr_uuid.u,
                .access_cb  = gatt_nus_chr_access,
                .val_handle = &s_tx_val_hdl,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

static int ble_nus_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    struct ble_gap_conn_desc desc;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_hdl = event->connect.conn_handle;
                s_tx_notify_enabled = true;
                s_conn_encrypted = false;
                s_link_state = VCI_BLE_LINK_STATE_ACTIVE;
                s_dle_max_tx_octets = 27;
                s_dle_max_rx_octets = 27;
                s_tx_phy = 1;
                s_rx_phy = 1;

                (void)ble_gattc_exchange_mtu(s_conn_hdl, NULL, NULL);

                /* Request Data Length Extension (DLE): connMaxTxOctets = 251, connMaxTxTime = 2120 us */
                (void)ble_gap_set_data_len(s_conn_hdl, 251, 2120);

                /* Schedule Active Connection Parameters update fallback within < 1.0s (800 ms) */
                if (s_conn_init_timer != NULL) {
                    (void)esp_timer_stop(s_conn_init_timer);
                    (void)esp_timer_start_once(s_conn_init_timer, 800000ULL); /* 800 ms */
                }

                ESP_LOGI(TAG, "BLE Central connected (conn_handle=%u)", s_conn_hdl);
            } else {
                s_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
                s_tx_notify_enabled = false;
                s_conn_encrypted = false;
                ble_nus_start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            esp_rom_printf("[NIMBLE GAP] DISCONNECT reason=%d (0x%04X)\n", event->disconnect.reason, event->disconnect.reason);
            ESP_LOGI(TAG, "BLE Central disconnected (reason=%d)", event->disconnect.reason);
            s_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
            s_tx_notify_enabled = false;
            s_conn_encrypted = false;
            s_link_state = VCI_BLE_LINK_STATE_ACTIVE;
            s_dle_max_tx_octets = 27;
            s_dle_max_rx_octets = 27;
            s_tx_phy = 1;
            s_rx_phy = 1;

            if (s_conn_init_timer != NULL && esp_timer_is_active(s_conn_init_timer)) {
                (void)esp_timer_stop(s_conn_init_timer);
            }
            if (s_inactivity_timer != NULL && esp_timer_is_active(s_inactivity_timer)) {
                (void)esp_timer_stop(s_inactivity_timer);
            }
            if (s_disconnect_cb != NULL) {
                s_disconnect_cb();
            }
            ble_nus_start_advertising();
            return 0;

        case BLE_GAP_EVENT_DATA_LEN_CHG:
            s_dle_max_tx_octets = event->data_len_chg.max_tx_octets;
            s_dle_max_rx_octets = event->data_len_chg.max_rx_octets;
            ESP_LOGI(TAG, "BLE DLE updated: tx_octets=%u, tx_time=%uus, rx_octets=%u, rx_time=%uus",
                     event->data_len_chg.max_tx_octets,
                     event->data_len_chg.max_tx_time,
                     event->data_len_chg.max_rx_octets,
                     event->data_len_chg.max_rx_time);
            return 0;

        case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
            s_tx_phy = event->phy_updated.tx_phy;
            s_rx_phy = event->phy_updated.rx_phy;
            ESP_LOGI(TAG, "BLE PHY updated: status=%d, tx_phy=%u (%s), rx_phy=%u (%s)",
                     event->phy_updated.status,
                     event->phy_updated.tx_phy,
                     (event->phy_updated.tx_phy == BLE_GAP_LE_PHY_2M) ? "2M" : "1M",
                     event->phy_updated.rx_phy,
                     (event->phy_updated.rx_phy == BLE_GAP_LE_PHY_2M) ? "2M" : "1M");
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            if (event->conn_update.status == 0) {
                struct ble_gap_conn_desc cd;
                if (ble_gap_conn_find(event->conn_update.conn_handle, &cd) == 0) {
                    ESP_LOGI(TAG, "BLE Conn Params updated: itvl=%.2fms (0x%04X), latency=%u, timeout=%ums",
                             cd.conn_itvl * 1.25f,
                             cd.conn_itvl,
                             cd.conn_latency,
                             cd.supervision_timeout * 10);
                    if (cd.conn_latency == 0) {
                        s_link_state = VCI_BLE_LINK_STATE_ACTIVE;
                    } else if (cd.conn_latency >= 4) {
                        s_link_state = VCI_BLE_LINK_STATE_IDLE;
                    }
                }
            } else {
                ESP_LOGW(TAG, "BLE Conn Params update failed (status=%d)", event->conn_update.status);
            }
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        case BLE_GAP_EVENT_L2CAP_UPDATE_REQ:
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0 &&
                ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                s_conn_encrypted = (desc.sec_state.encrypted != 0);
                if (s_conn_encrypted) {
                    /* Within 1.0s of link establishment, negotiate Active Conn Params */
                    struct ble_gap_upd_params active_params = {
                        .itvl_min = 6,              /* 7.5 ms */
                        .itvl_max = 12,             /* 15.0 ms */
                        .latency = 0,               /* 0 */
                        .supervision_timeout = 500, /* 5000 ms */
                        .min_ce_len = 0,
                        .max_ce_len = 0,
                    };
                    int rc = ble_gap_update_params(event->enc_change.conn_handle, &active_params);
                    ESP_LOGI(TAG, "Initiated Active Conn Params on encryption (rc=%d)", rc);
                    if (s_conn_init_timer != NULL && esp_timer_is_active(s_conn_init_timer)) {
                        (void)esp_timer_stop(s_conn_init_timer);
                    }
                    if (s_inactivity_timer != NULL) {
                        (void)esp_timer_stop(s_inactivity_timer);
                        (void)esp_timer_start_once(s_inactivity_timer, 10000000ULL);
                    }
                }
            } else {
                s_conn_encrypted = false;
            }
            return 0;

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            /* Delete stale bond for this peer and allow repeat pairing to succeed seamlessly */
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
                (void)ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_nus_start_advertising();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_tx_val_hdl) {
                s_tx_notify_enabled = (event->subscribe.cur_notify != 0);
                ESP_LOGI(TAG, "BLE NUS TX notifications %s", s_tx_notify_enabled ? "ENABLED" : "DISABLED");
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "BLE ATT MTU updated: %u bytes", event->mtu.value);
            return 0;

        default:
            return 0;
    }
}

static void ble_nus_start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    /* Flags: LE General Discoverable Mode + BR/EDR Not Supported (LE-only controller) */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    /* Put 128-bit NUS Service UUID in the Scan Response packet (<= 31 bytes) */
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t *)&s_nus_svc_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    /* 100 ms advertising interval (0x00A0 * 0.625 ms = 100.0 ms) */
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = 0x00A0; /* 100 ms */
    adv_params.itvl_max  = 0x00A0; /* 100 ms */

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_nus_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE Advertising started as '%s' (interval=100ms, flags=0x%02X)", name, fields.flags);
    }
}

static void ble_nus_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    ble_nus_start_advertising();
}

static void ble_nus_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset (reason=%d)", reason);
}

static void ble_nus_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_nus_init(ble_nus_rx_cb_t rx_cb)
{
    s_rx_cb = rx_cb;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return ret;
    }

    ble_hs_cfg.reset_cb          = ble_nus_on_reset;
    ble_hs_cfg.sync_cb           = ble_nus_on_sync;
    /* LE Security Mode 1, Level 2: Unauthenticated Pairing with Encryption (Just Works + NVS Bonding) */
    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_store_config_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_dis_init();

    /* Configure Bluetooth LE Power Class 2: maximum output power <= +4 dBm (set to +3 dBm / ESP_PWR_LVL_P3) */
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_tx_power_set(DEFAULT, P3) returned: %d", ret);
    }
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_tx_power_set(ADV, P3) returned: %d", ret);
    }

    /* Retrieve 8-character alphanumeric serial number from NVS */
    (void)vci_get_or_create_nvs_serial(s_dis_serial_str, sizeof(s_dis_serial_str));
    snprintf(s_ble_device_name, sizeof(s_ble_device_name), "%s-%s", VCI_BLE_DEVICE_NAME, s_dis_serial_str);

    (void)ble_svc_dis_manufacturer_name_set("OpenVCI");
    (void)ble_svc_dis_model_number_set(VCI_BLE_DEVICE_NAME);
    (void)ble_svc_dis_serial_number_set(s_dis_serial_str);
    (void)ble_svc_dis_hardware_revision_set("v1.0.0");
    (void)ble_svc_dis_firmware_revision_set("v2.1.0");
    (void)ble_svc_dis_software_revision_set("ISO-22900-2-D-PDU");

    int rc = ble_gatts_count_cfg(s_gatt_nus_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gatt_nus_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    ble_svc_gap_device_name_set(s_ble_device_name);
    ble_att_set_preferred_mtu(256);

    /* Timers for connection parameter negotiation and dynamic power states */
    esp_timer_create_args_t init_timer_args = {
        .callback = on_conn_init_timer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "vci_ble_init",
        .skip_unhandled_events = true,
    };
    (void)esp_timer_create(&init_timer_args, &s_conn_init_timer);

    esp_timer_create_args_t idle_timer_args = {
        .callback = on_inactivity_timer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "vci_ble_idle",
        .skip_unhandled_events = true,
    };
    (void)esp_timer_create(&idle_timer_args, &s_inactivity_timer);

    nimble_port_freertos_init(ble_nus_host_task);
    return ESP_OK;
}

void ble_nus_set_disconnect_cb(ble_nus_disconnect_cb_t disconnect_cb)
{
    s_disconnect_cb = disconnect_cb;
}

bool ble_nus_is_connected(void)
{
    return (s_conn_hdl != BLE_HS_CONN_HANDLE_NONE);
}

bool ble_nus_is_encrypted(void)
{
    if (s_conn_hdl == BLE_HS_CONN_HANDLE_NONE) {
        return false;
    }
    if (!s_conn_encrypted) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(s_conn_hdl, &desc) == 0 && desc.sec_state.encrypted) {
            s_conn_encrypted = true;
        }
    }
    return s_conn_encrypted;
}

void ble_nus_notify_activity(void)
{
    if (s_conn_hdl == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (s_link_state == VCI_BLE_LINK_STATE_IDLE) {
        s_link_state = VCI_BLE_LINK_STATE_ACTIVE;
        s_active_wake_count++;
        struct ble_gap_upd_params active_params = {
            .itvl_min = 6,              /* 7.5 ms */
            .itvl_max = 12,             /* 15.0 ms */
            .latency = 0,               /* 0 */
            .supervision_timeout = 500, /* 5000 ms */
            .min_ce_len = 0,
            .max_ce_len = 0,
        };
        int rc = ble_gap_update_params(s_conn_hdl, &active_params);
        ESP_LOGI(TAG, "Link Activity: Restored Peripheral_Latency=0 (rc=%d)", rc);
    }
    if (s_inactivity_timer != NULL) {
        if (esp_timer_is_active(s_inactivity_timer)) {
            (void)esp_timer_restart(s_inactivity_timer, 10000000ULL);
        } else {
            (void)esp_timer_start_once(s_inactivity_timer, 10000000ULL);
        }
    }
}

esp_err_t ble_nus_get_link_status(ble_link_status_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_conn_hdl == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(s_conn_hdl, &desc) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    out_status->conn_itvl = desc.conn_itvl;
    out_status->conn_latency = desc.conn_latency;
    out_status->supervision_timeout = desc.supervision_timeout;
    out_status->mtu = ble_att_mtu(s_conn_hdl);
    out_status->max_tx_octets = s_dle_max_tx_octets;
    out_status->max_rx_octets = s_dle_max_rx_octets;
    out_status->tx_phy = s_tx_phy;
    out_status->rx_phy = s_rx_phy;
    out_status->is_idle = (s_link_state == VCI_BLE_LINK_STATE_IDLE);
    out_status->idle_entry_count = s_idle_entry_count;
    out_status->active_wake_count = s_active_wake_count;
    return ESP_OK;
}

esp_err_t ble_nus_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || s_conn_hdl == BLE_HS_CONN_HANDLE_NONE ||
        !s_tx_notify_enabled || !ble_nus_is_encrypted()) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_nus_notify_activity();

    uint16_t mtu = ble_att_mtu(s_conn_hdl);
    uint16_t max_chunk = (mtu > 3U) ? (mtu - 3U) : 20U;
    if (max_chunk > 248U) {
        max_chunk = 248U;
    }

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > max_chunk) {
            chunk = max_chunk;
        }
        int rc = BLE_HS_ENOMEM;
        for (int retry = 0; retry < 5; retry++) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&data[offset], (uint16_t)chunk);
            if (om != NULL) {
                rc = ble_gatts_notify_custom(s_conn_hdl, s_tx_val_hdl, om);
                if (rc != BLE_HS_ENOMEM) {
                    break;
                }
            }
            vTaskDelay(1);
        }
        if (rc != 0) {
            return ESP_FAIL;
        }
        offset += chunk;
    }

    return ESP_OK;
}

