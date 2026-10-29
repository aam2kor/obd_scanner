/**
 * @file ble_nus.c
 * @brief Layer 1: Bluetooth LE Nordic UART Service (NUS) Transport via ESP-IDF NimBLE (Core 0).
 */

#include "ble_nus.h"
#include "esp_log.h"
#include "nvs_flash.h"
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

static const char *TAG = "ble_nus";

static ble_nus_rx_cb_t         s_rx_cb              = NULL;
static ble_nus_disconnect_cb_t s_disconnect_cb      = NULL;
static uint16_t                s_conn_hdl           = BLE_HS_CONN_HANDLE_NONE;
static uint16_t                s_tx_val_hdl         = 0;
static uint8_t                 s_own_addr_type      = BLE_OWN_ADDR_PUBLIC;
static bool                    s_tx_notify_enabled  = false;
static volatile bool           s_conn_encrypted     = false;
static char                    s_dis_serial_str[24] = {0};

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
                (void)ble_gattc_exchange_mtu(s_conn_hdl, NULL, NULL);
                ESP_LOGI(TAG, "BLE Central connected (conn_handle=%u)", s_conn_hdl);
            } else {
                s_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
                s_tx_notify_enabled = false;
                s_conn_encrypted = false;
                ble_nus_start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Central disconnected (reason=%d)", event->disconnect.reason);
            s_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
            s_tx_notify_enabled = false;
            s_conn_encrypted = false;
            if (s_disconnect_cb != NULL) {
                s_disconnect_cb();
            }
            ble_nus_start_advertising();
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0 &&
                ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                s_conn_encrypted = (desc.sec_state.encrypted != 0);
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

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = 0x20; /* 20 ms */
    adv_params.itvl_max  = 0x40; /* 40 ms */

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_nus_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE Advertising started as '%s'", name);
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

    /* Populate Device Information Service (0x180A) characteristics */
    uint8_t mac[6] = {0};
    (void)esp_efuse_mac_get_default(mac);
    snprintf(s_dis_serial_str, sizeof(s_dis_serial_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

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

    ble_svc_gap_device_name_set(VCI_BLE_DEVICE_NAME);
    ble_att_set_preferred_mtu(256);

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

esp_err_t ble_nus_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || s_conn_hdl == BLE_HS_CONN_HANDLE_NONE ||
        !s_tx_notify_enabled || !ble_nus_is_encrypted()) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t mtu = ble_att_mtu(s_conn_hdl);
    uint16_t max_chunk = (mtu > 3U) ? (mtu - 3U) : 20U;
    if (max_chunk > 240U) {
        max_chunk = 240U;
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

