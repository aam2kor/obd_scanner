/**
 * @file ble_nus.h
 * @brief Layer 1: Bluetooth LE Nordic UART Service (NUS) Transport via ESP-IDF NimBLE (Core 0).
 *
 * Service UUID : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX Char UUID : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write / Write Without Response)
 * TX Char UUID : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify)
 */

#ifndef BLE_NUS_H
#define BLE_NUS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define VCI_BLE_DEVICE_NAME     "ESP32-OBD-VCI"

/**
 * @brief Link Layer connection and negotiation status structure.
 */
typedef struct {
    uint16_t conn_itvl;             /* in 1.25 ms units */
    uint16_t conn_latency;          /* slave latency count */
    uint16_t supervision_timeout;   /* in 10 ms units */
    uint16_t mtu;                   /* negotiated ATT MTU */
    uint16_t max_tx_octets;         /* DLE max TX octets */
    uint16_t max_rx_octets;         /* DLE max RX octets */
    uint8_t  tx_phy;                /* 1 = 1M, 2 = 2M */
    uint8_t  rx_phy;                /* 1 = 1M, 2 = 2M */
    bool     is_idle;               /* true if currently in Idle power state */
    uint32_t idle_entry_count;      /* number of transitions into Idle state */
    uint32_t active_wake_count;     /* number of wakeups back to Active state */
} ble_link_status_t;

/**
 * @brief Callback invoked on Core 0 whenever raw bytes arrive on the BLE NUS RX characteristic.
 */
typedef void (*ble_nus_rx_cb_t)(const uint8_t *data, size_t len);

/**
 * @brief Callback invoked on Core 0 when an active BLE GATT connection drops (Link Supervision Timeout / disconnect).
 */
typedef void (*ble_nus_disconnect_cb_t)(void);

/**
 * @brief Initialize NVS, NimBLE Host & Controller, GATT Nordic UART Service, and start advertising.
 */
esp_err_t ble_nus_init(ble_nus_rx_cb_t rx_cb);

/**
 * @brief Register a disconnect callback to trigger autonomous PDUDestruct within 100 ms.
 */
void ble_nus_set_disconnect_cb(ble_nus_disconnect_cb_t disconnect_cb);

/**
 * @brief Check whether a BLE Central (PC/Mobile App) is currently connected.
 */
bool ble_nus_is_connected(void);

/**
 * @brief Check whether the active BLE connection is encrypted (LE Security Mode 1, Level 2+).
 */
bool ble_nus_is_encrypted(void);

/**
 * @brief Send raw D-PDU wire bytes to the connected BLE Central via NUS TX notifications.
 *        Automatically fragments payloads larger than (ATT_MTU - 3).
 */
esp_err_t ble_nus_send(const uint8_t *data, size_t len);

/**
 * @brief Notify the BLE link management engine of diagnostic or CAN bus activity.
 *        Wakes the link from Idle state (restores latency 0) and resets the 10-second inactivity timer.
 */
void ble_nus_notify_activity(void);

/**
 * @brief Query current Link Layer negotiation and power state parameters.
 */
esp_err_t ble_nus_get_link_status(ble_link_status_t *out_status);

#endif /* BLE_NUS_H */


