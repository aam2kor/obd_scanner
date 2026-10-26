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
 * @brief Callback invoked on Core 0 whenever raw bytes arrive on the BLE NUS RX characteristic.
 */
typedef void (*ble_nus_rx_cb_t)(const uint8_t *data, size_t len);

/**
 * @brief Initialize NVS, NimBLE Host & Controller, GATT Nordic UART Service, and start advertising.
 */
esp_err_t ble_nus_init(ble_nus_rx_cb_t rx_cb);

/**
 * @brief Check whether a BLE Central (PC/Mobile App) is currently connected.
 */
bool ble_nus_is_connected(void);

/**
 * @brief Send raw D-PDU wire bytes to the connected BLE Central via NUS TX notifications.
 *        Automatically fragments payloads larger than (ATT_MTU - 3).
 */
esp_err_t ble_nus_send(const uint8_t *data, size_t len);

#endif /* BLE_NUS_H */

