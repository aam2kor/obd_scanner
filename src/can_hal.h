/**
 * @file can_hal.h
 * @brief Layer 4: CAN Hardware Abstraction Layer (ESP-IDF TWAI + Virtual ECU Loopback).
 */

#ifndef CAN_HAL_H
#define CAN_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint32_t id;
    bool     is_extd;
    uint8_t  dlc;
    uint8_t  data[8];
} vci_can_frame_t;

/**
 * @brief Initialize CAN HAL queues for Virtual ECU loopback mode.
 */
void can_hal_init(void);

/**
 * @brief Start the CAN bus interface in either Virtual ECU Simulator mode (0)
 *        or Physical ESP32 TWAI controller mode (1) at the given baudrate (250/500/1000 kbps).
 */
esp_err_t can_hal_start(uint8_t bus_mode, uint16_t baud_khz);

/**
 * @brief Stop the active CAN interface.
 */
void can_hal_stop(void);

/**
 * @brief Transmit a Classic CAN frame from the Tester/VCI side.
 */
esp_err_t can_hal_send(const vci_can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Receive a Classic CAN frame on the Tester/VCI side.
 */
esp_err_t can_hal_recv(vci_can_frame_t *frame, uint32_t timeout_ms);

/* Virtual ECU Simulator Bus Hooks (used by ecu_sim.c when bus_mode == 0) */
esp_err_t can_hal_sim_ecu_recv(vci_can_frame_t *frame, uint32_t timeout_ms);
esp_err_t can_hal_sim_ecu_send(const vci_can_frame_t *frame, uint32_t timeout_ms);

#endif /* CAN_HAL_H */

