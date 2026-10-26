/**
 * @file led_status.h
 * @brief Non-blocking status LED controller on GPIO 2.
 */

#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>

typedef enum {
    VCI_LED_DISCONNECTED = 0,   /* Double-blink heartbeat every 1s */
    VCI_LED_CONNECTED_IDLE,     /* Steady 1 Hz blink (500ms ON / 500ms OFF) */
} vci_led_mode_t;

/**
 * @brief Initialize GPIO 2 and start the background status LED task on Core 0.
 */
void led_status_init(void);

/**
 * @brief Set the base status LED mode (Disconnected vs. Connected).
 */
void led_status_set_mode(vci_led_mode_t mode);

/**
 * @brief Trigger a brief 10 Hz activity burst on the LED when D-PDU / CAN packets flow.
 */
void led_status_pulse_activity(void);

#endif /* LED_STATUS_H */

