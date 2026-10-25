/**
 * @file main.c
 * @brief ESP32 LED Blink (1-second interval) in pure C using ESP-IDF & FreeRTOS.
 *
 * Foundation for the ESP32 OBD-II CAN (TWAI) Scanner project.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define BLINK_GPIO      GPIO_NUM_2
#define BLINK_PERIOD_MS 1000

static const char *TAG = "obd_scanner";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing onboard LED on GPIO %d...", BLINK_GPIO);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLINK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    uint32_t led_level = 0;

    while (1) {
        led_level = !led_level;
        ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, led_level));
        ESP_LOGI(TAG, "LED state: %s (GPIO%d=%lu)", led_level ? "ON" : "OFF", BLINK_GPIO, (unsigned long)led_level);
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
    }
}

