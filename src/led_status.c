/**
 * @file led_status.c
 * @brief Non-blocking status LED controller on GPIO 2.
 */

#include "led_status.h"
#include "vci_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static volatile vci_led_mode_t s_led_mode = VCI_LED_DISCONNECTED;
static volatile int64_t s_activity_until_us = 0;

static void led_status_task(void *arg)
{
    (void)arg;
    uint32_t tick = 0;

    while (1) {
        int64_t now_us = esp_timer_get_time();
        uint32_t level = 0;

        if (now_us < s_activity_until_us) {
            /* Fast 10 Hz activity flash (50ms ON, 50ms OFF) */
            level = (tick & 1U) ? 1U : 0U;
        } else if (s_led_mode == VCI_LED_CONNECTED_IDLE) {
            /* Steady 1 Hz blink (500ms ON, 500ms OFF at 50ms tick) */
            level = ((tick % 20U) < 10U) ? 1U : 0U;
        } else {
            /* Disconnected: brief double-pulse every 1s (ticks 0, 2 ON out of 20) */
            uint32_t phase = tick % 20U;
            level = (phase == 0U || phase == 2U) ? 1U : 0U;
        }

        gpio_set_level(VCI_LED_GPIO, level);
        tick++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void led_status_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VCI_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(VCI_LED_GPIO, 0);

    xTaskCreatePinnedToCore(
        led_status_task,
        "led_status",
        2048,
        NULL,
        VCI_PRIO_LED_TASK,
        NULL,
        VCI_CORE_HOST_IO
    );
}

void led_status_set_mode(vci_led_mode_t mode)
{
    s_led_mode = mode;
}

void led_status_pulse_activity(void)
{
    s_activity_until_us = esp_timer_get_time() + 250000LL; /* 250ms burst */
}

