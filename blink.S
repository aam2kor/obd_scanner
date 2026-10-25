/*
 * blink.S — Bare-Metal ESP32 LED Blink Firmware (1.000s Interval)
 * Target: ESP32-WROOM-32 (Xtensa LX6 @ 40 MHz XTAL boot clock)
 * GPIO:   GPIO 2 (Onboard Blue LED)
 * UART:   UART0 @ 115200 baud ("ON\r\n" / "OFF\r\n" status output)
 */

    .section .text
    .global _start

_start:
    /* 1. Disable Timer Group 0 & RTC Watchdogs */
    l32r    a8, .L_TIMG0_WDTWPROTECT
    l32r    a2, .L_WDT_WKEY
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_TIMG0_WDTCONFIG0
    l32r    a2, .L_ZERO
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_RTC_WDTWPROTECT
    l32r    a2, .L_WDT_WKEY
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_RTC_WDTCONFIG0
    l32r    a2, .L_ZERO
    memw
    s32i.n  a2, a8, 0

    /* 2. Configure GPIO 2 as Output */
    l32r    a8, .L_IO_MUX_GPIO2
    l32r    a2, .L_IO_MUX_GPIO2_VAL
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_GPIO_FUNC2_OUT_SEL
    l32r    a2, .L_GPIO_SIMPLE_OUT
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_GPIO_ENABLE_W1TS
    l32r    a2, .L_GPIO2_MASK
    memw
    s32i.n  a2, a8, 0

.L_blink_loop:
    /* 3. Turn LED ON (GPIO 2 = HIGH) and transmit "ON\r\n" */
    l32r    a8, .L_GPIO_OUT_W1TS
    l32r    a2, .L_GPIO2_MASK
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_UART0_FIFO
    l32r    a2, .L_CHAR_O
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_N
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_CR
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_LF
    s32i.n  a2, a8, 0

    /* Delay 1.000 second (8,000,000 iterations * 5 cycles = 40,000,000 cycles @ 40 MHz) */
    l32r    a11, .L_ONE
    l32r    a2, .L_TICKS_1S
    sub     a10, a10, a10
    .balign 4
.L_delay_on:
    add     a10, a10, a11
    bltu    a10, a2, .L_delay_on

    /* 4. Turn LED OFF (GPIO 2 = LOW) and transmit "OFF\r\n" */
    l32r    a8, .L_GPIO_OUT_W1TC
    l32r    a2, .L_GPIO2_MASK
    memw
    s32i.n  a2, a8, 0

    l32r    a8, .L_UART0_FIFO
    l32r    a2, .L_CHAR_O
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_F
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_F
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_CR
    s32i.n  a2, a8, 0
    l32r    a2, .L_CHAR_LF
    s32i.n  a2, a8, 0

    /* Delay 1.000 second */
    l32r    a11, .L_ONE
    l32r    a2, .L_TICKS_1S
    sub     a10, a10, a10
    .balign 4
.L_delay_off:
    add     a10, a10, a11
    bltu    a10, a2, .L_delay_off

    /* Repeat forever */
    sub     a10, a10, a10
    bltu    a10, a2, .L_blink_loop

