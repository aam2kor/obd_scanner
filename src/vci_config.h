/**
 * @file vci_config.h
 * @brief Hardware pin definitions, FreeRTOS task/queue parameters, and Lean D-PDU constants.
 */

#ifndef VCI_CONFIG_H
#define VCI_CONFIG_H

#include "driver/gpio.h"
#include "driver/uart.h"

/* Hardware Pin Mapping */
#define VCI_LED_GPIO            GPIO_NUM_2
#define VCI_CAN_TX_GPIO         GPIO_NUM_21
#define VCI_CAN_RX_GPIO         GPIO_NUM_22

/* Host UART Configuration */
#define VCI_HOST_UART_NUM       UART_NUM_0
#define VCI_HOST_UART_BAUD      115200
#define VCI_HOST_UART_RX_BUF    2048
#define VCI_HOST_UART_TX_BUF    2048

/* FreeRTOS Core Assignment */
#define VCI_CORE_HOST_IO        0   /* PRO_CPU: BLE / UART Host Comms */
#define VCI_CORE_REALTIME       1   /* APP_CPU: D-PDU, ISO-TP, CAN TWAI */

/* FreeRTOS Task Priorities */
#define VCI_PRIO_LED_TASK       3
#define VCI_PRIO_HOST_IO_TASK   10
#define VCI_PRIO_VCI_RT_TASK    18
#define VCI_PRIO_ECU_SIM_TASK   19

/* Queue Depths */
#define VCI_CMD_QUEUE_DEPTH     8
#define VCI_EVT_QUEUE_DEPTH     8

#endif /* VCI_CONFIG_H */

