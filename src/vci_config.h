/**
 * @file vci_config.h
 * @brief Hardware pin definitions, FreeRTOS task parameters, and ISO 22900-2 App-Device IF constants.
 */

#ifndef VCI_CONFIG_H
#define VCI_CONFIG_H

#include "driver/gpio.h"
#include "driver/uart.h"

/* Hardware Pin Mapping */
#define VCI_LED_GPIO                GPIO_NUM_2
#define VCI_CAN_TX_GPIO             GPIO_NUM_21
#define VCI_CAN_RX_GPIO             GPIO_NUM_22

/* Host UART Configuration */
#define VCI_HOST_UART_NUM           UART_NUM_0
#define VCI_HOST_UART_BAUD          115200
#define VCI_HOST_UART_RX_BUF        2048
#define VCI_HOST_UART_TX_BUF        2048

/* FreeRTOS Core Assignment */
#define VCI_CORE_HOST_IO            0   /* PRO_CPU: BLE / UART Host Comms & Synchronous ACK/NACK */
#define VCI_CORE_REALTIME           1   /* APP_CPU: 8-CLL Round-Robin D-PDU, ISO-TP, CAN TWAI */

/* FreeRTOS Task Priorities */
#define VCI_PRIO_LED_TASK           3
#define VCI_PRIO_HOST_IO_TASK       10
#define VCI_PRIO_VCI_RT_TASK        18
#define VCI_PRIO_ECU_SIM_TASK       19

/* App-Device IF Resource Limits */
#define VCI_MAX_CLLS                8U      /* Up to 8 concurrent ComLogicalLinks (CLL_ID 1..8) */
#define VCI_CLL_COP_QUEUE_DEPTH     4U      /* Up to 4 queued ComPrimitives per CLL */
#define VCI_CLL_SF_BUF_SIZE         8U      /* 1 static 8-byte Single-Frame buffer per CLL */
#define VCI_MF_POOL_COUNT           3U      /* 3x 4,095-byte Multi-Frame RX assembly buffers */
#define VCI_ISOTP_MAX_PDU_LEN       4095U   /* Max standard ISO 15765-2 12-bit FF payload */
#define VCI_MAX_PACKET_PAYLOAD      4103U   /* 8B header fields + 4095B N-PDU */

/* Firmware & Hardware Identification (for PDUGetVersion / IOCTL_GET_CABLE_ID) */
#define VCI_FW_VERSION_U32          0x00020100UL /* v2.1.0 */
#define VCI_HW_VERSION_U32          0x00010000UL /* ESP32 Rev 1.0 */
#define VCI_SAE_J1962_CABLE_ID      0x000007AAUL /* 1962 (SAE J1962 16-pin DLC) */
#define VCI_DEFAULT_VBATT_MV        12600UL      /* Nominal Pin 16 VBATT in mV (12.60 V) */

#endif /* VCI_CONFIG_H */
