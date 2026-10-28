# ESP32-OBD-VCI — Host Interface Specification (ISO 22900-2:2022 App-Device IF)

This document specifies the transport, binary wire framing (`0xAA55`), command/event opcodes, IOCTLs, multi-CLL queue architecture, and ComParam configuration required for a PC or mobile diagnostic application to communicate with the **ESP32-OBD-VCI**.

---

## 1. Host Transport Interfaces

Both transports carry the **exact same binary ISO 22900-2:2022 `App-Device IF` packets**. No ASCII/AT adapter is used.

### 1.1 Bluetooth LE (Nordic UART Service — NUS)
| Parameter | Value |
| :--- | :--- |
| **BLE Advertised Name** | `ESP32-OBD-VCI` |
| **NUS Service UUID** | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| **RX Characteristic UUID (Host → VCI)** | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` (`Write` / `Write Without Response`) |
| **TX Characteristic UUID (VCI → Host)** | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` (`Notify` — subscribe to CCCD on connect) |
| **Preferred ATT MTU** | `256` bytes (payloads larger than `MTU - 3` are fragmented automatically; store-and-forward reassembly via the stream parser) |
| **Wireless Link-Loss Safety** | If BLE GATT disconnects (`BLE_GAP_EVENT_DISCONNECT`), the VCI autonomously executes an internal `PDUDestruct` within `< 100 ms` (halting cyclic primitives, aborting in-flight ISO-TP transfers, clearing all 8 CLLs, and stopping CAN). |

### 1.2 USB-UART (Wired Fallback)
| Parameter | Value |
| :--- | :--- |
| **Serial Configuration** | `115200` baud, `8N1`, no hardware flow control (`RTS=False`, `DTR=False`) |

---

## 2. Binary Wire Frame Format (`0xAA55`)

All multi-byte integers on the wire are **Little-Endian** (`<H` for `uint16`, `<I` for `uint32`).

```text
+---------------+----------+--------+--------+--------+-------------+---------------+-------------+
|      SOF      | Msg_Type | CLL_ID | COP_ID | Status | Payload_Len |    Payload    |   CRC-16    |
|   2 B (<H)    |   1 B    |  1 B   |  1 B   |  1 B   |  2 B (<H)   |  0 .. 4103 B  |  2 B (<H)   |
+---------------+----------+--------+--------+--------+-------------+---------------+-------------+
| 0xAA55 (55AA) |  opcode  | 0..8   | 0..255 | 0x00.. |      N      |    N bytes    | CCITT-FALSE |
+---------------+----------+--------+--------+--------+-------------+---------------+-------------+
|<------------------------------- 8-Byte Header ------------------->|<-- N Bytes -->|<- 2 Bytes ->|
```

- **`SOF` (`2B LE`):** `0xAA55` (`0x55` followed by `0xAA` on the wire).
- **`Msg_Type` (`1B`):**
  - `0x01 .. 0x7F`: Host → VCI Command Request
  - `0x80`: VCI → Host Synchronous Command ACK/NACK (`< 20 ms`)
  - `0x81`: VCI → Host Asynchronous `PDU_IT_RESULT` Event Item
  - `0x82`: VCI → Host Asynchronous `PDU_IT_STATUS` Event Item
  - `0x83`: VCI → Host Asynchronous `PDU_IT_ERROR` Event Item
- **`CLL_ID` (`1B`):** Target ComLogicalLink Identifier (`0x00` = Module-level, `0x01 .. 0x08` = Logical Link 1..8).
- **`COP_ID` (`1B`):** Target ComPrimitive Identifier (`0x00` = Link/Module-level, `0x01 .. 0xFF` = Active Primitive).
- **`Status` (`1B`):** Set to `0x00` in Host requests; carries synchronous D-PDU return status (`PDU_STATUS_NOERROR`, `PDU_ERR_COMM_PC_TO_VCI_OVERRUN`, etc.) in `0x80` responses.
- **`Payload_Len` (`2B LE`):** Length of `Payload` in bytes (`0 .. 4,103`).
- **`CRC-16` (`2B LE`):** **CRC-16/CCITT-FALSE** (`poly = 0x1021`, `init = 0xFFFF`, `refin = false`, `refout = false`, `xorout = 0x0000`) computed from **`Msg_Type` (byte offset 2) through the end of `Payload`**.

---

## 3. Opcodes, IOCTLs, Status Codes & Error Codes

### 3.1 Host → VCI Command `Msg_Type`s (`0x01 .. 0x08`)
| `Msg_Type` | Name | `CLL_ID` | Request Payload Format | Description |
| :-: | :--- | :-: | :--- | :--- |
| `0x01` | `PDU_CMD_CONNECT` | `1..8` | `[bus_mode:1B]` *(or `[bus_mode:1B][baud_khz:2B]`)* | Transition CLL to `PDU_CLLST_ONLINE`. First CLL online locks `CP_Baudrate`; conflicting baudrate on subsequent CLLs is rejected with `0x05`. `bus_mode`: `0x00` = Virtual ECU Loopback, `0x01` = Physical CAN (`GPIO 21` TX / `GPIO 22` RX). |
| `0x02` | `PDU_CMD_DISCONNECT` | `1..8` or `0` | *(Empty — 0 bytes)* | Transition CLL to `PDU_CLLST_OFFLINE` (or `CLL_ID=0` for full `PDUDestruct`). |
| `0x03` | `PDU_CMD_SET_COMPARAM` | `1..8` | `[Param_ID:2B (<H)][Param_Val:4B (<I)]` | Set a 32-bit ComParam on `CLL_ID`. |
| `0x04` | `PDU_CMD_START_COP` | `1..8` | `[NumSendCycles:1B][NumRecvCycles:1B][CycleTime_ms:2B (<H)][CAN_ID:4B (<I)][N-PDU:1..4095B]` | Enqueue a `SEND_ONLY` (`NumRecvCycles=0`), `SENDRECV` (`NumSendCycles=1, NumRecvCycles>=1`), or Cyclic (`NumSendCycles>1` or `0xFF`, `CycleTime_ms>0`) ComPrimitive into the CLL's 4-slot FIFO queue. |
| `0x05` | `PDU_CMD_CANCEL_COP` | `1..8` or `0` | *(Empty — 0 bytes)* | Out-of-band priority cancellation of a specific `COP_ID` (or all primitives on `CLL_ID` when `COP_ID=0`). |
| `0x06` | `PDU_CMD_GET_STATUS` | `1..8` or `0` | *(Empty — 0 bytes)* | Query VCI status (`[online_mask:1B][bus_mode:1B][baud_khz:2B][rx_cmd_cnt:4B][crc_err_cnt:2B][uptime_ms:4B][can_req_id:2B]`). |
| `0x07` | `PDU_CMD_GET_COMPARAM` | `1..8` | `[Param_ID:2B (<H)]` | Read a 32-bit ComParam (`[Param_ID:2B][Param_Val:4B]`). |
| `0x08` | `PDU_CMD_MOD_CTRL` | `0` | `[IOCTL_ID:2B (<H)]` | Execute a module-level IOCTL or `PDUGetVersion`. |

### 3.2 Module IOCTLs (`PDU_CMD_MOD_CTRL = 0x08`, `CLL_ID = 0x00`)
| `IOCTL_ID` | Name | `0x80` Response Payload | Description |
| :-: | :--- | :--- | :--- |
| `0x0001` | `PDU_IOCTL_READ_VBATT` | `[VBatt_mV:4B (<I)]` | Returns Pin 16 vehicle battery voltage in millivolts (`12600` mV nominal). |
| `0x0002` | `PDU_IOCTL_GET_CABLE_ID` | `[Cable_ID:4B (<I)]` | Hardcoded to SAE J1962 16-pin DLC (`0x000007AA` / `1962`). |
| `0x0003` | `PDU_IOCTL_GET_VERSION` | `[FW_Ver:4B][HW_Ver:4B][Serial_MAC:8B]` | Returns firmware version (`0x00020100`), hardware version (`0x00010000`), and 6-byte factory eFuse MAC serial. |
| *Other* | *Unsupported IOCTL* | *(0 bytes)* | Immediately rejected with `Status = 0x04` (`PDU_ERR_FCT_NOT_SUPPORTED`). |

### 3.3 VCI → Host Packet `Msg_Type`s (`0x80 .. 0x83`)
| `Msg_Type` | Name | Payload Format | Description |
| :-: | :--- | :--- | :--- |
| `0x80` | `PDU_EVT_CMD_ACK` | `0B` *(or `4B`/`6B`/`16B` for queries)* | Synchronous Command ACK/NACK (`Status` in Header Byte 5). Emitted within `< 20 ms`. |
| `0x81` | `PDU_IT_RESULT` | `[RX_Timestamp_us:4B (<I)][CAN_ID:4B (<I)][Extra_Addr_Info:1B][Reassembled N-PDU:1..4095B]` | Asynchronous complete diagnostic response PDU with `100 µs` hardware timestamp resolution. |
| `0x82` | `PDU_IT_STATUS` | `[Event_Timestamp_us:4B (<I)][D-PDU_Status_Code:2B (<H)]` | Asynchronous state transition (`PDU_COPST_EXECUTING = 0x8002`, `PDU_COPST_FINISHED = 0x8003`, `PDU_COPST_CANCELLED = 0x8004`, `PDU_CLLST_OFFLINE = 0x8010`, `PDU_CLLST_ONLINE = 0x8011`). |
| `0x83` | `PDU_IT_ERROR` | `[Error_Timestamp_us:4B (<I)][D-PDU_Error_Code:2B (<H)][Vendor_SubCode:2B (<H)]` | Asynchronous bus, timing, or framing error event with `100 µs` hardware timestamp resolution. |

### 3.4 Synchronous `0x80` Status Codes & Asynchronous `0x83` Error Codes
- **`0x80` Synchronous `Status` Codes (Header Byte 5):**
  - `0x00` `PDU_STATUS_NOERROR` — Command accepted
  - `0x01` `PDU_STATUS_INVALID_ARG` — Out-of-range parameter or malformed payload
  - `0x02` `PDU_STATUS_NOT_CONN` — `PDU_CMD_START_COP` sent while target `CLL_ID` is offline
  - `0x03` `PDU_ERR_COMM_PC_TO_VCI_OVERRUN` — Target CLL's 4-slot ComPrimitive queue is full
  - `0x04` `PDU_ERR_FCT_NOT_SUPPORTED` — Unsupported protocol, `Msg_Type`, ComParam, or IOCTL
  - `0x05` `PDU_ERR_BAUDRATE_CONFLICT` — Attempted to connect or change `CP_Baudrate` to a rate conflicting with another online CLL
- **`0x83` `PDU_IT_ERROR` Codes (`D-PDU_Error_Code` / `Vendor_SubCode`):**
  - `D-PDU_Error_Code`: `0x0001` (`PDU_ERR_EVT_RX_TIMEOUT`), `0x0002` (`PDU_ERR_EVT_PROT_ERR`), `0x0003` (`PDU_ERR_EVT_TX_ERROR`), `0x0004` (`PDU_ERR_EVT_RX_ERROR`)
  - `Vendor_SubCode`: `0x0001` (`PDU_ERR_P2_TIMEOUT`), `0x0002` (`PDU_ERR_ISOTP_TIMEOUT`), `0x0003` (`PDU_ERR_CAN_BUS_OFF`), `0x0004` (`PDU_ERR_CRC_MISMATCH`), `0x0005` (`PDU_ERR_MALFORMED_FRAME`), `0x0006` (`PDU_ERR_CANCELLED`)

---

## 4. ComParam Table (ISO 22900-2 Annex B Aligned)

All ComParams use 16-bit `Param_ID` (`<H`) and 32-bit `Param_Value` (`<I`) in `PDU_CMD_SET_COMPARAM` (`0x03`) and `PDU_CMD_GET_COMPARAM` (`0x07`):

| `Param_ID` | ComParam Name | Default | Valid Range / Units | Description |
| :-: | :--- | :--- | :--- | :--- |
| `0x0001` | `CP_BAUDRATE` | `500000` | `125000`, `250000`, `500000`, `1000000` bps | CAN Classic bus bit rate |
| `0x0002` | `CP_CAN_PHYS_REQ_ID` | `0x7DF` | 11-bit (`0x7DF`, `0x7E0..0x7E7`) or 29-bit (`0x18DB33F1`, `0x18DAxxF1`) | Outgoing Tester CAN ID |
| `0x0003` | `CP_CAN_RESP_USDT_ID` | `0x7E8` | 11-bit (`0x7E8..0x7EF`) or 29-bit (`0x18DAF1xx`) | Expected ECU response CAN ID |
| `0x0004` | `CP_ADDRESSING_TYPE` | `0` | `0` = `FUNCTIONAL`, `1` = `PHYSICAL` | Addressing mode (`P3_Func` vs `P3_Phys` & FC routing) |
| `0x0005` | `CP_CAN_FILLER_BYTE` | `0xCC` | `0x00 .. 0xFF` | Padding byte for unused CAN frame bytes (`DLC = 8`) |
| `0x0006` | `CP_BLOCK_SIZE` | `0` | `0 .. 255` (`0` = continuous) | ISO-TP Flow Control Block Size (`BS`) |
| `0x0007` | `CP_ST_MIN` | `0` | `0x00..0x7F` (`0..127 ms`) or `0xF1..0xF9` (`100..900 µs`) | ISO-TP Flow Control Separation Time Min (`STmin`, `100 µs` resolution) |
| `0x0008` | `CP_N_BS_MS` | `1000` | `1 .. 60000` ms | ISO-TP Sender timeout waiting for Flow Control (`N_Bs`) |
| `0x0009` | `CP_N_CR_MS` | `1000` | `1 .. 60000` ms | ISO-TP Receiver timeout waiting for Consecutive Frame (`N_Cr`) |
| `0x000A` | `CP_P2_MAX_MS` | `50` | `1 .. 60000` ms | Max time from request TX to initial ECU response |
| `0x000B` | `CP_P2_STAR_MS` | `5000` | `1 .. 60000` ms | Extended timeout when ECU replies `0x7F [SID] 0x78` |
| `0x000C` | `CP_P3_PHYS_MS` | `5` | `0 .. 60000` ms | Minimum inter-request gap after a Physical response |
| `0x000D` | `CP_P3_FUNC_MS` | `50` | `0 .. 60000` ms | Minimum inter-request window for Functional requests |

---

## 5. Multi-CLL Architecture, Buffer Pool & 4-Slot Queue Behavior

- **8 Concurrent ComLogicalLinks (`CLL_ID = 1 .. 8`):**
  - Up to 8 independent logical links can be open concurrently sharing the physical CAN transceiver (e.g. `CLL 1` for functional broadcast `0x7DF`, `CLL 2` for ECM `0x7E0 -> 0x7E8`, `CLL 3` for TCM `0x7E1 -> 0x7E9`).
  - Core 1 arbitrates ready ComPrimitives across all 8 CLLs using a **Round-Robin Scheduler**, respecting each CLL's `P3_Phys` / `P3_Func` and `CycleTime_ms` deadlines.
- **Per-CLL `8-Byte` Single-Frame Buffer + Shared `3 × 4,095-Byte` Multi-Frame Pool:**
  - Each of the 8 CLLs owns a dedicated static **`8-byte` Single-Frame (`SF`) buffer** (`sf_rx_buf[8]`) for fast single-frame responses (`<= 7B` N-PDU).
  - A shared pool of **`3 × 4,095-byte` Multi-Frame (`MF`) buffers** (`s_mf_pool[3]`) dynamically services concurrent segmented ISO 15765-2 (`FF`/`FC`/`CF`) transfers up to the full 12-bit ISO-TP limit (`4,095` bytes).
- **4-Slot ComPrimitive FIFO Queue per CLL & Overrun NACK (`0x03`):**
  - Each CLL maintains a **4-slot FIFO queue** (`cop_queue[4]`).
  - When a `5th` `StartComPrimitive` is sent to a CLL whose 4 slots are full, Core 0 immediately rejects it with a synchronous `0x80` NACK (`Status = PDU_ERR_COMM_PC_TO_VCI_OVERRUN = 0x03`).
- **Out-of-Band Priority `CancelComPrimitive` (`0x05`):**
  - Marks the target `COP_ID` (or all 4 slots on `CLL_ID` when `COP_ID = 0`) for immediate cancellation, aborts any in-flight CAN/ISO-TP wait within `<= 5 ms`, and emits `0x82` `PDU_IT_STATUS(PDU_COPST_CANCELLED = 0x8004)` for every cancelled primitive.

> **Reference Implementation:** See [`pc_app/dpdu_client.py`](file:///home/abey/Documents/obd_scanner/pc_app/dpdu_client.py) and [`pc_app/obd_scanner_app.py`](file:///home/abey/Documents/obd_scanner/pc_app/obd_scanner_app.py) for complete Python Serial and `bleak` Bluetooth LE client implementations.

---

## 6. ESP32-OBD-VCI Memory Footprint

Measured from the compiled ESP-IDF 6.1.0 release ELF/object files (`.pio/build/esp32dev/`):

### 6.1 Core VCI & ISO 22900-2 App-Device IF Stack Breakdown
| Firmware Module | Source File | Flash (`.text` + `.data`) | Static RAM (`.bss` + `.data`) | Responsibility |
| :--- | :--- | ---: | ---: | :--- |
| **D-PDU App-Device IF Engine** | [`src/dpdu_protocol.c`](file:///home/abey/Documents/obd_scanner/src/dpdu_protocol.c) | **7,896 B** (`7.71 KB`) | **273 B** (`0.27 KB`) | `0xAA55` parser, CRC-16, IOCTLs, 8-CLL Round-Robin, Cyclic COP, `100 µs` timestamps |
| **ISO 15765-2 (ISO-TP + Pool)** | [`src/isotp.c`](file:///home/abey/Documents/obd_scanner/src/isotp.c) | **2,134 B** (`2.08 KB`) | **12,372 B** (`12.08 KB`) | `3 × 4,095B` MF pool + 8B SF binding + `100 µs` `STmin` (`0xF1..0xF9`) |
| **CAN Classic HAL** | [`src/can_hal.c`](file:///home/abey/Documents/obd_scanner/src/can_hal.c) | **1,234 B** (`1.21 KB`) | **11 B** | ESP32 TWAI driver wrapper & loopback queues |
| **BLE GATT NUS Server** | [`src/ble_nus.c`](file:///home/abey/Documents/obd_scanner/src/ble_nus.c) | **2,541 B** (`2.48 KB`) | **110 B** | NimBLE peripheral, NUS & link-loss disconnect hook |
| **Dual-Core Host I/O & 8-CLL State** | [`src/main.c`](file:///home/abey/Documents/obd_scanner/src/main.c) | **1,944 B** (`1.90 KB`) | **4,475 B** (`4.37 KB`) | 8-CLL × 4-slot state (`s_vci_state`), UART/BLE parsers & RTOS tasks |
| **Status LED Driver** | [`src/led_status.c`](file:///home/abey/Documents/obd_scanner/src/led_status.c) | **377 B** (`0.37 KB`) | **12 B** | `GPIO 2` state indicator |
| **Virtual Multi-ECU Simulator** | [`src/ecu_sim.c`](file:///home/abey/Documents/obd_scanner/src/ecu_sim.c) | **2,885 B** (`2.82 KB`) | **543 B** (`0.53 KB`) | Multi-ECU (`0x7E8..0x7EF` & 29-bit) OBD-II/UDS simulator |
| **Total Custom VCI Stack** | *(All 7 modules above)* | **19,011 B (`18.57 KB`)** | **17,796 B (`17.38 KB`)** | **Complete 8-CLL VCI + D-PDU + `3 × 4,095B` ISO-TP + Multi-ECU Sim** |

### 6.2 Total ESP32 Firmware Image (Including ESP-IDF + FreeRTOS + NimBLE BLE Stack)
- **Total Firmware Image (with ESP-IDF RTOS + NimBLE Bluetooth LE Controller/Host):**
  - **Flash Used:** `499,773 bytes` (**`488.1 KB`**, or **`47.7%`** of the 1 MB factory app partition on the 4 MB flash).
  - **Static RAM Used:** `44,908 bytes` (**`43.86 KB`**, or **`13.7%`** of the 320 KB ESP32 DRAM).
