# ESP32-OBD-VCI — Host Interface Specification (Lean D-PDU API)

This document specifies the transport, binary wire framing, command/event opcodes, and ComParam configuration required for a PC or mobile diagnostic application to communicate with the **ESP32-OBD-VCI**.

---

## 1. Host Transport Interfaces

Both transports carry the **exact same binary Lean D-PDU frames**. No ASCII/AT adapter is used.

### 1.1 Bluetooth LE (Nordic UART Service — NUS)
| Parameter | Value |
| :--- | :--- |
| **BLE Advertised Name** | `ESP32-OBD-VCI` |
| **NUS Service UUID** | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| **RX Characteristic UUID (Host → VCI)** | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` (`Write` / `Write Without Response`) |
| **TX Characteristic UUID (VCI → Host)** | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` (`Notify` — subscribe to CCCD on connect) |
| **Preferred ATT MTU** | `256` bytes (payloads larger than `MTU - 3` are fragmented automatically; reassemble via the stream parser) |

### 1.2 USB-UART (Wired Fallback)
| Parameter | Value |
| :--- | :--- |
| **Serial Configuration** | `115200` baud, `8N1`, no hardware flow control (`RTS=False`, `DTR=False`) |

---

## 2. Binary Wire Frame Format

All multi-byte integers on the wire are **Little-Endian** (`<H` for `uint16`, `<I` for `uint32`).

```text
+------+------+--------+--------+-----------+-----------+---------------+-------------+
| SOF  | Ver  | Opcode | Handle |  SeqTag   |  Length   |    Payload    |   CRC-16    |
| 1 B  | 1 B  |  1 B   |  1 B   | 2 B (<H)  | 2 B (<H)  |  0 .. 512 B   |  2 B (<H)   |
+------+------+--------+--------+-----------+-----------+---------------+-------------+
| 0xA5 | 0x01 |  cmd   |  0x01  | 0x0001..  |   N       |   N bytes     | CCITT-FALSE |
+------+------+--------+--------+-----------+-----------+---------------+-------------+
|<----------------------- 8-Byte Header --------------->|<-- N Bytes -->|<- 2 Bytes ->|
```

- **`SOF` (`1B`):** Always `0xA5`.
- **`Ver` (`1B`):** Protocol version, always `0x01`.
- **`Opcode` (`1B`):** Command (`0x01..0x07`) or Event (`0x80..0x82`).
- **`Handle` (`1B`):** Logical Link handle (`0x01`).
- **`SeqTag` (`2B LE`):** Host sequence counter echoed in the VCI's `PDU_EVT_CMD_ACK`, `PDU_EVT_RESULT`, and `PDU_EVT_ERROR` responses.
- **`Length` (`2B LE`):** Payload byte length (`0..512`).
- **`CRC-16` (`2B LE`):** **CRC-16/CCITT-FALSE** (`poly = 0x1021`, `init = 0xFFFF`, `refin = false`, `refout = false`, `xorout = 0x0000`) computed over the **8-byte Header + Payload**.

---

## 3. Opcodes, Status Codes & Error Codes

### 3.1 Host → VCI Command Opcodes
| Opcode | Name | Request Payload Format | Description |
| :-: | :--- | :--- | :--- |
| `0x01` | `PDU_CMD_CONNECT` | `[bus_mode:1B]` *(or `[bus_mode:1B][baud_khz:2B][flags:1B]`)* | Open Logical Link using `CP_BAUDRATE`. `bus_mode`: `0x00` = Virtual ECU Loopback, `0x01` = Physical CAN (`GPIO 21` TX / `GPIO 22` RX). |
| `0x02` | `PDU_CMD_DISCONNECT` | *(Empty — 0 bytes)* | Close Logical Link and stop CAN controller. |
| `0x03` | `PDU_CMD_SET_COMPARAM` | `[param_id:1B][param_val:4B (<I)]` | Set a ComParam in the VCI ComParam table. |
| `0x04` | `PDU_CMD_START_COP` | `[cop_type:1B][flags:1B][pdu:1..510B]` | Start a ComPrimitive. `cop_type`: `0x01` = `SENDRECV`, `0x02` = `SEND_ONLY`. `pdu` is the raw OBD-II/UDS service request (e.g., `01 0C` or `09 02`). |
| `0x05` | `PDU_CMD_CANCEL_COP` | *(Empty — 0 bytes)* | Abort an in-flight ComPrimitive immediately. |
| `0x06` | `PDU_CMD_GET_STATUS` | *(Empty — 0 bytes)* | Query VCI status (`connected`, `bus_mode`, `baud_khz`, `rx_cmd_count`, `crc_err_count`, `uptime_ms`, `can_req_id`). |
| `0x07` | `PDU_CMD_GET_COMPARAM` | `[param_id:1B]` | Read a 32-bit ComParam value from the VCI. |

### 3.2 VCI → Host Event Opcodes
| Opcode | Name | Event Payload Format | Description |
| :-: | :--- | :--- | :--- |
| `0x80` | `PDU_EVT_CMD_ACK` | `[orig_opcode:1B][status:1B][extra...]` | Immediate command acknowledgment. `status = 0x00` (`PDU_STATUS_NOERROR`) on success. |
| `0x81` | `PDU_EVT_RESULT` | `[rx_can_id:4B (<I)][rx_time_ms:2B (<H)][pdu:N B]` | Asynchronous ECU response after ISO-TP Single-Frame or Multi-Frame reassembly. |
| `0x82` | `PDU_EVT_ERROR` | `[err_code:1B][detail:1B]` | Asynchronous bus, timing, or framing error event. |

### 3.3 Status & Error Codes
- **`PDU_EVT_CMD_ACK` Status Codes:**
  - `0x00` `PDU_STATUS_NOERROR` — Command accepted
  - `0x01` `PDU_STATUS_INVALID_ARG` — Out-of-range parameter or malformed payload
  - `0x02` `PDU_STATUS_NOT_CONN` — `PDU_CMD_START_COP` sent while Logical Link is disconnected
  - `0x03` `PDU_STATUS_BUSY` — VCI busy
  - `0x04` `PDU_STATUS_UNSUPPORTED` — Unknown command opcode
- **`PDU_EVT_ERROR` Error Codes:**
  - `0x01` `PDU_ERR_P2_TIMEOUT` — No ECU response within `CP_P2_MAX_MS` (or `CP_P2_STAR_MS`)
  - `0x02` `PDU_ERR_ISOTP_TIMEOUT` — ISO 15765-2 Multi-Frame `CP_N_BS_MS` or `CP_N_CR_MS` timeout
  - `0x03` `PDU_ERR_CAN_BUS_OFF` — CAN transmit / bus-off error
  - `0x04` `PDU_ERR_CRC_MISMATCH` — Host command frame failed CRC-16 check
  - `0x05` `PDU_ERR_MALFORMED_FRAME` — Invalid header or ISO-TP sequence number (`SN`) error
  - `0x06` `PDU_ERR_CANCELLED` — Active ComPrimitive aborted via `PDU_CMD_CANCEL_COP`

---

## 4. ComParam Table (ISO 22900-2 Aligned)

All ComParams use `uint32` (`<I`) values in `PDU_CMD_SET_COMPARAM` (`0x03`) and `PDU_CMD_GET_COMPARAM` (`0x07`):

| ID | ComParam Name | Default | Valid Range / Units | Description |
| :-: | :--- | :--- | :--- | :--- |
| `0x01` | `CP_BAUDRATE` | `500000` | `125000`, `250000`, `500000`, `1000000` bps | CAN Classic bus bit rate |
| `0x02` | `CP_CAN_PHYS_REQ_ID` | `0x7DF` | 11-bit (`0x7DF`, `0x7E0`) or 29-bit (`0x18DB33F1`, `0x18DAxxF1`) | Outgoing Tester CAN ID |
| `0x03` | `CP_CAN_RESP_USDT_ID` | `0x7E8` | 11-bit (`0x7E8`) or 29-bit (`0x18DAF110`) | Expected ECU response CAN ID |
| `0x04` | `CP_ADDRESSING_TYPE` | `0` | `0` = `FUNCTIONAL`, `1` = `PHYSICAL` | Addressing mode (`P3_Func` vs `P3_Phys` & FC routing) |
| `0x05` | `CP_CAN_FILLER_BYTE` | `0xCC` | `0x00 .. 0xFF` | Padding byte for unused CAN frame bytes (`DLC = 8`) |
| `0x06` | `CP_BLOCK_SIZE` | `0` | `0 .. 255` (`0` = continuous) | ISO-TP Flow Control Block Size (`BS`) |
| `0x07` | `CP_ST_MIN` | `0` | `0 .. 127` ms or `0xF1 .. 0xF9` (`100..900 us`) | ISO-TP Flow Control Separation Time Min (`STmin`) |
| `0x08` | `CP_N_BS_MS` | `1000` | `1 .. 60000` ms | ISO-TP Sender timeout waiting for Flow Control (`N_Bs`) |
| `0x09` | `CP_N_CR_MS` | `1000` | `1 .. 60000` ms | ISO-TP Receiver timeout waiting for Consecutive Frame (`N_Cr`) |
| `0x0A` | `CP_P2_MAX_MS` | `50` | `1 .. 60000` ms | Max time from request TX to initial ECU response |
| `0x0B` | `CP_P2_STAR_MS` | `5000` | `1 .. 60000` ms | Extended timeout when ECU replies `0x7F [SID] 0x78` |
| `0x0C` | `CP_P3_PHYS_MS` | `5` | `0 .. 60000` ms | Minimum inter-request gap after a Physical response |
| `0x0D` | `CP_P3_FUNC_MS` | `50` | `0 .. 60000` ms | Minimum inter-request window for Functional requests |

---

## 5. Typical Session Flow & Wire Examples

1. **Configure Baud Rate (Optional — defaults to 500 kbps):**
   - Send `PDU_CMD_SET_COMPARAM` (`0x03`) with `param_id = 0x01` (`CP_BAUDRATE`), `param_val = 500000`.
   - Receive `PDU_EVT_CMD_ACK` (`0x80`).
2. **Connect Logical Link:**
   - Send `PDU_CMD_CONNECT` (`0x01`) with payload `01` (`VCI_BUS_MODE_PHYSICAL_CAN`) or `00` (`VCI_BUS_MODE_SIM_LOOPBACK`).
   - Receive `PDU_EVT_CMD_ACK` (`0x80`).
3. **Request OBD-II Engine RPM (Mode `01` PID `0C`):**
   - **Host TX (`PDU_CMD_START_COP`, `seq_tag = 1`, payload = `01 00 01 0C`):**
     `A5 01 04 01 01 00 04 00 01 00 01 0C 6C 6B`
   - **VCI RX #1 (`PDU_EVT_CMD_ACK` — Command Accepted):**
     `A5 01 80 01 01 00 02 00 04 00 84 C6`
   - **VCI RX #2 (`PDU_EVT_RESULT` — `CAN_ID = 0x7E8`, `1 ms`, `PDU = 41 0C 1A F8` = `1726.0 RPM`):**
     `A5 01 81 01 01 00 0A 00 E8 07 00 00 01 00 41 0C 1A F8 D3 29`
4. **Request 17-Character VIN (Mode `09` PID `02` — Multi-Frame ISO-TP handled automatically by VCI):**
   - Send `PDU_CMD_START_COP` (`0x04`) with payload `01 00 09 02`.
   - VCI transmits `02 09 02 CC CC CC CC CC`, receives First Frame (`FF`), automatically transmits physical Flow Control (`30 00 00 CC CC CC CC CC`), reassembles all Consecutive Frames (`CF`), and emits **one** `PDU_EVT_RESULT` (`0x81`) containing the 20-byte PDU `49 02 01 <17-byte ASCII VIN>`.

> **Reference Implementation:** See [`pc_app/dpdu_client.py`](file:///home/abey/Documents/obd_scanner/pc_app/dpdu_client.py) and [`pc_app/obd_scanner_app.py`](file:///home/abey/Documents/obd_scanner/pc_app/obd_scanner_app.py) for complete Python Serial and `bleak` Bluetooth LE client implementations.

---

## 6. ESP32-OBD-VCI Memory Footprint

Measured from the compiled ESP-IDF 6.1.0 release ELF/object files (`.pio/build/esp32dev/`):

### 6.1 Core VCI & Lean D-PDU Stack Breakdown
| Firmware Module | Source File | Flash (`.text` + `.data`) | Static RAM (`.bss` + `.data`) | Responsibility |
| :--- | :--- | ---: | ---: | :--- |
| **Lean D-PDU Engine** | [`src/dpdu_protocol.c`](file:///home/abey/Documents/obd_scanner/src/dpdu_protocol.c) | **4,081 B** (`3.99 KB`) | **2,090 B** (`2.04 KB`) | Binary parser, CRC-16, 13 ComParams, `P2`/`P2*`/`P3` timers |
| **ISO 15765-2 (ISO-TP)** | [`src/isotp.c`](file:///home/abey/Documents/obd_scanner/src/isotp.c) | **1,550 B** (`1.51 KB`) | **0 B** *(in `dpdu` ctx)* | Bidirectional `SF`/`FF`/`FC`/`CF` + 11/29-bit addressing |
| **CAN Classic HAL** | [`src/can_hal.c`](file:///home/abey/Documents/obd_scanner/src/can_hal.c) | **1,234 B** (`1.21 KB`) | **11 B** | ESP32 TWAI driver wrapper & loopback queues |
| **BLE GATT NUS Server** | [`src/ble_nus.c`](file:///home/abey/Documents/obd_scanner/src/ble_nus.c) | **2,513 B** (`2.45 KB`) | **106 B** | NimBLE peripheral & Nordic UART Service |
| **Dual-Core Host I/O & RT** | [`src/main.c`](file:///home/abey/Documents/obd_scanner/src/main.c) | **2,323 B** (`2.27 KB`) | **2,846 B** (`2.78 KB`) | FreeRTOS task isolation & UART0/BLE mux |
| **Status LED Driver** | [`src/led_status.c`](file:///home/abey/Documents/obd_scanner/src/led_status.c) | **377 B** (`0.37 KB`) | **12 B** | `GPIO 2` state indicator |
| **Virtual ECU Simulator** | [`src/ecu_sim.c`](file:///home/abey/Documents/obd_scanner/src/ecu_sim.c) | **2,757 B** (`2.69 KB`) | **543 B** (`0.53 KB`) | On-chip OBD-II/UDS simulator for loopback test |
| **Total Custom VCI Stack** | *(All 7 modules above)* | **14,835 B (`14.49 KB`)** | **5,608 B (`5.48 KB`)** | **Complete VCI + D-PDU + ISO-TP + Sim** |

### 6.2 Total ESP32 Firmware Image (Including ESP-IDF + FreeRTOS + NimBLE BLE Stack)
- **Pure Lean D-PDU + ISO-TP Core (`dpdu_protocol.c` + `isotp.c`):** **`5.50 KB` Flash** and **`2.04 KB` static RAM**.
- **Total Firmware Image (with ESP-IDF RTOS + NimBLE Bluetooth LE Controller/Host):**
  - **Flash Used:** `496,269 bytes` (**`484.6 KB`**, or **`47.3%`** of the 1 MB factory app partition on the 4 MB flash).
  - **Static RAM Used:** `32,724 bytes` (**`31.96 KB`**, or **`10.0%`** of the 320 KB ESP32 DRAM, leaving **~`288 KB`** of free RAM/heap).
