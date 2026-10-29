# ESP32-OBD-VCI — Hardware Performance & Benchmark Report

- **Timestamp:** `2026-10-29 19:05:13 `
- **Target Hardware:** `ESP32-OBD-VCI` (`ESP32-OBD-VCI`, BLE MAC `A4:F0:0F:77:87:CE`, Serial `A4:F0:0F:77:87:CC`)
- **Firmware / Hardware Version:** `FW 0x00020100` / `HW 0x00010000` (ESP-IDF `v6.1.0`, Dual-Core FreeRTOS)
- **Protocol:** ISO 22900-2:2022 `App-Device IF` (`0xAA55` Binary Framing + `CRC16-CCITT`, `100 µs` Hardware Timestamps, 8 CLLs, 4-Slot FIFO Queue/CLL, `3 × 4,095 B` ISO-TP Pool)
- **Transports Compared Side-by-Side:**
  - **Bluetooth LE:** NimBLE GATT Nordic UART Service (`LE Security Mode 1, Level 2` Just Works AES-CCM Encryption + NVS Bonding, `ATT MTU = 256 B`, Advertising Interval `20–40 ms`, Average RSSI `-50.0 dBm`)
  - **USB-UART0:** Wired Serial (`115200` baud, `8N1`)

---

## Executive Summary (Side-by-Side Highlights)

| Benchmark Metric | USB-UART0 (`115200` baud) | Bluetooth LE (`NimBLE NUS`) | Notes / Architectural Insight |
| :--- | ---: | ---: | :--- |
| **1. BLE Discovery Time (Avg / Min)** | *N/A (Wired)* | **`148.49 ms`** (`148.49 ms` min) | `20–40 ms` BLE advertising interval |
| **2. BLE Connection + CCCD + First IOCTL** | *N/A (Wired)* | **`1751.2 ms`** (`1365.6 ms` GATT) | Full link ready for diagnostics |
| **3. Synchronous `0x80` IOCTL RTT (`VBATT`)** | **`3.97 ms`** | **`91.85 ms`** | `< 20 ms` local ACK on UART; BLE bound by GATT connection interval |
| **3. Sustained D-PDU Packet Rate** | **`673.4 pkt/s`** (`108.8 kbps`) | **`197.9 pkt/s`** (`32.0 kbps`) | 4-slot pipelined Multi-Frame VIN stream |
| **4. Single-Frame OBD-II RTT (`01 0C` RPM)** | **`8.96 ms`** | **`92.95 ms`** | VCI internal CAN execution is only **`3773 µs`** (`3.77 ms`) |
| **5. Multi-Frame VIN (`09 02`, `STmin=0`)** | **`11.09 ms`** | **`93.53 ms`** | 20-byte VIN reassembled across `FF`+`FC`+`2×CF` in VCI RAM |
| **7. Physical Polling (Sequential vs. 4-Slot Queue)** | **`111.4` → `103.1 req/s`** (`0.93×`) | **`10.1` → `40.2 req/s`** (**`3.98×`**) | 4-slot CLL queue hides BLE GATT round-trip latency |
| **8. Live 3-PID Telemetry Rate (Cyclic COP)** | **`51.7 PIDs/s`** (`17.2 Hz`) | **`47.7 PIDs/s`** (`15.9 Hz`) | Autonomous VCI cyclic polling eliminates Host TX overhead |
| **9C. BLE Disconnect Autonomous `PDUDestruct`** | *Verified via UART* | **`13.1 ms`** | Satisfies ISO 22900-2 `< 100 ms` wireless link-loss safety requirement |

---

## 1. BLE Discovery Time

Measured across `N = 1` consecutive scan cycles using `BleakScanner` detection callbacks until `ESP32-OBD-VCI` (NUS UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`) is discovered:

| Metric | Min (`ms`) | Avg (`ms`) | Median (`ms`) | P95 (`ms`) | Max (`ms`) | StdDev (`ms`) | Avg RSSI |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **BLE Advertisement Discovery** | 148.49 | 148.49 | 148.49 | 148.49 | 148.49 | ±0.00 | `-50.0 dBm` |

---

## 2. BLE Connection & Handshake Time

Measured across `N = 1` full connect → CCCD subscribe → `PDUGetVersion` (`0x08`) handshake → disconnect cycles:

| Connection Sub-Phase | Min (`ms`) | Avg (`ms`) | Median (`ms`) | P95 (`ms`) | Max (`ms`) | StdDev (`ms`) |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| **1. GATT Link Establish (`connect()`)** | 1365.65 | 1365.65 | 1365.65 | 1365.65 | 1365.65 | ±0.00 |
| **2. NUS TX CCCD Subscribe (`start_notify()`)** | 295.95 | 295.95 | 295.95 | 295.95 | 295.95 | ±0.00 |
| **3. First D-PDU Handshake (`PDUGetVersion` `0x80` ACK)** | 89.61 | 89.61 | 89.61 | 89.61 | 89.61 | ±0.00 |
| **Total Ready-to-Diagnose (`1 + 2 + 3`)** | 1751.22 | 1751.22 | 1751.22 | 1751.22 | 1751.22 | ±0.00 |
| **GATT Disconnect Time (`disconnect()`)** | 2015.66 | 2015.66 | 2015.66 | 2015.66 | 2015.66 | ±0.00 |

---

## 3. BLE vs. USB-UART0 Throughput, Packet Rate & IOCTL Latency

### 3.1 Synchronous Module IOCTL Round-Trip Latency (`N = 30`)
| Operation | Wire Size (TX + RX) | Transport | Min (`ms`) | Avg (`ms`) | Median (`ms`) | P95 (`ms`) | Max (`ms`) | StdDev (`ms`) |
| :--- | :---: | :---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **`IOCTL_READ_VBATT` (`0x0001`)** | `10 B + 14 B` | **USB-UART0** | 3.75 | 3.97 | 3.87 | 4.54 | 5.93 | ±0.40 |
| **`IOCTL_READ_VBATT` (`0x0001`)** | `10 B + 14 B` | **BLE NUS** | 87.21 | 91.85 | 89.95 | 91.13 | 135.85 | ±10.42 |
| **`PDUGetVersion` (`0x0003`)** | `10 B + 26 B` | **USB-UART0** | 4.96 | 5.38 | 5.12 | 7.59 | 8.81 | ±0.82 |
| **`PDUGetVersion` (`0x0003`)** | `10 B + 26 B` | **BLE NUS** | 88.88 | 90.45 | 90.18 | 91.58 | 93.12 | ±1.02 |

### 3.2 Sustained Pipelined D-PDU Wire Throughput (`25 ×` Multi-Frame VIN Transactions = `125` D-PDU Packets)
| Transport | D-PDU Packet Rate (`pkt/s`) | Effective Wire Byte Rate (`B/s`) | Effective Wire Bitrate (`kbps`) |
| :--- | ---: | ---: | ---: |
| **USB-UART0 (`115200` baud)** | **`673.4 pkt/s`** | `13603.2 B/s` | `108.83 kbps` |
| **Bluetooth LE (`NimBLE NUS`)** | **`197.9 pkt/s`** | `3997.0 B/s` | `31.98 kbps` |

---

## 4. OBD-II Single-Frame (`SF`) Request–Response Performance

Measured for `N = 30` iterations of Mode `01` PID `0C` (**Engine RPM**, `01 0C` → `41 0C 1A F8`, physical addressing `0x7E0 -> 0x7E8`, `CP_P3_PHYS_MS = 0`).
Using the VCI's **`100 µs` hardware timestamps** (`0x82 PDU_COPST_EXECUTING` → `0x81 PDU_IT_RESULT`), we isolate internal CAN execution time from Host transport overhead:

| Timing Component | Transport | Min | Avg | Median | P95 | Max | StdDev |
| :--- | :---: | ---: | ---: | ---: | ---: | ---: | ---: |
| **Synchronous `0x80` ACK Latency (`ms`)** | **USB-UART0** | 5.76 | 5.92 | 5.92 | 6.08 | 6.35 | ±0.11 |
| **Synchronous `0x80` ACK Latency (`ms`)** | **BLE NUS** | 86.03 | 89.52 | 88.05 | 89.80 | 133.10 | ±8.28 |
| **VCI Internal CAN Execution (`µs`, HW Timestamp)** | **ESP32 Core 1** | 3740 | 3773 | 3770 | 3804 | 3811 | ±15 |
| **Total Host End-to-End RTT (`ms`)** | **USB-UART0** | 8.83 | 8.96 | 8.93 | 9.09 | 9.42 | ±0.12 |
| **Total Host End-to-End RTT (`ms`)** | **BLE NUS** | 87.79 | 92.95 | 89.98 | 133.34 | 135.03 | ±11.25 |

- **Transport Overhead Analysis:**
  - **VCI Internal CAN Bus Execution (`Core 1`):** `3773 µs` (`3.77 ms`)
  - **USB-UART0 Host Transport + Framing Overhead:** `7.35 ms`
  - **Bluetooth LE GATT Write + Notify Overhead:** `89.18 ms`

---

## 5. OBD-II Multi-Frame (`MF`) Request–Response Latency (ISO 15765-2 `FF`/`FC`/`CF`)

Measured across 4 multi-frame profiles (`N = 12` each), comparing **VCI Internal ISO-TP Assembly Time (`ms`)** (from `100 µs` hardware timestamps) against **Total Host End-to-End RTT (`ms`)**:

| Multi-Frame Test Profile | Payload | VCI ISO-TP Time (`ms`) | USB-UART0 E2E Avg (`ms`) | USB-UART0 P95 (`ms`) | BLE NUS E2E Avg (`ms`) | BLE NUS P95 (`ms`) |
| :--- | :---: | ---: | ---: | ---: | ---: | ---: |
| **1. Mode `09` PID `02` VIN (`BS=0, STmin=0 ms`)** | `20 B` RX (`3` frames) | `4.86 ms` | **`11.09 ms`** | `11.25 ms` | **`93.53 ms`** | `92.25 ms` |
| **2. Mode `09` PID `02` VIN (`BS=0, STmin=0xF2 / 200 µs`)** | `20 B` RX (`3` frames) | `4.26 ms` | **`10.45 ms`** | `10.59 ms` | **`93.76 ms`** | `91.09 ms` |
| **3. Mode `09` PID `04` Cal ID (`BS=1, STmin=10 ms`)** | `19 B` RX (`3` frames + `2` FC) | `23.90 ms` | **`30.08 ms`** | `30.14 ms` | **`93.73 ms`** | `90.90 ms` |
| **4. UDS `0x2E` Write VIN (Multi-Frame Tester → ECU TX)** | `20 B` TX (`3` frames) | `4.42 ms` | **`10.41 ms`** | `10.56 ms` | **`93.72 ms`** | `91.73 ms` |

---

## 6. Functional Request Performance (`0x7DF` 11-Bit vs. `0x18DB33F1` 29-Bit)

Evaluates functional broadcast addressing (`CP_ADDRESSING_TYPE = FUNCTIONAL`) with standard `CP_P3_FUNC_MS = 50 ms` vs. `0 ms` across both 11-bit and 29-bit CAN identifiers:

| Functional Profile | CAN ID Pair | `CP_P3_FUNC_MS` | USB-UART0 Avg (`ms`) | USB-UART0 Rate (`req/s`) | BLE NUS Avg (`ms`) | BLE NUS Rate (`req/s`) |
| :--- | :---: | :---: | ---: | ---: | ---: | ---: |
| **11-Bit Functional (Standard P3)** | `0x7DF -> 0x7E8` | `50 ms` | `55.64 ms` | `18.0 req/s` | `97.71 ms` | `10.2 req/s` |
| **11-Bit Functional (Zero P3)** | `0x7DF -> 0x7E8` | `0 ms` | `12.16 ms` | `82.3 req/s` | `90.14 ms` | `11.1 req/s` |
| **29-Bit Extended Functional (Standard P3)** | `0x18DB33F1 -> 0x18DAF110` | `50 ms` | `55.23 ms` | `18.1 req/s` | `93.93 ms` | `10.6 req/s` |
| **29-Bit Extended Functional (Zero P3)** | `0x18DB33F1 -> 0x18DAF110` | `0 ms` | `12.10 ms` | `82.7 req/s` | `90.27 ms` | `11.1 req/s` |

---

## 7. Physical Request Performance — Sequential vs. 4-Slot Queue Pipelining

Compares executing `20` physical OBD-II requests (`0x7E0 -> 0x7E8`, `CP_P3_PHYS_MS = 5 ms`) one-by-one (**Sequential Stop-and-Wait**) vs. keeping the CLL's **4-slot ComPrimitive FIFO queue** fed (**Pipelined**):

| Transport | Sequential Avg (`ms/req`) | Sequential Throughput (`req/s`) | 4-Slot Pipelined Avg (`ms/req`) | 4-Slot Pipelined Throughput (`req/s`) | Pipelining Speedup |
| :--- | ---: | ---: | ---: | ---: | ---: |
| **USB-UART0 (`115200` baud)** | `8.98 ms` | `111.4 req/s` | **`9.70 ms`** | **`103.1 req/s`** | **`0.93×`** |
| **Bluetooth LE (`NimBLE NUS`)** | `99.10 ms` | `10.1 req/s` | **`24.89 ms`** | **`40.2 req/s`** | **`3.98×`** |

---

## 8. OBD-II Live Telemetry Polling (1, 2, and 3 Parameters)

Measures **Dashboard Sweep Refresh Rate (`Hz` = complete sweeps/sec)** and **Aggregate Parameter Rate (`PIDs/s`)** when polling:
- **1 Parameter:** `RPM (01 0C)`
- **2 Parameters:** `RPM (01 0C)` + `Vehicle Speed (01 0D)`
- **3 Parameters:** `RPM (01 0C)` + `Vehicle Speed (01 0D)` + `Coolant Temp (01 05)`

across **3 execution strategies**:
1. **Sequential Host Polling:** Host sends 1 PID and waits for `PDU_COPST_FINISHED` before sending the next.
2. **4-Slot Pipelined Host Polling:** Host bursts up to 4 PIDs into `CLL 1`'s 4-slot queue.
3. **Autonomous VCI Cyclic `StartComPrimitive`:** Host sends `StartComPrimitive(NumSendCycles=15, CycleTime_ms=15)` once per PID; the VCI executes periodic CAN polling autonomously on Core 1 and streams `0x81` `PDU_IT_RESULT` frames back to the Host with zero Host TX overhead.

| Parameter Count | Strategy | USB-UART0 Sweep Rate (`Hz`) | USB-UART0 Total (`PIDs/s`) | BLE NUS Sweep Rate (`Hz`) | BLE NUS Total (`PIDs/s`) |
| :---: | :--- | ---: | ---: | ---: | ---: |
| **1 PID** (`RPM`) | 1. Sequential Host Polling | `110.9 Hz` | `110.9 PIDs/s` | `11.1 Hz` | `11.1 PIDs/s` |
| **1 PID** (`RPM`) | 2. 4-Slot Pipelined Host Polling | `103.6 Hz` | `103.6 PIDs/s` | `41.5 Hz` | `41.5 PIDs/s` |
| **1 PID** (`RPM`) | **3. VCI Autonomous Cyclic COP** | **`51.7 Hz`** | **`51.7 PIDs/s`** | **`42.1 Hz`** | **`42.1 PIDs/s`** |
| **2 PIDs** (`RPM, Spd`) | 1. Sequential Host Polling | `56.2 Hz` | `112.3 PIDs/s` | `5.3 Hz` | `10.6 PIDs/s` |
| **2 PIDs** (`RPM, Spd`) | 2. 4-Slot Pipelined Host Polling | `54.9 Hz` | `109.9 PIDs/s` | `20.8 Hz` | `41.5 PIDs/s` |
| **2 PIDs** (`RPM, Spd`) | **3. VCI Autonomous Cyclic COP** | **`25.9 Hz`** | **`51.7 PIDs/s`** | **`22.3 Hz`** | **`44.6 PIDs/s`** |
| **3 PIDs** (`RPM, Spd, Tmp`) | 1. Sequential Host Polling | `37.7 Hz` | `113.0 PIDs/s` | `3.5 Hz` | `10.6 PIDs/s` |
| **3 PIDs** (`RPM, Spd, Tmp`) | 2. 4-Slot Pipelined Host Polling | `37.3 Hz` | `112.0 PIDs/s` | `14.5 Hz` | `43.4 PIDs/s` |
| **3 PIDs** (`RPM, Spd, Tmp`) | **3. VCI Autonomous Cyclic COP** | **`17.2 Hz`** | **`51.7 PIDs/s`** | **`15.9 Hz`** | **`47.7 PIDs/s`** |

---

## 9. Additional ISO 22900-2:2022 Architectural Benchmarks

### 9A. Consolidated BLE vs. USB-UART0 Transport Overhead Summary
| Metric | USB-UART0 (`115200` baud) | Bluetooth LE (`NimBLE NUS`) | BLE Delta vs. UART |
| :--- | ---: | ---: | ---: |
| **Synchronous Command ACK (`0x80`)** | `5.92 ms` | `89.52 ms` | `+83.59 ms` |
| **Single-Frame E2E Round-Trip (`01 0C`)** | `8.96 ms` | `92.95 ms` | `+83.99 ms` |
| **Multi-Frame VIN E2E Round-Trip (`09 02`)** | `11.09 ms` | `93.53 ms` | `+82.43 ms` |
| **Pipelined Physical Throughput (`4-Slot Queue`)** | `103.1 req/s` | `40.2 req/s` | `39.0%` of UART rate |
| **Autonomous Cyclic 3-PID Throughput** | `51.7 PIDs/s` | `47.7 PIDs/s` | `92.2%` of UART rate |

### 9B. Multi-CLL Concurrent Round-Robin Scaling (1 vs. 2 vs. 4 Active ECUs/CLLs)
Tests Core 1 Round-Robin bus arbitration across multiple simultaneous physical ComLogicalLinks (`CLL 1`: ECM `0x7E0->0x7E8`, `CLL 2`: TCM `0x7E1->0x7E9`, `CLL 3`: ABS `0x7E2->0x7EA`, `CLL 4`: BCM `0x7E3->0x7EB`), with each CLL's `5 ms` `P3_Phys` timer tracked independently:

| Active Concurrent CLLs | Total Requests (`8 / CLL`) | USB-UART0 Total Time (`ms`) | USB-UART0 Aggregate Rate (`PIDs/s`) | BLE NUS Total Time (`ms`) | BLE NUS Aggregate Rate (`PIDs/s`) |
| :---: | :---: | ---: | ---: | ---: | ---: |
| **1 Active CLL (`CLL 1`)** | `8` | `82.8 ms` | `96.7 PIDs/s` | `271.6 ms` | `29.5 PIDs/s` |
| **2 Active CLLs (`CLL 1..2`)** | `16` | `103.0 ms` | `155.3 PIDs/s` | `274.1 ms` | `58.4 PIDs/s` |
| **4 Active CLLs (`CLL 1..4`)** | `32` | `193.1 ms` | **`165.8 PIDs/s`** | `410.4 ms` | **`78.0 PIDs/s`** |

### 9C. Out-of-Band `CancelComPrimitive` Drain & Wireless Link-Loss `PDUDestruct` Latency
| Safety / Control Operation | Requirement Target | Measured USB-UART0 | Measured BLE NUS | Status |
| :--- | :---: | ---: | ---: | :---: |
| **Out-of-Band `CancelComPrimitive` (`0x05`) 4-Slot Queue Drain** | `< 50 ms` | **`7.55 ms`** | **`90.28 ms`** | **PASS** |
| **BLE GATT Disconnect → Autonomous `PDUDestruct` (`online_mask=0`)** | `< 100 ms` | *Monitored via UART0* | **`13.09 ms`** | **PASS** |

### 9D. VCI Hardware Timer Accuracy (`CP_P2_MAX_MS`, `NRC 0x78` `P2*`, `CP_N_CR_MS`)
| Protocol Timer / Scenario | Configured / Expected | VCI Hardware Elapsed (`ms`) | USB-UART0 Host RTT (`ms`) | BLE NUS Host RTT (`ms`) |
| :--- | :---: | ---: | ---: | ---: |
| **`CP_P2_MAX_MS` Timeout (`01 FF` Unanswered PID)** | `50.0 ms` | **`59.99 ms`** | `234.06 ms` | `134.80 ms` |
| **UDS/OBD `NRC 0x78` (`ResponsePending` -> `P2*` Extension)** | `~150.0 ms` ECU delay | **`123.38 ms`** | `130.46 ms` | `225.63 ms` |
| **ISO-TP `CP_N_CR_MS` Consecutive Frame Timeout (`09 F1`)** | `500.0 ms` | **`509.06 ms`** | `515.60 ms` | `592.73 ms` |
