# ESP32-OBD-VCI — Engineering Lessons Learned & Technical Design Guide

**Document Scope:** Architectural decisions, root-cause failure analyses, memory optimization patterns, real-time dual-core scheduling, Bluetooth LE (NimBLE) & USB-UART0 transport mechanics, and ISO 22900-2:2022 (`D-PDU API`) / ISO 15765-2 (`ISO-TP`) / ISO 15765-4 (`OBD-II on CAN`) implementation guidance for embedded VCI engineers.

---

## Table of Contents
1. [Architectural Overview & Core Partitioning](#1-architectural-overview--core-partitioning)
2. [Bluetooth LE (NimBLE GATT NUS) Lessons Learned](#2-bluetooth-le-nimble-gatt-nus-lessons-learned)
3. [USB-UART0 High-Speed Binary Framing Lessons Learned](#3-usb-uart0-high-speed-binary-framing-lessons-learned)
4. [ISO 22900-2:2022 (`App-Device IF`) Protocol & Queue Sizing Lessons](#4-iso-22900-22022-app-device-if-protocol--queue-sizing-lessons)
5. [Zero-Copy Static Memory Architecture on Constrained SoCs](#5-zero-copy-static-memory-architecture-on-constrained-socs)
6. [ISO 15765-2 (`ISO-TP`) & Sub-Millisecond Timer Precision](#6-iso-15765-2-iso-tp--sub-millisecond-timer-precision)
7. [Host-Side Transport Mitigation Strategies (Pipelining vs. Cyclic COP)](#7-host-side-transport-mitigation-strategies-pipelining-vs-cyclic-cop)
8. [Summary Checklist for Future VCI Designers](#8-summary-checklist-for-future-vci-designers)

---

## 1. Architectural Overview & Core Partitioning

### 1.1 Asymmetric Dual-Core Partitioning (`ESP32` 240 MHz Xtensa LX6)
A diagnostic Vehicle Communication Interface (VCI) serves two fundamentally conflicting timing domains:
1. **Host Transport Domain (Variable Latency, Burst-Oriented):** Bluetooth LE 4.2/5.0 GATT notifications, ATT MTU fragmentation, BlueZ/OS D-Bus scheduling, and USB-UART serial buffers.
2. **Vehicle CAN / ISO-TP Domain (Hard Real-Time, Sub-Millisecond):** ISO 15765-2 Consecutive Frame separation (`STmin` down to `100 µs`), `N_Bs` / `N_Cr` timeouts (`1000 ms`), UDS/OBD `P2_Max` (`50 ms`), `NRC 0x78` (`ResponsePending` -> `P2*_Max = 5000 ms`), and `P3_Phys` / `P3_Func` inter-request spacing.

**Design Rule:** Never run ISO-TP segmentation/reassembly or `P2`/`P3` timers on the same CPU core or task as the Bluetooth LE stack.
- **Core 0 (`PRO_CPU`):** Runs the ESP-IDF NimBLE Host/Controller task, the `host_io_task` (UART0 + BLE NUS stream parsing, CRC-16 verification, and immediate `< 20 ms` `0x80` `CMD_ACK`/`NACK` admission), and status LED control.
- **Core 1 (`APP_CPU`):** Runs the dedicated `vci_rt_task` (8-CLL Round-Robin scheduler, ISO 15765-2 state machine, `esp_timer_get_time()` microsecond timers, and TWAI/CAN bus I/O) plus the virtual ECU task when operating in loopback mode.

```
+-------------------------------------------------------------------------+
|                        ESP32 Dual-Core VCI SoC                          |
|                                                                         |
|  Core 0 (PRO_CPU): Host I/O & Wireless     Core 1 (APP_CPU): Real-Time  |
|  +-----------------------------------+     +-------------------------+  |
|  | NimBLE GATT Task (ATT/GAP)        |     | 8-CLL Round-Robin Sched |  |
|  |        | (SPSC Lock-Free Ring)    |     | 4-Slot FIFO Queue / CLL |  |
|  |        v                          |     | ISO 15765-2 Engine      |  |
|  | host_io_task (0xAA55 + CRC16)     |====>| 3 x 4,095B MF RX Pool   |  |
|  | Immediate 0x80 ACK/NACK (< 20 ms) |     | 100 us HW Timestamps    |  |
|  +-----------------------------------+     +-------------------------+  |
+-------------------------------------------------------------------------+
```

---

## 2. Bluetooth LE (NimBLE GATT NUS) Lessons Learned

### Lesson 2.1: Never Modify `ctxt->om` In-Place Inside a NimBLE GATT Write Callback
- **Symptom Observed:** During sustained BLE stress testing (`[BLE 3/9]` in `vci_perf_benchmark.py`), the BLE connection reliably dropped on the **25th GATT write** (`12 iterations × 2 writes/iteration = 24 writes`), whereas shorter tests (8–15 writes) passed without issue.
- **Root Cause:** In NimBLE (`ble_att_svr.c`), when a GATT Characteristic Write (`BLE_GATT_ACCESS_OP_WRITE_CHR`) arrives, NimBLE passes a chained `os_mbuf` via `ctxt->om` to the application's `access_cb` (`gatt_nus_chr_access` in `src/ble_nus.c`). After `access_cb` returns, NimBLE frees the incoming buffer by calling `os_mbuf_free_chain(ctxt.om)`.
  - Our initial code traversed the mbuf chain by advancing `ctxt->om = SLIST_NEXT(ctxt->om, om_next);` directly.
  - By the end of the loop, `ctxt->om` was `NULL`. When `gatt_nus_chr_access` returned `0`, NimBLE called `os_mbuf_free_chain(NULL)` (a no-op), **leaking one `os_mbuf` on every single GATT write**.
  - Because ESP-IDF's default NimBLE `msys_1` pool has **24 blocks**, the 25th GATT write exhausted the pool and severed the BLE connection.
- **Technical Fix:** Always iterate over `ctxt->om` using a local `const struct os_mbuf *cur = ctxt->om;` pointer (or `ble_hs_mbuf_to_flat()`) so `ctxt->om` remains pointing to the head of the chain when returning to NimBLE:
  ```c
  const struct os_mbuf *cur = ctxt->om;
  while (cur != NULL) {
      s_rx_cb(cur->om_data, cur->om_len);
      cur = SLIST_NEXT(cur, om_next);
  }
  ```

### Lesson 2.2: Decouple GATT RX Callbacks from Outgoing GATT TX Notifications
- **Symptom Observed:** Calling `ble_gatts_notify_custom()` synchronously from inside `gatt_nus_chr_access()` (to send the immediate `0x80` `CMD_ACK` while still inside the NimBLE GATT write callback) creates re-entrant contention on NimBLE's internal host lock and mbuf pool during rapid `Write Without Response` bursts.
- **Technical Fix:** Use a lock-free Single-Producer / Single-Consumer (SPSC) ringbuffer (`s_ble_rx_ring[512]` in `src/main.c`) inside `on_ble_nus_rx()` and wake `host_io_task` via `xTaskNotifyGive()`.
  - `gatt_nus_chr_access()` completes in `< 2 µs` and immediately returns control to NimBLE so the RX `os_mbuf` is freed right away.
  - `host_io_task` parses the `0xAA55` frame and invokes `ble_nus_send()` cleanly outside the NimBLE ATT callback context.
  - In `ble_nus_send()`, wrap `ble_hs_mbuf_from_flat()` / `ble_gatts_notify_custom()` in a short retry loop (`BLE_HS_ENOMEM` backoff with `vTaskDelay(1)`) so burst pipelined notifications (`0x80` ACK + `0x82` EXECUTING + `0x81` RESULT + `0x82` FINISHED) never drop a packet under heavy load.

### Lesson 2.3: Initiate ATT MTU Exchange from the Peripheral (`ble_gattc_exchange_mtu`)
- **Symptom Observed:** On Linux (`BlueZ` + `Bleak`), when reconnecting to an already-cached BLE device, `BlueZ` often skips GATT service discovery and uses the default BLE ATT MTU of `23 bytes` (`20-byte` payload per notification) unless prompted. At `MTU = 23`, a `26-byte` `PDUGetVersion` ACK or `39-byte` Multi-Frame VIN Result splits into multiple notifications.
- **Technical Fix:** Inside `BLE_GAP_EVENT_CONNECT` on the ESP32 peripheral (`src/ble_nus.c`), call:
  ```c
  ble_att_set_preferred_mtu(256);
  /* Inside BLE_GAP_EVENT_CONNECT: */
  (void)ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
  ```
  Proactively initiating MTU negotiation from the VCI peripheral guarantees a `256-byte` ATT MTU (`240-byte` chunk size) on every connection, fitting all single-frame and typical multi-frame D-PDU packets into a single BLE air packet.

### Lesson 2.4: Wireless Link-Loss Safety (`PDUDestruct < 100 ms`)
- **Requirement (ISO 22900-2):** If a wireless diagnostic session drops while cyclic or multi-frame ComPrimitives are active on the vehicle CAN bus, the VCI must autonomously abort all in-flight CAN activity and transition all CLLs to `PDU_CLLST_OFFLINE` within `< 100 ms`.
- **Implementation Pattern:** In `BLE_GAP_EVENT_DISCONNECT`, set `cancel_requested = true` across all 8 CLLs × 4 queue slots, set `s_ble_destruct_pending = true`, and immediately wake `vci_rt_task` on Core 1 via `xTaskNotifyGive()`. Hardware benchmarks confirmed full autonomous teardown in **`48.4 ms`** (`< 100 ms` target).

---

## 3. USB-UART0 High-Speed Binary Framing Lessons Learned

### Lesson 3.1: Silence Runtime `ESP_LOG*` on Shared Binary UART Channels
- **Symptom Observed:** When `UART0` carries binary `0xAA55` D-PDU packets between the Host and VCI, any asynchronous `ESP_LOGI()` / `ESP_LOGW()` output (e.g., NimBLE connection logs or TWAI state logs) injects ASCII bytes into the middle of binary `0xAA55` frames, triggering `PDU_ERR_CRC_MISMATCH` (`0x0001`) or `PDU_ERR_MALFORMED_FRAME` (`0x0002`) on the Host parser.
- **Technical Fix:** Call `esp_log_level_set("*", ESP_LOG_NONE);` at the beginning of `app_main()` in `src/main.c` once boot diagnostics have finished. Early ROM bootloader messages during the first ~400 ms after reset are flushed by the Host's `reset_vci_board()` routine before binary framing begins.

### Lesson 3.2: Avoid Fixed-Size Block Reads (`read(256)`) with Multi-Millisecond Timeouts
- **Symptom Observed:** Initial USB-UART0 benchmarks showed a uniform **`22.17 ms`** latency for a 10-byte `IOCTL_READ_VBATT` command even though 24 total bytes (`10 B` TX + `14 B` RX) at `115200` baud only require `2.08 ms` on the wire!
- **Root Cause (Both Host and MCU):**
  1. **Host (`pyserial`):** Calling `self.ser.read(256)` with `timeout = 0.02` (`20 ms`) receives the 14-byte response in `2 ms`, and then **blocks for the remaining `18 ms`** waiting for 242 more bytes until the `20 ms` timeout expires!
  2. **ESP32 (`uart_read_bytes`):** Calling `uart_read_bytes(UART_NUM_0, rx_chunk, 256, pdMS_TO_TICKS(5))` loops internally inside ESP-IDF's `xRingbufferReceiveUpTo()` until either all `256` bytes are read or the full tick timeout expires.
- **Technical Fix:**
  - **On the Host (`pc_app/dpdu_client.py`):** Query `n_avail = self.ser.in_waiting` and call `self.ser.read(n_avail if n_avail > 0 else 1)`. When `in_waiting == 0`, `read(1)` wakes immediately on the first incoming byte; on the next loop pass, `read(in_waiting)` drains the rest of the frame with **zero timeout wait**.
  - **On the ESP32 (`src/main.c`):** Query `uart_get_buffered_data_len()` first and read the exact buffered byte count with a `0`-tick timeout; only block for `1 byte` (`uart_read_bytes(..., 1, 1)`) when the RX FIFO is empty.
  - **Result:** USB-UART0 synchronous IOCTL latency dropped from **`22.17 ms` to `3.93 ms` (`5.6×` faster)**, single-frame OBD-II RTT dropped from **`23.89 ms` to `9.00 ms`**, and sustained wire throughput reached **`691.2 pkt/s` (`111.69 kbps` — 97% of the `115.2 kbps` 8N1 line rate)**.

---

## 4. ISO 22900-2:2022 (`App-Device IF`) Protocol & Queue Sizing Lessons

### Lesson 4.1: Emit `0x80` `CMD_ACK` *Before* Publishing a ComPrimitive to Core 1
- **Symptom Observed:** When `PDU_CMD_START_COP` (`0x08`) was admitted on Core 0, our initial code set `slot->in_use = true; cll->cop_count++;` *before* calling `emit_ack()` (`0x80`). Because Core 1 (`vci_rt_task`) runs concurrently at 240 MHz, Core 1 picked up the new slot and emitted `0x82 PDU_IT_STATUS (PDU_COPST_EXECUTING)` (and even `0x81 PDU_IT_RESULT` in loopback mode!) in `< 500 µs` — arriving on the wire *before* Core 0's `0x80` `CMD_ACK`!
- **Technical Fix:** In `src/dpdu_protocol.c`, populate the slot fields first, call `emit_ack(send_fn, ..., PDU_STATUS_NOERROR)` while `slot->in_use` is still `false`, and only set `slot->in_use = true; cll->cop_count++;` inside a critical section (`portENTER_CRITICAL`) **after** the `0x80` `CMD_ACK` has been serialized onto the Host wire. This guarantees strict causal ordering on the wire:
  1. `0x80` `PDU_EVT_CMD_ACK`
  2. `0x82` `PDU_IT_STATUS` (`PDU_COPST_EXECUTING`)
  3. `0x81` `PDU_IT_RESULT` (or `0x83` `PDU_IT_ERROR`)
  4. `0x82` `PDU_IT_STATUS` (`PDU_COPST_FINISHED` / `PDU_COPST_CANCELLED`)

### Lesson 4.2: Why `8 CLLs × 4-Slot Queue/CLL` Beats a Single Global 16-Slot Queue
- **Comparison:**
  - A **single global 16-slot FIFO queue** allows one slow ECU (e.g., an ECU responding with `NRC 0x78 ResponsePending` taking up to `P2*_Max = 5000 ms`, or a slow multi-frame transfer) to **head-of-line block** every other ECU in the vehicle.
  - **Per-CLL 4-Slot FIFO Queues across 8 CLLs (`32` total slots)** combined with a **Core 1 Round-Robin Scheduler** isolate each ECU's timing domain (`P2`, `P2*`, `P3_Phys`, `STmin`). While `CLL 1` (`0x7E0`) is waiting for its `5 ms` `P3_Phys` inter-request delay or an `STmin` timer, Core 1 immediately services `CLL 2` (`0x7E1`), `CLL 3` (`0x7E2`), and `CLL 4` (`0x7E3`).
- **Benchmark Proof (`Test 9B`):** Scaling from 1 active CLL to 4 concurrent CLLs increased aggregate diagnostic throughput from **`97.3 PIDs/s` to `168.4 PIDs/s` (`+73%`)** on USB-UART0 and from **`40.7 PIDs/s` to `62.9 PIDs/s` (`+55%`)** on Bluetooth LE.
- **Explicit Overrun NACK (`0x03`):** A 4-slot queue per CLL provides deterministic backpressure (`PDU_ERR_COMM_PC_TO_VCI_OVERRUN = 0x03` on the 5th unserviced command) without wasting SRAM on deep idle queues.

---

## 5. Zero-Copy Static Memory Architecture on Constrained SoCs

### Lesson 5.1: Tiered Buffer Sizing (`8 B` Single-Frame per CLL + `3 × 4,095 B` Shared Multi-Frame Pool)
- **Why Not Allocate `4,095 B` Per CLL or Per Queue Slot?**
  - Reserving `4,095 B` for all 8 CLLs (`32.8 KB`) or all 32 queue slots (`131 KB`) wastes precious ESP32 internal DRAM (especially when NimBLE + FreeRTOS already consume ~30 KB).
  - Conversely, 95%+ of live OBD-II traffic (Mode `01` PIDs, UDS `0x10`/`0x3E`/`0x22` single-frame responses) fits inside **`<= 7 bytes`** of a single Classic CAN 2.0B frame (`8 B`).
- **Optimal Memory Architecture Implemented:**
  1. **Per-CLL Single-Frame Buffer (`8 B × 8 CLLs = 64 B` total):** Every CLL owns a dedicated `uint8_t sf_rx_buf[8]` so single-frame responses never contend for shared memory.
  2. **Shared Multi-Frame RX Pool (`3 × 4,095 B = 12,285 B` total):** Allocated dynamically from a 3-slot static pool (`s_mf_pool[3]` in `src/isotp.c`) **only** when a CAN First Frame (`PCI = 0x10..0x1F`) arrives, and released immediately after `0x81 PDU_IT_RESULT`, `0x83 PDU_IT_ERROR`, `CancelComPrimitive`, or `Disconnect`. Supports up to 3 ECUs responding simultaneously with multi-frame payloads to a functional broadcast (`0x7DF`).
  3. **Queue Slot TX Payloads (`16 B × 32 slots = 512 B` + `1 × 4,095 B` Shared Large TX Buffer):** Since standard OBD-II/UDS requests are `2–8 bytes`, each queue slot holds a `16 B` inline TX buffer (`tx_small_pdu[16]`), while rare multi-frame Tester→ECU writes (`> 16 B`, such as UDS `0x2E` / `0x36`) borrow a single shared `4,095 B` TX buffer.

### Lesson 5.2: Eliminate Hidden Intermediate Copy Buffers (`> 20 KB` DRAM Saved)
- **Anti-Pattern to Avoid:** Even with a `3 × 4,095 B` ISO-TP pool, it is easy to accidentally create multiple `4.1 KB` staging buffers across layers:
  - A `4,098 B` parser payload buffer for UART + another `4,098 B` for BLE (`8.2 KB`)
  - A `4,095 B` ISO-TP RX copy buffer on the stack/BSS (`4.1 KB`)
  - A `4,104 B` `PDU_IT_RESULT` header-prepend buffer (`4.1 KB`)
  - A `4,114 B` `host_send_packet()` wire encoding buffer (`4.1 KB`)
- **Zero-Copy Solution Implemented (`45,428 B` Total Firmware RAM = `13.9%` of 320 KB DRAM):**
  1. **In-Place Result Header Padding (`result_hdr_pad[9]` in `isotp_mf_buf_t`):** Precede the `uint8_t data[4095]` array in each multi-frame pool buffer with `uint8_t result_hdr_pad[9]`. When `emit_it_result()` formats a multi-frame `0x81` `PDU_IT_RESULT`, it writes `[Rx_Timestamp_us (4B) | Rx_CAN_ID (4B) | Extra_Addr_Info (1B)]` directly into the 9 bytes immediately preceding `data[0]` — **zero copy**!
  2. **Chunked Wire Streaming in `host_send_packet()`:** For payloads `> 256 B`, compute the `CRC16-CCITT` over the 8-byte header and in-place payload, and stream `[Header (8B)]` + `[Payload (N B)]` + `[CRC (2B)]` directly to `uart_write_bytes()` and `ble_nus_send()` under `s_host_tx_mutex` without allocating a `4.1 KB` contiguous wire buffer.

---

## 6. ISO 15765-2 (`ISO-TP`) & Sub-Millisecond Timer Precision

### Lesson 6.1: Hardware Microsecond Timestamps (`100 µs` Resolution) & Sub-ms `STmin` (`0xF1..0xF9`)
- **Requirement:** ISO 15765-2 defines two ranges for Flow Control `STmin`:
  - `0x00..0x7F`: `0` to `127 ms`
  - `0xF1..0xF9`: **`100 µs` to `900 µs`** (in `100 µs` steps)
  - All other values (`0x80..0xF0`, `0xFA..0xFF`) are invalid/reserved and must be rejected (`PDU_STATUS_INVALID_ARG`) by `SetComParam(CP_ST_MIN)` or clamped to `127 ms` (`0x7F`) when received from an external ECU's Flow Control frame.
- **Why FreeRTOS Tick Timers (`vTaskDelay`) Are Insufficient:** Standard FreeRTOS ticks on ESP32 run at `100 Hz` (`10 ms` per tick) or `1000 Hz` (`1 ms` per tick) — far too coarse for `100–900 µs` `STmin` spacing or accurate `P2_Max` (`50 ms`) / `P3_Phys` (`5 ms`) measurement.
- **Technical Solution:**
  - Use ESP-IDF's hardware high-resolution timer `esp_timer_get_time()` (`1 µs` resolution, 64-bit counter) for all protocol deadlines (`p2_deadline_us`, `n_cr_deadline_us`, `next_allowed_tx_us`) and `uint32_t` packet timestamps (`Rx_Timestamp_us`).
  - For sub-millisecond `STmin` (`0xF1..0xF9`), use `esp_rom_delay_us()` for the sub-ms busy-wait window so Consecutive Frames (`CF`) are spaced accurately to within `±10 µs`.
  - **Diagnostic Value of Hardware Timestamps:** Embedding `Rx_Timestamp_us` in `0x82` (`PDU_COPST_EXECUTING`) and `0x81` (`PDU_IT_RESULT`) allowed our benchmark suite to separate **VCI internal CAN execution time (`3,811 µs ± 14 µs`)** from Host transport latency (`9.00 ms` on UART vs. `89.88 ms` on BLE).

### Lesson 6.2: Absorbing UDS/OBD `NRC 0x78` (`RequestCorrectlyReceived-ResponsePending`) Inside the VCI
- When an ECU responds with `[0x7F, <SID>, 0x78]`, the VCI must **not** complete the ComPrimitive or forward the `0x7F .. 0x78` frame as the final result. Instead, the VCI absorbs the pending frame and extends the active response deadline from `CP_P2_MAX_MS` (`50 ms`) to `CP_P2_STAR_MS` (`5000 ms`). Handling this inside the VCI on Core 1 prevents wireless BLE latency from causing false `P2_Max` timeouts.

---

## 7. Host-Side Transport Mitigation Strategies (Pipelining vs. Cyclic COP)

Our side-by-side hardware benchmarks (`VCI_PERFORMANCE_REPORT.md`) revealed a critical insight for anyone building a wireless BLE diagnostic scanner:

| Polling Strategy (3 PIDs: RPM, Speed, Coolant) | USB-UART0 (`115200` baud) | Bluetooth LE (`NimBLE NUS`) | BLE Efficiency vs. Wired |
| :--- | ---: | ---: | ---: |
| **1. Sequential Stop-and-Wait (`1 PID at a time`)** | `112.9 PIDs/s` (`37.6 Hz`) | `11.1 PIDs/s` (`3.7 Hz`) | `9.8%` |
| **2. 4-Slot CLL Queue Pipelining (`Burst of 4`)** | `112.1 PIDs/s` (`37.4 Hz`) | `43.5 PIDs/s` (`14.5 Hz`) | `38.8%` (**`3.9×` BLE speedup**) |
| **3. Autonomous VCI Cyclic `StartComPrimitive`** | `51.7 PIDs/s` (`17.2 Hz`) | **`47.7 PIDs/s` (`15.9 Hz`)** | **`92.3%` (**`4.3×` BLE speedup**)** |

### Key Takeaway for Application Developers:
1. **Never use Sequential Stop-and-Wait for live dashboards over BLE:** Each stop-and-wait round trip incurs ~`90 ms` of BLE GATT write + notify scheduling latency, limiting a 3-PID dashboard to a sluggish `3.7 Hz` update rate.
2. **Use the 4-Slot CLL FIFO Queue for One-Shot Bursts (e.g., PID Discovery or Multi-DID Reads):** Pipelining 4 requests into the CLL queue amortizes BLE round-trip latency across 4 transactions, boosting BLE throughput by **`3.63×–3.9×`** (`40.3–43.5 req/s`).
3. **Use Autonomous VCI Cyclic `StartComPrimitive` (`CycleTime_ms > 0`) for Live Telemetry Dashboards:** Sending a single `StartComPrimitive` with `NumSendCycles` and `CycleTime_ms` offloads periodic CAN transmission to Core 1 on the VCI. The VCI streams `0x81` `PDU_IT_RESULT` notifications continuously without waiting for Host GATT writes, achieving **`47.7 PIDs/s` (`15.9 Hz` 3-PID sweep rate) over Bluetooth LE — 92.3% of wired serial performance**.

---

## 8. Summary Checklist for Future VCI Designers

- [ ] **Core Isolation:** Pin wireless/Host I/O (`BLE` + `UART`) to Core 0 and the `D-PDU` / `ISO-TP` / `CAN` real-time state machine to Core 1.
- [ ] **NimBLE `os_mbuf` Hygiene:** Never mutate `ctxt->om` in-place inside `BLE_GATT_ACCESS_OP_WRITE_CHR`; always iterate with a local pointer so NimBLE frees the mbuf chain.
- [ ] **Callback Decoupling:** Enqueue BLE RX bytes into a lock-free ringbuffer inside the GATT callback and transmit GATT notifications from a dedicated I/O task.
- [ ] **Peripheral MTU Negotiation:** Call `ble_gattc_exchange_mtu()` on `BLE_GAP_EVENT_CONNECT` so cached reconnects still use a `256-byte` ATT MTU.
- [ ] **Non-Blocking Serial Reads:** Use `in_waiting` (Host) and `uart_get_buffered_data_len()` (MCU) instead of fixed-size `read(256)` calls that stall until the read timeout expires.
- [ ] **Strict Event Ordering:** Emit `0x80` `CMD_ACK` *before* marking a queue slot `in_use = true` for the real-time core.
- [ ] **Zero-Copy Multi-Frame Pool:** Pre-allocate `9 bytes` of header padding directly in front of each `4,095 B` ISO-TP RX buffer (`result_hdr_pad[9]`) and stream large packets in chunks to avoid `4.1 KB` intermediate copies.
- [ ] **Safety on Link Loss:** Hook `BLE_GAP_EVENT_DISCONNECT` to cancel all active ComPrimitives and transition all CLLs offline within `< 100 ms` (`48.4 ms` measured).
