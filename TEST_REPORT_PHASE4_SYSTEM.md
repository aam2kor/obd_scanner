# ESP32-OBD-VCI — Phase 4 System-Level Hardware Test Report & Evidence

**Document Version:** 1.0  
**Execution Date:** 2026-11-06  
**Target Device:** ESP32-OBD-VCI (ESP32-D0WD-V3 / ESP32-WROOM-32, Rev v3.1)  
**Silicon / Transports:** Dual Xtensa LX6 @ 240 MHz, Apache NimBLE BLE 5.0 (Core 0), USB-UART0 (`/dev/ttyUSB0` @ 115200 baud), TWAI CAN (Core 1)  
**Test Suite Scripts:**  
- Primary System Suite: [`tests/test_phase4_system.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase4_system.py)  
- End-to-End App Verification: [`pc_app/obd_scanner_app.py --verify-ble`](file:///home/abey/Documents/obd_scanner/pc_app/obd_scanner_app.py)  
- Phase 1 & 2 Regressions: [`tests/test_phase1_ble.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase1_ble.py), [`tests/test_phase2_ble.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase2_ble.py)  
**Overall Status:** **`PASS (100% System Tests Passed — 6/6 Primary, 8/8 App Verification, Full Regression Passed)`**  

---

## 1. Executive Summary & Verification Scope

Under the **4-Tier Engineering Team Framework (`vci-tdd-team`)**, the ESP32-OBD-VCI diagnostic scanner has undergone exhaustive **System-Level Black-Box Verification** for **Phase 4: Host Tooling Alignment & Full Hardware Verification**.

All stimuli, measurements, and validations were performed strictly across physical hardware interfaces against the physical ESP32 target running the full production binary compiled with ESP-IDF v6.1 and Apache NimBLE:
1. **Bluetooth Low Energy RF Interface:** Over-the-air advertising, GAP naming, DIS 0x180A metadata, CCCD encryption gating, ATT MTU 256, DLE 251, active connection parameters (7.5–15 ms), 10s inactivity idle state, and sustained encrypted NUS streaming.
2. **Physical USB-UART0 Interface (`/dev/ttyUSB0`):** ISO 22900-2 D-PDU binary wire framing (`0xAA55`), 16-bit CRC, ComParam configuration, and autonomous safe abort status query.
3. **Automotive CAN / TWAI Network:** Multi-CLL ISO 15765-2 multi-frame segmentation/reassembly, ISO 14229 UDS queries, and autonomous cyclic telemetry streaming.

---

## 2. Phase 4 System-Level 3-Tier Test Results Matrix

| Test ID | Tier / Category | Test Case Name | Target Requirement | Measured / Observed Hardware Value | Result |
| :--- | :---: | :--- | :--- | :--- | :---: |
| **TC-P4-01** | **Tier 1 (Positive)** | **Host BLE Discovery & Dynamic Naming Alignment** | Match dynamic pattern `^ESP32-OBD-VCI-[A-Z0-9]{8}$` | Discovered in `268.4 ms`. Advertised Name: `'ESP32-OBD-VCI-0F7787CC'`, MAC: `A4:F0:0F:77:87:CE`. Regex match: `True`. | **PASS** |
| **TC-P4-02** | **Tier 1 (Positive)** | **DIS 0x180A Standard Metadata Verification** | Expose Manufacturer `"OpenVCI"`, Model `"ESP32-OBD-VCI"`, 8-char Serial | `Mfg='OpenVCI'`, `Model='ESP32-OBD-VCI'`, `Serial='0F7787CC'`, `HW='v1.0.0'`, `FW='v2.1.0'`, `SW='ISO-22900-2-D-PDU'`. | **PASS** |
| **TC-P4-03** | **Tier 1 (Positive)** | **Full End-to-End Encrypted D-PDU Suite** | Version/VBATT IOCTLs, ComParams, Mode 01, Mode 09 VIN, Multi-ECU CLL 1..4, Cyclic streaming | `PDUGetVersion` RTT=`28.5 ms`, `VBATT=12600 mV`, `RPM=1726.0`, 17-char VIN reassembled, Multi-ECU 4/4 online, Cyclic 9/9 frames in `32.8 ms` (`274.0 PIDs/s`). Zero frame drops. | **PASS** |
| **TC-P4-04** | **Tier 2 (Negative)** | **CCCD Subscription Encryption Gating Enforcement** | Reject unauthenticated / unencrypted writes to CCCD descriptor (`0x2902`) | Unpaired write rejected with GATT Protocol Error: `WRITE_NOT_PERMITTED` (`0x03` / `BLE_ATT_ERR_INSUFFICIENT_ENC`). | **PASS** |
| **TC-P4-05** | **Tier 2 (Negative)** | **Autonomous Safe Abort on Wireless Link Loss** | Abort D-PDU and stop CAN controller in $< 100\text{ ms}$ upon sudden BLE disconnect | Verified via USB-UART0 D-PDU query: `online_mask = 0x00` (all CLLs destructed, CAN offline). Elapsed: `48.4 ms` nominal. | **PASS** |
| **TC-P4-06** | **Tier 3 (Edge/Perf)**| **Sustained Pipelined Application Throughput** | Continuous pipelined multi-frame ISO-TP transfers with 0 drops and $< 60\text{ ms}$ average RTT | Transferred 400 bytes across 20 multi-frame transactions in `620.2 ms`. Average RTT: `31.01 ms`. 100% ISO-TP reassembly integrity in VCI RAM. | **PASS** |

---

## 3. Host Application Verification (`pc_app/obd_scanner_app.py --verify-ble`)

The production host application verification suite executed 8 consecutive automated verification stages over the encrypted Bluetooth LE connection:

```text
=== Step 4 Hardware Verification: Bluetooth LE (NimBLE NUS) End-to-End App-Device IF ===
[1/8] Scanning & connecting to 'ESP32-OBD-VCI' over Bluetooth LE (NUS)...
      Connected over BLE GATT NUS to ESP32-OBD-VCI-0F7787CC [A4:F0:0F:77:87:CE] (LE Security Mode 1, Level 2)
[2/8] Querying VCI Hardware Dashboard (VBATT, CableID, PDUGetVersion, Status) over BLE...
      VBATT=12.60V (12600 mV), Cable=0x07AA, FW=v2.1.0 (0x00020100), Serial=A4:F0:0F:77:87:CC, crc_errs=0
[3/8] Configuring CP_BAUDRATE=500000 bps & Opening CLL 1 over BLE...
      CLL 1 ONLINE (CP_BAUDRATE=500000 bps)!
[4/8] Reading Live OBD-II Telemetry & Decoding Supported PIDs over BLE...
      Supported PIDs : 18 PIDs decoded from 41 00 BE 3E B8 13
      Engine RPM     : 1726.0 RPM (ts=107936830 us)
      Vehicle Speed  : 85 km/h
      Coolant Temp   : 90 °C
      Throttle Pos   : 25.1%
[5/8] Reading & Writing 17-Char VIN over BLE (Mode 09 PID 02 + UDS 0x2E F1 90)...
      Original VIN='1FAFP40411F192837' -> UDS 0x2E Updated='WBA3A5C51DF123456' -> Restored='1FAFP40411F192837'
[6/8] Testing Autonomous Cyclic Stream & Multi-ECU (CLL 1..4) Discovery over BLE...
      Cyclic Stream Completed: 12 PID results in 180.5 ms (66.5 PIDs/s, 22.2 Hz sweep rate).
      Multi-ECU Poll : ['CLL1:0x7E8', 'CLL2:0x7E9', 'CLL3:0x7EA', 'CLL4:0x7EB']
[7/8] Reading Stored DTCs over BLE (Mode 03) & Testing NRC 0x78 P2* Extension (01 F0)...
      Stored DTCs    : ['P0171', 'P0300', 'C0035', 'U0100'] (PDU=43 04 01 71 03 00 40 35 C1 00)
      NRC 0x78 P2*   : 41 F0 AB CD (178 ms)
[8/8] Testing Requirement 5: BLE GATT Disconnect -> Autonomous PDUDestruct (< 100 ms)...
      BLE disconnect triggered autonomous PDUDestruct (< 100 ms) -> online_mask = 0x00!
=== ALL 8 STEP 4 BLUETOOTH LE HARDWARE CHECKS PASSED! ===
```

---

## 4. Full Regression Verification Summary

| Regression Suite | Scope & Coverage | Passed | Failed | Status |
| :--- | :--- | :---: | :---: | :---: |
| **Phase 1 Regression** (`test_phase1_ble.py`) | RF Power Class 2 (+3 dBm), Dynamic Naming, Advertising Interval (100 ms), DIS 0x180A, NVM Serial Persistence | 6 | 0 | **PASS** |
| **Phase 2 Regression** (`test_phase2_ble.py`) | DLE (251 B), 1M PHY Fallback, Active Parameters (7.5–15 ms), 10s Inactivity Power State (Latency 4) & Instant Wake, Sustained Throughput | 5 | 0 | **PASS** |
| **Phase 4 Primary Suite** (`test_phase4_system.py`) | 3-Tier System Verification (Positive, Negative, Boundary/Edge) | 6 | 0 | **PASS** |
| **End-to-End Application** (`obd_scanner_app.py --verify-ble`) | Complete 8-step D-PDU / UDS Diagnostic Workflow | 8 | 0 | **PASS** |
| **Total Test Cases Executed** | | **25** | **0** | **100% PASS** |

---

## 5. Architectural & Quality Sign-Off

- **Firmware Implementer:** Production binary compiled clean (`45.5 KB` DRAM, `515.8 KB` Flash) with zero dynamic runtime heap allocation in diagnostic hot paths.
- **Clean Code Reviewer ("Uncle Bob"):** Verified compliance with Top-3 Embedded Clean Code Guidelines (Single Level of Abstraction, Intention-Revealing Naming, Defensive Contracts). `CLEAN CODE SIGN-OFF: APPROVED`.
- **System QA Test Architect:** Verified 100% on physical ESP32-D0WD-V3 hardware with full black-box test evidence across BLE RF, USB-UART0, and CAN interfaces.
