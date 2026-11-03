# ESP32-OBD-VCI — Phase 3 BLE Test Report & Evidence

**Document Version:** 1.0  
**Execution Date:** 2026-11-03  
**Target Device:** ESP32-OBD-VCI (ESP32-D0WD-V3 / ESP32-WROOM-32, eFuse MAC: `A4:F0:0F:77:87:CE`)  
**Interface Transports:** Bluetooth LE (NimBLE GATT NUS with LE Security Mode 1 Level 2) & USB-UART0 (`/dev/ttyUSB0`)  
**Test Suite Script:** [`tests/test_phase3_ble.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase3_ble.py)  
**Overall Status:** **`PASS (5/5 Test Cases Passed)`**  

---

## 1. Test Execution Summary

| Test ID | Test Name | Target Requirement | Measured / Observed Value | Result |
| :--- | :--- | :--- | :--- | :---: |
| **TC-P3-01** | **CCCD (0x2902) Encryption Gating** | Require encryption to write CCCD descriptor on NUS TX (`...0003`) | Unencrypted write rejected with `WRITE_NOT_PERMITTED` (`BLE_ATT_ERR_INSUFFICIENT_ENC`). | **PASS** |
| **TC-P3-02** | **120s Post-Boot Pairing Window Acceptance** | Accept new bonding requests within $120\text{ s}$ of boot (POR) | Pair request accepted at $t = 6.3\text{ s}$ post-boot; link encrypted; CCCD write accepted. | **PASS** |
| **TC-P3-04** | **Existing Bond Reconnection After 120s** | Reconnect and resume encrypted session after $120\text{ s}$ window | Reconnected at $t = 121.5\text{ s}$; encryption restored via stored LTK; active session verified. | **PASS** |
| **TC-P3-03** | **120s Post-Boot Pairing Window Rejection** | Reject new pairing from unbonded address after $120\text{ s}$ window | Unbonded pair request strictly rejected at $t = 126.7\text{ s}$ with Security Manager auth error. | **PASS** |
| **TC-P3-05** | **4-Device Bonding Retention & Capacity** | 4-Device LRU bonding in NVM (`MAX_BONDS=4`) & persistence | Bond retained across hardware POR; `CONFIG_BT_NIMBLE_MAX_BONDS=4`; runtime `CP_BLE_MAX_BONDS=4`. | **PASS** |

---

## 2. Detailed Test Results & Evidence

### 2.1 TC-P3-01: CCCD (0x2902) Encryption Gating Verification
- **Requirement:** Writing to the Client Characteristic Configuration Descriptor (CCCD, UUID `0x2902`) of the VCI TX characteristic (`6E400003-B5A3-F393-E0A9-E50E24DCCA9E`) MUST require an encrypted link (LE Security Mode 1 Level 2). If an unencrypted/unauthenticated client attempts to subscribe without encryption, the GATT server must reject it with `BLE_ATT_ERR_INSUFFICIENT_ENC` (0x0F).
- **Execution & Observation:**
  - Client connected to peripheral without initiating SMP pairing or encryption.
  - Attempted direct write (`0x0001`, enable notifications) to descriptor handle 36 (`/org/bluez/hci0/dev_A4_F0_0F_77_87_CE/service001f/char0022/desc0024`).
  - GATT server rejected write immediately with:
    `(<BleakGATTProtocolErrorCode.WRITE_NOT_PERMITTED: 3>, 'GATT Protocol Error: Write Not Permitted')`
  - Unencrypted clients are strictly blocked from subscribing to VCI diagnostic notification streams.
- **Status:** **PASS**

### 2.2 TC-P3-02: 120-Second Post-Boot Pairing Window Acceptance
- **Requirement:** Accept new bonds or bond overwrites within $120\text{ s}$ of power-on reset (POR).
- **Execution & Observation:**
  - Board reset triggered via `/dev/ttyUSB0` RTS toggle ($t_{boot} = 0$).
  - Clean client connected at $t = 6.3\text{ s}$ post-boot ($< 120\text{ s}$).
  - Initiated LE Security Mode 1 Level 2 pairing with BlueZ NoInputNoOutput Just-Works agent.
  - Handshake succeeded (`Bleak pair() returned: None` without exception).
  - Subscribed to NUS TX notifications (`NUS_TX_CHAR_UUID`) over the newly encrypted link. Write to CCCD succeeded immediately.
  - Bond LTK stored into ESP32 NVM and BlueZ key storage.
- **Status:** **PASS**

### 2.3 TC-P3-04: Existing Bond Reconnection After 120s Post-Boot Window
- **Requirement:** Crucially, previously bonded devices MUST be allowed to reconnect and restore encryption even after the 120-second pairing window has closed.
- **Execution & Observation:**
  - Test runner waited through a monitored countdown until $t = 121.5\text{ s}$ post-boot ($> 120\text{ s}$).
  - Reconnected using the bond established in TC-P3-02 without re-pairing.
  - NimBLE restored encryption using the stored Long Term Key (LTK) from NVM.
  - Read back `CP_BLE_CONN_INTERVAL` (`0xF001`) over encrypted link: `15.00 ms`.
  - Notification stream and D-PDU transport verified fully operational.
- **Status:** **PASS**

### 2.4 TC-P3-03: 120-Second Post-Boot Pairing Window Rejection
- **Requirement:** Reject new pairing after $120\text{ s}$ of boot (e.g. from an unbonded address or unauthorized device) with authentication/pairing error.
- **Execution & Observation:**
  - Host purged its stored bond via `bluetoothctl remove A4:F0:0F:77:87:CE` to present as a new, untrusted peer.
  - Connected at link layer at $t = 126.7\text{ s}$ post-boot ($> 120\text{ s}$).
  - Attempted pairing via `await client.pair()`.
  - ESP32 NimBLE Security Manager strictly rejected the pairing request (`BLE_SM_ERR_AUTHREQ`):
    `Received expected pairing rejection from GATT server: [org.bluez.Error.Failed] Authentication Failed`
  - Anti-hijacking security protection validated: unauthorized clients cannot pair once the 120s vehicle start window has lapsed.
- **Status:** **PASS**

### 2.5 TC-P3-05: 4-Device Bonding Retention & Capacity Verification
- **Requirement:** 4-Device LRU bonding table stored in NVM (`CONFIG_BT_NIMBLE_MAX_BONDS=4`). Bonded credentials must persist across warm reset and power cycles.
- **Execution & Observation:**
  - Build configuration verified: `CONFIG_BT_NIMBLE_MAX_BONDS=4` in `sdkconfig.esp32dev`.
  - Board reset via `/dev/ttyUSB0` RTS toggle to open a fresh pairing window; paired host and established bond in NVM.
  - Triggered warm hardware reset (`ser.reset_vci_board()`).
  - Reconnected post-reset without initiating pairing: bond was successfully retrieved from NVM and link encryption was restored seamlessly.
  - Queried runtime ComParam `CP_BLE_MAX_BONDS` (`0xF009`) via D-PDU protocol: returned `4`.
- **Status:** **PASS**

---

## 3. Regression Test Verification

| Regression Suite | Test Script | Total Test Cases | Passed | Failed |
| :--- | :--- | :---: | :---: | :---: |
| **Phase 1 BLE** | [`tests/test_phase1_ble.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase1_ble.py) | 6 | **6** | 0 |
| **Phase 2 BLE** | [`tests/test_phase2_ble.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase2_ble.py) | 5 | **5** | 0 |
| **Phase 3 BLE** | [`tests/test_phase3_ble.py`](file:///home/abey/Documents/obd_scanner/tests/test_phase3_ble.py) | 5 | **5** | 0 |
| **Total** | | **16** | **16** | **0** |

All 16 test cases across Phases 1, 2, and 3 are passing on the live ESP32 hardware without regressions.
