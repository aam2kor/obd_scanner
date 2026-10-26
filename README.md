# OBD Scanner — Hardware & Board Information

## Connected Target Board: ESP32-WROOM-32

### Hardware & USB Summary

| Property | Value |
| :--- | :--- |
| **Module** | `ESP32-WROOM-32` |
| **SoC / Revision** | ESP32 (Chip Revision 3.0, ROM Magic `0x00F01D83`, ROM Date `Jul 29 2019`) |
| **Base MAC Address** | `a4:f0:0f:77:87:cc` |
| **eFuse / APB Registers** | `EFUSE_BLK0_RDATA3: 0x0000A200`, `APB_CTL_DATE: 0x96042000` |
| **SPI Flash** | 4 MB (`DIO` mode @ 40 MHz) |
| **USB-to-UART Bridge** | Silicon Labs CP2102 USB to UART Bridge Controller (`ID 10c4:ea60`, Serial `0001`) |
| **Linux Device Node** | `/dev/ttyUSB0` |
| **Persistent Device Path** | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| **Default Baud Rate** | `115200` |
| **Auto-Reset / Bootloader** | Supported via DTR/RTS (`ROM Bootloader SYNC: True`) |

---

## Factory Flash Partition Table

| # | Label | Usage | Type | Subtype | Offset | Length |
| :- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0 | `phy_init` | RF data | `01` | `01` | `0x0000f000` | `0x00001000` (4 KB) |
| 1 | `otadata` | OTA data | `01` | `00` | `0x00010000` | `0x00002000` (8 KB) |
| 2 | `nvs` | WiFi data | `01` | `02` | `0x00012000` | `0x0000e000` (56 KB) |
| 3 | `at_customize` | unknown | `40` | `00` | `0x00020000` | `0x000e0000` (896 KB) |
| 4 | `ota_0` | OTA app | `00` | `10` | `0x00100000` | `0x00180000` (1.5 MB) |
| 5 | `ota_1` | OTA app | `00` | `11` | `0x00280000` | `0x00180000` (1.5 MB) |

---

## Factory Boot Log (115200 Baud)

```text
ets Jul 29 2019 12:21:46

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0030,len:7672
load:0x40078000,len:16432
load:0x40080400,len:3704
entry 0x4008067c
I (27) boot: ESP-IDF qa-test-v4.3.3-20220423 2nd stage bootloader
I (27) boot: compile time 11:11:52
I (28) boot: chip revision: 3
I (31) boot_comm: chip revision: 3, min. bootloader chip revision: 0
I (38) boot.esp32: SPI Speed      : 40MHz
I (43) boot.esp32: SPI Mode       : DIO
I (48) boot.esp32: SPI Flash Size : 4MB
I (52) boot: Enabling RNG early entropy source...
I (58) boot: Partition Table:
I (61) boot: ## Label            Usage          Type ST Offset   Length
I (68) boot:  0 phy_init         RF data          01 01 0000f000 00001000
I (76) boot:  1 otadata          OTA data         01 00 00010000 00002000
I (83) boot:  2 nvs              WiFi data        01 02 00012000 0000e000
I (91) boot:  3 at_customize     unknown          40 00 00020000 000e0000
I (98) boot:  4 ota_0            OTA app          00 10 00100000 00180000
I (106) boot:  5 ota_1            OTA app          00 11 00280000 00180000
I (113) boot: End of partition table
I (118) boot_comm: chip revision: 3, min. application chip revision: 0
I (125) esp_image: segment 0: paddr=00100020 vaddr=3f400020 size=295d8h (169432) map
I (195) esp_image: segment 1: paddr=00129600 vaddr=3ffbdb60 size=04474h ( 17524) load
I (202) esp_image: segment 2: paddr=0012da7c vaddr=40080000 size=0259ch (  9628) load
I (206) esp_image: segment 3: paddr=00130020 vaddr=400d0020 size=111630h (1119792) map
I (614) esp_image: segment 4: paddr=00241658 vaddr=4008259c size=1b568h (111976) load
I (660) esp_image: segment 5: paddr=0025cbc8 vaddr=400c0000 size=00064h (   100) load
I (661) esp_image: segment 6: paddr=0025cc34 vaddr=50000000 size=00010h (    16) load
I (681) boot: Loaded app from partition at offset 0x100000
I (681) boot: Disabling RNG early entropy source...
module_name:WROOM-32
max tx power=78,ret=0
2.4.0
```

---

## Development Environment (PlatformIO + ESP-IDF in C)

This project implements a dual-core **ESP32 OBD-II Vehicle Communication Interface (VCI)** in C (`framework = espidf`) and a **PC Bluetooth LE & Serial OBD-II Scanner Application** (`pc_app/`) communicating over a **Binary Lean D-PDU API (ISO 22900-2 subset)**.

### 4-Layer VCI Firmware Architecture (`src/`)
- **Core 0 (`PRO_CPU`) — Host Transport & Status:**
  - `src/ble_nus.h` / `src/ble_nus.c` — Bluetooth LE Nordic UART Service (`ESP32-OBD-VCI`, Service UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`) via ESP-IDF NimBLE
  - `src/main.c` — Dual-transport Host I/O task (UART0 + BLE NUS) and FreeRTOS queue routing
  - `src/led_status.h` / `src/led_status.c` — Non-blocking status LED state machine on `GPIO 2`
- **Core 1 (`APP_CPU`) — Hard Real-Time Automotive Stack:**
  - `src/dpdu_protocol.h` / `src/dpdu_protocol.c` — Binary Lean D-PDU (`0xA5` SOF + 8B Header + Payload + CRC-16/CCITT-FALSE), 13-parameter ISO 22900-2 ComParam table (`CP_BAUDRATE`, `CP_CAN_PHYS_REQ_ID`, `CP_CAN_RESP_USDT_ID`, `CP_ADDRESSING_TYPE`, `CP_CAN_FILLER_BYTE`, `CP_BLOCK_SIZE`, `CP_ST_MIN`, `CP_N_BS_MS`, `CP_N_CR_MS`, `CP_P2_MAX_MS`, `CP_P2_STAR_MS`, `CP_P3_PHYS_MS`, `CP_P3_FUNC_MS`), SID response validation, and in-flight `PDU_CMD_CANCEL_COP`
  - `src/isotp.h` / `src/isotp.c` — Bidirectional ISO 15765-2 (ISO-TP) Single-Frame (`SF`) and Multi-Frame (`FF` / `FC` / `CF`) segmentation & reassembly supporting both 11-bit (`0x7DF`/`0x7E8..0x7EF`) and 29-bit (`0x18DB33F1`/`0x18DAF1xx`) CAN addressing
  - `src/can_hal.h` / `src/can_hal.c` — CAN Classic 2.0B HAL (`driver/twai.h` on `GPIO 21` TX / `GPIO 22` RX at `125` / `250` / `500` / `1000 kbps` + Virtual Loopback mode)
  - `src/ecu_sim.h` / `src/ecu_sim.c` — Virtual OBD-II / UDS ECU simulator (11-bit & 29-bit CAN, Modes `01`, `03`, `04`, `09` 17-char VIN & Cal ID, UDS `0x2E`)

### PC OBD-II Diagnostic Scanner (`pc_app/`)
- `pc_app/dpdu_client.py` — Binary Lean D-PDU serial client & automated hardware verification suites (Steps 1–3)
- `pc_app/obd_scanner_app.py` — Bluetooth LE (`bleak` GATT NUS) & Serial OBD-II Scanner CLI and Step 4 BLE verification suite

### Build, Flash, and Run Commands

```bash
# Build the ESP-IDF C firmware
.venv/bin/pio run

# Run Step 2 & Step 3 USB-UART D-PDU & ISO-TP hardware verification
.venv/bin/python pc_app/dpdu_client.py --all

# Run Step 4 Bluetooth LE (NimBLE NUS) end-to-end hardware verification
.venv/bin/python pc_app/obd_scanner_app.py --verify-ble

# Launch interactive Bluetooth LE OBD-II Scanner CLI
.venv/bin/python pc_app/obd_scanner_app.py
```

---

## Host Serial Port Setup (Linux)

Ensure your user is in the `dialout` group to access `/dev/ttyUSB0`:

```bash
sudo usermod -aG dialout $USER && sudo chmod a+rw /dev/ttyUSB0
```

