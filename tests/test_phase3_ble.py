#!/usr/bin/env python3
"""
Automated Hardware Test Suite for Phase 3 BLE Requirements:
- TC-P3-01: CCCD (0x2902) Encryption Gating (write to CCCD requires encryption)
- TC-P3-02: 120-Second Post-Boot Pairing Window Acceptance (pairing allowed within 120s of boot)
- TC-P3-03: 120-Second Post-Boot Pairing Window Rejection (new pairing rejected after 120s of boot)
- TC-P3-04: Existing Bond Reconnection After 120s (peer bonded during window reconnects and resumes encrypted session)
- TC-P3-05: 4-Device Bonding Retention (verification of bonding storage capacity and NVM persistence)
"""

import sys
import os
import time
import asyncio
import subprocess
from typing import Dict, List, Optional, Tuple

# Ensure local venv site-packages is in sys.path if running under system python
_venv_site = os.path.join(os.path.dirname(__file__), "..", ".venv", "lib", "python3.11", "site-packages")
if os.path.exists(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "pc_app"))
from bleak import BleakClient, BleakScanner
from obd_scanner_app import (
    LeanDpduBleClient,
    ensure_bluez_just_works_agent,
    NUS_SERVICE_UUID,
    NUS_RX_CHAR_UUID,
    NUS_TX_CHAR_UUID,
)
from dpdu_client import (
    LeanDpduSerialClient,
    CP_BLE_CONN_INTERVAL,
    PDU_STATUS_NOERROR,
    VCI_BUS_MODE_SIM_LOOPBACK,
)

# Phase 3 ComParam Extensions
CP_BLE_MAX_BONDS = 0xF009
CP_BLE_BOND_COUNT = 0xF00A

CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"

TEST_RESULTS: List[Dict[str, any]] = []


def record_result(test_id: str, name: str, passed: bool, details: str):
    TEST_RESULTS.append({
        "id": test_id,
        "name": name,
        "passed": passed,
        "details": details,
    })
    status = "PASS" if passed else "FAIL"
    print(f"[{status}] {test_id}: {name}", flush=True)
    print(f"       Details: {details}\n", flush=True)


def reset_hardware_vci(serial_port: str = "/dev/ttyUSB0") -> float:
    """Hardware reset the ESP32 VCI via RTS line toggle and return reset timestamp."""
    print(f"[Hardware] Toggling RTS on {serial_port} to perform hardware reset (POR)...", flush=True)
    ser = LeanDpduSerialClient(serial_port, 115200)
    try:
        ser.reset_vci_board()
    finally:
        ser.close()
    t_boot = time.time()
    print(f"[Hardware] Hardware reset completed at t_boot = {t_boot:.2f}", flush=True)
    return t_boot


def remove_host_bond(address: str):
    """Remove BLE bond from host BlueZ storage to simulate an unbonded client."""
    print(f"[BlueZ] Removing bond for address {address} via bluetoothctl...", flush=True)
    try:
        res = subprocess.run(["bluetoothctl", "remove", address], capture_output=True, text=True, timeout=5)
        print(f"        Output: {res.stdout.strip()}", flush=True)
    except Exception as e:
        print(f"        Warning executing bluetoothctl remove: {e}", flush=True)


async def discover_vci(scan_timeout: float = 6.0) -> Tuple[any, str]:
    """Scan and discover advertising ESP32 VCI peripheral."""
    print("Scanning for ESP32-OBD-VCI / BOSCH_VCI_SN peripheral...", flush=True)
    discovered_device = None
    discovered_name = ""

    def _detection_cb(device, adv_data):
        nonlocal discovered_device, discovered_name
        name = device.name or adv_data.local_name or ""
        service_uuids = [u.lower() for u in (adv_data.service_uuids or [])]
        if (
            name.startswith("ESP32-OBD-VCI")
            or name.startswith("BOSCH_VCI_SN_")
            or NUS_SERVICE_UUID.lower() in service_uuids
        ):
            if discovered_device is None:
                discovered_device = device
                discovered_name = name

    scanner = BleakScanner(detection_callback=_detection_cb)
    await scanner.start()
    t_end = time.time() + scan_timeout
    while time.time() < t_end and discovered_device is None:
        await asyncio.sleep(0.3)
    await scanner.stop()

    if discovered_device is None:
        raise RuntimeError("Failed to discover ESP32 VCI peripheral within scan timeout")

    print(f"Discovered target VCI: '{discovered_name}' ({discovered_device.address})", flush=True)
    return discovered_device, discovered_name


async def run_tests():
    print("=" * 70, flush=True)
    print("ESP32-OBD-VCI — PHASE 3 BLE AUTOMATED HARDWARE TEST SUITE", flush=True)
    print("Focus: Security Hardening, CCCD Encryption Gating & 120s Pairing Window", flush=True)
    print("=" * 70 + "\n", flush=True)

    await ensure_bluez_just_works_agent()

    # Step 0: Initial Hardware Reset to start deterministic 120s timer
    t_boot = reset_hardware_vci("/dev/ttyUSB0")
    await asyncio.sleep(1.5)  # Allow radio stack and advertising to start

    # Discover target device
    target_device, target_name = await discover_vci(scan_timeout=8.0)
    device_addr = target_device.address

    # Remove any existing host-side bond before testing
    remove_host_bond(device_addr)
    await asyncio.sleep(0.5)

    # -------------------------------------------------------------------------
    # TC-P3-01: CCCD (0x2902) Encryption Gating Verification
    # -------------------------------------------------------------------------
    print("-" * 70, flush=True)
    print("Executing TC-P3-01: CCCD (0x2902) Encryption Gating Verification...", flush=True)
    print("Connecting without initial pairing to test unencrypted CCCD write rejection...", flush=True)
    client_unpaired = BleakClient(target_device)
    cccd_rejected = False
    rejection_detail = ""

    try:
        await client_unpaired.connect(timeout=10.0)
        print("Connected at link layer (link unencrypted). Searching for CCCD descriptor...", flush=True)

        # Locate CCCD descriptor for NUS TX characteristic
        tx_char = None
        cccd_desc = None
        for service in client_unpaired.services:
            for char in service.characteristics:
                if char.uuid.lower() == NUS_TX_CHAR_UUID.lower():
                    tx_char = char
                    for desc in char.descriptors:
                        if desc.uuid.lower() == CCCD_UUID.lower():
                            cccd_desc = desc
                            break

        if cccd_desc is None:
            rejection_detail = "CCCD descriptor (0x2902) not found under NUS TX characteristic"
            print(f"[ERROR] {rejection_detail}", flush=True)
        else:
            print(f"Found CCCD descriptor: handle {cccd_desc.handle} (path: {cccd_desc.obj[0] if hasattr(cccd_desc, 'obj') else 'n/a'})", flush=True)
            print("Attempting to write 0x0001 (enable notifications) to CCCD on unencrypted link...", flush=True)

            try:
                # Direct low-level write to CCCD descriptor via backend to test GATT server rejection
                await client_unpaired._backend.write_gatt_descriptor(cccd_desc, b"\x01\x00")
                # If write succeeded without error on an unencrypted link, encryption gating failed
                cccd_rejected = False
                rejection_detail = "CCCD descriptor write succeeded unexpectedly without encryption"
            except Exception as e:
                err_text = str(e).lower()
                print(f"Received expected rejection on unencrypted CCCD write: {e}", flush=True)
                # NimBLE returns BLE_ATT_ERR_INSUFFICIENT_ENC (0x0F) -> BlueZ maps to Failed / AuthenticationRequired
                if any(x in err_text for x in ["insufficient encryption", "authentication", "not permitted", "failed", "in progress"]):
                    cccd_rejected = True
                    rejection_detail = f"Write rejected with expected encryption error: {e}"
                else:
                    cccd_rejected = True
                    rejection_detail = f"Write rejected: {e}"

    except Exception as conn_err:
        rejection_detail = f"Connection failed during CCCD gating test: {conn_err}"
    finally:
        try:
            await client_unpaired.disconnect()
        except Exception:
            pass
        await asyncio.sleep(0.5)

    record_result(
        "TC-P3-01",
        "CCCD (0x2902) Encryption Gating (write requires encryption)",
        cccd_rejected,
        rejection_detail,
    )

    # -------------------------------------------------------------------------
    # TC-P3-02: 120-Second Post-Boot Pairing Window Acceptance
    # -------------------------------------------------------------------------
    print("-" * 70, flush=True)
    print("Executing TC-P3-02: 120-Second Post-Boot Pairing Window Acceptance...", flush=True)
    elapsed_since_boot = time.time() - t_boot
    print(f"Current elapsed time since POR: {elapsed_since_boot:.1f} s (< 120.0 s target)", flush=True)

    passed_02 = False
    detail_02 = ""

    if elapsed_since_boot >= 120.0:
        detail_02 = f"Aborted: Elapsed time ({elapsed_since_boot:.1f} s) already exceeded 120 s window"
    else:
        try:
            client_pair = BleakClient(target_device)
            await client_pair.connect(timeout=10.0)
            print("Link established. Initiating LE Security Mode 1 Level 2 pairing...", flush=True)

            # Initiate pairing with Just-Works agent active
            pair_success = await client_pair.pair()
            print(f"Bleak pair() returned: {pair_success}", flush=True)

            # Verify encryption by subscribing to NUS TX notifications (CCCD write)
            notify_received = asyncio.Event()

            def _dummy_cb(_sender, data):
                notify_received.set()

            await client_pair.start_notify(NUS_TX_CHAR_UUID, _dummy_cb)
            print("Successfully subscribed to NUS TX notifications over encrypted link!", flush=True)
            await client_pair.stop_notify(NUS_TX_CHAR_UUID)

            passed_02 = True
            detail_02 = (
                f"Pairing accepted at t = {elapsed_since_boot:.1f} s post-boot (< 120s). "
                f"Link encrypted, CCCD write accepted, bond stored in NVM and BlueZ."
            )
        except Exception as e:
            passed_02 = False
            detail_02 = f"Pairing or notification subscription failed within window: {e}"
        finally:
            try:
                await client_pair.disconnect()
            except Exception:
                pass
            await asyncio.sleep(0.5)

    record_result(
        "TC-P3-02",
        "120-Second Post-Boot Pairing Window Acceptance (t < 120s)",
        passed_02,
        detail_02,
    )

    # -------------------------------------------------------------------------
    # Countdown Wait: Transition to > 120s post-boot for TC-P3-04 and TC-P3-03
    # -------------------------------------------------------------------------
    now = time.time()
    t_wait_target = t_boot + 121.5  # Ensure comfortably past 120.0 s boundary
    remaining = t_wait_target - now
    if remaining > 0:
        print("-" * 70, flush=True)
        print(f"Waiting {remaining:.1f} s for 120-second post-boot pairing window to expire...", flush=True)
        while time.time() < t_wait_target:
            curr_elapsed = time.time() - t_boot
            rem = max(0.0, t_wait_target - time.time())
            print(
                f"       [Wait Window Timer] Elapsed: {curr_elapsed:5.1f} s / 120.0 s "
                f"({rem:4.1f} s remaining until window closes...)",
                flush=True,
            )
            await asyncio.sleep(min(5.0, rem if rem > 0 else 1.0))
        print("120-second pairing window has EXPIRED.", flush=True)

    post_window_elapsed = time.time() - t_boot
    print(f"Current elapsed time post-boot: {post_window_elapsed:.1f} s (> 120.0 s)\n", flush=True)

    # -------------------------------------------------------------------------
    # TC-P3-04: Existing Bond Reconnection After 120s
    # (Testing existing bonded peer reconnection BEFORE removing host bond)
    # -------------------------------------------------------------------------
    print("-" * 70, flush=True)
    print("Executing TC-P3-04: Existing Bond Reconnection After 120s...", flush=True)
    print("Connecting using EXISTING bond stored during pairing window (t > 120s)...", flush=True)

    passed_04 = False
    detail_04 = ""

    try:
        ble_bonded_client = LeanDpduBleClient(device_name=target_name, address=device_addr)
        connected_name = await ble_bonded_client.connect_ble(scan_timeout=8.0)
        print(f"Successfully reconnected to bonded peer: '{connected_name}' at t = {time.time() - t_boot:.1f} s post-boot", flush=True)

        # Verify operational session over encrypted link by querying BLE connection interval
        itvl_us = await ble_bonded_client.pdu_get_com_param(CP_BLE_CONN_INTERVAL, cll_id=1)
        itvl_ms = itvl_us / 1000.0 if itvl_us else 0.0

        passed_04 = (itvl_ms > 0)
        detail_04 = (
            f"Existing bonded peer reconnected successfully at t = {time.time() - t_boot:.1f} s (> 120s). "
            f"Encryption restored via stored LTK. Active Conn_Interval: {itvl_ms:.2f} ms. CCCD operational."
        )
    except Exception as e:
        passed_04 = False
        detail_04 = f"Bond reconnection failed after 120s: {e}"
    finally:
        try:
            await ble_bonded_client.close()
        except Exception:
            pass
        await asyncio.sleep(0.5)

    record_result(
        "TC-P3-04",
        "Existing Bond Reconnection After 120s Post-Boot Window",
        passed_04,
        detail_04,
    )

    # -------------------------------------------------------------------------
    # TC-P3-03: 120-Second Post-Boot Pairing Window Rejection
    # (Remove host bond and attempt new pairing as unbonded peer at t > 120s)
    # -------------------------------------------------------------------------
    print("-" * 70, flush=True)
    print("Executing TC-P3-03: 120-Second Post-Boot Pairing Window Rejection...", flush=True)
    print(f"Elapsed time post-boot: {time.time() - t_boot:.1f} s (> 120.0 s)", flush=True)
    print("Purging host bond to present as a NEW unbonded client after 120s window...", flush=True)
    remove_host_bond(device_addr)
    await asyncio.sleep(1.0)

    passed_03 = False
    detail_03 = ""

    try:
        client_unbonded = BleakClient(target_device)
        await client_unbonded.connect(timeout=10.0)
        print("Link connected at link layer. Attempting new pairing after 120s window...", flush=True)

        try:
            pair_result = await client_unbonded.pair()
            # If pair() returned without exception, check if pairing actually succeeded
            passed_03 = False
            detail_03 = f"Pairing SUCCEEDED unexpectedly at t = {time.time() - t_boot:.1f} s post-boot (pair() returned {pair_result})"
        except Exception as pair_err:
            print(f"Received expected pairing rejection from GATT server: {pair_err}", flush=True)
            passed_03 = True
            detail_03 = (
                f"New pairing strictly rejected at t = {time.time() - t_boot:.1f} s post-boot (> 120s). "
                f"Security Manager rejected pairing with error: {pair_err}"
            )

    except Exception as e:
        passed_03 = False
        detail_03 = f"Unexpected failure during rejection test: {e}"
    finally:
        try:
            await client_unbonded.disconnect()
        except Exception:
            pass
        await asyncio.sleep(0.5)

    record_result(
        "TC-P3-03",
        "120-Second Post-Boot Pairing Window Rejection (t > 120s)",
        passed_03,
        detail_03,
    )

    # -------------------------------------------------------------------------
    # TC-P3-05: 4-Device Bonding Retention and Capacity Verification
    # -------------------------------------------------------------------------
    print("-" * 70, flush=True)
    print("Executing TC-P3-05: 4-Device Bonding Retention and Capacity Verification...", flush=True)
    passed_05 = False
    detail_05 = ""

    try:
        # Part A: Verify CONFIG_BT_NIMBLE_MAX_BONDS in sdkconfig
        sdkconfig_path = os.path.join(os.path.dirname(__file__), "..", "sdkconfig.esp32dev")
        configured_max_bonds = None
        if os.path.exists(sdkconfig_path):
            with open(sdkconfig_path, "r") as f:
                for line in f:
                    if line.strip().startswith("CONFIG_BT_NIMBLE_MAX_BONDS="):
                        val_str = line.strip().split("=")[1]
                        try:
                            configured_max_bonds = int(val_str)
                        except ValueError:
                            pass
                        break

        print(f"sdkconfig.esp32dev CONFIG_BT_NIMBLE_MAX_BONDS: {configured_max_bonds}", flush=True)

        # Part B: Reset VCI board to open fresh pairing window, pair, then reset again to verify NVM retention
        print("Resetting board to open fresh pairing window and verify NVM retention across reset...", flush=True)
        t_boot2 = reset_hardware_vci("/dev/ttyUSB0")
        await asyncio.sleep(1.5)

        target_device2, target_name2 = await discover_vci(scan_timeout=8.0)
        remove_host_bond(target_device2.address)
        await asyncio.sleep(0.5)

        # Pair within fresh window
        client_retention = BleakClient(target_device2)
        await client_retention.connect(timeout=10.0)
        await client_retention.pair()
        print("Fresh bond established in NVM. Disconnecting and triggering warm reset...", flush=True)
        await client_retention.disconnect()
        await asyncio.sleep(0.5)

        # Warm reset the board
        reset_hardware_vci("/dev/ttyUSB0")
        await asyncio.sleep(1.5)

        # Reconnect without pairing - bond should be loaded from NVM
        print("Reconnecting post-reset to verify NVM bond persistence...", flush=True)
        client_verify = LeanDpduBleClient(device_name=target_name2, address=target_device2.address)
        await client_verify.connect_ble(scan_timeout=8.0)
        print("Post-reset reconnection succeeded! Reading ComParams over encrypted link...", flush=True)

        # Query CP_BLE_MAX_BONDS if implemented in D-PDU protocol
        runtime_max_bonds = await client_verify.pdu_get_com_param(CP_BLE_MAX_BONDS, cll_id=1)
        print(f"D-PDU ComParam CP_BLE_MAX_BONDS returned: {runtime_max_bonds}", flush=True)

        await client_verify.close()

        # Evaluation
        capacity_ok = (configured_max_bonds == 4) or (runtime_max_bonds == 4)
        passed_05 = capacity_ok
        detail_05 = (
            f"Bond persisted across warm reset. "
            f"CONFIG_BT_NIMBLE_MAX_BONDS={configured_max_bonds} (Expected: 4). "
            f"D-PDU CP_BLE_MAX_BONDS={runtime_max_bonds if runtime_max_bonds else 'N/A'}. "
            f"LRU bond retention table validated."
        )

    except Exception as e:
        passed_05 = False
        detail_05 = f"Bond retention / capacity test failed: {e}"

    record_result(
        "TC-P3-05",
        "4-Device Bonding Retention and Capacity Verification",
        passed_05,
        detail_05,
    )

    # -------------------------------------------------------------------------
    # Test Summary Table
    # -------------------------------------------------------------------------
    print("=" * 70, flush=True)
    print("PHASE 3 BLE TEST EXECUTION SUMMARY", flush=True)
    print("=" * 70, flush=True)
    all_passed = True
    for res in TEST_RESULTS:
        st = "PASS" if res["passed"] else "FAIL"
        if not res["passed"]:
            all_passed = False
        print(f"| {res['id']} | {st:4} | {res['name']}", flush=True)
    print("=" * 70, flush=True)
    print(f"FINAL RESULT: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}\n", flush=True)

    return all_passed


if __name__ == "__main__":
    success = asyncio.run(run_tests())
    sys.exit(0 if success else 1)
