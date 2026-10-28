#!/usr/bin/env python3
"""
PC Bluetooth LE & USB-Serial OBD-II Diagnostic Scanner Application (ISO 22900-2:2022 App-Device IF).

Communicates with the ESP32 OBD-II VCI ("ESP32-OBD-VCI") over Bluetooth LE
Nordic UART Service (NUS) or USB-UART0 using the binary App-Device IF protocol:
[ SOF (2B: 0xAA55) | Msg_Type (1B) | CLL_ID (1B) | COP_ID (1B) | Status (1B) | Payload_Len (2B) | Payload | CRC16-CCITT (2B) ]
"""

import argparse
import asyncio
import struct
import sys
import time
from typing import Optional, Tuple

from dpdu_client import (
    CP_ADDRESSING_TYPE,
    CP_BAUDRATE,
    CP_BLOCK_SIZE,
    CP_CAN_FILLER_BYTE,
    CP_CAN_PHYS_REQ_ID,
    CP_CAN_RESP_USDT_ID,
    CP_N_BS_MS,
    CP_N_CR_MS,
    CP_P2_MAX_MS,
    CP_P2_STAR_MS,
    CP_P3_FUNC_MS,
    CP_P3_PHYS_MS,
    CP_ST_MIN,
    DPDU_ADDR_FUNCTIONAL,
    DPDU_ADDR_PHYSICAL,
    DpduFrame,
    DpduResultEvent,
    DpduStreamParser,
    LeanDpduSerialClient,
    PDU_CLLST_OFFLINE,
    PDU_CLLST_ONLINE,
    PDU_CMD_CONNECT,
    PDU_CMD_DISCONNECT,
    PDU_CMD_GET_COMPARAM,
    PDU_CMD_GET_STATUS,
    PDU_CMD_MOD_CTRL,
    PDU_CMD_SET_COMPARAM,
    PDU_CMD_START_COP,
    PDU_COPST_CANCELLED,
    PDU_COPST_FINISHED,
    PDU_COPT_SENDRECV,
    PDU_COPT_SEND_ONLY,
    PDU_EVT_CMD_ACK,
    PDU_IOCTL_GET_CABLE_ID,
    PDU_IOCTL_GET_VERSION,
    PDU_IOCTL_READ_VBATT,
    PDU_IT_ERROR,
    PDU_IT_RESULT,
    PDU_IT_STATUS,
    PDU_STATUS_NOERROR,
    VCI_BUS_MODE_PHYSICAL_CAN,
    VCI_BUS_MODE_SIM_LOOPBACK,
    decode_obd_dtcs,
)

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
VCI_BLE_NAME = "ESP32-OBD-VCI"


class LeanDpduBleClient:
    """Async PC Host ISO 22900-2 App-Device IF Client over Bluetooth LE (Nordic UART Service)."""

    def __init__(self, device_name: str = VCI_BLE_NAME, address: Optional[str] = None):
        self.device_name = device_name
        self.address = address
        self.client = None
        self.parser = DpduStreamParser()
        self._rx_queue: asyncio.Queue[DpduFrame] = asyncio.Queue()
        self._deferred: list[DpduFrame] = []
        self._next_cop_id = 1

    def alloc_cop_id(self) -> int:
        cid = self._next_cop_id
        self._next_cop_id = (self._next_cop_id % 254) + 1
        return cid

    def _on_nus_tx_notify(self, _sender, data: bytearray):
        for b in data:
            frame = self.parser.feed_byte(b)
            if frame is not None:
                self._rx_queue.put_nowait(frame)

    async def connect_ble(self, scan_timeout: float = 8.0) -> str:
        from bleak import BleakClient, BleakScanner

        target_device = None
        if self.address:
            target_device = await BleakScanner.find_device_by_address(self.address, timeout=scan_timeout)
        else:
            devices = await BleakScanner.discover(timeout=scan_timeout, return_adv=True)
            for dev, adv in devices.values():
                adv_name = dev.name or adv.local_name or ""
                service_uuids = [u.lower() for u in (adv.service_uuids or [])]
                if adv_name == self.device_name or NUS_SERVICE_UUID in service_uuids:
                    target_device = dev
                    break

        if target_device is None:
            raise RuntimeError(
                f"Could not find BLE VCI '{self.device_name}' (NUS UUID {NUS_SERVICE_UUID}). "
                f"Ensure the ESP32 is powered and advertising."
            )

        self.client = BleakClient(target_device)
        await self.client.connect()
        self.parser.reset()
        self._deferred.clear()
        while not self._rx_queue.empty():
            self._rx_queue.get_nowait()
        await self.client.start_notify(NUS_TX_CHAR_UUID, self._on_nus_tx_notify)
        return f"{target_device.name or self.device_name} [{target_device.address}]"

    async def close(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(NUS_TX_CHAR_UUID)
            except Exception:
                pass
            await self.client.disconnect()

    async def recv_frame(self, timeout: float = 2.0) -> DpduFrame:
        if self._deferred:
            return self._deferred.pop(0)
        return await asyncio.wait_for(self._rx_queue.get(), timeout=timeout)

    async def send_and_recv_ack(
        self,
        msg_type: int,
        payload: bytes = b"",
        cll_id: int = 1,
        cop_id: int = 0,
        timeout: float = 2.0,
    ) -> DpduFrame:
        req = DpduFrame(msg_type=msg_type, cll_id=cll_id, cop_id=cop_id, status=0, payload=payload)
        await self.client.write_gatt_char(NUS_RX_CHAR_UUID, req.encode(), response=False)
        skipped: list[DpduFrame] = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            f = await self.recv_frame(timeout=max(0.05, deadline - time.time()))
            if f.msg_type == PDU_EVT_CMD_ACK and f.cll_id == cll_id and f.cop_id == cop_id:
                self._deferred = skipped + self._deferred
                return f
            skipped.append(f)
        raise TimeoutError("Timed out waiting for 0x80 synchronous Command ACK over BLE")

    async def pdu_ioctl(self, ioctl_id: int) -> DpduFrame:
        return await self.send_and_recv_ack(
            PDU_CMD_MOD_CTRL,
            payload=struct.pack("<H", ioctl_id),
            cll_id=0,
            cop_id=0,
        )

    async def pdu_get_status(self, cll_id: int = 1) -> dict:
        rsp = await self.send_and_recv_ack(PDU_CMD_GET_STATUS, cll_id=cll_id)
        assert rsp.status == PDU_STATUS_NOERROR and len(rsp.payload) == 16
        online_mask, bus_mode, baud_khz, rx_cnt, crc_err, uptime_ms, req_id = struct.unpack(
            "<BBHIHIH", rsp.payload
        )
        return {
            "online_mask": online_mask,
            "connected": bool(online_mask & (1 << (max(1, cll_id) - 1))),
            "bus_mode": bus_mode,
            "baud_khz": baud_khz,
            "rx_cmd_count": rx_cnt,
            "crc_err_count": crc_err,
            "uptime_ms": uptime_ms,
            "can_req_id": req_id,
        }

    async def pdu_connect(
        self,
        bus_mode: int = VCI_BUS_MODE_SIM_LOOPBACK,
        baud_khz: int = 0,
        flags: int = 0,
        cll_id: int = 1,
    ) -> DpduFrame:
        if baud_khz == 0 and flags == 0:
            payload = struct.pack("<B", bus_mode)
        else:
            payload = struct.pack("<BHB", bus_mode, baud_khz, flags)
        ack = await self.send_and_recv_ack(PDU_CMD_CONNECT, payload=payload, cll_id=cll_id)
        if ack.status == PDU_STATUS_NOERROR:
            st_evt = await self.recv_frame(timeout=1.0)
            assert st_evt.msg_type == PDU_IT_STATUS
        return ack

    async def pdu_disconnect(self, cll_id: int = 1) -> DpduFrame:
        ack = await self.send_and_recv_ack(PDU_CMD_DISCONNECT, cll_id=cll_id)
        if ack.status == PDU_STATUS_NOERROR:
            try:
                f = await self.recv_frame(timeout=0.1)
                if f.msg_type != PDU_IT_STATUS:
                    self._deferred.insert(0, f)
            except asyncio.TimeoutError:
                pass
        return ack

    async def pdu_set_com_param(self, param_id: int, param_val: int, cll_id: int = 1) -> DpduFrame:
        payload = struct.pack("<HI", param_id, param_val)
        return await self.send_and_recv_ack(PDU_CMD_SET_COMPARAM, payload=payload, cll_id=cll_id)

    async def pdu_get_com_param(self, param_id: int, cll_id: int = 1) -> int:
        rsp = await self.send_and_recv_ack(PDU_CMD_GET_COMPARAM, struct.pack("<H", param_id), cll_id=cll_id)
        assert rsp.status == PDU_STATUS_NOERROR and len(rsp.payload) == 6
        _, val = struct.unpack("<HI", rsp.payload)
        return val

    async def pdu_start_com_primitive(
        self,
        pdu_data: bytes,
        cop_type: int = PDU_COPT_SENDRECV,
        flags: int = 0,
        timeout: float = 2.5,
        cll_id: int = 1,
        can_id: int = 0,
    ) -> Tuple[DpduFrame, Optional[DpduFrame]]:
        _ = flags
        num_recv = 0 if cop_type == PDU_COPT_SEND_ONLY else 1
        cop_id = self.alloc_cop_id()
        payload = LeanDpduSerialClient.build_start_cop_payload(
            pdu_data,
            num_send_cycles=1,
            num_recv_cycles=num_recv,
            cycle_time_ms=0,
            can_id=can_id,
        )
        t0 = time.time()
        ack = await self.send_and_recv_ack(PDU_CMD_START_COP, payload=payload, cll_id=cll_id, cop_id=cop_id, timeout=timeout)
        if ack.status != PDU_STATUS_NOERROR:
            return ack, None

        outcome_evt: Optional[DpduFrame] = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            f = await self.recv_frame(timeout=max(0.05, deadline - time.time()))
            if f.cll_id != cll_id or f.cop_id != cop_id:
                continue
            if f.msg_type in (PDU_IT_RESULT, PDU_IT_ERROR):
                outcome_evt = f
                setattr(outcome_evt, "_elapsed_ms", max(1, int((time.time() - t0) * 1000)))
            elif f.msg_type == PDU_IT_STATUS:
                st = LeanDpduSerialClient.parse_status_event(f)
                if st.status_code in (PDU_COPST_FINISHED, PDU_COPST_CANCELLED):
                    break
        return ack, outcome_evt

    @staticmethod
    def parse_result_event(evt: DpduFrame) -> DpduResultEvent:
        return LeanDpduSerialClient.parse_result_event(evt)


async def run_step4_ble_verification(address: Optional[str] = None):
    print("=== Step 4 Hardware Verification: Bluetooth LE (NimBLE NUS) End-to-End App-Device IF ===")
    ble = LeanDpduBleClient(device_name=VCI_BLE_NAME, address=address)
    try:
        print(f"[1/8] Scanning & connecting to '{VCI_BLE_NAME}' over Bluetooth LE (NUS)...")
        peer = await ble.connect_ble(scan_timeout=8.0)
        print(f"      Connected over BLE GATT NUS to {peer}")

        print("[2/8] Querying VCI Status & IOCTLs (VBATT, CableID, GetVersion) over BLE...")
        st = await ble.pdu_get_status()
        vbatt_ack = await ble.pdu_ioctl(PDU_IOCTL_READ_VBATT)
        vbatt_mv = struct.unpack("<I", vbatt_ack.payload)[0]
        print(
            f"      VCI Status: uptime={st['uptime_ms']} ms, rx_cmds={st['rx_cmd_count']}, "
            f"crc_errs={st['crc_err_count']}, VBATT={vbatt_mv} mV"
        )
        assert st["crc_err_count"] == 0 and vbatt_mv == 12600

        print("[3/8] Configuring CP_BAUDRATE=500000 bps & Opening CLL 1 over BLE...")
        await ble.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=1)
        ack = await ble.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=1)
        assert ack.status == PDU_STATUS_NOERROR
        print("      CLL 1 ONLINE (CP_BAUDRATE=500000 bps)!")

        print("[4/8] Reading Live OBD-II Telemetry over BLE (Mode 01 PIDs 00, 0C, 0D, 05, 11)...")
        _, evt_00 = await ble.pdu_start_com_primitive(b"\x01\x00")
        res_00 = ble.parse_result_event(evt_00)

        _, evt_rpm = await ble.pdu_start_com_primitive(b"\x01\x0C")
        res_rpm = ble.parse_result_event(evt_rpm)
        rpm = ((res_rpm.pdu[2] << 8) | res_rpm.pdu[3]) / 4.0

        _, evt_spd = await ble.pdu_start_com_primitive(b"\x01\x0D")
        res_spd = ble.parse_result_event(evt_spd)
        speed_kmh = res_spd.pdu[2]

        _, evt_tmp = await ble.pdu_start_com_primitive(b"\x01\x05")
        res_tmp = ble.parse_result_event(evt_tmp)
        coolant_c = res_tmp.pdu[2] - 40

        _, evt_thr = await ble.pdu_start_com_primitive(b"\x01\x11")
        res_thr = ble.parse_result_event(evt_thr)
        throttle_pct = (res_thr.pdu[2] * 100.0) / 255.0

        print(f"      Supported PIDs : {res_00.pdu.hex(' ').upper()} (ts={res_00.rx_timestamp_us} us)")
        print(f"      Engine RPM     : {rpm:.1f} RPM")
        print(f"      Vehicle Speed  : {speed_kmh} km/h")
        print(f"      Coolant Temp   : {coolant_c} °C")
        print(f"      Throttle Pos   : {throttle_pct:.1f}%")
        assert rpm == 1726.0 and speed_kmh == 85 and coolant_c == 90

        print("[5/8] Reading 17-Char VIN over BLE via Multi-Frame ISO-TP (Mode 09 PID 0x02)...")
        _, evt_vin = await ble.pdu_start_com_primitive(b"\x09\x02")
        res_vin = ble.parse_result_event(evt_vin)
        vin_str = res_vin.pdu[3:].decode("ascii")
        print(f"      VIN            : '{vin_str}' ({len(res_vin.pdu)} bytes)")
        assert len(res_vin.pdu) == 20

        print("[6/8] Reading Calibration ID over BLE via Multi-Frame ISO-TP (Mode 09 PID 0x04)...")
        _, evt_cal = await ble.pdu_start_com_primitive(b"\x09\x04")
        res_cal = ble.parse_result_event(evt_cal)
        cal_str = res_cal.pdu[3:].decode("ascii")
        print(f"      Calibration ID : '{cal_str}' ({len(res_cal.pdu)} bytes)")
        assert cal_str == "ESP32_VCI_CAL_01"

        print("[7/8] Reading Stored DTCs over BLE (Mode 03) & Testing NRC 0x78 P2* Extension (01 F0)...")
        _, evt_dtc = await ble.pdu_start_com_primitive(b"\x03")
        res_dtc = ble.parse_result_event(evt_dtc)
        dtcs = decode_obd_dtcs(res_dtc.pdu)
        print(f"      Stored DTCs    : {dtcs} (PDU={res_dtc.pdu.hex(' ').upper()})")

        _, evt_p2s = await ble.pdu_start_com_primitive(b"\x01\xF0")
        res_p2s = ble.parse_result_event(evt_p2s)
        print(f"      NRC 0x78 P2*   : {res_p2s.pdu.hex(' ').upper()} ({res_p2s.rx_time_ms} ms)")
        assert res_p2s.pdu == b"\x41\xF0\xAB\xCD"

        print("[8/8] Testing Requirement 5: BLE GATT Disconnect -> Autonomous PDUDestruct (< 100 ms)...")
        # Leave CLL 1 ONLINE and disconnect BLE abruptly without sending PDU_CMD_DISCONNECT!
        await ble.close()
        await asyncio.sleep(0.1)  # Wait 100 ms and check over USB-UART that all CLLs were cleared!
        ser_check = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
        try:
            st_after = ser_check.pdu_get_status(cll_id=1)
            assert st_after["online_mask"] == 0, f"Expected online_mask=0 after BLE disconnect, got {st_after['online_mask']}"
            print("      BLE disconnect triggered autonomous PDUDestruct (< 100 ms) -> online_mask = 0x00!")
        finally:
            ser_check.close()
        print("=== ALL 8 STEP 4 BLUETOOTH LE HARDWARE CHECKS PASSED! ===")
    finally:
        await ble.close()


async def run_interactive_scanner(use_ble: bool, serial_port: str, ble_addr: Optional[str], physical_can: bool):
    bus_mode = VCI_BUS_MODE_PHYSICAL_CAN if physical_can else VCI_BUS_MODE_SIM_LOOPBACK
    bus_label = "PHYSICAL CAN (TWAI GPIO21/22)" if physical_can else "VIRTUAL ECU LOOPBACK"

    if use_ble:
        print(f"Scanning for Bluetooth LE VCI '{VCI_BLE_NAME}'...")
        ble = LeanDpduBleClient(device_name=VCI_BLE_NAME, address=ble_addr)
        peer = await ble.connect_ble()
        print(f"Connected over Bluetooth LE to {peer}")
        await ble.pdu_connect(bus_mode=bus_mode, baud_khz=500, cll_id=1)
        print(f"Logical Link Active: {bus_label} @ 500 kbps")
        try:
            while True:
                print("\n--- ESP32 Bluetooth OBD-II Scanner Menu ---")
                print("  1) Live Telemetry Snapshot (RPM, Speed, Coolant, Throttle)")
                print("  2) Read Vehicle Info (17-Char VIN & Calibration ID)")
                print("  3) Read Stored Diagnostic Trouble Codes (Mode 03)")
                print("  4) Clear Diagnostic Trouble Codes (Mode 04)")
                print("  5) Send Custom Hex OBD-II / UDS Request (e.g. '01 0C' or '09 02')")
                print("  q) Disconnect & Quit")
                choice = input("Select option: ").strip().lower()
                if choice == "1":
                    _, e_rpm = await ble.pdu_start_com_primitive(b"\x01\x0C")
                    _, e_spd = await ble.pdu_start_com_primitive(b"\x01\x0D")
                    _, e_tmp = await ble.pdu_start_com_primitive(b"\x01\x05")
                    _, e_thr = await ble.pdu_start_com_primitive(b"\x01\x11")
                    r_rpm = ble.parse_result_event(e_rpm)
                    r_spd = ble.parse_result_event(e_spd)
                    r_tmp = ble.parse_result_event(e_tmp)
                    r_thr = ble.parse_result_event(e_thr)
                    print(f"  Engine RPM    : {((r_rpm.pdu[2] << 8) | r_rpm.pdu[3]) / 4.0:.1f} RPM")
                    print(f"  Vehicle Speed : {r_spd.pdu[2]} km/h")
                    print(f"  Coolant Temp  : {r_tmp.pdu[2] - 40} °C")
                    print(f"  Throttle Pos  : {(r_thr.pdu[2] * 100.0) / 255.0:.1f}%")
                elif choice == "2":
                    _, e_vin = await ble.pdu_start_com_primitive(b"\x09\x02")
                    _, e_cal = await ble.pdu_start_com_primitive(b"\x09\x04")
                    print(f"  VIN           : {ble.parse_result_event(e_vin).pdu[3:].decode('ascii', errors='replace')}")
                    print(f"  Calibration ID: {ble.parse_result_event(e_cal).pdu[3:].decode('ascii', errors='replace')}")
                elif choice == "3":
                    _, e_dtc = await ble.pdu_start_com_primitive(b"\x03")
                    print(f"  Stored DTCs   : {decode_obd_dtcs(ble.parse_result_event(e_dtc).pdu)}")
                elif choice == "4":
                    await ble.pdu_start_com_primitive(b"\x04")
                    print("  Diagnostic Trouble Codes cleared.")
                elif choice == "5":
                    raw_hex = input("  Enter hex bytes (e.g. 01 0C): ").strip()
                    req_bytes = bytes.fromhex(raw_hex)
                    _, evt = await ble.pdu_start_com_primitive(req_bytes)
                    if evt and evt.msg_type == PDU_IT_RESULT:
                        res = ble.parse_result_event(evt)
                        print(f"  RX CAN 0x{res.rx_can_id:03X} ({res.rx_timestamp_us} us): {res.pdu.hex(' ').upper()}")
                    elif evt:
                        print(f"  Error Event: {evt.payload.hex(' ')}")
                elif choice == "q":
                    await ble.pdu_disconnect(cll_id=1)
                    break
        finally:
            await ble.close()
    else:
        ser = LeanDpduSerialClient(serial_port, 115200)
        ser.pdu_connect(bus_mode=bus_mode, baud_khz=500, cll_id=1)
        print(f"Connected over Serial ({serial_port}) — {bus_label} @ 500 kbps")
        ser.pdu_disconnect(cll_id=1)
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="ESP32 Bluetooth LE & Serial OBD-II Diagnostic Scanner")
    parser.add_argument("--verify-ble", action="store_true", help="Run automated 8-step BLE NUS verification suite")
    parser.add_argument("--ble", action="store_true", help="Connect interactively over Bluetooth LE (default)")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="Serial port path for wired mode")
    parser.add_argument("--address", default=None, help="Optional BLE MAC address of ESP32-OBD-VCI")
    parser.add_argument("--physical-can", action="store_true", help="Use Physical CAN (TWAI GPIO21/22) instead of SimLoopback")
    args = parser.parse_args()

    if args.verify_ble:
        asyncio.run(run_step4_ble_verification(address=args.address))
    else:
        asyncio.run(run_interactive_scanner(use_ble=True, serial_port=args.serial, ble_addr=args.address, physical_can=args.physical_can))


if __name__ == "__main__":
    main()
