#!/usr/bin/env python3
"""
PC Bluetooth LE & USB-Serial OBD-II Diagnostic Scanner Application (Step 4).

Communicates with the ESP32 OBD-II VCI ("ESP32-OBD-VCI") over Bluetooth LE
Nordic UART Service (NUS) or USB-UART0 using the pure binary Lean D-PDU API
(0xA5 SOF + 8B Header + Payload + CRC-16/CCITT-FALSE).
"""

import argparse
import asyncio
import struct
import sys
import time
from typing import Optional, Tuple

from dpdu_client import (
    CP_CAN_PHYS_REQ_ID,
    CP_CAN_RESP_USDT_ID,
    CP_ISOTP_BS,
    CP_ISOTP_STMIN_MS,
    CP_P2_MAX_MS,
    CP_P2_STAR_MS,
    DpduFrame,
    DpduResultEvent,
    DpduStreamParser,
    LeanDpduSerialClient,
    PDU_CMD_CONNECT,
    PDU_CMD_DISCONNECT,
    PDU_CMD_GET_COMPARAM,
    PDU_CMD_GET_STATUS,
    PDU_CMD_SET_COMPARAM,
    PDU_CMD_START_COP,
    PDU_COPT_SENDRECV,
    PDU_COPT_SEND_ONLY,
    PDU_ERR_ISOTP_TIMEOUT,
    PDU_ERR_P2_TIMEOUT,
    PDU_EVT_CMD_ACK,
    PDU_EVT_ERROR,
    PDU_EVT_RESULT,
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
    """Async PC Host Lean D-PDU Client communicating over Bluetooth LE (Nordic UART Service)."""

    def __init__(self, device_name: str = VCI_BLE_NAME, address: Optional[str] = None):
        self.device_name = device_name
        self.address = address
        self.client = None
        self.parser = DpduStreamParser()
        self._rx_queue: asyncio.Queue[DpduFrame] = asyncio.Queue()
        self._seq = 1

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        return s

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
        return await asyncio.wait_for(self._rx_queue.get(), timeout=timeout)

    async def send_and_recv(
        self,
        opcode: int,
        payload: bytes = b"",
        handle: int = 1,
        corrupt_crc: bool = False,
        timeout: float = 2.0,
    ) -> DpduFrame:
        seq = self._next_seq()
        req = DpduFrame(opcode=opcode, handle=handle, seq_tag=seq, payload=payload)
        wire = req.encode(corrupt_crc=corrupt_crc)
        await self.client.write_gatt_char(NUS_RX_CHAR_UUID, wire, response=False)
        return await self.recv_frame(timeout=timeout)

    async def pdu_get_status(self) -> dict:
        rsp = await self.send_and_recv(PDU_CMD_GET_STATUS)
        assert rsp.opcode == PDU_EVT_CMD_ACK
        orig_op, status = rsp.payload[0], rsp.payload[1]
        assert orig_op == PDU_CMD_GET_STATUS and status == PDU_STATUS_NOERROR
        connected, bus_mode, baud_khz, rx_cnt, crc_err, uptime_ms, req_id = struct.unpack(
            "<BBHIIII", rsp.payload[2:22]
        )
        return {
            "connected": bool(connected),
            "bus_mode": bus_mode,
            "baud_khz": baud_khz,
            "rx_cmd_count": rx_cnt,
            "crc_err_count": crc_err,
            "uptime_ms": uptime_ms,
            "can_req_id": req_id,
        }

    async def pdu_connect(
        self, bus_mode: int = VCI_BUS_MODE_SIM_LOOPBACK, baud_khz: int = 500, flags: int = 0
    ) -> DpduFrame:
        payload = struct.pack("<BHB", bus_mode, baud_khz, flags)
        return await self.send_and_recv(PDU_CMD_CONNECT, payload)

    async def pdu_disconnect(self) -> DpduFrame:
        return await self.send_and_recv(PDU_CMD_DISCONNECT)

    async def pdu_set_com_param(self, param_id: int, param_val: int) -> DpduFrame:
        payload = struct.pack("<BI", param_id, param_val)
        return await self.send_and_recv(PDU_CMD_SET_COMPARAM, payload)

    async def pdu_get_com_param(self, param_id: int) -> int:
        rsp = await self.send_and_recv(PDU_CMD_GET_COMPARAM, struct.pack("<B", param_id))
        assert rsp.opcode == PDU_EVT_CMD_ACK and rsp.payload[1] == PDU_STATUS_NOERROR
        _, val = struct.unpack("<BI", rsp.payload[2:7])
        return val

    async def pdu_start_com_primitive(
        self,
        pdu_data: bytes,
        cop_type: int = PDU_COPT_SENDRECV,
        flags: int = 0,
        timeout: float = 2.5,
    ) -> Tuple[DpduFrame, Optional[DpduFrame]]:
        payload = struct.pack("<BB", cop_type, flags) + pdu_data
        ack = await self.send_and_recv(PDU_CMD_START_COP, payload, timeout=timeout)
        if ack.opcode != PDU_EVT_CMD_ACK or ack.payload[1] != PDU_STATUS_NOERROR:
            return ack, None
        if cop_type == PDU_COPT_SEND_ONLY:
            return ack, None
        evt = await self.recv_frame(timeout=timeout)
        return ack, evt

    @staticmethod
    def parse_result_event(evt: DpduFrame) -> DpduResultEvent:
        return LeanDpduSerialClient.parse_result_event(evt)


async def run_step4_ble_verification(address: Optional[str] = None):
    print("=== Step 4 Hardware Verification: Bluetooth LE (NimBLE NUS) End-to-End OBD-II Scanner ===")
    ble = LeanDpduBleClient(device_name=VCI_BLE_NAME, address=address)
    try:
        print(f"[1/8] Scanning & connecting to '{VCI_BLE_NAME}' over Bluetooth LE (NUS)...")
        peer = await ble.connect_ble(scan_timeout=8.0)
        print(f"      Connected over BLE GATT NUS to {peer}")

        print("[2/8] Querying VCI Status over BLE (PDU_CMD_GET_STATUS)...")
        st = await ble.pdu_get_status()
        print(
            f"      VCI Status: uptime={st['uptime_ms']} ms, rx_cmds={st['rx_cmd_count']}, "
            f"crc_errs={st['crc_err_count']}, can_req_id=0x{st['can_req_id']:03X}"
        )

        print("[3/8] Opening Logical Link over BLE (PDU_CMD_CONNECT: SimLoopback, 500 kbps)...")
        ack = await ble.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, baud_khz=500)
        assert ack.opcode == PDU_EVT_CMD_ACK and ack.payload[1] == PDU_STATUS_NOERROR
        print("      Logical Link Connected!")

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

        print(f"      Supported PIDs : {res_00.pdu.hex(' ').upper()} (CAN {res_00.rx_time_ms} ms)")
        print(f"      Engine RPM     : {rpm:.1f} RPM (CAN {res_rpm.rx_time_ms} ms)")
        print(f"      Vehicle Speed  : {speed_kmh} km/h (CAN {res_spd.rx_time_ms} ms)")
        print(f"      Coolant Temp   : {coolant_c} °C (CAN {res_tmp.rx_time_ms} ms)")
        print(f"      Throttle Pos   : {throttle_pct:.1f}% (CAN {res_thr.rx_time_ms} ms)")
        assert rpm == 1726.0 and speed_kmh == 85 and coolant_c == 90

        print("[5/8] Reading 17-Char VIN over BLE via Multi-Frame ISO-TP (Mode 09 PID 0x02)...")
        _, evt_vin = await ble.pdu_start_com_primitive(b"\x09\x02")
        res_vin = ble.parse_result_event(evt_vin)
        vin_str = res_vin.pdu[3:].decode("ascii")
        print(f"      VIN            : '{vin_str}' ({len(res_vin.pdu)} bytes reassembled in {res_vin.rx_time_ms} ms)")
        assert len(res_vin.pdu) == 20

        print("[6/8] Reading Calibration ID over BLE via Multi-Frame ISO-TP (Mode 09 PID 0x04)...")
        _, evt_cal = await ble.pdu_start_com_primitive(b"\x09\x04")
        res_cal = ble.parse_result_event(evt_cal)
        cal_str = res_cal.pdu[3:].decode("ascii")
        print(f"      Calibration ID : '{cal_str}' ({len(res_cal.pdu)} bytes reassembled in {res_cal.rx_time_ms} ms)")
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

        print("[8/8] Disconnecting Logical Link & closing BLE GATT session...")
        ack_disc = await ble.pdu_disconnect()
        assert ack_disc.opcode == PDU_EVT_CMD_ACK and ack_disc.payload[1] == PDU_STATUS_NOERROR
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
        await ble.pdu_connect(bus_mode=bus_mode, baud_khz=500)
        print(f"Logical Link Active: {bus_label} @ 500 kbps")
        try:
            while True:
                print("\n--- ESP32 Bluetooth OBD-II Scanner Menu ---")
                print("  1) Live Telemetry Snapshot (RPM, Speed, Coolant, Throttle)")
                print("  2) Read Vehicle Info (17-Char VIN & Calibration ID)")
                print("  3) Read Stored Diagnostic Trouble Codes (Mode 03)")
                print("  4) Clear Diagnostic Trouble Codes (Mode 04)")
                print("  5) Send Custom Hex OBD-II / UDS Request (e.g. '01 0C' or '09 02')")
                print("  6) VCI Status & ComParams")
                print("  q) Quit")
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
                    print(f"  Engine RPM     : {((r_rpm.pdu[2] << 8) | r_rpm.pdu[3]) / 4.0:.1f} RPM")
                    print(f"  Vehicle Speed  : {r_spd.pdu[2]} km/h")
                    print(f"  Coolant Temp   : {r_tmp.pdu[2] - 40} °C")
                    print(f"  Throttle Pos   : {(r_thr.pdu[2] * 100.0) / 255.0:.1f}%")
                elif choice == "2":
                    _, e_vin = await ble.pdu_start_com_primitive(b"\x09\x02")
                    _, e_cal = await ble.pdu_start_com_primitive(b"\x09\x04")
                    r_vin = ble.parse_result_event(e_vin)
                    r_cal = ble.parse_result_event(e_cal)
                    print(f"  VIN (Mode 09 02)    : {r_vin.pdu[3:].decode('ascii', errors='replace')}")
                    print(f"  Cal ID (Mode 09 04) : {r_cal.pdu[3:].decode('ascii', errors='replace')}")
                elif choice == "3":
                    _, e_dtc = await ble.pdu_start_com_primitive(b"\x03")
                    r_dtc = ble.parse_result_event(e_dtc)
                    dtcs = decode_obd_dtcs(r_dtc.pdu)
                    print(f"  Stored DTCs ({len(dtcs)}) : {', '.join(dtcs) if dtcs else 'None'}")
                elif choice == "4":
                    _, e_clr = await ble.pdu_start_com_primitive(b"\x04")
                    r_clr = ble.parse_result_event(e_clr)
                    print(f"  Clear DTCs Response : {r_clr.pdu.hex(' ').upper()}")
                elif choice == "5":
                    raw_hex = input("  Enter hex PDU bytes (e.g. 01 0C): ").strip()
                    pdu_req = bytes.fromhex(raw_hex)
                    _, evt = await ble.pdu_start_com_primitive(pdu_req)
                    if evt.opcode == PDU_EVT_RESULT:
                        res = ble.parse_result_event(evt)
                        print(f"  RX [0x{res.rx_can_id:03X}, {res.rx_time_ms} ms]: {res.pdu.hex(' ').upper()}")
                    else:
                        print(f"  ERROR Event: err=0x{evt.payload[0]:02X}, detail=0x{evt.payload[1]:02X}")
                elif choice == "6":
                    st = await ble.pdu_get_status()
                    print(f"  Status: {st}")
                elif choice in ("q", "quit", "exit"):
                    break
        finally:
            await ble.pdu_disconnect()
            await ble.close()
    else:
        ser = LeanDpduSerialClient(serial_port, 115200)
        ser.pdu_connect(bus_mode=bus_mode, baud_khz=500)
        print(f"Connected over USB-Serial ({serial_port}) — {bus_label} @ 500 kbps")
        ser.pdu_disconnect()
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="PC Bluetooth LE & Serial OBD-II Scanner App")
    parser.add_argument("--verify-ble", action="store_true", help="Run automated Step 4 BLE verification suite")
    parser.add_argument("--ble-addr", type=str, default=None, help="Optional BLE MAC address of ESP32-OBD-VCI")
    parser.add_argument("--serial", type=str, default=None, help="Use USB-Serial port instead of BLE (e.g. /dev/ttyUSB0)")
    parser.add_argument("--physical-can", action="store_true", help="Use Physical TWAI CAN bus instead of Virtual ECU")
    args = parser.parse_args()

    if args.verify_ble:
        asyncio.run(run_step4_ble_verification(address=args.ble_addr))
    else:
        use_ble = args.serial is None
        asyncio.run(run_interactive_scanner(use_ble, args.serial or "/dev/ttyUSB0", args.ble_addr, args.physical_can))


if __name__ == "__main__":
    main()

