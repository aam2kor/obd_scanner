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
from typing import Dict, List, Optional, Tuple

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
    PDU_CMD_CANCEL_COP,
    PDU_CMD_CONNECT,
    PDU_CMD_DISCONNECT,
    PDU_CMD_GET_COMPARAM,
    PDU_CMD_GET_STATUS,
    PDU_CMD_MOD_CTRL,
    PDU_CMD_SET_COMPARAM,
    PDU_CMD_START_COP,
    PDU_COPST_CANCELLED,
    PDU_COPST_EXECUTING,
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

# Standard OBD-II Mode 01 PID names for Supported PID Bitmap Decoding (PIDs 0x01..0x20)
OBD_MODE01_PID_NAMES: Dict[int, str] = {
    0x01: "Monitor Status Since DTCs Cleared",
    0x03: "Fuel System Status",
    0x04: "Calculated Engine Load",
    0x05: "Engine Coolant Temperature",
    0x06: "Short Term Fuel Trim — Bank 1",
    0x07: "Long Term Fuel Trim — Bank 1",
    0x0B: "Intake Manifold Absolute Pressure",
    0x0C: "Engine RPM",
    0x0D: "Vehicle Speed",
    0x0E: "Timing Advance",
    0x0F: "Intake Air Temperature",
    0x10: "MAF Air Flow Rate",
    0x11: "Throttle Position",
    0x13: "Oxygen Sensors Present (2 Banks)",
    0x15: "Oxygen Sensor 2 (Voltage & Fuel Trim)",
    0x1C: "OBD Standards This Vehicle Conforms To",
    0x1F: "Run Time Since Engine Start",
    0x20: "PIDs Supported [21 - 40]",
}

# All 13 ISO 22900-2 ComParams supported per CLL
COMPARAM_CATALOG: List[Tuple[int, str, str]] = [
    (CP_BAUDRATE, "CP_Baudrate", "bps"),
    (CP_ADDRESSING_TYPE, "CP_AddressingType", "0=Func, 1=Phys"),
    (CP_CAN_PHYS_REQ_ID, "CP_CanPhysReqId", "hex CAN ID"),
    (CP_CAN_RESP_USDT_ID, "CP_CanRespUsdtId", "hex CAN ID"),
    (CP_CAN_FILLER_BYTE, "CP_CanFillerByte", "hex byte"),
    (CP_BLOCK_SIZE, "CP_BlockSize", "frames (0=unlim)"),
    (CP_ST_MIN, "CP_STmin", "0-7F ms / F1-F9 100us"),
    (CP_N_BS_MS, "CP_N_Bs", "ms"),
    (CP_N_CR_MS, "CP_N_Cr", "ms"),
    (CP_P2_MAX_MS, "CP_P2Max", "ms"),
    (CP_P2_STAR_MS, "CP_P2Star", "ms"),
    (CP_P3_PHYS_MS, "CP_P3Phys", "ms"),
    (CP_P3_FUNC_MS, "CP_P3Func", "ms"),
]

# Standard Multi-ECU Physical CAN ID Table (ISO 15765-4)
STANDARD_ECU_PROFILES: Dict[int, Tuple[str, int, int]] = {
    1: ("ECM (Engine Control Module)", 0x7E0, 0x7E8),
    2: ("TCM (Transmission Control Module)", 0x7E1, 0x7E9),
    3: ("ABS / ESC Module", 0x7E2, 0x7EA),
    4: ("BCM (Body Control Module)", 0x7E3, 0x7EB),
}


def decode_supported_pids(pdu: bytes) -> List[Tuple[int, str]]:
    """Decode a 6-byte Mode 01 PID 00 response (`41 00 B0 B1 B2 B3`) into supported PID tuples."""
    if len(pdu) < 6 or pdu[0] != 0x41 or pdu[1] != 0x00:
        return []
    bitmap = struct.unpack(">I", pdu[2:6])[0]
    supported: List[Tuple[int, str]] = []
    for bit_idx in range(32):
        pid = bit_idx + 1
        if bitmap & (1 << (31 - bit_idx)):
            supported.append((pid, OBD_MODE01_PID_NAMES.get(pid, f"SAE Standard PID 0x{pid:02X}")))
    return supported


def format_version_u32(ver: int) -> str:
    """Format a 32-bit version integer (`0x00MMmmpp`) as `vMajor.Minor.Patch`."""
    major = (ver >> 16) & 0xFF
    minor = (ver >> 8) & 0xFF
    patch = ver & 0xFF
    return f"v{major}.{minor}.{patch} (0x{ver:08X})"


def format_stmin_value(val: int) -> str:
    """Format an ISO 15765-2 STmin value (`0x00..0x7F` ms or `0xF1..0xF9` 100-900 us)."""
    if 0xF1 <= val <= 0xF9:
        return f"0x{val:02X} ({(val - 0xF0) * 100} µs)"
    return f"0x{val:02X} ({val} ms)"


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
        if msg_type != PDU_CMD_CANCEL_COP and cop_id == 0:
            cop_id = self.alloc_cop_id()
        req = DpduFrame(msg_type=msg_type, cll_id=cll_id, cop_id=cop_id, status=0, payload=payload)
        encoded = req.encode()
        skipped: list[DpduFrame] = []
        for _attempt in range(3):
            await self.client.write_gatt_char(NUS_RX_CHAR_UUID, encoded, response=False)
            attempt_deadline = time.time() + min(0.5, timeout)
            try:
                while time.time() < attempt_deadline:
                    f = await self.recv_frame(timeout=max(0.05, attempt_deadline - time.time()))
                    if f.msg_type == PDU_EVT_CMD_ACK and f.cll_id == cll_id and f.cop_id == cop_id:
                        self._deferred = skipped + self._deferred
                        return f
                    skipped.append(f)
            except asyncio.TimeoutError:
                continue
        raise TimeoutError("Timed out waiting for 0x80 synchronous Command ACK over BLE")

    async def pdu_ioctl(self, ioctl_id: int) -> DpduFrame:
        return await self.send_and_recv_ack(
            PDU_CMD_MOD_CTRL,
            payload=struct.pack("<H", ioctl_id),
            cll_id=0,
            cop_id=0,
        )

    async def read_vbatt_mv(self) -> int:
        """Read vehicle Pin 16 battery voltage in millivolts via IOCTL_READ_VBATT (0x0001)."""
        ack = await self.pdu_ioctl(PDU_IOCTL_READ_VBATT)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 4
        return struct.unpack("<I", ack.payload)[0]

    async def get_cable_id(self) -> int:
        """Query connected diagnostic cable ID via IOCTL_GET_CABLE_ID (0x0002)."""
        ack = await self.pdu_ioctl(PDU_IOCTL_GET_CABLE_ID)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 4
        return struct.unpack("<I", ack.payload)[0]

    async def get_version_info(self) -> dict:
        """Query VCI Firmware Version, Hardware Version, and eFuse MAC/Serial via PDUGetVersion (0x0003)."""
        ack = await self.pdu_ioctl(PDU_IOCTL_GET_VERSION)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 16
        fw_ver, hw_ver = struct.unpack("<II", ack.payload[:8])
        mac_str = ":".join(f"{b:02X}" for b in ack.payload[8:14])
        return {
            "fw_version": fw_ver,
            "fw_str": format_version_u32(fw_ver),
            "hw_version": hw_ver,
            "hw_str": format_version_u32(hw_ver),
            "serial_mac": mac_str,
        }

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
            deadline = time.time() + 1.0
            while time.time() < deadline:
                st_evt = await self.recv_frame(timeout=max(0.05, deadline - time.time()))
                if st_evt.msg_type == PDU_IT_STATUS and st_evt.cll_id == cll_id:
                    break
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

    async def pdu_cancel_com_primitive(self, cll_id: int = 1, cop_id: int = 0) -> DpduFrame:
        """Send Out-of-Band CancelComPrimitive (0x05); cop_id=0 drains all 4 queue slots on cll_id."""
        return await self.send_and_recv_ack(PDU_CMD_CANCEL_COP, cll_id=cll_id, cop_id=cop_id)

    async def pdu_start_cyclic_cop(
        self,
        pdu_data: bytes,
        num_send_cycles: int = 20,
        cycle_time_ms: int = 25,
        cll_id: int = 1,
        can_id: int = 0,
    ) -> Tuple[int, DpduFrame]:
        """Enqueue an autonomous VCI Cyclic StartComPrimitive and return (cop_id, ack)."""
        cop_id = self.alloc_cop_id()
        payload = LeanDpduSerialClient.build_start_cop_payload(
            pdu_data,
            num_send_cycles=num_send_cycles,
            num_recv_cycles=1,
            cycle_time_ms=cycle_time_ms,
            can_id=can_id,
        )
        ack = await self.send_and_recv_ack(PDU_CMD_START_COP, payload=payload, cll_id=cll_id, cop_id=cop_id)
        return cop_id, ack

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
        ack = await self.send_and_recv_ack(
            PDU_CMD_START_COP,
            payload=payload,
            cll_id=cll_id,
            cop_id=cop_id,
            timeout=timeout,
        )
        if ack.status != PDU_STATUS_NOERROR:
            return ack, None

        exec_ts_us: Optional[int] = None
        outcome_evt: Optional[DpduFrame] = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            f = await self.recv_frame(timeout=max(0.05, deadline - time.time()))
            if f.cll_id != cll_id or f.cop_id != cop_id:
                continue
            if f.msg_type in (PDU_IT_RESULT, PDU_IT_ERROR):
                outcome_evt = f
                setattr(outcome_evt, "_elapsed_ms", max(1, int((time.time() - t0) * 1000)))
                if exec_ts_us is not None and f.msg_type == PDU_IT_RESULT:
                    res_tmp = LeanDpduSerialClient.parse_result_event(f)
                    setattr(outcome_evt, "_vci_can_us", (res_tmp.rx_timestamp_us - exec_ts_us) & 0xFFFFFFFF)
            elif f.msg_type == PDU_IT_STATUS:
                st = LeanDpduSerialClient.parse_status_event(f)
                if st.status_code == PDU_COPST_EXECUTING and exec_ts_us is None:
                    exec_ts_us = st.timestamp_us
                elif st.status_code in (PDU_COPST_FINISHED, PDU_COPST_CANCELLED):
                    break
        return ack, outcome_evt

    @staticmethod
    def parse_result_event(evt: DpduFrame) -> DpduResultEvent:
        return LeanDpduSerialClient.parse_result_event(evt)


class AsyncSerialVciAdapter:
    """Async adapter wrapping LeanDpduSerialClient so the interactive scanner runs identically on BLE and Serial."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        self.ser_client = LeanDpduSerialClient(port, baud)
        self.port = port

    async def close(self):
        self.ser_client.close()

    async def recv_frame(self, timeout: float = 1.0) -> DpduFrame:
        return self.ser_client.recv_frame(timeout=timeout)

    async def pdu_ioctl(self, ioctl_id: int) -> DpduFrame:
        return self.ser_client.pdu_ioctl(ioctl_id)

    async def read_vbatt_mv(self) -> int:
        ack = self.ser_client.pdu_ioctl(PDU_IOCTL_READ_VBATT)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 4
        return struct.unpack("<I", ack.payload)[0]

    async def get_cable_id(self) -> int:
        ack = self.ser_client.pdu_ioctl(PDU_IOCTL_GET_CABLE_ID)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 4
        return struct.unpack("<I", ack.payload)[0]

    async def get_version_info(self) -> dict:
        ack = self.ser_client.pdu_ioctl(PDU_IOCTL_GET_VERSION)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 16
        fw_ver, hw_ver = struct.unpack("<II", ack.payload[:8])
        mac_str = ":".join(f"{b:02X}" for b in ack.payload[8:14])
        return {
            "fw_version": fw_ver,
            "fw_str": format_version_u32(fw_ver),
            "hw_version": hw_ver,
            "hw_str": format_version_u32(hw_ver),
            "serial_mac": mac_str,
        }

    async def pdu_get_status(self, cll_id: int = 1) -> dict:
        return self.ser_client.pdu_get_status(cll_id=cll_id)

    async def pdu_connect(
        self,
        bus_mode: int = VCI_BUS_MODE_SIM_LOOPBACK,
        baud_khz: int = 0,
        flags: int = 0,
        cll_id: int = 1,
    ) -> DpduFrame:
        return self.ser_client.pdu_connect(bus_mode=bus_mode, baud_khz=baud_khz, flags=flags, cll_id=cll_id)

    async def pdu_disconnect(self, cll_id: int = 1) -> DpduFrame:
        return self.ser_client.pdu_disconnect(cll_id=cll_id)

    async def pdu_set_com_param(self, param_id: int, param_val: int, cll_id: int = 1) -> DpduFrame:
        return self.ser_client.pdu_set_com_param(param_id, param_val, cll_id=cll_id)

    async def pdu_get_com_param(self, param_id: int, cll_id: int = 1) -> int:
        return self.ser_client.pdu_get_com_param(param_id, cll_id=cll_id)

    async def pdu_cancel_com_primitive(self, cll_id: int = 1, cop_id: int = 0) -> DpduFrame:
        return self.ser_client.pdu_cancel_com_primitive(cll_id=cll_id, cop_id=cop_id)

    async def pdu_start_cyclic_cop(
        self,
        pdu_data: bytes,
        num_send_cycles: int = 20,
        cycle_time_ms: int = 25,
        cll_id: int = 1,
        can_id: int = 0,
    ) -> Tuple[int, DpduFrame]:
        cop_id = self.ser_client.alloc_cop_id()
        payload = LeanDpduSerialClient.build_start_cop_payload(
            pdu_data,
            num_send_cycles=num_send_cycles,
            num_recv_cycles=1,
            cycle_time_ms=cycle_time_ms,
            can_id=can_id,
        )
        ack = self.ser_client.send_and_recv_ack(PDU_CMD_START_COP, payload=payload, cll_id=cll_id, cop_id=cop_id)
        return cop_id, ack

    async def pdu_start_com_primitive(
        self,
        pdu_data: bytes,
        cop_type: int = PDU_COPT_SENDRECV,
        flags: int = 0,
        timeout: float = 2.5,
        cll_id: int = 1,
        can_id: int = 0,
    ) -> Tuple[DpduFrame, Optional[DpduFrame]]:
        return self.ser_client.pdu_start_com_primitive(
            pdu_data,
            cop_type=cop_type,
            flags=flags,
            timeout=timeout,
            cll_id=cll_id,
            can_id=can_id,
        )

    @staticmethod
    def parse_result_event(evt: DpduFrame) -> DpduResultEvent:
        return LeanDpduSerialClient.parse_result_event(evt)


async def display_vci_hardware_dashboard(vci, active_cll: int = 1):
    """Feature 1: Display Battery Voltage, Cable ID, Firmware/Hardware Version, and VCI Health Status."""
    vbatt_mv = await vci.read_vbatt_mv()
    cable_id = await vci.get_cable_id()
    ver = await vci.get_version_info()
    st = await vci.pdu_get_status(cll_id=active_cll)

    cable_desc = "SAE J1962 16-Pin OBD-II DLC" if cable_id == 0x07AA else f"Custom (0x{cable_id:04X})"
    online_clls = [str(c) for c in range(1, 9) if st["online_mask"] & (1 << (c - 1))]
    online_str = ", ".join(f"CLL {c}" for c in online_clls) if online_clls else "None (All Offline)"

    print("\n  ================ VCI HARDWARE & BATTERY DASHBOARD ================")
    print(f"  Pin 16 Battery Voltage : {vbatt_mv / 1000.0:.2f} V ({vbatt_mv} mV)")
    print(f"  Diagnostic Cable ID    : 0x{cable_id:04X} ({cable_desc})")
    print(f"  Firmware Version       : {ver['fw_str']}")
    print(f"  Hardware Version       : {ver['hw_str']}")
    print(f"  Factory Serial / MAC   : {ver['serial_mac']}")
    print(f"  VCI Uptime             : {st['uptime_ms'] / 1000.0:.2f} s ({st['uptime_ms']} ms)")
    print(f"  Active CLL Mask        : 0x{st['online_mask']:02X} ({online_str})")
    print(f"  Locked CAN Bus Rate    : {st['baud_khz']} kbps (Mode={st['bus_mode']})")
    print(f"  Total Commands / Errors: {st['rx_cmd_count']} RX cmds | {st['crc_err_count']} CRC errors")
    print("  ==================================================================")


async def display_telemetry_snapshot_and_pids(vci, active_cll: int = 1):
    """Feature 2: Decode Supported PIDs (01 00) and read live telemetry with 100 µs VCI timestamps."""
    _, e_00 = await vci.pdu_start_com_primitive(b"\x01\x00", cll_id=active_cll)
    _, e_rpm = await vci.pdu_start_com_primitive(b"\x01\x0C", cll_id=active_cll)
    _, e_spd = await vci.pdu_start_com_primitive(b"\x01\x0D", cll_id=active_cll)
    _, e_tmp = await vci.pdu_start_com_primitive(b"\x01\x05", cll_id=active_cll)
    _, e_thr = await vci.pdu_start_com_primitive(b"\x01\x11", cll_id=active_cll)

    r_00 = vci.parse_result_event(e_00)
    r_rpm = vci.parse_result_event(e_rpm)
    r_spd = vci.parse_result_event(e_spd)
    r_tmp = vci.parse_result_event(e_tmp)
    r_thr = vci.parse_result_event(e_thr)

    rpm = ((r_rpm.pdu[2] << 8) | r_rpm.pdu[3]) / 4.0
    speed_kmh = r_spd.pdu[2]
    coolant_c = r_tmp.pdu[2] - 40
    throttle_pct = (r_thr.pdu[2] * 100.0) / 255.0
    vbatt_mv = await vci.read_vbatt_mv()

    supported = decode_supported_pids(r_00.pdu)
    print(f"\n  --- Live OBD-II Telemetry Snapshot (CLL {active_cll}, RX CAN 0x{r_rpm.rx_can_id:X}) ---")
    print(f"  Battery Voltage : {vbatt_mv / 1000.0:.2f} V ({vbatt_mv} mV)")
    print(f"  Engine RPM      : {rpm:7.1f} RPM   [VCI ts={r_rpm.rx_timestamp_us} µs]")
    print(f"  Vehicle Speed   : {speed_kmh:7d} km/h  [VCI ts={r_spd.rx_timestamp_us} µs]")
    print(f"  Coolant Temp    : {coolant_c:7d} °C    [VCI ts={r_tmp.rx_timestamp_us} µs]")
    print(f"  Throttle Pos    : {throttle_pct:7.1f} %     [VCI ts={r_thr.rx_timestamp_us} µs]")
    print(f"\n  Supported Mode 01 PIDs ({len(supported)} decoded from {r_00.pdu.hex(' ').upper()}):")
    for pid, name in supported:
        print(f"    - PID 0x{pid:02X}: {name}")


async def run_cyclic_live_stream(vci, active_cll: int = 1, cycles_per_pid: int = 15, cycle_time_ms: int = 25):
    """Feature 3: Autonomous VCI Cyclic StartComPrimitive streaming for RPM, Speed & Coolant Temp."""
    print(
        f"\n  Starting Autonomous VCI Cyclic Stream on CLL {active_cll} "
        f"({cycles_per_pid} cycles × 3 PIDs @ {cycle_time_ms} ms interval)..."
    )
    cop_rpm, ack1 = await vci.pdu_start_cyclic_cop(b"\x01\x0C", cycles_per_pid, cycle_time_ms, cll_id=active_cll)
    cop_spd, ack2 = await vci.pdu_start_cyclic_cop(b"\x01\x0D", cycles_per_pid, cycle_time_ms, cll_id=active_cll)
    cop_tmp, ack3 = await vci.pdu_start_cyclic_cop(b"\x01\x05", cycles_per_pid, cycle_time_ms, cll_id=active_cll)
    assert ack1.status == PDU_STATUS_NOERROR and ack2.status == PDU_STATUS_NOERROR and ack3.status == PDU_STATUS_NOERROR

    active_cops = {cop_rpm, cop_spd, cop_tmp}
    finished_cops = set()
    latest = {"rpm": 0.0, "speed": 0, "coolant": 0, "ts_us": 0}
    rx_results = 0
    t_start = time.perf_counter()

    try:
        while len(finished_cops) < len(active_cops):
            f = await vci.recv_frame(timeout=2.0)
            if f.cll_id != active_cll or f.cop_id not in active_cops:
                continue
            if f.msg_type == PDU_IT_RESULT:
                res = vci.parse_result_event(f)
                rx_results += 1
                latest["ts_us"] = res.rx_timestamp_us
                if len(res.pdu) >= 4 and res.pdu[0] == 0x41 and res.pdu[1] == 0x0C:
                    latest["rpm"] = ((res.pdu[2] << 8) | res.pdu[3]) / 4.0
                elif len(res.pdu) >= 3 and res.pdu[0] == 0x41 and res.pdu[1] == 0x0D:
                    latest["speed"] = res.pdu[2]
                elif len(res.pdu) >= 3 and res.pdu[0] == 0x41 and res.pdu[1] == 0x05:
                    latest["coolant"] = res.pdu[2] - 40
                if rx_results % 3 == 0:
                    elapsed = max(0.001, time.perf_counter() - t_start)
                    rate = rx_results / elapsed
                    print(
                        f"    [Sweep #{rx_results // 3:02d} | {rate:5.1f} PIDs/s | VCI ts={latest['ts_us']} µs] "
                        f"RPM={latest['rpm']:.1f} | Speed={latest['speed']} km/h | Coolant={latest['coolant']} °C"
                    )
            elif f.msg_type == PDU_IT_STATUS:
                st = LeanDpduSerialClient.parse_status_event(f)
                if st.status_code in (PDU_COPST_FINISHED, PDU_COPST_CANCELLED):
                    finished_cops.add(f.cop_id)
    except KeyboardInterrupt:
        print("\n  Aborting stream via Out-of-Band CancelComPrimitive (0x05)...")
        await vci.pdu_cancel_com_primitive(cll_id=active_cll, cop_id=0)

    total_s = max(0.001, time.perf_counter() - t_start)
    print(
        f"  Cyclic Stream Completed: {rx_results} PID results in {total_s * 1000.0:.1f} ms "
        f"({rx_results / total_s:.1f} PIDs/s, {(rx_results / 3.0) / total_s:.1f} Hz sweep rate)."
    )
    return rx_results


async def write_and_verify_vin_uds(vci, new_vin: str, active_cll: int = 1) -> str:
    """Feature 5: Write a 17-character VIN via UDS 0x2E F1 90 (Multi-Frame TX) and verify via Mode 09 PID 02."""
    vin_clean = new_vin.strip().encode("ascii", errors="replace")[:17].ljust(17, b"0")
    uds_req = b"\x2E\xF1\x90" + vin_clean
    _, evt_w = await vci.pdu_start_com_primitive(uds_req, cll_id=active_cll)
    res_w = vci.parse_result_event(evt_w)
    assert res_w.pdu == b"\x6E\xF1\x90", f"Expected positive UDS 0x6E F1 90 response, got {res_w.pdu.hex(' ')}"

    _, evt_r = await vci.pdu_start_com_primitive(b"\x09\x02", cll_id=active_cll)
    res_r = vci.parse_result_event(evt_r)
    readback_vin = res_r.pdu[3:].decode("ascii", errors="replace")
    return readback_vin


async def discover_and_poll_multi_cll(vci, bus_mode: int = VCI_BUS_MODE_SIM_LOOPBACK) -> List[dict]:
    """Feature 3 (Multi-ECU): Open 4 concurrent physical CLLs (ECM, TCM, ABS, BCM) and poll in parallel."""
    results = []
    for cll_id, (ecu_name, req_id, resp_id) in STANDARD_ECU_PROFILES.items():
        await vci.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=cll_id)
        await vci.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL, cll_id=cll_id)
        await vci.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, req_id, cll_id=cll_id)
        await vci.pdu_set_com_param(CP_CAN_RESP_USDT_ID, resp_id, cll_id=cll_id)
        await vci.pdu_set_com_param(CP_P3_PHYS_MS, 5, cll_id=cll_id)
        await vci.pdu_connect(bus_mode=bus_mode, cll_id=cll_id)

    for cll_id, (ecu_name, req_id, resp_id) in STANDARD_ECU_PROFILES.items():
        _, evt = await vci.pdu_start_com_primitive(b"\x01\x0C", cll_id=cll_id)
        res = vci.parse_result_event(evt)
        rpm = ((res.pdu[2] << 8) | res.pdu[3]) / 4.0
        results.append(
            {
                "cll_id": cll_id,
                "ecu_name": ecu_name,
                "req_id": req_id,
                "rx_can_id": res.rx_can_id,
                "ts_us": res.rx_timestamp_us,
                "rpm": rpm,
            }
        )

    # Disconnect auxiliary CLLs 2..4 while keeping CLL 1 online
    for cll_id in (2, 3, 4):
        await vci.pdu_disconnect(cll_id=cll_id)
    return results


async def inspect_com_params(vci, cll_id: int = 1) -> Dict[int, int]:
    """Feature 4: Query and print all 13 ISO 22900-2 ComParams on `cll_id`."""
    values: Dict[int, int] = {}
    print(f"\n  ================ ISO 22900-2 COMPARAM TABLE (CLL {cll_id}) ================")
    for idx, (param_id, name, unit) in enumerate(COMPARAM_CATALOG, start=1):
        val = await vci.pdu_get_com_param(param_id, cll_id=cll_id)
        values[param_id] = val
        if param_id in (CP_CAN_PHYS_REQ_ID, CP_CAN_RESP_USDT_ID):
            val_str = f"0x{val:X}"
        elif param_id == CP_CAN_FILLER_BYTE:
            val_str = f"0x{val:02X}"
        elif param_id == CP_ST_MIN:
            val_str = format_stmin_value(val)
        elif param_id == CP_ADDRESSING_TYPE:
            val_str = "PHYSICAL (1)" if val == DPDU_ADDR_PHYSICAL else "FUNCTIONAL (0)"
        else:
            val_str = f"{val}"
        print(f"  {idx:2d}) [0x{param_id:04X}] {name:<20} = {val_str:<18} ({unit})")
    print("  =====================================================================")
    return values


async def run_step4_ble_verification(address: Optional[str] = None):
    print("=== Step 4 Hardware Verification: Bluetooth LE (NimBLE NUS) End-to-End App-Device IF ===")
    ble = LeanDpduBleClient(device_name=VCI_BLE_NAME, address=address)
    try:
        print(f"[1/8] Scanning & connecting to '{VCI_BLE_NAME}' over Bluetooth LE (NUS)...")
        peer = await ble.connect_ble(scan_timeout=8.0)
        print(f"      Connected over BLE GATT NUS to {peer}")

        print("[2/8] Querying VCI Hardware Dashboard (VBATT, CableID, PDUGetVersion, Status) over BLE...")
        st = await ble.pdu_get_status()
        vbatt_mv = await ble.read_vbatt_mv()
        cable_id = await ble.get_cable_id()
        ver = await ble.get_version_info()
        print(
            f"      VBATT={vbatt_mv / 1000.0:.2f}V ({vbatt_mv} mV), Cable=0x{cable_id:04X}, "
            f"FW={ver['fw_str']}, Serial={ver['serial_mac']}, crc_errs={st['crc_err_count']}"
        )
        assert st["crc_err_count"] == 0 and vbatt_mv == 12600 and cable_id == 0x07AA

        print("[3/8] Configuring CP_BAUDRATE=500000 bps & Opening CLL 1 over BLE...")
        await ble.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=1)
        ack = await ble.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=1)
        assert ack.status == PDU_STATUS_NOERROR
        print("      CLL 1 ONLINE (CP_BAUDRATE=500000 bps)!")

        print("[4/8] Reading Live OBD-II Telemetry & Decoding Supported PIDs over BLE...")
        _, evt_00 = await ble.pdu_start_com_primitive(b"\x01\x00")
        res_00 = ble.parse_result_event(evt_00)
        supported = decode_supported_pids(res_00.pdu)

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

        print(f"      Supported PIDs : {len(supported)} PIDs decoded from {res_00.pdu.hex(' ').upper()}")
        print(f"      Engine RPM     : {rpm:.1f} RPM (ts={res_rpm.rx_timestamp_us} us)")
        print(f"      Vehicle Speed  : {speed_kmh} km/h")
        print(f"      Coolant Temp   : {coolant_c} °C")
        print(f"      Throttle Pos   : {throttle_pct:.1f}%")
        assert rpm == 1726.0 and speed_kmh == 85 and coolant_c == 90 and len(supported) > 0

        print("[5/8] Reading & Writing 17-Char VIN over BLE (Mode 09 PID 02 + UDS 0x2E F1 90)...")
        _, evt_vin = await ble.pdu_start_com_primitive(b"\x09\x02")
        res_vin = ble.parse_result_event(evt_vin)
        orig_vin = res_vin.pdu[3:].decode("ascii")
        updated_vin = await write_and_verify_vin_uds(ble, "WBA3A5C51DF123456", active_cll=1)
        restored_vin = await write_and_verify_vin_uds(ble, orig_vin, active_cll=1)
        print(f"      Original VIN='{orig_vin}' -> UDS 0x2E Updated='{updated_vin}' -> Restored='{restored_vin}'")
        assert updated_vin == "WBA3A5C51DF123456" and restored_vin == orig_vin

        print("[6/8] Testing Autonomous Cyclic Stream & Multi-ECU (CLL 1..4) Discovery over BLE...")
        rx_cnt = await run_cyclic_live_stream(ble, active_cll=1, cycles_per_pid=4, cycle_time_ms=25)
        assert rx_cnt == 12
        ecu_list = await discover_and_poll_multi_cll(ble, bus_mode=VCI_BUS_MODE_SIM_LOOPBACK)
        ecu_summary = [f"CLL{e['cll_id']}:0x{e['rx_can_id']:X}" for e in ecu_list]
        print(f"      Multi-ECU Poll : {ecu_summary}")
        assert len(ecu_list) == 4

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
        await ble.close()
        await asyncio.sleep(0.1)
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
        vci = LeanDpduBleClient(device_name=VCI_BLE_NAME, address=ble_addr)
        peer = await vci.connect_ble()
        transport_label = f"Bluetooth LE NUS ({peer})"
    else:
        vci = AsyncSerialVciAdapter(serial_port, 115200)
        transport_label = f"USB-UART0 ({serial_port} @ 115200 baud)"

    active_cll = 1
    await vci.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=active_cll)
    await vci.pdu_connect(bus_mode=bus_mode, cll_id=active_cll)

    vbatt_mv = await vci.read_vbatt_mv()
    ver = await vci.get_version_info()
    print(f"Connected over {transport_label}")
    print(
        f"Active Link: CLL {active_cll} | {bus_label} @ 500 kbps | "
        f"VBATT={vbatt_mv / 1000.0:.2f} V | FW {ver['fw_str']}"
    )

    try:
        while True:
            print(f"\n--- ESP32 ISO 22900-2 OBD-II Scanner Menu [Active CLL {active_cll}] ---")
            print("  1) VCI Hardware, Battery Voltage (VBATT), Cable ID & Health Dashboard")
            print("  2) Live Telemetry Snapshot + Supported Mode 01 PID Bitmap Decoder")
            print("  3) High-Speed Continuous Dashboard Stream (Autonomous VCI Cyclic COP)")
            print("  4) Read / Write Vehicle Identity (Mode 09 VIN & Cal ID + UDS 0x2E Write VIN)")
            print("  5) Read & Clear Diagnostic Trouble Codes (Mode 03 / Mode 04)")
            print("  6) Multi-ECU / Multi-CLL Discovery & Active CLL Switcher (CLL 1..8)")
            print("  7) Runtime ISO 22900-2 ComParam Inspector & Tuner (13 ComParams)")
            print("  8) Send Custom Hex OBD-II / UDS Request (11-bit or 29-bit CAN)")
            print("  q) Disconnect & Quit")
            choice = input("Select option: ").strip().lower()

            if choice == "1":
                await display_vci_hardware_dashboard(vci, active_cll=active_cll)

            elif choice == "2":
                await display_telemetry_snapshot_and_pids(vci, active_cll=active_cll)

            elif choice == "3":
                raw_cyc = input("  Number of sweeps per PID [default 15]: ").strip()
                n_cyc = int(raw_cyc) if raw_cyc.isdigit() and int(raw_cyc) > 0 else 15
                raw_ms = input("  Cycle interval in ms [default 25]: ").strip()
                c_ms = int(raw_ms) if raw_ms.isdigit() and int(raw_ms) >= 5 else 25
                await run_cyclic_live_stream(vci, active_cll=active_cll, cycles_per_pid=n_cyc, cycle_time_ms=c_ms)

            elif choice == "4":
                _, e_vin = await vci.pdu_start_com_primitive(b"\x09\x02", cll_id=active_cll)
                _, e_cal = await vci.pdu_start_com_primitive(b"\x09\x04", cll_id=active_cll)
                print(f"  Current 17-Char VIN : {vci.parse_result_event(e_vin).pdu[3:].decode('ascii', errors='replace')}")
                print(f"  Calibration ID      : {vci.parse_result_event(e_cal).pdu[3:].decode('ascii', errors='replace')}")
                sub = input("  Write new VIN via UDS 0x2E F1 90? (y/N): ").strip().lower()
                if sub == "y":
                    new_vin = input("  Enter 17-character VIN: ").strip()
                    if new_vin:
                        rb_vin = await write_and_verify_vin_uds(vci, new_vin, active_cll=active_cll)
                        print(f"  UDS 0x2E Write Confirmed! Mode 09 PID 02 Readback VIN: '{rb_vin}'")

            elif choice == "5":
                _, e_dtc = await vci.pdu_start_com_primitive(b"\x03", cll_id=active_cll)
                dtcs = decode_obd_dtcs(vci.parse_result_event(e_dtc).pdu)
                print(f"  Stored DTCs ({len(dtcs)}): {dtcs if dtcs else 'None (No Fault Codes)'}")
                if dtcs:
                    clr = input("  Clear Stored DTCs via Mode 04? (y/N): ").strip().lower()
                    if clr == "y":
                        await vci.pdu_start_com_primitive(b"\x04", cll_id=active_cll)
                        _, e_after = await vci.pdu_start_com_primitive(b"\x03", cll_id=active_cll)
                        print(f"  DTCs after Mode 04 Clear: {decode_obd_dtcs(vci.parse_result_event(e_after).pdu)}")

            elif choice == "6":
                print("\n  Scanning 4 Standard Physical ECUs (CLL 1..4) via VCI Round-Robin Scheduler...")
                ecu_list = await discover_and_poll_multi_cll(vci, bus_mode=bus_mode)
                for e in ecu_list:
                    print(
                        f"    [CLL {e['cll_id']}] {e['ecu_name']:<34} "
                        f"TX=0x{e['req_id']:03X} -> RX=0x{e['rx_can_id']:03X} | "
                        f"RPM={e['rpm']:.1f} (ts={e['ts_us']} µs)"
                    )
                sw = input(f"  Enter CLL ID (1..8) to switch active link [current={active_cll}, Enter to keep]: ").strip()
                if sw.isdigit() and 1 <= int(sw) <= 8:
                    active_cll = int(sw)
                    await vci.pdu_connect(bus_mode=bus_mode, cll_id=active_cll)
                    print(f"  Switched active diagnostic link to CLL {active_cll} (ONLINE).")

            elif choice == "7":
                await inspect_com_params(vci, cll_id=active_cll)
                print("  Presets: [a] 11-bit Physical (0x7E0->0x7E8)  [b] 11-bit Functional (0x7DF->0x7E8)")
                print("           [c] 29-bit Functional (0x18DB33F1)  [d] Sub-ms STmin 200 µs (0xF2)")
                print("           [s] Set custom ComParam by index (1..13)  [Enter] Back")
                p_cmd = input("  Select preset or action: ").strip().lower()
                if p_cmd == "a":
                    await vci.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL, cll_id=active_cll)
                    await vci.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7E0, cll_id=active_cll)
                    await vci.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x7E8, cll_id=active_cll)
                    print("  Applied 11-bit Physical preset (0x7E0 -> 0x7E8).")
                elif p_cmd == "b":
                    await vci.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_FUNCTIONAL, cll_id=active_cll)
                    await vci.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7DF, cll_id=active_cll)
                    await vci.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x7E8, cll_id=active_cll)
                    print("  Applied 11-bit Functional preset (0x7DF -> 0x7E8).")
                elif p_cmd == "c":
                    await vci.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_FUNCTIONAL, cll_id=active_cll)
                    await vci.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x18DB33F1, cll_id=active_cll)
                    await vci.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x18DAF110, cll_id=active_cll)
                    print("  Applied 29-bit Extended Functional preset (0x18DB33F1 -> 0x18DAF110).")
                elif p_cmd == "d":
                    await vci.pdu_set_com_param(CP_ST_MIN, 0xF2, cll_id=active_cll)
                    print("  Applied Sub-ms STmin = 0xF2 (200 µs).")
                elif p_cmd == "s":
                    idx_s = input("  Enter ComParam index (1..13): ").strip()
                    val_s = input("  Enter new value (decimal or 0x hex): ").strip()
                    if idx_s.isdigit() and 1 <= int(idx_s) <= len(COMPARAM_CATALOG) and val_s:
                        param_id, p_name, _ = COMPARAM_CATALOG[int(idx_s) - 1]
                        new_val = int(val_s, 0)
                        ack = await vci.pdu_set_com_param(param_id, new_val, cll_id=active_cll)
                        print(f"  Set {p_name} = {new_val} (0x{new_val:X}) -> status=0x{ack.status:02X}")

            elif choice == "8":
                raw_hex = input("  Enter hex bytes (e.g. '01 0C', '09 02', '01 F0'): ").strip()
                if raw_hex:
                    req_bytes = bytes.fromhex(raw_hex)
                    _, evt = await vci.pdu_start_com_primitive(req_bytes, cll_id=active_cll)
                    if evt and evt.msg_type == PDU_IT_RESULT:
                        res = vci.parse_result_event(evt)
                        vci_us = getattr(evt, "_vci_can_us", None)
                        us_str = f", VCI CAN={vci_us} µs" if vci_us is not None else ""
                        print(
                            f"  RX CAN 0x{res.rx_can_id:X} (ts={res.rx_timestamp_us} µs, "
                            f"RTT={res.rx_time_ms} ms{us_str}): {res.pdu.hex(' ').upper()}"
                        )
                    elif evt:
                        print(f"  Error Event (0x83): {evt.payload.hex(' ')}")

            elif choice == "q":
                await vci.pdu_disconnect(cll_id=0)
                break
    finally:
        await vci.close()


async def run_serial_scanner_verification(serial_port: str = "/dev/ttyUSB0"):
    print("=== Serial Scanner Verification: Exercising All Interactive Scanner Features over USB-UART0 ===")
    vci = AsyncSerialVciAdapter(serial_port, 115200)
    try:
        await vci.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=1)
        await vci.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=1)
        await display_vci_hardware_dashboard(vci, active_cll=1)
        await display_telemetry_snapshot_and_pids(vci, active_cll=1)
        await inspect_com_params(vci, cll_id=1)
        await run_cyclic_live_stream(vci, active_cll=1, cycles_per_pid=4, cycle_time_ms=25)
        ecus = await discover_and_poll_multi_cll(vci, bus_mode=VCI_BUS_MODE_SIM_LOOPBACK)
        assert len(ecus) == 4
        await vci.pdu_disconnect(cll_id=0)
        print("=== ALL SERIAL SCANNER FEATURES VERIFIED! ===")
    finally:
        await vci.close()


def main():
    parser = argparse.ArgumentParser(description="ESP32 Bluetooth LE & Serial OBD-II Diagnostic Scanner")
    parser.add_argument("--verify-ble", action="store_true", help="Run automated 8-step BLE NUS verification suite")
    parser.add_argument("--verify-serial", action="store_true", help="Run automated Serial scanner feature verification")
    parser.add_argument("--ble", action="store_true", help="Connect interactively over Bluetooth LE (default unless --use-serial)")
    parser.add_argument("--use-serial", action="store_true", help="Connect interactively over wired USB-UART0 instead of BLE")
    parser.add_argument("--serial", default="/dev/ttyUSB0", help="Serial port path for wired mode (default: /dev/ttyUSB0)")
    parser.add_argument("--address", default=None, help="Optional BLE MAC address of ESP32-OBD-VCI")
    parser.add_argument("--physical-can", action="store_true", help="Use Physical CAN (TWAI GPIO21/22) instead of SimLoopback")
    args = parser.parse_args()

    if args.verify_ble:
        asyncio.run(run_step4_ble_verification(address=args.address))
    elif args.verify_serial:
        asyncio.run(run_serial_scanner_verification(serial_port=args.serial))
    else:
        use_ble_mode = not args.use_serial
        asyncio.run(
            run_interactive_scanner(
                use_ble=use_ble_mode,
                serial_port=args.serial,
                ble_addr=args.address,
                physical_can=args.physical_can,
            )
        )


if __name__ == "__main__":
    main()
