#!/usr/bin/env python3
"""
pc_app/dpdu_client.py — PC Host Lean D-PDU API Client & Hardware Verification Suite.

Implements the binary Lean D-PDU wire protocol (0xA5 SOF + 8-byte header + payload
+ CRC-16/CCITT-FALSE) over Serial (/dev/ttyUSB0) and BLE GATT.
"""

from dataclasses import dataclass
import struct
import time
from typing import Optional, Tuple
import serial

# Wire Framing Constants
DPDU_SOF = 0xA5
DPDU_VERSION = 0x01
DPDU_HEADER_SIZE = 8
DPDU_CRC_SIZE = 2
DPDU_MAX_PAYLOAD = 512

# Host -> VCI Opcodes
PDU_CMD_CONNECT      = 0x01
PDU_CMD_DISCONNECT   = 0x02
PDU_CMD_SET_COMPARAM = 0x03
PDU_CMD_START_COP    = 0x04
PDU_CMD_CANCEL_COP   = 0x05
PDU_CMD_GET_STATUS   = 0x06
PDU_CMD_GET_COMPARAM = 0x07

# VCI -> Host Event Opcodes
PDU_EVT_CMD_ACK      = 0x80
PDU_EVT_RESULT       = 0x81
PDU_EVT_ERROR        = 0x82

# Status Codes
PDU_STATUS_NOERROR      = 0x00
PDU_STATUS_INVALID_ARG  = 0x01
PDU_STATUS_NOT_CONN     = 0x02
PDU_STATUS_BUSY         = 0x03
PDU_STATUS_UNSUPPORTED  = 0x04

# Error Codes
PDU_ERR_P2_TIMEOUT      = 0x01
PDU_ERR_ISOTP_TIMEOUT   = 0x02
PDU_ERR_CAN_BUS_OFF     = 0x03
PDU_ERR_CRC_MISMATCH    = 0x04
PDU_ERR_MALFORMED_FRAME = 0x05
PDU_ERR_CANCELLED       = 0x06

# 13 ISO 22900-2 Aligned ComParam IDs
CP_BAUDRATE         = 0x01
CP_CAN_PHYS_REQ_ID  = 0x02
CP_CAN_RESP_USDT_ID = 0x03
CP_ADDRESSING_TYPE  = 0x04
CP_CAN_FILLER_BYTE  = 0x05
CP_BLOCK_SIZE       = 0x06
CP_ST_MIN           = 0x07
CP_N_BS_MS          = 0x08
CP_N_CR_MS          = 0x09
CP_P2_MAX_MS        = 0x0A
CP_P2_STAR_MS       = 0x0B
CP_P3_PHYS_MS       = 0x0C
CP_P3_FUNC_MS       = 0x0D

# Backwards-compatible aliases
CP_ISOTP_BS         = CP_BLOCK_SIZE
CP_ISOTP_STMIN_MS   = CP_ST_MIN

# Addressing Types (CP_ADDRESSING_TYPE)
DPDU_ADDR_FUNCTIONAL = 0x00
DPDU_ADDR_PHYSICAL   = 0x01

# Bus Modes
VCI_BUS_MODE_SIM_LOOPBACK = 0x00
VCI_BUS_MODE_PHYSICAL_CAN = 0x01

# ComPrimitive Types
PDU_COPT_SENDRECV  = 0x01
PDU_COPT_SEND_ONLY = 0x02


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class DpduFrame:
    opcode: int
    handle: int
    seq_tag: int
    payload: bytes

    def encode(self, corrupt_crc: bool = False) -> bytes:
        hdr = struct.pack(
            "<BBBBHH",
            DPDU_SOF,
            DPDU_VERSION,
            self.opcode,
            self.handle,
            self.seq_tag,
            len(self.payload),
        )
        body = hdr + self.payload
        crc = crc16_ccitt(body)
        if corrupt_crc:
            crc ^= 0x0001
        return body + struct.pack("<H", crc)


@dataclass
class DpduResultEvent:
    rx_can_id: int
    rx_time_ms: int
    pdu: bytes


class DpduStreamParser:
    """Byte-stream parser for 0xA5 binary D-PDU frames."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._state = "SOF"
        self._hdr = bytearray()
        self._payload = bytearray()
        self._crc = bytearray()
        self._expected_len = 0

    def feed_byte(self, b: int) -> Optional[DpduFrame]:
        if self._state == "SOF":
            if b == DPDU_SOF:
                self._hdr = bytearray([b])
                self._state = "HDR"
            return None

        if self._state == "HDR":
            self._hdr.append(b)
            if len(self._hdr) == DPDU_HEADER_SIZE:
                _, ver, _, _, _, length = struct.unpack("<BBBBHH", self._hdr)
                if ver != DPDU_VERSION or length > DPDU_MAX_PAYLOAD:
                    self.reset()
                    if b == DPDU_SOF:
                        self._hdr = bytearray([b])
                        self._state = "HDR"
                    return None
                self._expected_len = length
                self._payload = bytearray()
                self._crc = bytearray()
                self._state = "PAYLOAD" if length > 0 else "CRC"
            return None

        if self._state == "PAYLOAD":
            self._payload.append(b)
            if len(self._payload) >= self._expected_len:
                self._state = "CRC"
            return None

        if self._state == "CRC":
            self._crc.append(b)
            if len(self._crc) >= DPDU_CRC_SIZE:
                rx_crc = struct.unpack("<H", self._crc)[0]
                calc_crc = crc16_ccitt(bytes(self._hdr) + bytes(self._payload))
                _, _, opcode, handle, seq_tag, _ = struct.unpack("<BBBBHH", self._hdr)
                payload_bytes = bytes(self._payload)
                self.reset()
                if rx_crc != calc_crc:
                    raise ValueError(f"Host RX CRC mismatch: rx=0x{rx_crc:04x} != calc=0x{calc_crc:04x}")
                return DpduFrame(opcode=opcode, handle=handle, seq_tag=seq_tag, payload=payload_bytes)
            return None

        self.reset()
        return None


class LeanDpduSerialClient:
    """PC Host Lean D-PDU Client communicating over USB-UART (or BLE in Step 4)."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.parser = DpduStreamParser()
        self._rx_queue: list[DpduFrame] = []
        self._seq = 1

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def reset_vci_board(self):
        """Hardware reset the ESP32 via RTS and wait for VCI + BLE initialization."""
        self.ser.dtr = False
        self.ser.rts = True
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.rts = False
        time.sleep(1.0)
        self.ser.reset_input_buffer()
        self.parser.reset()
        self._rx_queue.clear()

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        return s

    def recv_frame(self, timeout: float = 1.0) -> DpduFrame:
        if self._rx_queue:
            return self._rx_queue.pop(0)
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(128)
            for b in chunk:
                frame = self.parser.feed_byte(b)
                if frame is not None:
                    self._rx_queue.append(frame)
            if self._rx_queue:
                return self._rx_queue.pop(0)
        raise TimeoutError("Timed out waiting for D-PDU frame from VCI")

    def send_raw_cmd(self, opcode: int, payload: bytes = b"", handle: int = 1) -> int:
        """Send a D-PDU command frame without waiting for a response; returns the assigned seq_tag."""
        seq = self._next_seq()
        req = DpduFrame(opcode=opcode, handle=handle, seq_tag=seq, payload=payload)
        self.ser.write(req.encode())
        self.ser.flush()
        return seq

    def send_and_recv(
        self,
        opcode: int,
        payload: bytes = b"",
        handle: int = 1,
        corrupt_crc: bool = False,
        timeout: float = 1.0,
    ) -> DpduFrame:
        seq = self._next_seq()
        req = DpduFrame(opcode=opcode, handle=handle, seq_tag=seq, payload=payload)
        wire = req.encode(corrupt_crc=corrupt_crc)
        self.ser.write(wire)
        self.ser.flush()
        return self.recv_frame(timeout=timeout)

    def pdu_get_status(self) -> dict:
        rsp = self.send_and_recv(PDU_CMD_GET_STATUS)
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

    def pdu_connect(self, bus_mode: int = VCI_BUS_MODE_SIM_LOOPBACK, baud_khz: int = 0, flags: int = 0) -> DpduFrame:
        if baud_khz == 0 and flags == 0:
            payload = struct.pack("<B", bus_mode)
        else:
            payload = struct.pack("<BHB", bus_mode, baud_khz, flags)
        return self.send_and_recv(PDU_CMD_CONNECT, payload)

    def pdu_disconnect(self) -> DpduFrame:
        return self.send_and_recv(PDU_CMD_DISCONNECT)

    def pdu_set_com_param(self, param_id: int, param_val: int) -> DpduFrame:
        payload = struct.pack("<BI", param_id, param_val)
        return self.send_and_recv(PDU_CMD_SET_COMPARAM, payload)

    def pdu_get_com_param(self, param_id: int) -> int:
        rsp = self.send_and_recv(PDU_CMD_GET_COMPARAM, struct.pack("<B", param_id))
        assert rsp.opcode == PDU_EVT_CMD_ACK and rsp.payload[1] == PDU_STATUS_NOERROR
        _, val = struct.unpack("<BI", rsp.payload[2:7])
        return val

    def pdu_start_com_primitive(
        self,
        pdu_data: bytes,
        cop_type: int = PDU_COPT_SENDRECV,
        flags: int = 0,
        timeout: float = 2.0,
    ) -> Tuple[DpduFrame, Optional[DpduFrame]]:
        """
        Send PDU_CMD_START_COP.
        Returns (ack_frame, event_frame) where event_frame is PDU_EVT_RESULT or PDU_EVT_ERROR
        for SENDRECV when connected.
        """
        payload = struct.pack("<BB", cop_type, flags) + pdu_data
        ack = self.send_and_recv(PDU_CMD_START_COP, payload, timeout=timeout)
        if ack.opcode != PDU_EVT_CMD_ACK or ack.payload[1] != PDU_STATUS_NOERROR:
            return ack, None
        if cop_type == PDU_COPT_SEND_ONLY:
            return ack, None
        evt = self.recv_frame(timeout=timeout)
        return ack, evt

    @staticmethod
    def parse_result_event(evt: DpduFrame) -> DpduResultEvent:
        if evt.opcode != PDU_EVT_RESULT or len(evt.payload) < 6:
            raise AssertionError(
                f"Expected PDU_EVT_RESULT (0x81), got opcode=0x{evt.opcode:02X} payload={evt.payload.hex(' ')}"
            )
        rx_can_id, rx_time_ms = struct.unpack("<IH", evt.payload[:6])
        return DpduResultEvent(rx_can_id=rx_can_id, rx_time_ms=rx_time_ms, pdu=evt.payload[6:])


def run_step2_verification():
    print("=== Step 2 Hardware Verification: CAN HAL + Single-Frame OBD-II Mode 01 + P2/P2* Timers ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/7] Resetting ESP32 VCI & connecting Logical Link (SimLoopback, 500 kbps)...")
        client.reset_vci_board()
        ack = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, baud_khz=500)
        assert ack.opcode == PDU_EVT_CMD_ACK and ack.payload[1] == PDU_STATUS_NOERROR
        print("      Connected (CAN_REQ_ID=0x7DF, CAN_RESP_ID=0x7E8, P2_Max=50ms, P2*_Max=5000ms)")

        print("[2/7] Querying OBD-II Mode 01 PID 0x00 (Supported PIDs 01..20)...")
        ack, evt = client.pdu_start_com_primitive(b"\x01\x00")
        res = client.parse_result_event(evt)
        print(f"      RX CAN ID=0x{res.rx_can_id:03X} ({res.rx_time_ms} ms) PDU={res.pdu.hex(' ').upper()}")
        assert res.rx_can_id == 0x7E8 and res.pdu == bytes.fromhex("41 00 BE 3E B8 13")

        print("[3/7] Querying OBD-II Mode 01 PID 0x0C (Engine RPM)...")
        ack, evt = client.pdu_start_com_primitive(b"\x01\x0C")
        res = client.parse_result_event(evt)
        rpm = ((res.pdu[2] << 8) | res.pdu[3]) / 4.0
        print(f"      RX CAN ID=0x{res.rx_can_id:03X} ({res.rx_time_ms} ms) PDU={res.pdu.hex(' ').upper()} -> {rpm:.1f} RPM")
        assert res.pdu[:2] == b"\x41\x0C" and rpm == 1726.0

        print("[4/7] Querying OBD-II Mode 01 PID 0x0D (Vehicle Speed) & PID 0x05 (Coolant Temp)...")
        _, evt_spd = client.pdu_start_com_primitive(b"\x01\x0D")
        res_spd = client.parse_result_event(evt_spd)
        speed_kmh = res_spd.pdu[2]

        _, evt_tmp = client.pdu_start_com_primitive(b"\x01\x05")
        res_tmp = client.parse_result_event(evt_tmp)
        coolant_c = res_tmp.pdu[2] - 40
        print(f"      Speed   : {res_spd.pdu.hex(' ').upper()} -> {speed_kmh} km/h ({res_spd.rx_time_ms} ms)")
        print(f"      Coolant : {res_tmp.pdu.hex(' ').upper()} -> {coolant_c} °C ({res_tmp.rx_time_ms} ms)")
        assert speed_kmh == 85 and coolant_c == 90

        print("[5/7] Querying OBD-II Mode 01 PID 0x11 (Throttle Position)...")
        _, evt_thr = client.pdu_start_com_primitive(b"\x01\x11")
        res_thr = client.parse_result_event(evt_thr)
        throttle_pct = (res_thr.pdu[2] * 100.0) / 255.0
        print(f"      Throttle: {res_thr.pdu.hex(' ').upper()} -> {throttle_pct:.1f}% ({res_thr.rx_time_ms} ms)")
        assert res_thr.pdu == b"\x41\x11\x40"

        print("[6/7] Testing UDS/OBD Negative Response 0x7F 0x01 0x78 (ResponsePending) -> P2* Timer Extension...")
        _, evt_p2star = client.pdu_start_com_primitive(b"\x01\xF0")
        res_p2star = client.parse_result_event(evt_p2star)
        print(
            f"      NRC 0x78 absorbed by VCI! Final positive response received at {res_p2star.rx_time_ms} ms "
            f"(> 50ms P2_Max): {res_p2star.pdu.hex(' ').upper()}"
        )
        assert res_p2star.pdu == b"\x41\xF0\xAB\xCD" and res_p2star.rx_time_ms >= 100

        print("[7/7] Testing P2_Max (50 ms) Timeout on unanswered PID (01 FF)...")
        _, evt_to = client.pdu_start_com_primitive(b"\x01\xFF")
        assert evt_to.opcode == PDU_EVT_ERROR and evt_to.payload[0] == PDU_ERR_P2_TIMEOUT
        print(
            f"      VCI enforced P2_Max timeout and emitted PDU_EVT_ERROR "
            f"(err=0x{evt_to.payload[0]:02x} PDU_ERR_P2_TIMEOUT, sid=0x{evt_to.payload[1]:02x})"
        )

        client.pdu_disconnect()
        print("=== ALL 7 STEP 2 HARDWARE CHECKS PASSED! ===")
    finally:
        client.close()


def decode_obd_dtcs(pdu: bytes) -> list[str]:
    """Decode SAE J1979 Mode 03 response [0x43, count, hi1, lo1, ...] into standard DTC strings."""
    assert len(pdu) >= 2 and pdu[0] == 0x43
    count = pdu[1]
    prefix_map = {0: "P", 1: "C", 2: "B", 3: "U"}
    dtcs: list[str] = []
    for i in range(count):
        idx = 2 + (i * 2)
        if idx + 1 >= len(pdu):
            break
        b1, b2 = pdu[idx], pdu[idx + 1]
        if b1 == 0 and b2 == 0:
            continue
        sys_prefix = prefix_map[(b1 >> 6) & 0x03]
        d1 = (b1 >> 4) & 0x03
        d2 = b1 & 0x0F
        d3 = (b2 >> 4) & 0x0F
        d4 = b2 & 0x0F
        dtcs.append(f"{sys_prefix}{d1:X}{d2:X}{d3:X}{d4:X}")
    return dtcs


def run_step3_verification():
    print("=== Step 3 Hardware Verification: Multi-Frame ISO 15765-2 (FF/FC/CF) for VIN, DTCs & UDS ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/8] Resetting ESP32 VCI & connecting Logical Link (SimLoopback, 500 kbps)...")
        client.reset_vci_board()
        ack = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, baud_khz=500)
        assert ack.opcode == PDU_EVT_CMD_ACK and ack.payload[1] == PDU_STATUS_NOERROR
        print("      Connected (ISO-TP BS=0, STmin=0ms, N_Cr=1000ms)")

        print("[2/8] Querying OBD-II Mode 09 PID 0x02 (17-Char VIN via Multi-Frame FF/FC/CF)...")
        _, evt_vin = client.pdu_start_com_primitive(b"\x09\x02")
        res_vin = client.parse_result_event(evt_vin)
        vin_str = res_vin.pdu[3:].decode("ascii")
        print(
            f"      RX CAN ID=0x{res_vin.rx_can_id:03X} ({res_vin.rx_time_ms} ms, {len(res_vin.pdu)} bytes) "
            f"VIN='{vin_str}'"
        )
        assert len(res_vin.pdu) == 20 and res_vin.pdu[:3] == b"\x49\x02\x01"
        assert vin_str == "1FAFP40411F192837"

        print("[3/8] Testing Flow Control BlockSize (BS=1) & STmin (10 ms) on Mode 09 PID 0x04 (Cal ID)...")
        client.pdu_set_com_param(CP_ISOTP_BS, 1)
        client.pdu_set_com_param(CP_ISOTP_STMIN_MS, 10)
        _, evt_cal = client.pdu_start_com_primitive(b"\x09\x04")
        res_cal = client.parse_result_event(evt_cal)
        cal_str = res_cal.pdu[3:].decode("ascii")
        print(
            f"      RX CAN ID=0x{res_cal.rx_can_id:03X} ({res_cal.rx_time_ms} ms, {len(res_cal.pdu)} bytes) "
            f"CalID='{cal_str}' (with BS=1 & STmin=10ms >= 20ms)"
        )
        assert cal_str == "ESP32_VCI_CAL_01" and res_cal.rx_time_ms >= 20
        client.pdu_set_com_param(CP_ISOTP_BS, 0)
        client.pdu_set_com_param(CP_ISOTP_STMIN_MS, 0)

        print("[4/8] Querying OBD-II Mode 03 (Read Stored Diagnostic Trouble Codes via Multi-Frame)...")
        _, evt_dtc = client.pdu_start_com_primitive(b"\x03")
        res_dtc = client.parse_result_event(evt_dtc)
        dtcs = decode_obd_dtcs(res_dtc.pdu)
        print(f"      RX PDU={res_dtc.pdu.hex(' ').upper()} ({len(res_dtc.pdu)} bytes) -> Stored DTCs: {dtcs}")
        assert dtcs == ["P0171", "P0300", "C0035", "U0100"]

        print("[5/8] Executing OBD-II Mode 04 (Clear DTCs) and re-querying Mode 03...")
        _, evt_clr = client.pdu_start_com_primitive(b"\x04")
        res_clr = client.parse_result_event(evt_clr)
        assert res_clr.pdu == b"\x44"
        _, evt_dtc_after = client.pdu_start_com_primitive(b"\x03")
        res_dtc_after = client.parse_result_event(evt_dtc_after)
        dtcs_after = decode_obd_dtcs(res_dtc_after.pdu)
        print(f"      Mode 04 ACK={res_clr.pdu.hex().upper()}, Mode 03 after clear -> DTCs: {dtcs_after}")
        assert dtcs_after == []

        print("[6/8] Testing Multi-Frame TX (Tester VCI -> ECU): UDS 0x2E WriteDataByIdentifier (VIN)...")
        new_vin = b"WBA3A5C51DF123456"
        uds_req = b"\x2E\xF1\x90" + new_vin  # 20-byte Multi-Frame request!
        _, evt_2e = client.pdu_start_com_primitive(uds_req)
        res_2e = client.parse_result_event(evt_2e)
        assert res_2e.pdu == b"\x6E\xF1\x90"
        _, evt_vin2 = client.pdu_start_com_primitive(b"\x09\x02")
        res_vin2 = client.parse_result_event(evt_vin2)
        updated_vin = res_vin2.pdu[3:].decode("ascii")
        print(
            f"      UDS 0x2E Positive Rsp={res_2e.pdu.hex(' ').upper()}, "
            f"Readback Mode 09 PID 0x02 VIN='{updated_vin}'"
        )
        assert updated_vin == "WBA3A5C51DF123456"

        print("[7/8] Testing Multi-Frame Sequence Number Mismatch Detection (09 F2)...")
        _, evt_seq = client.pdu_start_com_primitive(b"\x09\xF2")
        assert evt_seq.opcode == PDU_EVT_ERROR and evt_seq.payload[0] == PDU_ERR_MALFORMED_FRAME
        print(
            f"      VCI detected bad CF sequence number and emitted PDU_EVT_ERROR "
            f"(err=0x{evt_seq.payload[0]:02x} PDU_ERR_MALFORMED_FRAME)"
        )

        print("[8/8] Testing Multi-Frame N_Cr (1000 ms) Consecutive Frame Timeout (09 F1)...")
        t0 = time.time()
        _, evt_ncr = client.pdu_start_com_primitive(b"\x09\xF1", timeout=2.5)
        elapsed_ms = int((time.time() - t0) * 1000)
        assert evt_ncr.opcode == PDU_EVT_ERROR and evt_ncr.payload[0] == PDU_ERR_ISOTP_TIMEOUT
        print(
            f"      VCI enforced N_Cr (1000 ms) timeout at {elapsed_ms} ms and emitted PDU_EVT_ERROR "
            f"(err=0x{evt_ncr.payload[0]:02x} PDU_ERR_ISOTP_TIMEOUT)"
        )

        client.pdu_disconnect()
        print("=== ALL 8 STEP 3 HARDWARE CHECKS PASSED! ===")
    finally:
        client.close()


def run_comparam_and_standards_verification():
    print("=== ISO 22900-2 Standards Verification: 13 ComParams, P3_Func/P3_Phys, 29-Bit CAN & Cancel ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/6] Resetting VCI & verifying all 13 default ISO 22900-2 ComParams...")
        client.reset_vci_board()
        expected_defaults = {
            CP_BAUDRATE:         500000,
            CP_CAN_PHYS_REQ_ID:  0x7DF,
            CP_CAN_RESP_USDT_ID: 0x7E8,
            CP_ADDRESSING_TYPE:  DPDU_ADDR_FUNCTIONAL,
            CP_CAN_FILLER_BYTE:  0xCC,
            CP_BLOCK_SIZE:       0,
            CP_ST_MIN:           0,
            CP_N_BS_MS:          1000,
            CP_N_CR_MS:          1000,
            CP_P2_MAX_MS:        50,
            CP_P2_STAR_MS:       5000,
            CP_P3_PHYS_MS:       5,
            CP_P3_FUNC_MS:       50,
        }
        for cp_id, exp_val in expected_defaults.items():
            actual = client.pdu_get_com_param(cp_id)
            assert actual == exp_val, f"ComParam 0x{cp_id:02X} mismatch: {actual} != {exp_val}"
        print("      All 13 ComParams verified at standard defaults!")

        print("[2/6] Setting CP_BAUDRATE = 250000 bps before PDUConnect(bus_mode)...")
        client.pdu_set_com_param(CP_BAUDRATE, 250000)
        ack = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK)
        assert ack.opcode == PDU_EVT_CMD_ACK and ack.payload[1] == PDU_STATUS_NOERROR
        st = client.pdu_get_status()
        print(f"      Connected via CP_BAUDRATE -> Active Bus Baud = {st['baud_khz']} kbps")
        assert st["baud_khz"] == 250

        print("[3/6] Verifying CP_CAN_FILLER_BYTE (0xAA and 0x55) on outgoing CAN frames (01 F3)...")
        client.pdu_set_com_param(CP_CAN_FILLER_BYTE, 0xAA)
        _, evt_pad1 = client.pdu_start_com_primitive(b"\x01\xF3")
        res_pad1 = client.parse_result_event(evt_pad1)
        client.pdu_set_com_param(CP_CAN_FILLER_BYTE, 0x55)
        _, evt_pad2 = client.pdu_start_com_primitive(b"\x01\xF3")
        res_pad2 = client.parse_result_event(evt_pad2)
        print(
            f"      Filler=0xAA -> ECU saw 0x{res_pad1.pdu[2]:02X}; "
            f"Filler=0x55 -> ECU saw 0x{res_pad2.pdu[2]:02X}"
        )
        assert res_pad1.pdu[2] == 0xAA and res_pad2.pdu[2] == 0x55
        client.pdu_set_com_param(CP_CAN_FILLER_BYTE, 0xCC)

        print("[4/6] Verifying CP_P3_FUNC_MS (100 ms) vs CP_P3_PHYS_MS (5 ms) Inter-Request Enforcement...")
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_FUNCTIONAL)
        client.pdu_set_com_param(CP_P3_FUNC_MS, 100)
        t0 = time.time()
        client.pdu_start_com_primitive(b"\x01\x0C")
        client.pdu_start_com_primitive(b"\x01\x0D")
        func_pair_ms = int((time.time() - t0) * 1000)

        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7E0)
        client.pdu_set_com_param(CP_P3_PHYS_MS, 5)
        time.sleep(0.11)  # Let previous Functional P3_Func window expire before timing Physical pair
        t1 = time.time()
        client.pdu_start_com_primitive(b"\x01\x0C")
        client.pdu_start_com_primitive(b"\x01\x0D")
        phys_pair_ms = int((time.time() - t1) * 1000)
        print(
            f"      Back-to-back Functional pair (CP_P3_FUNC_MS=100ms): {func_pair_ms} ms (>= 100 ms)\n"
            f"      Back-to-back Physical pair   (CP_P3_PHYS_MS=5ms)  : {phys_pair_ms} ms (< 70 ms)"
        )
        assert func_pair_ms >= 100 and phys_pair_ms < 70

        print("[5/6] Verifying 29-Bit Extended OBD-II CAN (0x18DB33F1 -> 0x18DAF110) SF & Multi-Frame VIN...")
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_FUNCTIONAL)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x18DB33F1)
        client.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x18DAF110)
        client.pdu_set_com_param(CP_P3_FUNC_MS, 10)
        _, evt_29_sf = client.pdu_start_com_primitive(b"\x01\x0C")
        res_29_sf = client.parse_result_event(evt_29_sf)
        _, evt_29_mf = client.pdu_start_com_primitive(b"\x09\x02")
        res_29_mf = client.parse_result_event(evt_29_mf)
        vin_29 = res_29_mf.pdu[3:].decode("ascii")
        print(
            f"      29-Bit SF RX ID=0x{res_29_sf.rx_can_id:08X} PDU={res_29_sf.pdu.hex(' ').upper()}\n"
            f"      29-Bit MF RX ID=0x{res_29_mf.rx_can_id:08X} VIN='{vin_29}' (FC routed to 0x18DA10F1)"
        )
        assert res_29_sf.rx_can_id == 0x18DAF110 and res_29_mf.rx_can_id == 0x18DAF110
        assert len(vin_29) == 17

        print("[6/8] Verifying Pipelined Asynchronous Burst (5 back-to-back START_COPs without waiting)...")
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7E0)
        client.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x7E8)
        client.pdu_set_com_param(CP_P3_PHYS_MS, 2)
        burst_pids = [b"\x01\x00", b"\x01\x0C", b"\x01\x0D", b"\x01\x05", b"\x01\x11"]
        burst_seqs = [
            client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + pid)
            for pid in burst_pids
        ]
        acks_seen = set()
        results_seen = {}
        while len(acks_seen) < 5 or len(results_seen) < 5:
            f = client.recv_frame(timeout=2.0)
            if f.opcode == PDU_EVT_CMD_ACK and f.payload[1] == PDU_STATUS_NOERROR:
                acks_seen.add(f.seq_tag)
            elif f.opcode == PDU_EVT_RESULT:
                results_seen[f.seq_tag] = client.parse_result_event(f)
        assert acks_seen == set(burst_seqs) and set(results_seen.keys()) == set(burst_seqs)
        print(f"      All 5 pipelined async requests ACKed & resolved in order (SeqTags={burst_seqs})!")

        print("[7/8] Verifying Out-of-Band Priority PDU_CMD_CANCEL_COP + Queue Drain of 3 Queued Primitives...")
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7DF)
        client.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x7E8)
        s_active = client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\xF4")
        time.sleep(0.03)  # Let s_active enter 5000ms P2* wait on Core 1
        s_q1 = client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\x0C")
        s_q2 = client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\x0D")
        s_q3 = client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\x05")
        time.sleep(0.02)
        t_cancel = time.time()
        s_cancel = client.send_raw_cmd(PDU_CMD_CANCEL_COP)
        cancelled_seqs = set()
        cancel_ack_seen = False
        while not (cancel_ack_seen and len(cancelled_seqs) == 4):
            f = client.recv_frame(timeout=1.5)
            if f.opcode == PDU_EVT_ERROR and f.payload[0] == PDU_ERR_CANCELLED:
                cancelled_seqs.add(f.seq_tag)
            elif f.opcode == PDU_EVT_CMD_ACK and f.seq_tag == s_cancel and f.payload[0] == PDU_CMD_CANCEL_COP:
                cancel_ack_seen = True
        cancel_ms = int((time.time() - t_cancel) * 1000)
        assert cancelled_seqs == {s_active, s_q1, s_q2, s_q3}
        print(
            f"      Out-of-band CANCEL_COP aborted active P2* + drained 3 queued primitives in {cancel_ms} ms!"
        )

        print("[8/8] Verifying Queue Depth = 16 & Explicit PDU_STATUS_BUSY (0x03) Backpressure on Overflow...")
        # 1 active in-flight on Core 1 + 16 queued in s_q_dpdu_cmd + 1 overflow (18th)
        s_in_flight = client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\xF4")
        time.sleep(0.03)  # Ensure Core 1 pops s_in_flight so all 16 queue slots are tested
        queued_16 = [
            client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\x0C")
            for _ in range(16)
        ]
        s_overflow = client.send_raw_cmd(PDU_CMD_START_COP, struct.pack("<BB", PDU_COPT_SENDRECV, 0) + b"\x01\x0D")
        busy_seen = False
        admitted_acks = set()
        while not busy_seen or len(admitted_acks) < 17:
            f = client.recv_frame(timeout=1.5)
            if f.opcode == PDU_EVT_CMD_ACK:
                if f.seq_tag == s_overflow and f.payload[1] == PDU_STATUS_BUSY:
                    busy_seen = True
                elif f.payload[1] == PDU_STATUS_NOERROR:
                    admitted_acks.add(f.seq_tag)
        assert busy_seen and len(admitted_acks) == 17
        # Cleanly cancel & drain the 17 active/queued commands
        s_cleanup = client.send_raw_cmd(PDU_CMD_CANCEL_COP)
        cleanup_cancelled = 0
        cleanup_acked = False
        while not (cleanup_acked and cleanup_cancelled == 17):
            f = client.recv_frame(timeout=1.5)
            if f.opcode == PDU_EVT_ERROR and f.payload[0] == PDU_ERR_CANCELLED:
                cleanup_cancelled += 1
            elif f.opcode == PDU_EVT_CMD_ACK and f.seq_tag == s_cleanup:
                cleanup_acked = True
        print(
            "      1 in-flight + 16 queued accepted (NOERROR); 18th received PDU_STATUS_BUSY (0x03); "
            "all 17 drained cleanly via CANCEL_COP!"
        )

        client.pdu_disconnect()
        print("=== ALL 8 ISO 22900-2 & ASYNC PIPELINING VERIFICATION CHECKS PASSED! ===")
    finally:
        client.close()


if __name__ == "__main__":
    import sys
    if "--all" in sys.argv:
        run_step2_verification()
        print()
        run_step3_verification()
        print()
    run_comparam_and_standards_verification()
