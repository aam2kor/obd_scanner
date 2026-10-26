#!/usr/bin/env python3
"""
pc_app/dpdu_client.py — PC Host Lean D-PDU API Client & Hardware Verification Suite.

Implements the binary Lean D-PDU wire protocol (0xA5 SOF + 8-byte header + payload
+ CRC-16/CCITT-FALSE) over Serial (/dev/ttyUSB0) and BLE GATT.
"""

from dataclasses import dataclass
import struct
import time
from typing import Optional
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

# ComParam IDs
CP_CAN_PHYS_REQ_ID  = 0x01
CP_CAN_RESP_USDT_ID = 0x02
CP_P2_MAX_MS        = 0x03
CP_P2_STAR_MS       = 0x04
CP_ISOTP_BS         = 0x05
CP_ISOTP_STMIN_MS   = 0x06

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
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = DpduStreamParser()
        self._seq = 1

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def reset_vci_board(self):
        """Hardware reset the ESP32 via RTS and wait for VCI initialization."""
        self.ser.dtr = False
        self.ser.rts = True
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.rts = False
        time.sleep(0.6)
        self.ser.reset_input_buffer()
        self.parser.reset()

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        return s

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

        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(128)
            for b in chunk:
                frame = self.parser.feed_byte(b)
                if frame is not None:
                    return frame
        raise TimeoutError(f"Timed out waiting for VCI response to opcode 0x{opcode:02x} (seq={seq})")

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

    def pdu_connect(self, bus_mode: int = VCI_BUS_MODE_SIM_LOOPBACK, baud_khz: int = 500, flags: int = 0) -> DpduFrame:
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

    def pdu_start_com_primitive(self, pdu_data: bytes, cop_type: int = PDU_COPT_SENDRECV, flags: int = 0) -> DpduFrame:
        payload = struct.pack("<BB", cop_type, flags) + pdu_data
        return self.send_and_recv(PDU_CMD_START_COP, payload)


def run_step1_verification():
    print("=== Step 1 Hardware Verification: Binary Lean D-PDU Protocol Core ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/7] Resetting ESP32 VCI and querying initial PDU_CMD_GET_STATUS...")
        client.reset_vci_board()
        st = client.pdu_get_status()
        print(f"      Status OK: {st}")
        assert st["connected"] is False
        assert st["baud_khz"] == 500
        assert st["can_req_id"] == 0x7DF

        print("[2/7] Verifying PDU_CMD_START_COP is rejected when disconnected...")
        rsp = client.pdu_start_com_primitive(b"\x01\x0C")
        assert rsp.opcode == PDU_EVT_CMD_ACK and rsp.payload[1] == PDU_STATUS_NOT_CONN
        print("      Correctly returned PDU_STATUS_NOT_CONN (0x02)")

        print("[3/7] Connecting Logical Link via PDU_CMD_CONNECT (SimLoopback, 500 kbps)...")
        rsp = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, baud_khz=500)
        assert rsp.opcode == PDU_EVT_CMD_ACK and rsp.payload[1] == PDU_STATUS_NOERROR
        print("      Connected! GPIO 2 LED switched to 1 Hz Connected heartbeat.")

        print("[4/7] Setting & reading back ComParams (CP_CAN_PHYS_REQ_ID=0x7E0, CP_P2_MAX_MS=75)...")
        rsp1 = client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x000007E0)
        rsp2 = client.pdu_set_com_param(CP_P2_MAX_MS, 75)
        assert rsp1.payload[1] == PDU_STATUS_NOERROR and rsp2.payload[1] == PDU_STATUS_NOERROR
        req_id = client.pdu_get_com_param(CP_CAN_PHYS_REQ_ID)
        p2_max = client.pdu_get_com_param(CP_P2_MAX_MS)
        print(f"      Read back: CP_CAN_PHYS_REQ_ID=0x{req_id:03X}, CP_P2_MAX_MS={p2_max} ms")
        assert req_id == 0x7E0 and p2_max == 75

        print("[5/7] Starting ComPrimitive (SENDRECV [01 0C]) while connected...")
        rsp = client.pdu_start_com_primitive(b"\x01\x0C")
        assert rsp.opcode == PDU_EVT_CMD_ACK and rsp.payload[1] == PDU_STATUS_NOERROR
        print("      ComPrimitive accepted (PDU_STATUS_NOERROR)")

        print("[6/7] Injecting intentional 1-bit CRC-16 error to verify VCI CRC protection...")
        err_rsp = client.send_and_recv(PDU_CMD_GET_STATUS, corrupt_crc=True)
        assert err_rsp.opcode == PDU_EVT_ERROR and err_rsp.payload[0] == PDU_ERR_CRC_MISMATCH
        print(f"      VCI detected corrupted frame and returned PDU_EVT_ERROR (err=0x{err_rsp.payload[0]:02x} CRC_MISMATCH)")

        print("[7/7] Querying final status & disconnecting via PDU_CMD_DISCONNECT...")
        st_final = client.pdu_get_status()
        print(f"      Final VCI Status: {st_final}")
        assert st_final["connected"] is True
        assert st_final["crc_err_count"] == 1
        disc_rsp = client.pdu_disconnect()
        assert disc_rsp.opcode == PDU_EVT_CMD_ACK and disc_rsp.payload[1] == PDU_STATUS_NOERROR
        print("=== ALL 7 STEP 1 HARDWARE CHECKS PASSED! ===")
    finally:
        client.close()


if __name__ == "__main__":
    run_step1_verification()

