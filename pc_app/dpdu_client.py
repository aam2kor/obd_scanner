#!/usr/bin/env python3
"""
pc_app/dpdu_client.py — PC Host ISO 22900-2:2022 App-Device IF Client & Hardware Verification Suite.

Wire Frame Format:
  [ SOF (2B: 0xAA55) | Msg_Type (1B) | CLL_ID (1B) | COP_ID (1B) | Status (1B) | Payload_Len (2B) | Payload (0..4103B) | CRC16-CCITT (2B) ]
  CRC16-CCITT (poly=0x1021, init=0xFFFF) computed over Msg_Type through the end of Payload.
"""

from dataclasses import dataclass
import struct
import time
from typing import Optional, Tuple
import serial

# Wire Framing Constants
DPDU_SOF_WORD    = 0xAA55
DPDU_SOF_BYTE_0  = 0x55
DPDU_SOF_BYTE_1  = 0xAA
DPDU_HEADER_SIZE = 8
DPDU_CRC_SIZE    = 2
DPDU_MAX_PAYLOAD = 4103

# Host -> VCI Command Msg_Types (0x01..0x7F)
PDU_CMD_CONNECT      = 0x01
PDU_CMD_DISCONNECT   = 0x02
PDU_CMD_SET_COMPARAM = 0x03
PDU_CMD_START_COP    = 0x04
PDU_CMD_CANCEL_COP   = 0x05
PDU_CMD_GET_STATUS   = 0x06
PDU_CMD_GET_COMPARAM = 0x07
PDU_CMD_MOD_CTRL     = 0x08

# VCI -> Host Packet Msg_Types (0x80..0xFF)
PDU_EVT_CMD_ACK      = 0x80  # Synchronous Command ACK/NACK (Status in Header Byte 5)
PDU_IT_RESULT        = 0x81  # [RX_Timestamp_us(4B)|CAN_ID(4B)|Extra_Addr(1B)|N-PDU]
PDU_IT_STATUS        = 0x82  # [Event_Timestamp_us(4B)|D-PDU_Status_Code(2B)]
PDU_IT_ERROR         = 0x83  # [Error_Timestamp_us(4B)|D-PDU_Error_Code(2B)|Vendor_SubCode(2B)]

# Legacy aliases
PDU_EVT_RESULT       = PDU_IT_RESULT
PDU_EVT_STATUS       = PDU_IT_STATUS
PDU_EVT_ERROR        = PDU_IT_ERROR

# Synchronous 0x80 Status Codes (Header Byte 5)
PDU_STATUS_NOERROR                 = 0x00
PDU_STATUS_INVALID_ARG             = 0x01
PDU_STATUS_NOT_CONN                = 0x02
PDU_ERR_COMM_PC_TO_VCI_OVERRUN     = 0x03
PDU_ERR_FCT_NOT_SUPPORTED          = 0x04
PDU_ERR_BAUDRATE_CONFLICT          = 0x05

PDU_STATUS_BUSY                    = PDU_ERR_COMM_PC_TO_VCI_OVERRUN
PDU_STATUS_UNSUPPORTED             = PDU_ERR_FCT_NOT_SUPPORTED

# Module IOCTL IDs (PDU_CMD_MOD_CTRL = 0x08, CLL_ID = 0x00)
PDU_IOCTL_READ_VBATT               = 0x0001
PDU_IOCTL_GET_CABLE_ID             = 0x0002
PDU_IOCTL_GET_VERSION              = 0x0003

# D-PDU Status Codes (2B LE in 0x82 PDU_IT_STATUS)
PDU_COPST_IDLE                     = 0x8001
PDU_COPST_EXECUTING                = 0x8002
PDU_COPST_FINISHED                 = 0x8003
PDU_COPST_CANCELLED                = 0x8004
PDU_COPST_WAITING                  = 0x8005
PDU_CLLST_OFFLINE                  = 0x8010
PDU_CLLST_ONLINE                   = 0x8011
PDU_CLLST_COMM_STARTED             = 0x8012

# D-PDU Error Codes (2B LE in 0x83 PDU_IT_ERROR)
PDU_ERR_EVT_RX_TIMEOUT             = 0x0001
PDU_ERR_EVT_PROT_ERR               = 0x0002
PDU_ERR_EVT_TX_ERROR               = 0x0003
PDU_ERR_EVT_RX_ERROR               = 0x0004

# Vendor Error SubCodes (2B LE in 0x83 PDU_IT_ERROR)
PDU_ERR_P2_TIMEOUT                 = 0x0001
PDU_ERR_ISOTP_TIMEOUT              = 0x0002
PDU_ERR_CAN_BUS_OFF                = 0x0003
PDU_ERR_CRC_MISMATCH               = 0x0004
PDU_ERR_MALFORMED_FRAME            = 0x0005
PDU_ERR_CANCELLED                  = 0x0006

# 13 ISO 22900-2 Annex B ComParam IDs (2B LE)
CP_BAUDRATE         = 0x0001
CP_CAN_PHYS_REQ_ID  = 0x0002
CP_CAN_RESP_USDT_ID = 0x0003
CP_ADDRESSING_TYPE  = 0x0004
CP_CAN_FILLER_BYTE  = 0x0005
CP_BLOCK_SIZE       = 0x0006
CP_ST_MIN           = 0x0007
CP_N_BS_MS          = 0x0008
CP_N_CR_MS          = 0x0009
CP_P2_MAX_MS        = 0x000A
CP_P2_STAR_MS       = 0x000B
CP_P3_PHYS_MS       = 0x000C
CP_P3_FUNC_MS       = 0x000D

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
    msg_type: int
    cll_id: int
    cop_id: int
    status: int
    payload: bytes

    @property
    def opcode(self) -> int:
        return self.msg_type

    def encode(self, corrupt_crc: bool = False) -> bytes:
        hdr = struct.pack(
            "<HBBBBH",
            DPDU_SOF_WORD,
            self.msg_type,
            self.cll_id,
            self.cop_id,
            self.status,
            len(self.payload),
        )
        # Requirement 6: CRC16-CCITT computed over Msg_Type (byte 2) through end of Payload
        crc_input = hdr[2:] + self.payload
        crc = crc16_ccitt(crc_input)
        if corrupt_crc:
            crc ^= 0x0001
        return hdr + self.payload + struct.pack("<H", crc)


@dataclass
class DpduResultEvent:
    rx_timestamp_us: int
    rx_can_id: int
    extra_addr_info: int
    pdu: bytes
    elapsed_ms: int = 0

    @property
    def rx_time_ms(self) -> int:
        return self.elapsed_ms


@dataclass
class DpduStatusEvent:
    timestamp_us: int
    status_code: int


@dataclass
class DpduErrorEvent:
    timestamp_us: int
    error_code: int
    vendor_subcode: int


class DpduStreamParser:
    """Byte-stream parser for 0xAA55 binary App-Device IF D-PDU packets."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._state = "SOF0"
        self._hdr = bytearray()
        self._payload = bytearray()
        self._crc = bytearray()
        self._expected_len = 0

    def feed_byte(self, b: int) -> Optional[DpduFrame]:
        if self._state == "SOF0":
            if b == DPDU_SOF_BYTE_0:
                self._hdr = bytearray([b])
                self._state = "SOF1"
            return None

        if self._state == "SOF1":
            if b == DPDU_SOF_BYTE_1:
                self._hdr.append(b)
                self._state = "HDR"
            elif b == DPDU_SOF_BYTE_0:
                self._hdr = bytearray([b])
            else:
                self.reset()
            return None

        if self._state == "HDR":
            self._hdr.append(b)
            if len(self._hdr) == DPDU_HEADER_SIZE:
                _, _, _, _, _, length = struct.unpack("<HBBBBH", self._hdr)
                if length > DPDU_MAX_PAYLOAD:
                    self.reset()
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
                calc_crc = crc16_ccitt(bytes(self._hdr[2:]) + bytes(self._payload))
                _, msg_type, cll_id, cop_id, status, _ = struct.unpack("<HBBBBH", self._hdr)
                payload_bytes = bytes(self._payload)
                self.reset()
                if rx_crc != calc_crc:
                    raise ValueError(f"Host RX CRC mismatch: rx=0x{rx_crc:04x} != calc=0x{calc_crc:04x}")
                return DpduFrame(
                    msg_type=msg_type,
                    cll_id=cll_id,
                    cop_id=cop_id,
                    status=status,
                    payload=payload_bytes,
                )
            return None

        self.reset()
        return None


class LeanDpduSerialClient:
    """PC Host ISO 22900-2 App-Device IF Client over USB-UART0."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.parser = DpduStreamParser()
        self._rx_queue: list[DpduFrame] = []
        self._next_cop_id = 1

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

    def alloc_cop_id(self) -> int:
        cid = self._next_cop_id
        self._next_cop_id = (self._next_cop_id % 254) + 1
        return cid

    def recv_frame(self, timeout: float = 1.0) -> DpduFrame:
        if self._rx_queue:
            return self._rx_queue.pop(0)
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(256)
            for b in chunk:
                frame = self.parser.feed_byte(b)
                if frame is not None:
                    self._rx_queue.append(frame)
            if self._rx_queue:
                return self._rx_queue.pop(0)
        raise TimeoutError("Timed out waiting for D-PDU frame from VCI")

    def send_raw_cmd(
        self,
        msg_type: int,
        payload: bytes = b"",
        cll_id: int = 1,
        cop_id: int = 0,
        corrupt_crc: bool = False,
    ) -> int:
        if msg_type == PDU_CMD_START_COP and cop_id == 0:
            cop_id = self.alloc_cop_id()
        req = DpduFrame(msg_type=msg_type, cll_id=cll_id, cop_id=cop_id, status=0, payload=payload)
        self.ser.write(req.encode(corrupt_crc=corrupt_crc))
        self.ser.flush()
        return cop_id

    def send_and_recv_ack(
        self,
        msg_type: int,
        payload: bytes = b"",
        cll_id: int = 1,
        cop_id: int = 0,
        timeout: float = 1.0,
    ) -> DpduFrame:
        self.send_raw_cmd(msg_type, payload=payload, cll_id=cll_id, cop_id=cop_id)
        deferred: list[DpduFrame] = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            f = self.recv_frame(timeout=max(0.05, deadline - time.time()))
            if f.msg_type == PDU_EVT_CMD_ACK and f.cll_id == cll_id and f.cop_id == cop_id:
                self._rx_queue = deferred + self._rx_queue
                return f
            deferred.append(f)
        raise TimeoutError("Timed out waiting for 0x80 synchronous Command ACK")

    def pdu_ioctl(self, ioctl_id: int) -> DpduFrame:
        return self.send_and_recv_ack(
            PDU_CMD_MOD_CTRL,
            payload=struct.pack("<H", ioctl_id),
            cll_id=0,
            cop_id=0,
        )

    def pdu_read_vbatt_mv(self) -> int:
        ack = self.pdu_ioctl(PDU_IOCTL_READ_VBATT)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 4
        return struct.unpack("<I", ack.payload)[0]

    def pdu_get_cable_id(self) -> int:
        ack = self.pdu_ioctl(PDU_IOCTL_GET_CABLE_ID)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 4
        return struct.unpack("<I", ack.payload)[0]

    def pdu_get_version(self) -> dict:
        ack = self.pdu_ioctl(PDU_IOCTL_GET_VERSION)
        assert ack.status == PDU_STATUS_NOERROR and len(ack.payload) == 16
        fw_ver, hw_ver = struct.unpack("<II", ack.payload[:8])
        serial_mac = ack.payload[8:14].hex(":").upper()
        return {"fw_version": fw_ver, "hw_version": hw_ver, "serial": serial_mac}

    def pdu_get_status(self, cll_id: int = 1) -> dict:
        rsp = self.send_and_recv_ack(PDU_CMD_GET_STATUS, cll_id=cll_id)
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

    def pdu_connect(
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
        ack = self.send_and_recv_ack(PDU_CMD_CONNECT, payload=payload, cll_id=cll_id)
        if ack.status == PDU_STATUS_NOERROR:
            # Consume the PDU_CLLST_ONLINE (0x82) state transition event
            st_evt = self.recv_frame(timeout=0.5)
            assert st_evt.msg_type == PDU_IT_STATUS
        return ack

    def pdu_disconnect(self, cll_id: int = 1) -> DpduFrame:
        ack = self.send_and_recv_ack(PDU_CMD_DISCONNECT, cll_id=cll_id)
        if ack.status == PDU_STATUS_NOERROR:
            # Drain any PDU_CLLST_OFFLINE (0x82) event(s)
            try:
                while True:
                    f = self.recv_frame(timeout=0.05)
                    if f.msg_type != PDU_IT_STATUS:
                        self._rx_queue.insert(0, f)
                        break
            except TimeoutError:
                pass
        return ack

    def pdu_set_com_param(self, param_id: int, param_val: int, cll_id: int = 1) -> DpduFrame:
        payload = struct.pack("<HI", param_id, param_val)
        return self.send_and_recv_ack(PDU_CMD_SET_COMPARAM, payload=payload, cll_id=cll_id)

    def pdu_get_com_param(self, param_id: int, cll_id: int = 1) -> int:
        rsp = self.send_and_recv_ack(PDU_CMD_GET_COMPARAM, struct.pack("<H", param_id), cll_id=cll_id)
        assert rsp.status == PDU_STATUS_NOERROR and len(rsp.payload) == 6
        _, val = struct.unpack("<HI", rsp.payload)
        return val

    @staticmethod
    def build_start_cop_payload(
        pdu_data: bytes,
        num_send_cycles: int = 1,
        num_recv_cycles: int = 1,
        cycle_time_ms: int = 0,
        can_id: int = 0,
    ) -> bytes:
        return struct.pack("<BBHI", num_send_cycles, num_recv_cycles, cycle_time_ms, can_id) + pdu_data

    def pdu_start_com_primitive(
        self,
        pdu_data: bytes,
        cop_type: int = PDU_COPT_SENDRECV,
        flags: int = 0,
        timeout: float = 2.0,
        cll_id: int = 1,
        cop_id: int = 0,
        num_send_cycles: int = 1,
        num_recv_cycles: Optional[int] = None,
        cycle_time_ms: int = 0,
        can_id: int = 0,
    ) -> Tuple[DpduFrame, Optional[DpduFrame]]:
        """
        Send StartComPrimitive (0x04) and wait for one transaction cycle to complete.
        Consumes PDU_IT_STATUS (PDU_COPST_EXECUTING / PDU_COPST_FINISHED) automatically
        and returns (ack_frame, result_or_error_frame).
        """
        _ = flags
        if num_recv_cycles is None:
            num_recv_cycles = 0 if cop_type == PDU_COPT_SEND_ONLY else 1
        if cop_id == 0:
            cop_id = self.alloc_cop_id()

        payload = self.build_start_cop_payload(
            pdu_data,
            num_send_cycles=num_send_cycles,
            num_recv_cycles=num_recv_cycles,
            cycle_time_ms=cycle_time_ms,
            can_id=can_id,
        )
        t0 = time.time()
        ack = self.send_and_recv_ack(PDU_CMD_START_COP, payload=payload, cll_id=cll_id, cop_id=cop_id, timeout=timeout)
        if ack.status != PDU_STATUS_NOERROR:
            return ack, None

        outcome_evt: Optional[DpduFrame] = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            f = self.recv_frame(timeout=max(0.05, deadline - time.time()))
            if f.cll_id != cll_id or f.cop_id != cop_id:
                continue
            if f.msg_type in (PDU_IT_RESULT, PDU_IT_ERROR):
                outcome_evt = f
                setattr(outcome_evt, "_elapsed_ms", max(1, int((time.time() - t0) * 1000)))
            elif f.msg_type == PDU_IT_STATUS:
                st = self.parse_status_event(f)
                if st.status_code in (PDU_COPST_FINISHED, PDU_COPST_CANCELLED):
                    break
        return ack, outcome_evt

    @staticmethod
    def parse_result_event(evt: DpduFrame) -> DpduResultEvent:
        if evt.msg_type != PDU_IT_RESULT or len(evt.payload) < 10:
            raise AssertionError(
                f"Expected PDU_IT_RESULT (0x81), got msg_type=0x{evt.msg_type:02X} payload={evt.payload.hex(' ')}"
            )
        rx_ts_us, rx_can_id, extra_addr = struct.unpack("<IIB", evt.payload[:9])
        elapsed_ms = getattr(evt, "_elapsed_ms", 1)
        return DpduResultEvent(
            rx_timestamp_us=rx_ts_us,
            rx_can_id=rx_can_id,
            extra_addr_info=extra_addr,
            pdu=evt.payload[9:],
            elapsed_ms=elapsed_ms,
        )

    @staticmethod
    def parse_status_event(evt: DpduFrame) -> DpduStatusEvent:
        assert evt.msg_type == PDU_IT_STATUS and len(evt.payload) == 6
        ts_us, code = struct.unpack("<IH", evt.payload)
        return DpduStatusEvent(timestamp_us=ts_us, status_code=code)

    @staticmethod
    def parse_error_event(evt: DpduFrame) -> DpduErrorEvent:
        assert evt.msg_type == PDU_IT_ERROR and len(evt.payload) == 8
        ts_us, err_code, subcode = struct.unpack("<IHH", evt.payload)
        return DpduErrorEvent(timestamp_us=ts_us, error_code=err_code, vendor_subcode=subcode)


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


def run_step2_verification():
    print("=== Step 2 Hardware Verification: CAN HAL + Single-Frame OBD-II Mode 01 + P2/P2* Timers ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/7] Resetting ESP32 VCI & connecting CLL 1 (SimLoopback, 500 kbps)...")
        client.reset_vci_board()
        ack = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, baud_khz=500, cll_id=1)
        assert ack.msg_type == PDU_EVT_CMD_ACK and ack.status == PDU_STATUS_NOERROR
        print("      Connected CLL 1 (CAN_REQ_ID=0x7DF, CAN_RESP_ID=0x7E8, P2_Max=50ms, P2*_Max=5000ms)")

        print("[2/7] Querying OBD-II Mode 01 PID 0x00 (Supported PIDs 01..20)...")
        _, evt = client.pdu_start_com_primitive(b"\x01\x00")
        res = client.parse_result_event(evt)
        print(f"      RX CAN ID=0x{res.rx_can_id:03X} (ts={res.rx_timestamp_us} us) PDU={res.pdu.hex(' ').upper()}")
        assert res.rx_can_id == 0x7E8 and res.pdu == bytes.fromhex("41 00 BE 3E B8 13")

        print("[3/7] Querying OBD-II Mode 01 PID 0x0C (Engine RPM)...")
        _, evt = client.pdu_start_com_primitive(b"\x01\x0C")
        res = client.parse_result_event(evt)
        rpm = ((res.pdu[2] << 8) | res.pdu[3]) / 4.0
        print(f"      RX CAN ID=0x{res.rx_can_id:03X} (ts={res.rx_timestamp_us} us) PDU={res.pdu.hex(' ').upper()} -> {rpm:.1f} RPM")
        assert res.pdu[:2] == b"\x41\x0C" and rpm == 1726.0

        print("[4/7] Querying OBD-II Mode 01 PID 0x0D (Vehicle Speed) & PID 0x05 (Coolant Temp)...")
        _, evt_spd = client.pdu_start_com_primitive(b"\x01\x0D")
        res_spd = client.parse_result_event(evt_spd)
        speed_kmh = res_spd.pdu[2]

        _, evt_tmp = client.pdu_start_com_primitive(b"\x01\x05")
        res_tmp = client.parse_result_event(evt_tmp)
        coolant_c = res_tmp.pdu[2] - 40
        print(f"      Speed   : {res_spd.pdu.hex(' ').upper()} -> {speed_kmh} km/h")
        print(f"      Coolant : {res_tmp.pdu.hex(' ').upper()} -> {coolant_c} °C")
        assert speed_kmh == 85 and coolant_c == 90

        print("[5/7] Querying OBD-II Mode 01 PID 0x11 (Throttle Position)...")
        _, evt_thr = client.pdu_start_com_primitive(b"\x01\x11")
        res_thr = client.parse_result_event(evt_thr)
        throttle_pct = (res_thr.pdu[2] * 100.0) / 255.0
        print(f"      Throttle: {res_thr.pdu.hex(' ').upper()} -> {throttle_pct:.1f}%")
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
        err_to = client.parse_error_event(evt_to)
        assert err_to.error_code == PDU_ERR_EVT_RX_TIMEOUT and err_to.vendor_subcode == PDU_ERR_P2_TIMEOUT
        print(
            f"      VCI enforced P2_Max timeout and emitted 0x83 PDU_IT_ERROR "
            f"(err=0x{err_to.error_code:04X}, subcode=0x{err_to.vendor_subcode:04X} PDU_ERR_P2_TIMEOUT)"
        )

        client.pdu_disconnect(cll_id=1)
        print("=== ALL 7 STEP 2 HARDWARE CHECKS PASSED! ===")
    finally:
        client.close()


def run_step3_verification():
    print("=== Step 3 Hardware Verification: Multi-Frame ISO 15765-2 (FF/FC/CF) for VIN, DTCs & UDS ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/8] Resetting ESP32 VCI & connecting CLL 1 (SimLoopback, 500 kbps)...")
        client.reset_vci_board()
        ack = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, baud_khz=500, cll_id=1)
        assert ack.status == PDU_STATUS_NOERROR
        print("      Connected (ISO-TP BS=0, STmin=0ms, N_Cr=1000ms)")

        print("[2/8] Querying OBD-II Mode 09 PID 0x02 (17-Char VIN via 4,095B Multi-Frame Pool)...")
        _, evt_vin = client.pdu_start_com_primitive(b"\x09\x02")
        res_vin = client.parse_result_event(evt_vin)
        vin_str = res_vin.pdu[3:].decode("ascii")
        print(f"      RX CAN ID=0x{res_vin.rx_can_id:03X} ({len(res_vin.pdu)} bytes) VIN='{vin_str}'")
        assert len(res_vin.pdu) == 20 and vin_str == "1FAFP40411F192837"

        print("[3/8] Testing Flow Control BlockSize (BS=1) & STmin (10 ms) on Mode 09 PID 0x04 (Cal ID)...")
        client.pdu_set_com_param(CP_BLOCK_SIZE, 1)
        client.pdu_set_com_param(CP_ST_MIN, 10)
        _, evt_cal = client.pdu_start_com_primitive(b"\x09\x04")
        res_cal = client.parse_result_event(evt_cal)
        cal_str = res_cal.pdu[3:].decode("ascii")
        print(f"      RX CAN ID=0x{res_cal.rx_can_id:03X} ({res_cal.rx_time_ms} ms) CalID='{cal_str}'")
        assert cal_str == "ESP32_VCI_CAL_01" and res_cal.rx_time_ms >= 20
        client.pdu_set_com_param(CP_BLOCK_SIZE, 0)
        client.pdu_set_com_param(CP_ST_MIN, 0)

        print("[4/8] Querying OBD-II Mode 03 (Read Stored Diagnostic Trouble Codes via Multi-Frame)...")
        _, evt_dtc = client.pdu_start_com_primitive(b"\x03")
        res_dtc = client.parse_result_event(evt_dtc)
        dtcs = decode_obd_dtcs(res_dtc.pdu)
        print(f"      RX PDU={res_dtc.pdu.hex(' ').upper()} -> Stored DTCs: {dtcs}")
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
        uds_req = b"\x2E\xF1\x90" + new_vin
        _, evt_2e = client.pdu_start_com_primitive(uds_req)
        res_2e = client.parse_result_event(evt_2e)
        assert res_2e.pdu == b"\x6E\xF1\x90"
        _, evt_vin2 = client.pdu_start_com_primitive(b"\x09\x02")
        res_vin2 = client.parse_result_event(evt_vin2)
        updated_vin = res_vin2.pdu[3:].decode("ascii")
        print(f"      UDS 0x2E Positive Rsp={res_2e.pdu.hex(' ').upper()}, Readback VIN='{updated_vin}'")
        assert updated_vin == "WBA3A5C51DF123456"

        print("[7/8] Testing Multi-Frame Sequence Number Mismatch Detection (09 F2)...")
        _, evt_seq = client.pdu_start_com_primitive(b"\x09\xF2")
        err_seq = client.parse_error_event(evt_seq)
        assert err_seq.error_code == PDU_ERR_EVT_PROT_ERR and err_seq.vendor_subcode == PDU_ERR_MALFORMED_FRAME
        print(f"      VCI detected bad CF sequence number -> 0x83 PDU_IT_ERROR (err=0x{err_seq.error_code:04X})")

        print("[8/8] Testing Multi-Frame N_Cr (1000 ms) Consecutive Frame Timeout (09 F1)...")
        t0 = time.time()
        _, evt_ncr = client.pdu_start_com_primitive(b"\x09\xF1", timeout=2.5)
        elapsed_ms = int((time.time() - t0) * 1000)
        err_ncr = client.parse_error_event(evt_ncr)
        assert err_ncr.error_code == PDU_ERR_EVT_RX_TIMEOUT and err_ncr.vendor_subcode == PDU_ERR_ISOTP_TIMEOUT
        print(f"      VCI enforced N_Cr (1000 ms) timeout at {elapsed_ms} ms -> 0x83 PDU_IT_ERROR")

        client.pdu_disconnect(cll_id=1)
        print("=== ALL 8 STEP 3 HARDWARE CHECKS PASSED! ===")
    finally:
        client.close()


def run_comparam_and_standards_verification():
    print("=== ISO 22900-2:2022 App-Device IF Verification: IOCTLs, 8 CLLs, 4-Slot Queue, 100us STmin & Cyclic COP ===")
    client = LeanDpduSerialClient("/dev/ttyUSB0", 115200)
    try:
        print("[1/10] Resetting VCI & verifying all 13 default ISO 22900-2 ComParams on CLL 1...")
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
            actual = client.pdu_get_com_param(cp_id, cll_id=1)
            assert actual == exp_val, f"ComParam 0x{cp_id:04X} mismatch: {actual} != {exp_val}"
        print("       All 13 ComParams verified at standard defaults!")

        print("[2/10] Verifying IOCTL_READ_VBATT, IOCTL_GET_CABLE_ID, PDUGetVersion & unsupported IOCTL NACK...")
        vbatt_mv = client.pdu_read_vbatt_mv()
        cable_id = client.pdu_get_cable_id()
        ver_info = client.pdu_get_version()
        bad_ioctl = client.pdu_ioctl(0x9999)
        assert vbatt_mv == 12600 and cable_id == 0x07AA
        assert ver_info["fw_version"] == 0x00020100 and bad_ioctl.status == PDU_ERR_FCT_NOT_SUPPORTED
        print(
            f"       VBATT={vbatt_mv} mV, CableID=0x{cable_id:04X} (SAE J1962), "
            f"FW=0x{ver_info['fw_version']:08X}, Serial={ver_info['serial']}, "
            f"Unknown IOCTL -> 0x{bad_ioctl.status:02X} (PDU_ERR_FCT_NOT_SUPPORTED)"
        )

        print("[3/10] Verifying 100 us Hardware Timestamps (uint32 us) & Sub-ms STmin (0xF2 = 200 us)...")
        client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=1)
        client.pdu_set_com_param(CP_ST_MIN, 0xF2, cll_id=1)  # 200 us STmin!
        _, evt_us1 = client.pdu_start_com_primitive(b"\x09\x02", cll_id=1)
        res_us1 = client.parse_result_event(evt_us1)
        _, evt_us2 = client.pdu_start_com_primitive(b"\x01\x0C", cll_id=1)
        res_us2 = client.parse_result_event(evt_us2)
        delta_us = (res_us2.rx_timestamp_us - res_us1.rx_timestamp_us) & 0xFFFFFFFF
        assert res_us1.pdu[3:].decode("ascii") == "1FAFP40411F192837" and delta_us > 1000
        client.pdu_set_com_param(CP_ST_MIN, 0, cll_id=1)
        print(f"       STmin=0xF2 (200 us) VIN OK; Hardware timestamps: t1={res_us1.rx_timestamp_us} us, t2={res_us2.rx_timestamp_us} us (delta={delta_us} us)")

        print("[4/10] Verifying CP_CAN_FILLER_BYTE (0xAA and 0x55) on outgoing CAN frames (01 F3)...")
        client.pdu_set_com_param(CP_CAN_FILLER_BYTE, 0xAA, cll_id=1)
        _, evt_pad1 = client.pdu_start_com_primitive(b"\x01\xF3", cll_id=1)
        res_pad1 = client.parse_result_event(evt_pad1)
        client.pdu_set_com_param(CP_CAN_FILLER_BYTE, 0x55, cll_id=1)
        _, evt_pad2 = client.pdu_start_com_primitive(b"\x01\xF3", cll_id=1)
        res_pad2 = client.parse_result_event(evt_pad2)
        assert res_pad1.pdu[2] == 0xAA and res_pad2.pdu[2] == 0x55
        client.pdu_set_com_param(CP_CAN_FILLER_BYTE, 0xCC, cll_id=1)
        print(f"       Filler=0xAA -> ECU saw 0x{res_pad1.pdu[2]:02X}; Filler=0x55 -> ECU saw 0x{res_pad2.pdu[2]:02X}")

        print("[5/10] Verifying CP_P3_FUNC_MS (100 ms) vs CP_P3_PHYS_MS (5 ms) Inter-Request Enforcement...")
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_FUNCTIONAL, cll_id=1)
        client.pdu_set_com_param(CP_P3_FUNC_MS, 100, cll_id=1)
        t0 = time.time()
        client.pdu_start_com_primitive(b"\x01\x0C", cll_id=1, can_id=0x7DF)
        client.pdu_start_com_primitive(b"\x01\x0D", cll_id=1, can_id=0x7DF)
        func_pair_ms = int((time.time() - t0) * 1000)

        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL, cll_id=1)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7E0, cll_id=1)
        client.pdu_set_com_param(CP_P3_PHYS_MS, 5, cll_id=1)
        time.sleep(0.11)
        t1 = time.time()
        client.pdu_start_com_primitive(b"\x01\x0C", cll_id=1, can_id=0x7E0)
        client.pdu_start_com_primitive(b"\x01\x0D", cll_id=1, can_id=0x7E0)
        phys_pair_ms = int((time.time() - t1) * 1000)
        assert func_pair_ms >= 100 and phys_pair_ms < 75
        print(f"       Functional pair (P3_Func=100ms): {func_pair_ms} ms; Physical pair (P3_Phys=5ms): {phys_pair_ms} ms")

        print("[6/10] Verifying 29-Bit Extended OBD-II CAN (0x18DB33F1 -> 0x18DAF110) SF & Multi-Frame VIN...")
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_FUNCTIONAL, cll_id=1)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x18DB33F1, cll_id=1)
        client.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x18DAF110, cll_id=1)
        client.pdu_set_com_param(CP_P3_FUNC_MS, 10, cll_id=1)
        _, evt_29_sf = client.pdu_start_com_primitive(b"\x01\x0C", cll_id=1, can_id=0x18DB33F1)
        res_29_sf = client.parse_result_event(evt_29_sf)
        _, evt_29_mf = client.pdu_start_com_primitive(b"\x09\x02", cll_id=1, can_id=0x18DB33F1)
        res_29_mf = client.parse_result_event(evt_29_mf)
        assert res_29_sf.rx_can_id == 0x18DAF110 and res_29_mf.rx_can_id == 0x18DAF110
        print(f"       29-Bit SF RX ID=0x{res_29_sf.rx_can_id:08X}; 29-Bit MF VIN='{res_29_mf.pdu[3:].decode('ascii')}'")

        print("[7/10] Verifying Multi-CLL (CLL 1..3), Baudrate Lock Conflict Rejection & Round-Robin Arbitration...")
        # CLL 1 is online at 500 kbps. Try connecting CLL 2 at conflicting 250 kbps -> must reject!
        client.pdu_set_com_param(CP_BAUDRATE, 250000, cll_id=2)
        conflict_ack = client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=2)
        assert conflict_ack.status == PDU_ERR_BAUDRATE_CONFLICT

        # Now set CLL 2 (ECM 0x7E0->0x7E8) and CLL 3 (TCM 0x7E1->0x7E9) to 500 kbps and connect both
        client.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=2)
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL, cll_id=2)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7E0, cll_id=2)
        client.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x7E8, cll_id=2)
        assert client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=2).status == PDU_STATUS_NOERROR

        client.pdu_set_com_param(CP_BAUDRATE, 500000, cll_id=3)
        client.pdu_set_com_param(CP_ADDRESSING_TYPE, DPDU_ADDR_PHYSICAL, cll_id=3)
        client.pdu_set_com_param(CP_CAN_PHYS_REQ_ID, 0x7E1, cll_id=3)
        client.pdu_set_com_param(CP_CAN_RESP_USDT_ID, 0x7E9, cll_id=3)
        assert client.pdu_connect(bus_mode=VCI_BUS_MODE_SIM_LOOPBACK, cll_id=3).status == PDU_STATUS_NOERROR

        # Pipeline commands simultaneously across CLL 2 (0x7E0) and CLL 3 (0x7E1)
        cop_c2 = client.send_raw_cmd(
            PDU_CMD_START_COP,
            client.build_start_cop_payload(b"\x01\x0C", can_id=0x7E0),
            cll_id=2,
            cop_id=21,
        )
        cop_c3 = client.send_raw_cmd(
            PDU_CMD_START_COP,
            client.build_start_cop_payload(b"\x01\x0D", can_id=0x7E1),
            cll_id=3,
            cop_id=31,
        )
        rx_can_ids = {}
        finished_cops = set()
        while len(finished_cops) < 2:
            f = client.recv_frame(timeout=1.5)
            if f.msg_type == PDU_IT_RESULT:
                res = client.parse_result_event(f)
                rx_can_ids[(f.cll_id, f.cop_id)] = res.rx_can_id
            elif f.msg_type == PDU_IT_STATUS:
                st = client.parse_status_event(f)
                if st.status_code == PDU_COPST_FINISHED:
                    finished_cops.add((f.cll_id, f.cop_id))
        assert rx_can_ids[(2, cop_c2)] == 0x7E8 and rx_can_ids[(3, cop_c3)] == 0x7E9
        print("       Baudrate conflict rejected (0x05); CLL 2 (0x7E8) & CLL 3 (0x7E9) Round-Robin verified!")

        print("[8/10] Verifying 4-Slot ComPrimitive FIFO Queue per CLL & Overrun NACK (0x03) on 5th Request...")
        # Queue 4 primitives on CLL 2 (first one enters 5000ms P2* wait so all 4 slots stay occupied)
        for idx, pid in enumerate([b"\x01\xF4", b"\x01\x0C", b"\x01\x0D", b"\x01\x05"], start=1):
            ack_q = client.send_and_recv_ack(
                PDU_CMD_START_COP,
                client.build_start_cop_payload(pid, can_id=0x7E0),
                cll_id=2,
                cop_id=40 + idx,
            )
            assert ack_q.status == PDU_STATUS_NOERROR
        # 5th primitive on CLL 2 must be rejected synchronously with PDU_ERR_COMM_PC_TO_VCI_OVERRUN (0x03)
        ack_overrun = client.send_and_recv_ack(
            PDU_CMD_START_COP,
            client.build_start_cop_payload(b"\x01\x11", can_id=0x7E0),
            cll_id=2,
            cop_id=45,
        )
        assert ack_overrun.status == PDU_ERR_COMM_PC_TO_VCI_OVERRUN
        print("       4 slots accepted on CLL 2; 5th rejected with 0x80 NACK (PDU_ERR_COMM_PC_TO_VCI_OVERRUN = 0x03)!")

        print("[9/10] Verifying Out-of-Band CancelComPrimitive (0x05) Draining All 4 Slots on CLL 2...")
        ack_cancel = client.send_and_recv_ack(PDU_CMD_CANCEL_COP, cll_id=2, cop_id=0)
        assert ack_cancel.status == PDU_STATUS_NOERROR
        cancelled_cops = set()
        while len(cancelled_cops) < 4:
            f = client.recv_frame(timeout=1.5)
            if f.msg_type == PDU_IT_STATUS and f.cll_id == 2:
                st = client.parse_status_event(f)
                if st.status_code == PDU_COPST_CANCELLED:
                    cancelled_cops.add(f.cop_id)
        assert cancelled_cops == {41, 42, 43, 44}
        print(f"       All 4 primitives on CLL 2 cancelled cleanly (COP_IDs={sorted(cancelled_cops)})!")

        print("[10/10] Verifying Cyclic StartComPrimitive (NumSendCycles=3, CycleTime_ms=30)...")
        ack_cyc = client.send_and_recv_ack(
            PDU_CMD_START_COP,
            client.build_start_cop_payload(
                b"\x01\x0C",
                num_send_cycles=3,
                num_recv_cycles=1,
                cycle_time_ms=30,
                can_id=0x7E0,
            ),
            cll_id=2,
            cop_id=99,
        )
        assert ack_cyc.status == PDU_STATUS_NOERROR
        cyc_results = []
        cyc_finished = False
        while not cyc_finished:
            f = client.recv_frame(timeout=1.5)
            if f.cll_id == 2 and f.cop_id == 99:
                if f.msg_type == PDU_IT_RESULT:
                    cyc_results.append(client.parse_result_event(f))
                elif f.msg_type == PDU_IT_STATUS:
                    st = client.parse_status_event(f)
                    if st.status_code == PDU_COPST_FINISHED:
                        cyc_finished = True
        assert len(cyc_results) == 3
        print("       Cyclic StartComPrimitive autonomously emitted 3 PDU_IT_RESULTs + PDU_COPST_FINISHED!")

        client.pdu_disconnect(cll_id=0)
        print("=== ALL 10 ISO 22900-2:2022 APP-DEVICE IF VERIFICATION CHECKS PASSED! ===")
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
