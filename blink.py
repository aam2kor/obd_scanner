#!/usr/bin/env python3
"""
blink.py — Assemble, flash, and verify a 1-second LED blink program on ESP32.

Builds a standalone ESP32 bootable firmware image (blink.bin), flashes it to
SPI flash offset 0x1000 via the ESP32 ROM bootloader over /dev/ttyUSB0, resets
the board into normal SPI_FAST_FLASH_BOOT mode, and verifies the 1.000s blink
intervals over UART0.
"""
import pathlib
import struct
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200
LOAD_ADDR = 0x40078000
FLASH_OFFSET = 0x1000
TICKS_1S = 8_000_000  # 8,000,000 * 5 cycles = 40,000,000 cycles = 1.000s @ 40 MHz


def slip_encode(pkt: bytes) -> bytes:
    return b"\xc0" + pkt.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"


def slip_read(s: serial.Serial, timeout: float = 0.5):
    deadline = time.time() + timeout
    buf = b""
    in_pkt = False
    while time.time() < deadline:
        ch = s.read(1)
        if not ch:
            continue
        if ch == b"\xc0":
            if in_pkt and len(buf) > 0:
                return buf.replace(b"\xdb\xdc", b"\xc0").replace(b"\xdb\xdd", b"\xdb")
            in_pkt = True
            buf = b""
        elif in_pkt:
            buf += ch
    return None


def send_cmd(s: serial.Serial, op: int, data: bytes, chk: int = 0, timeout: float = 2.0):
    hdr = struct.pack("<BBHI", 0x00, op, len(data), chk)
    s.write(slip_encode(hdr + data))
    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = slip_read(s, max(0.05, deadline - time.time()))
        if resp and resp[0] == 0x01 and resp[1] == op:
            status = resp[8:10] if len(resp) >= 10 else resp[-4:-2]
            if status[0] != 0:
                raise RuntimeError(f"ROM cmd 0x{op:02x} returned error {status.hex()}")
            return resp
    raise RuntimeError(f"ROM cmd 0x{op:02x} timed out")


def sync_bootloader(s: serial.Serial) -> bool:
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.dtr = True
    s.rts = False
    time.sleep(0.05)
    s.dtr = False
    time.sleep(0.1)
    s.reset_input_buffer()

    payload = b"\x07\x07\x12\x20" + (b"\x55" * 32)
    hdr = struct.pack("<BBHI", 0x00, 0x08, len(payload), 0)
    pkt = slip_encode(hdr + payload)
    for _ in range(10):
        s.write(pkt)
        resp = slip_read(s, 0.15)
        if resp and len(resp) >= 8 and resp[0] == 0x01 and resp[1] == 0x08:
            while slip_read(s, 0.1):
                pass
            return True
    return False


def build_blink_segment(base_addr: int = LOAD_ADDR, ticks_per_sec: int = TICKS_1S):
    literals = []

    def add_lit(val: int) -> int:
        if val not in literals:
            literals.append(val)
        return literals.index(val)

    REG_TIMG0_WDTWPROTECT = add_lit(0x3FF5F064)
    REG_TIMG0_WDTCONFIG0  = add_lit(0x3FF5F048)
    REG_RTC_WDTWPROTECT   = add_lit(0x3FF480A4)
    REG_RTC_WDTCONFIG0    = add_lit(0x3FF4808C)
    VAL_WDT_WKEY          = add_lit(0x50D83AA1)
    VAL_ZERO              = add_lit(0x00000000)

    REG_IO_MUX_GPIO2      = add_lit(0x3FF49040)
    VAL_IO_MUX_GPIO2      = add_lit(0x00002800)  # MCU_SEL=2 (GPIO2), FUN_DRV=2
    REG_GPIO_FUNC2_OUT    = add_lit(0x3FF44538)
    VAL_GPIO_SIMPLE_OUT   = add_lit(0x00000100)  # Simple GPIO output
    REG_GPIO_ENABLE_W1TS  = add_lit(0x3FF44024)
    REG_GPIO_OUT_W1TS     = add_lit(0x3FF44008)
    REG_GPIO_OUT_W1TC     = add_lit(0x3FF4400C)
    VAL_GPIO2_MASK        = add_lit(1 << 2)

    REG_UART0_FIFO        = add_lit(0x3FF40000)
    VAL_CHAR_O            = add_lit(ord("O"))
    VAL_CHAR_N            = add_lit(ord("N"))
    VAL_CHAR_F            = add_lit(ord("F"))
    VAL_CHAR_CR           = add_lit(ord("\r"))
    VAL_CHAR_LF           = add_lit(ord("\n"))
    VAL_ONE               = add_lit(1)
    VAL_TICKS             = add_lit(ticks_per_sec)

    lit_size = len(literals) * 4
    code_start = base_addr + lit_size
    code = bytearray()

    def emit_l32r(reg: int, lit_idx: int):
        pc = code_start + len(code)
        lit_addr = base_addr + lit_idx * 4
        aligned_pc = (pc + 3) & ~3
        off_words = (lit_addr - aligned_pc) // 4
        assert -32768 <= off_words < 0
        code.append(((reg & 0xF) << 4) | 0x01)
        code.extend(struct.pack("<h", off_words))

    def emit_write_reg(reg_lit_idx: int, val_lit_idx: int):
        emit_l32r(8, reg_lit_idx)
        emit_l32r(2, val_lit_idx)
        code.extend(b"\xc0\x20\x00")  # memw
        code.extend(b"\x29\x08")      # s32i.n a2, a8, 0

    def emit_delay():
        emit_l32r(11, VAL_ONE)        # a11 = 1
        emit_l32r(2, VAL_TICKS)       # a2 = ticks_per_sec
        code.extend(b"\xa0\xaa\xc0")  # sub a10, a10, a10 (a10 = 0)
        while (code_start + len(code)) % 4 != 0:
            code.extend(b"\xc0\x20\x00")  # 3-byte memw NOP to 4-byte align loop
        code.extend(b"\xb0\xaa\x80")  # add a10, a10, a11 (a10 += 1)
        code.extend(b"\x27\x3a\xf9")  # bltu a10, a2, -7

    # 1. Disable hardware watchdogs
    emit_write_reg(REG_TIMG0_WDTWPROTECT, VAL_WDT_WKEY)
    emit_write_reg(REG_TIMG0_WDTCONFIG0, VAL_ZERO)
    emit_write_reg(REG_RTC_WDTWPROTECT, VAL_WDT_WKEY)
    emit_write_reg(REG_RTC_WDTCONFIG0, VAL_ZERO)

    # 2. Configure GPIO 2 as push-pull output
    emit_write_reg(REG_IO_MUX_GPIO2, VAL_IO_MUX_GPIO2)
    emit_write_reg(REG_GPIO_FUNC2_OUT, VAL_GPIO_SIMPLE_OUT)
    emit_write_reg(REG_GPIO_ENABLE_W1TS, VAL_GPIO2_MASK)

    # 3. Main 1-second blink loop
    loop_pc = code_start + len(code)

    # Turn GPIO 2 LED ON + output "ON\r\n"
    emit_write_reg(REG_GPIO_OUT_W1TS, VAL_GPIO2_MASK)
    emit_l32r(8, REG_UART0_FIFO)
    for ch_lit in (VAL_CHAR_O, VAL_CHAR_N, VAL_CHAR_CR, VAL_CHAR_LF):
        emit_l32r(2, ch_lit)
        code.extend(b"\x29\x08")
    emit_delay()

    # Turn GPIO 2 LED OFF + output "OFF\r\n"
    emit_write_reg(REG_GPIO_OUT_W1TC, VAL_GPIO2_MASK)
    emit_l32r(8, REG_UART0_FIFO)
    for ch_lit in (VAL_CHAR_O, VAL_CHAR_F, VAL_CHAR_F, VAL_CHAR_CR, VAL_CHAR_LF):
        emit_l32r(2, ch_lit)
        code.extend(b"\x29\x08")
    emit_delay()

    # Unconditional jump back to loop_pc
    code.extend(b"\xa0\xaa\xc0")  # sub a10, a10, a10 (a10 = 0)
    branch_pc = code_start + len(code)
    rel_off = loop_pc - (branch_pc + 4)
    assert -128 <= rel_off < 0
    code.extend(bytes([0x27, 0x3A, rel_off & 0xFF]))

    while len(code) % 4 != 0:
        code.append(0x00)

    lit_bytes = b"".join(struct.pack("<I", w) for w in literals)
    return lit_bytes + bytes(code), code_start


def build_esp32_image() -> bytes:
    seg_data, entry_addr = build_blink_segment()
    # 24-byte ESP32 image header (magic=0xE9, segments=1, DIO=2, 4MB/40MHz=0x20)
    img_hdr = struct.pack(
        "<BBBBIB3sHB8sB",
        0xE9,         # magic
        1,            # segment_count
        0x02,         # spi_mode = DIO
        0x20,         # spi_speed_size = 4MB, 40MHz
        entry_addr,   # entry_addr
        0xEE,         # wp_pin = disabled
        b"\x00\x00\x00",  # spi_pin_drv
        0x0000,       # chip_id = ESP32
        0x00,         # min_chip_rev
        b"\x00" * 8,  # reserved
        0x00,         # hash_appended = 0
    )
    seg_hdr = struct.pack("<II", LOAD_ADDR, len(seg_data))
    raw = bytearray(img_hdr + seg_hdr + seg_data)

    # Pad so that (len(raw) + 1) is a multiple of 16 bytes, then append XOR checksum
    while (len(raw) + 1) % 16 != 0:
        raw.append(0x00)
    chk = 0xEF
    for b in seg_data:
        chk ^= b
    raw.append(chk)
    return bytes(raw)


def flash_and_verify():
    image = build_esp32_image()
    bin_path = pathlib.Path(__file__).with_name("blink.bin")
    bin_path.write_bytes(image)
    print(f"Built {bin_path.name} ({len(image)} bytes)")

    with serial.Serial(PORT, BAUD, timeout=0.1) as s:
        print(f"Syncing with ESP32 ROM bootloader on {PORT}...")
        if not sync_bootloader(s):
            raise RuntimeError("Failed to sync with ESP32 bootloader")

        # Attach SPI flash (0x0D) and configure 4MB flash parameters (0x0B)
        send_cmd(s, 0x0D, struct.pack("<II", 0, 0))
        send_cmd(
            s,
            0x0B,
            struct.pack("<IIIIII", 0, 4 * 1024 * 1024, 65536, 4096, 256, 0xFFFF),
        )

        # Flash image to offset 0x1000 via FLASH_BEGIN (0x02), FLASH_DATA (0x03), FLASH_END (0x04)
        block_size = 1024
        padded = image + (b"\xff" * ((block_size - (len(image) % block_size)) % block_size))
        num_blocks = len(padded) // block_size
        print(f"Flashing {len(image)} bytes to 0x{FLASH_OFFSET:04x}...")
        send_cmd(
            s,
            0x02,
            struct.pack("<IIII", len(padded), num_blocks, block_size, FLASH_OFFSET),
            timeout=5.0,
        )
        for seq in range(num_blocks):
            block = padded[seq * block_size : (seq + 1) * block_size]
            chk = 0xEF
            for b in block:
                chk ^= b
            send_cmd(
                s,
                0x03,
                struct.pack("<IIII", len(block), seq, 0, 0) + block,
                chk=chk,
                timeout=3.0,
            )
        send_cmd(s, 0x04, struct.pack("<I", 1))
        print("Flash complete! Resetting board into SPI_FAST_FLASH_BOOT...")

        # Hardware reset via RTS
        s.dtr = False
        s.rts = True
        time.sleep(0.1)
        s.reset_input_buffer()
        s.rts = False

        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.5:
            line = s.readline()
            if line:
                dt = time.monotonic() - t0
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"[{dt:6.3f}s] {text}")


if __name__ == "__main__":
    flash_and_verify()

