"""
test_rgf_readwrite.py - Write values to FPGA RGF registers, read them back, and verify.

Message formats:
  MSG_RGF_WRITE (16 bytes):
    {W<addr><off_hi><off_lo>,V<dh1><dh0><pad>,V<dl1><dl0><pad>}
    byte0:  0x7B  '{'
    byte1:  0x57  'W'
    byte2:  addr  (base address - selects RGF block)
    byte3:  off_h (offset high byte)
    byte4:  off_l (offset low byte)
    byte5:  0x2C  ','
    byte6:  0x56  'V'
    byte7:  dh1   (data_high MSB)
    byte8:  dh0   (data_high LSB)
    byte9:  0x00  (padding)
    byte10: 0x2C  ','
    byte11: 0x56  'V'
    byte12: dl1   (data_low MSB)
    byte13: dl0   (data_low LSB)
    byte14: 0x00  (padding)
    byte15: 0x7D  '}'

  MSG_RGF_READ (6 bytes):
    {R<addr><off_hi><off_lo>}
    byte0: 0x7B  '{'
    byte1: 0x52  'R'
    byte2: addr
    byte3: off_h
    byte4: off_l
    byte5: 0x7D  '}'

  Read response from FPGA (16 bytes):
    {R<addr><off_h><off_l>,V<pad><d31:24><d23:16>,V<pad><d15:8><d7:0>}
    data = (byte8 << 24) | (byte9 << 16) | (byte13 << 8) | byte14

RGF Address Map (base addresses):
    0x10 = LED
    0x20 = SYS
    0x30 = CTRL
    0x40 = PWM
    0x50 = IMG
    0x60 = TX_FIFO

Usage:
    python test_rgf_readwrite.py
"""

import serial
import time
import sys

# =============================================================================
# Configuration
# =============================================================================
PORT = 'COM5'
BAUD_RATE = 5500000
TIMEOUT_READ = 0.5   # seconds to wait for read response

# Frame constants
CHAR_OPEN  = 0x7B  # '{'
CHAR_CLOSE = 0x7D  # '}'
CHAR_COMMA = 0x2C  # ','
OP_W       = 0x57  # 'W'
OP_R       = 0x52  # 'R'
CHAR_V     = 0x56  # 'V'

# RGF base addresses
RGF_LED    = 0x10
RGF_SYS    = 0x20
RGF_CTRL   = 0x30
RGF_PWM    = 0x40
RGF_IMG    = 0x50
RGF_TXFIFO = 0x60


# =============================================================================
# Message builders
# =============================================================================
def build_rgf_write(base_addr, offset, data32):
    """
    Build MSG_RGF_WRITE: {W<addr><off_h><off_l>,V<dh1><dh0><pad>,V<dl1><dl0><pad>}
    data32: 32-bit value to write.
      data_high = data32[31:16], data_low = data32[15:0]
    """
    dh = (data32 >> 16) & 0xFFFF
    dl = data32 & 0xFFFF

    msg = bytearray(16)
    msg[0]  = CHAR_OPEN
    msg[1]  = OP_W
    msg[2]  = base_addr & 0xFF
    msg[3]  = (offset >> 8) & 0xFF
    msg[4]  = offset & 0xFF
    msg[5]  = CHAR_COMMA
    msg[6]  = CHAR_V
    msg[7]  = (dh >> 8) & 0xFF   # data_high MSB
    msg[8]  = dh & 0xFF          # data_high LSB
    msg[9]  = 0x00               # padding
    msg[10] = CHAR_COMMA
    msg[11] = CHAR_V
    msg[12] = (dl >> 8) & 0xFF   # data_low MSB
    msg[13] = dl & 0xFF          # data_low LSB
    msg[14] = 0x00               # padding
    msg[15] = CHAR_CLOSE
    return bytes(msg)


def build_rgf_read(base_addr, offset):
    """
    Build MSG_RGF_READ: {R<addr><off_h><off_l>}
    Only 6 bytes - short message ending at position 5.
    """
    msg = bytearray(6)
    msg[0] = CHAR_OPEN
    msg[1] = OP_R
    msg[2] = base_addr & 0xFF
    msg[3] = (offset >> 8) & 0xFF
    msg[4] = offset & 0xFF
    msg[5] = CHAR_CLOSE
    return bytes(msg)


def parse_rgf_response(raw_bytes):
    """
    Parse a 16-byte RGF read response from the FPGA.
    Format: {R<addr><off_h><off_l>,V<pad><d31:24><d23:16>,V<pad><d15:8><d7:0>}

    Returns (addr, offset, data32) or None on failure.
    """
    if len(raw_bytes) < 16:
        return None

    # Validate frame
    if raw_bytes[0] != CHAR_OPEN:
        return None
    if raw_bytes[1] != OP_R:
        return None
    if raw_bytes[5] != CHAR_COMMA:
        return None
    if raw_bytes[6] != CHAR_V:
        return None
    if raw_bytes[10] != CHAR_COMMA:
        return None
    if raw_bytes[11] != CHAR_V:
        return None
    if raw_bytes[15] != CHAR_CLOSE:
        return None

    addr   = raw_bytes[2]
    offset = (raw_bytes[3] << 8) | raw_bytes[4]
    data32 = (raw_bytes[8] << 24) | (raw_bytes[9] << 16) | (raw_bytes[13] << 8) | raw_bytes[14]
    return (addr, offset, data32)


def receive_response(ser, timeout=TIMEOUT_READ):
    """
    Read a 16-byte response from FPGA.
    Searches for '{' start delimiter, then reads 15 more bytes.
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        byte = ser.read(1)
        if byte and byte[0] == CHAR_OPEN:
            # Found start, read remaining 15 bytes
            rest = ser.read(15)
            if len(rest) == 15:
                full_msg = bytes([CHAR_OPEN]) + rest
                return full_msg
            else:
                return None
    return None


def addr_name(base_addr):
    """Human-readable name for RGF base address."""
    names = {
        RGF_LED:    "LED",
        RGF_SYS:    "SYS",
        RGF_CTRL:   "CTRL",
        RGF_PWM:    "PWM",
        RGF_IMG:    "IMG",
        RGF_TXFIFO: "TX_FIFO",
    }
    return names.get(base_addr, f"0x{base_addr:02X}")


# =============================================================================
# Test definitions
# =============================================================================
# Each test: (base_addr, offset, write_value, mask, description)
# mask: bits that the RGF actually stores (for comparison)
#
# Valid register maps:
#   LED  (0x10): 0x0=LED_CTRL[5:0], 0x4=PATTERN_CFG[1:0]
#   SYS  (0x20): 0x0=CTRL[1:0]
#   CTRL (0x30): 0x0=RX_PKT_CNT[31:0], 0x4=TX_PKT_CNT[31:0], 0x8=COLOR_CNT[31:0], 0xC=CFG_CNT[31:0]
#   PWM  (0x40): 0x0=PWM_CFG[31:0], 0x4=RED_LUT[23:0], 0x8=GREEN_LUT[23:0], 0xC=BLUE_LUT[23:0]
#   IMG  (0x50): 0x0=IMG_STATUS[20:0], 0x4=IMG_TX_MON(RO), 0x8=IMG_CTRL[0]
TESTS = [
    # LED register tests
    (RGF_LED,  0x0000, 0x0000003F, 0x0000003F, "LED_CTRL all writable bits"),
    (RGF_LED,  0x0004, 0x00000003, 0x00000003, "LED_PATTERN_CFG mode bits"),

    # SYS register test (only 2 bits writable)
    (RGF_SYS,  0x0000, 0x00000002, 0x00000003, "SYS_CTRL enable bit"),

    # CTRL (counter) register tests - full 32-bit
    (RGF_CTRL, 0x0000, 0xCAFEBABE, 0xFFFFFFFF, "CNT_RX_PKT full 32-bit"),
    (RGF_CTRL, 0x0004, 0x12345678, 0xFFFFFFFF, "CNT_TX_PKT full 32-bit"),
    (RGF_CTRL, 0x0008, 0xA5A5A5A5, 0xFFFFFFFF, "CNT_COLOR full 32-bit"),
    (RGF_CTRL, 0x000C, 0xDEADBEEF, 0xFFFFFFFF, "CNT_CONFIG full 32-bit"),

    # PWM register tests
    (RGF_PWM,  0x0000, 0x00FF00FF, 0xFFFFFFFF, "PWM_CFG time/sweep"),
    (RGF_PWM,  0x0004, 0x00ABCDEF, 0x00FFFFFF, "PWM_RED_LUT 24-bit"),
    (RGF_PWM,  0x0008, 0x00123456, 0x00FFFFFF, "PWM_GREEN_LUT 24-bit"),
    (RGF_PWM,  0x000C, 0x00FEDCBA, 0x00FFFFFF, "PWM_BLUE_LUT 24-bit"),

    # IMG register tests
    (RGF_IMG,  0x0000, 0x001AAAAA, 0x001FFFFF, "IMG_STATUS height/width/ready"),
    (RGF_IMG,  0x0008, 0x00000001, 0x00000001, "IMG_CTRL start_read bit"),
]


# =============================================================================
# Main
# =============================================================================
def main():
    print("=" * 60)
    print("  FPGA RGF Write/Read Verification Test")
    print("=" * 60)
    print(f"  Port: {PORT}   Baud: {BAUD_RATE}")
    print(f"  Tests: {len(TESTS)}")
    print()

    # Open serial port
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT_READ,
            write_timeout=1,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )
        ser.set_buffer_size(rx_size=65536)
        print(f"  Connected to {PORT}\n")
    except Exception as e:
        print(f"  ERROR: {e}")
        input("\nPress Enter to exit...")
        return

    time.sleep(0.1)
    ser.reset_input_buffer()

    pass_count = 0
    fail_count = 0
    no_resp_count = 0

    # =========================================================================
    # Phase 1: WRITE all values
    # =========================================================================
    print("-" * 60)
    print("  PHASE 1: Writing values to RGF registers")
    print("-" * 60)
    for i, (base, offset, value, mask, desc) in enumerate(TESTS):
        msg = build_rgf_write(base, offset, value)
        ser.write(msg)
        ser.flush()
        name = addr_name(base)
        print(f"  [{i+1:2d}/{len(TESTS)}] WRITE {name}[0x{offset:04X}] = 0x{value:08X}  ({desc})")
        time.sleep(0.005)  # 5ms between writes for handshake completion

    print()
    time.sleep(0.1)  # let pipeline settle
    ser.reset_input_buffer()  # clear any stale data

    # =========================================================================
    # Phase 2: READ back and verify
    # =========================================================================
    print("-" * 60)
    print("  PHASE 2: Reading back and verifying")
    print("-" * 60)

    for i, (base, offset, value, mask, desc) in enumerate(TESTS):
        name = addr_name(base)
        expected = value & mask  # only compare writable bits

        # Send read request
        msg = build_rgf_read(base, offset)
        ser.write(msg)
        ser.flush()

        # Wait for response
        raw = receive_response(ser, timeout=TIMEOUT_READ)

        if raw is None:
            print(f"  [{i+1:2d}/{len(TESTS)}] READ  {name}[0x{offset:04X}]  => NO RESPONSE  ({desc})")
            no_resp_count += 1
            time.sleep(0.05)
            continue

        parsed = parse_rgf_response(raw)
        if parsed is None:
            print(f"  [{i+1:2d}/{len(TESTS)}] READ  {name}[0x{offset:04X}]  => BAD FORMAT: {raw.hex()}  ({desc})")
            fail_count += 1
            time.sleep(0.05)
            continue

        r_addr, r_offset, r_data = parsed
        match = (r_data & mask) == expected

        if match:
            print(f"  [{i+1:2d}/{len(TESTS)}] READ  {name}[0x{offset:04X}]  => 0x{r_data:08X}  PASS  ({desc})")
            pass_count += 1
        else:
            print(f"  [{i+1:2d}/{len(TESTS)}] READ  {name}[0x{offset:04X}]  => 0x{r_data:08X}  FAIL  ({desc})")
            print(f"           expected: 0x{expected:08X} (mask: 0x{mask:08X})")
            fail_count += 1

        # Debug: show raw bytes on failure or first test
        if not match or i == 0:
            print(f"           raw: {raw.hex()}")

        time.sleep(0.05)  # delay between reads

    # =========================================================================
    # Summary
    # =========================================================================
    print()
    print("=" * 60)
    print(f"  RESULTS:  {pass_count} PASS  /  {fail_count} FAIL  /  {no_resp_count} NO RESPONSE")
    print(f"  Total:    {len(TESTS)} tests")
    if fail_count == 0 and no_resp_count == 0:
        print("  STATUS:   ALL TESTS PASSED")
    else:
        print("  STATUS:   SOME TESTS FAILED")
    print("=" * 60)

    ser.close()
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")
