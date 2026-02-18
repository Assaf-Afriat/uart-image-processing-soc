"""
receive_burst_image.py - Request and receive 256x256 image from FPGA via UART

Protocol:
1. Send MSG_START_BURST_RD: {R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}
2. Receive MSG_BURST_PIXEL_WR (repeated): {R0,G0,B0,R1, G1,B1,R2,G2, B2,R3,G3,B3}
   - 4 pixels per message, interleaved RGB order
   - For 256x256 = 65536 pixels / 4 = 16384 messages

Output saved to: ../img_output/received_image_<timestamp>.ppm

Usage:
    python receive_burst_image.py
"""

import serial
import time
import sys
import os
from pathlib import Path
from datetime import datetime

# =============================================================================
# Configuration
# =============================================================================
PORT = 'COM5'           # Serial port - adjust as needed
BAUD_RATE = 5500000     # 5.5M baud - matches FPGA setting
IMG_WIDTH = 256
IMG_HEIGHT = 256
PIXELS_PER_MSG = 4      # 4 pixels per burst message
TIMEOUT_SEC = 30        # Timeout for receiving image

# UART Frame Configuration
DATA_BITS = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_ONE

# Message framing constants
CHAR_OPEN  = 0x7B  # '{'
CHAR_CLOSE = 0x7D  # '}'
CHAR_COMMA = 0x2C  # ','
OP_R       = 0x52  # 'R' - Read / Burst Read
CHAR_H     = 0x48  # 'H' - Height
CHAR_W     = 0x57  # 'W' - Width


def build_start_burst_read_msg(height, width, addr=0, offset=0):
    """
    Build MSG_START_BURST_RD message.
    Format: {R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}
    """
    msg = bytearray(16)

    # Frame start
    msg[0] = CHAR_OPEN

    # Opcode + address + offset
    msg[1] = OP_R                              # 'R'
    msg[2] = addr & 0xFF                       # Address (8-bit)
    msg[3] = (offset >> 8) & 0xFF              # Offset high byte
    msg[4] = offset & 0xFF                     # Offset low byte

    # Comma separator
    msg[5] = CHAR_COMMA

    # Height field: H<h2><h1><h0>
    msg[6] = CHAR_H                            # 'H'
    msg[7] = (height >> 16) & 0xFF             # Height byte 2 (MSB)
    msg[8] = (height >> 8) & 0xFF              # Height byte 1
    msg[9] = height & 0xFF                     # Height byte 0 (LSB)

    # Comma separator
    msg[10] = CHAR_COMMA

    # Width field: W<w2><w1><w0>
    msg[11] = CHAR_W                           # 'W'
    msg[12] = (width >> 16) & 0xFF             # Width byte 2 (MSB)
    msg[13] = (width >> 8) & 0xFF              # Width byte 1
    msg[14] = width & 0xFF                     # Width byte 0 (LSB)

    # Frame end
    msg[15] = CHAR_CLOSE

    return bytes(msg)


def parse_burst_pixel_msg(packet, debug=False):
    """
    Parse MSG_BURST_PIXEL_WR message (interleaved format).

    FPGA TX Format: {R0,G0,B0,R1, G1,B1,R2,G2, B2,R3,G3,B3}

    Byte indices after '{':
      0=R0, 1=G0, 2=B0, 3=R1, 4=',', 5=G1, 6=B1, 7=R2, 8=G2, 9=',', 10=B2, 11=R3, 12=G3, 13=B3, 14='}'

    packet: 15 bytes (after '{')
    Returns: list of 4 tuples [(R,G,B), ...] or None if invalid
    """
    # Validate frame structure
    if len(packet) < 15:
        if debug:
            print(f"    Parse error: packet too short ({len(packet)} bytes)")
        return None

    # Check delimiters: comma at index 4, 9 and close at 14
    if packet[4] != CHAR_COMMA or packet[9] != CHAR_COMMA or packet[14] != CHAR_CLOSE:
        if debug:
            print(f"    Parse error: wrong delimiters at [4]={packet[4]:02X}, [9]={packet[9]:02X}, [14]={packet[14]:02X}")
        return None

    # Extract 4 pixels - interleaved RGB order
    p0 = (packet[0],  packet[1],  packet[2])   # R0, G0, B0
    p1 = (packet[3],  packet[5],  packet[6])   # R1, G1, B1
    p2 = (packet[7],  packet[8],  packet[10])  # R2, G2, B2
    p3 = (packet[11], packet[12], packet[13])  # R3, G3, B3

    if debug:
        print(f"    Parsed: P0={p0}, P1={p1}, P2={p2}, P3={p3}")

    return [p0, p1, p2, p3]


def save_ppm(image_data, filename):
    """Save image data as PPM file (P3 text format)."""
    print(f"Saving to {filename}...")
    with open(filename, 'w') as f:
        f.write("P3\n")  # P3 for RGB text format
        f.write(f"{IMG_WIDTH} {IMG_HEIGHT}\n")
        f.write("255\n")
        for row in image_data:
            for pixel in row:
                f.write(f"{pixel[0]} {pixel[1]} {pixel[2]} ")
            f.write("\n")
    print("Save complete.")


def get_output_path():
    """Get output file path with timestamp."""
    script_dir = Path(__file__).parent
    output_dir = script_dir.parent / "img_output"

    if not output_dir.exists():
        output_dir.mkdir(parents=True)
        print(f"Created output folder: {output_dir}")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"received_image_{timestamp}.ppm"

    return output_dir / filename


def main():
    total_pixels = IMG_WIDTH * IMG_HEIGHT
    total_messages = total_pixels // PIXELS_PER_MSG

    # Debug mode
    DEBUG_MODE = True

    print(f"FPGA Burst Image Receiver")
    print(f"=" * 50)
    print(f"Image size: {IMG_WIDTH}x{IMG_HEIGHT} = {total_pixels} pixels")
    print(f"Expected messages: {total_messages}")
    print(f"Debug mode: {DEBUG_MODE}")
    print()

    # Open serial port
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=DATA_BITS,
            parity=PARITY,
            stopbits=STOP_BITS,
            timeout=2,           # 2 second read timeout
            write_timeout=1,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False
        )
        # Increase OS serial buffer to prevent overflow at 5.5Mbaud
        ser.set_buffer_size(rx_size=524288)  # 512KB receive buffer
        print(f"Connected to {PORT} at {BAUD_RATE} baud (rx buffer: 512KB)")
    except Exception as e:
        print(f"ERROR: Could not open serial port: {e}")
        input("\nPress Enter to exit...")
        return

    # Small delay for connection to stabilize
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Build and send burst read request
    read_msg = build_start_burst_read_msg(IMG_HEIGHT, IMG_WIDTH)
    print(f"\nSending START_BURST_RD request: {read_msg.hex()}")
    print(f"  Decoded: {{R addr=0, H={IMG_HEIGHT}, W={IMG_WIDTH}}}")
    ser.write(read_msg)
    ser.flush()

    print(f"\nWaiting for image data...")

    # =========================================================================
    # BULK READ: Read all raw bytes first, then parse offline.
    # Avoids byte-by-byte Python overhead that causes serial buffer overflow.
    # =========================================================================
    total_bytes_expected = total_messages * 16  # 16384 * 16 = 262144 bytes

    raw_buffer = bytearray()
    start_time = time.time()
    no_data_count = 0

    while len(raw_buffer) < total_bytes_expected:
        elapsed = time.time() - start_time
        if elapsed > TIMEOUT_SEC:
            print(f"\n  Timeout after {elapsed:.1f}s")
            break

        # Read large chunks for efficiency
        bytes_remaining = total_bytes_expected - len(raw_buffer)
        chunk_size = min(bytes_remaining, 65536)  # Read up to 64KB at a time
        chunk = ser.read(chunk_size)

        if chunk:
            raw_buffer.extend(chunk)
            no_data_count = 0

            # Progress update
            progress = len(raw_buffer) / total_bytes_expected * 100
            rate = len(raw_buffer) / elapsed if elapsed > 0 else 0
            print(f"  Receiving: {progress:.1f}% ({len(raw_buffer)}/{total_bytes_expected} bytes) - {rate/1000:.0f} KB/s", end='\r')
        else:
            no_data_count += 1
            if no_data_count >= 3:
                print(f"\n  No more data after {elapsed:.1f}s ({len(raw_buffer)} bytes received)")
                break

    receive_time = time.time() - start_time
    print(f"\n  Raw data received: {len(raw_buffer)} bytes in {receive_time:.2f}s")

    # Debug: show first 50 raw bytes
    if DEBUG_MODE and len(raw_buffer) >= 50:
        print(f"\n[DEBUG] First 50 raw bytes:")
        for i in range(0, min(50, len(raw_buffer)), 16):
            chunk = raw_buffer[i:i+16]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            print(f"  {i:04X}: {hex_str:<48} {ascii_str}")

    # =========================================================================
    # PARSE: Walk through raw buffer, find '{' delimiters, extract messages
    # =========================================================================
    print(f"\nParsing messages...")

    image = [[[0, 0, 0] for _ in range(IMG_WIDTH)] for _ in range(IMG_HEIGHT)]

    pixels_received = 0
    messages_received = 0
    sync_errors = 0
    pos = 0

    try:
        while pos < len(raw_buffer) - 15 and pixels_received < total_pixels:
            # Find next frame start '{'
            if raw_buffer[pos] != CHAR_OPEN:
                pos += 1
                continue

            # Extract 15 bytes after '{'
            packet = raw_buffer[pos+1 : pos+16]

            # Parse pixel data
            pixels = parse_burst_pixel_msg(packet, debug=(messages_received < 5))
            if pixels is None:
                sync_errors += 1
                pos += 1  # Skip this '{' and look for next
                continue

            messages_received += 1
            pos += 16  # Advance past this message

            # Store pixels in image buffer (row-major order)
            for pixel in pixels:
                row = pixels_received // IMG_WIDTH
                col = pixels_received % IMG_WIDTH

                if row < IMG_HEIGHT and col < IMG_WIDTH:
                    image[row][col] = list(pixel)
                    pixels_received += 1

            # Progress update every 4096 messages
            if messages_received % 4096 == 0:
                progress = pixels_received / total_pixels * 100
                print(f"  Parsed: {progress:.1f}% ({messages_received}/{total_messages} messages)")

            # Debug: Print first few packets
            if messages_received <= 3:
                print(f"  Msg {messages_received}: pixels {pixels}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    finally:
        ser.close()

        print(f"\n")
        print(f"=" * 50)
        print(f"Transfer complete!")
        print(f"  Raw bytes received: {len(raw_buffer)}/{total_bytes_expected}")
        print(f"  Pixels received: {pixels_received}/{total_pixels}")
        print(f"  Messages received: {messages_received}/{total_messages}")
        print(f"  Sync errors: {sync_errors}")
        print(f"  Receive time: {receive_time:.2f}s")
        if receive_time > 0 and len(raw_buffer) > 0:
            print(f"  Raw data rate: {len(raw_buffer) / receive_time / 1000:.1f} KB/s")
        if pixels_received == total_pixels:
            print(f"  STATUS: COMPLETE - all pixels received!")
        else:
            missing = total_pixels - pixels_received
            print(f"  STATUS: INCOMPLETE - {missing} pixels missing ({missing*100/total_pixels:.1f}%)")

        # Save image
        output_path = get_output_path()
        save_ppm(image, output_path)
        print(f"\nImage saved to: {output_path}")

        input("\nPress Enter to exit...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")
