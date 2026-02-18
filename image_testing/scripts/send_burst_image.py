"""
send_burst_image.py - Send 256x256 image to FPGA via UART burst messages

Protocol:
1. Send MSG_START_BURST_WR: {I<X><X><X>,H<h2><h1><h0>,W<w2><w1><w0>}
2. Send MSG_BURST_PIXEL_WR (repeated): {R0,G0,B0,R1, G1,B1,R2,G2, B2,R3,G3,B3}
   - 4 pixels per message, interleaved RGB order
   - For 256x256 = 65536 pixels / 4 = 16384 messages

Usage:
    python send_burst_image.py
"""

import serial
import time
import sys
import os
from pathlib import Path

# =============================================================================
# Configuration
# =============================================================================
PORT = 'COM5'           # Serial port - adjust as needed
BAUD_RATE = 5500000     # 5.5M baud - matches FPGA setting
IMG_WIDTH = 256
IMG_HEIGHT = 256
PIXELS_PER_MSG = 4      # 4 pixels per burst message

# UART Frame Configuration
DATA_BITS = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_ONE

# Message framing constants
CHAR_OPEN  = 0x7B  # '{'
CHAR_CLOSE = 0x7D  # '}'
CHAR_COMMA = 0x2C  # ','
OP_I       = 0x49  # 'I' - Image Write
CHAR_H     = 0x48  # 'H' - Height
CHAR_W     = 0x57  # 'W' - Width


def build_start_burst_write_msg(height, width):
    """
    Build MSG_START_BURST_WR message.
    Format: {I<X><X><X>,H<h2><h1><h0>,W<w2><w1><w0>}
    """
    msg = bytearray(16)

    # Frame start
    msg[0] = CHAR_OPEN

    # Opcode + padding
    msg[1] = OP_I                              # 'I'
    msg[2] = 0x00                              # Padding
    msg[3] = 0x00                              # Padding
    msg[4] = 0x00                              # Padding

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


def build_burst_pixel_msg(pixels):
    """
    Build MSG_BURST_PIXEL_WR message for 4 pixels.
    Format: {R0,G0,B0,R1, G1,B1,R2,G2, B2,R3,G3,B3}
    Interleaved pixel order.

    pixels: list of 4 tuples [(R,G,B), (R,G,B), (R,G,B), (R,G,B)]
    """
    if len(pixels) != 4:
        raise ValueError(f"Expected 4 pixels, got {len(pixels)}")

    p0, p1, p2, p3 = pixels

    msg = bytearray(16)

    # Frame start
    msg[0] = CHAR_OPEN

    # Group 1: R0, G0, B0, R1
    msg[1] = p0[0]  # R0
    msg[2] = p0[1]  # G0
    msg[3] = p0[2]  # B0
    msg[4] = p1[0]  # R1

    # Comma separator
    msg[5] = CHAR_COMMA

    # Group 2: G1, B1, R2, G2
    msg[6] = p1[1]  # G1
    msg[7] = p1[2]  # B1
    msg[8] = p2[0]  # R2
    msg[9] = p2[1]  # G2

    # Comma separator
    msg[10] = CHAR_COMMA

    # Group 3: B2, R3, G3, B3
    msg[11] = p2[2]  # B2
    msg[12] = p3[0]  # R3
    msg[13] = p3[1]  # G3
    msg[14] = p3[2]  # B3

    # Frame end
    msg[15] = CHAR_CLOSE

    return bytes(msg)


def send_message(ser, msg):
    """Send a message as a single block."""
    ser.write(msg)
    ser.flush()


def find_input_image():
    """Find first image in img_input folder."""
    script_dir = Path(__file__).parent
    input_dir = script_dir.parent / "img_input"

    if not input_dir.exists():
        input_dir.mkdir(parents=True)
        print(f"Created input folder: {input_dir}")
        print(f"Please place an image file in: {input_dir}")
        return None

    # Supported formats
    extensions = ['*.png', '*.jpg', '*.jpeg', '*.bmp', '*.pgm', '*.ppm']
    for ext in extensions:
        files = list(input_dir.glob(ext))
        if files:
            return files[0]

    print(f"No image files found in: {input_dir}")
    print(f"Supported formats: PNG, JPG, BMP, PGM, PPM")
    return None


def load_image(path):
    """Load image and convert to 256x256 RGB pixel list."""
    from PIL import Image

    img = Image.open(path)
    print(f"Loaded: {path.name} ({img.size[0]}x{img.size[1]}, mode={img.mode})")

    # Resize to 256x256
    if img.size != (IMG_WIDTH, IMG_HEIGHT):
        img = img.resize((IMG_WIDTH, IMG_HEIGHT), Image.LANCZOS)
        print(f"Resized to {IMG_WIDTH}x{IMG_HEIGHT}")

    # Convert to RGB
    if img.mode != 'RGB':
        img = img.convert('RGB')
        print(f"Converted to RGB")

    # Extract pixels in row-major order
    pixels = list(img.getdata())
    return pixels


def main():
    total_pixels = IMG_WIDTH * IMG_HEIGHT
    total_messages = total_pixels // PIXELS_PER_MSG

    print(f"FPGA Burst Image Sender")
    print(f"=" * 50)

    # Find and load image
    img_path = find_input_image()
    if img_path is None:
        input("\nPress Enter to exit...")
        return

    pixels = load_image(img_path)

    print(f"\nImage: {IMG_WIDTH}x{IMG_HEIGHT} = {total_pixels} pixels")
    print(f"Messages to send: {total_messages}")

    # Debug: show first few pixel values
    print(f"\n[DEBUG] First 8 pixels from image:")
    for i in range(min(8, len(pixels))):
        r, g, b = pixels[i]
        print(f"  Pixel {i}: R={r:3d}(0x{r:02X}) G={g:3d}(0x{g:02X}) B={b:3d}(0x{b:02X})")

    # Open serial port
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=DATA_BITS,
            parity=PARITY,
            stopbits=STOP_BITS,
            timeout=1,
            write_timeout=1,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False
        )
        print(f"\nConnected to {PORT} at {BAUD_RATE} baud")
    except Exception as e:
        print(f"ERROR: Could not open serial port: {e}")
        input("\nPress Enter to exit...")
        return

    # Small delay for connection to stabilize
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Send start burst write message
    start_msg = build_start_burst_write_msg(IMG_HEIGHT, IMG_WIDTH)
    print(f"\nSending START_BURST_WR: {start_msg.hex()}")
    send_message(ser, start_msg)

    # Brief delay for FPGA to process start message
    time.sleep(0.01)

    # Send pixel data in groups of 4
    print(f"\nSending {total_messages} burst pixel messages...")
    start_time = time.time()

    for msg_idx in range(total_messages):
        # Get 4 pixels for this message
        base_idx = msg_idx * PIXELS_PER_MSG
        pixel_group = pixels[base_idx:base_idx + PIXELS_PER_MSG]

        # Build and send burst message
        burst_msg = build_burst_pixel_msg(pixel_group)
        send_message(ser, burst_msg)

        # Debug: print first 3 messages
        if msg_idx < 3:
            print(f"  [DEBUG] Msg {msg_idx}: {burst_msg.hex()}")
            print(f"          P0=({pixel_group[0][0]:3d},{pixel_group[0][1]:3d},{pixel_group[0][2]:3d}) "
                  f"P1=({pixel_group[1][0]:3d},{pixel_group[1][1]:3d},{pixel_group[1][2]:3d}) "
                  f"P2=({pixel_group[2][0]:3d},{pixel_group[2][1]:3d},{pixel_group[2][2]:3d}) "
                  f"P3=({pixel_group[3][0]:3d},{pixel_group[3][1]:3d},{pixel_group[3][2]:3d})")

        # Progress update every 1024 messages
        if (msg_idx + 1) % 1024 == 0:
            progress = (msg_idx + 1) / total_messages * 100
            elapsed = time.time() - start_time
            rate = (msg_idx + 1) * 4 / elapsed if elapsed > 0 else 0
            print(f"  Progress: {progress:.1f}% ({(msg_idx+1)*4}/{total_pixels} pixels) - {rate:.0f} px/s")

    elapsed = time.time() - start_time
    ser.close()

    print(f"\n{'=' * 50}")
    print(f"Transfer complete!")
    print(f"  Messages sent: {total_messages}")
    print(f"  Pixels sent: {total_pixels}")
    print(f"  Time: {elapsed:.2f}s")
    if elapsed > 0:
        print(f"  Rate: {total_pixels / elapsed:.0f} pixels/sec")

    input("\nPress Enter to exit...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")
