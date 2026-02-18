#!/usr/bin/env python3
"""
Simple RX Test - Send a single MSG_START_BURST_WR message without flow control
to test if the FPGA receives and recognizes it.
"""
import serial
import time

# Configuration
PORT = 'COM5'
BAUD_RATE = 5500000

# Build MSG_START_BURST_WR: {I<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}
# For 256x256 image at address 0
def build_test_msg():
    msg = bytearray(16)
    msg[0] = 0x7B   # '{'
    msg[1] = 0x49   # 'I'
    msg[2] = 0x00   # addr = 0
    msg[3] = 0x00   # offset high = 0
    msg[4] = 0x00   # offset low = 0
    msg[5] = 0x2C   # ','
    msg[6] = 0x48   # 'H'
    msg[7] = 0x00   # height byte 2 (MSB) = 0
    msg[8] = 0x01   # height byte 1 = 1 (256 = 0x000100)
    msg[9] = 0x00   # height byte 0 (LSB) = 0
    msg[10] = 0x2C  # ','
    msg[11] = 0x57  # 'W'
    msg[12] = 0x00  # width byte 2 (MSB) = 0
    msg[13] = 0x01  # width byte 1 = 1 (256 = 0x000100)
    msg[14] = 0x00  # width byte 0 (LSB) = 0
    msg[15] = 0x7D  # '}'
    return bytes(msg)

def main():
    print("=" * 50)
    print("Simple RX Test - No Flow Control")
    print("=" * 50)
    
    # Open serial WITHOUT flow control
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            rtscts=False,   # NO hardware flow control
            dsrdtr=False,
            xonxoff=False
        )
        print(f"Connected to {PORT} at {BAUD_RATE} baud (no flow control)")
    except Exception as e:
        print(f"ERROR: {e}")
        input("\nPress Enter to exit...")
        return
    
    time.sleep(0.1)
    
    # Build and send test message
    msg = build_test_msg()
    print(f"\nSending MSG_START_BURST_WR:")
    print(f"  Hex: {msg.hex()}")
    print(f"  Decoded: {{I addr=0, H=256, W=256}}")
    
    print("\n>>> Watch LEDs on FPGA:")
    print("    LED 12: Should pulse (PHY received byte)")
    print("    LED 9:  Should pulse (MAC assembled message)")
    print("    LED 10: Should turn ON (burst mode activated)")
    
    # Send the message slowly - one byte at a time with small delay
    print("\nSending bytes one-by-one (10ms delay between)...")
    for i, byte in enumerate(msg):
        ser.write(bytes([byte]))
        print(f"  Byte {i:2d}: 0x{byte:02X}", end='')
        if i == 0:
            print("  '{'")
        elif i == 1:
            print("  'I' opcode")
        elif i == 5:
            print("  ','")
        elif i == 6:
            print("  'H' field")
        elif i == 10:
            print("  ','")
        elif i == 11:
            print("  'W' field")
        elif i == 15:
            print("  '}'")
        else:
            print()
        time.sleep(0.01)  # 10ms between bytes
    
    ser.flush()
    
    print("\n>>> Message sent!")
    print(">>> Check LEDs now:")
    print("    - LED 10 should be ON (burst_on = 1)")
    print("    - LED 12 should have pulsed 16 times")
    
    # Wait and check for any response
    time.sleep(0.5)
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"\nReceived {len(response)} bytes back: {response.hex()}")
    else:
        print("\nNo response received (expected for this message type)")
    
    ser.close()
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
