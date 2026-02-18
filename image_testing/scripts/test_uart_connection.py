"""
test_uart_connection.py - Simple UART connectivity test

Sends a simple RGF read message and checks for any response.
This tests if the basic UART RX->TX path is working.
"""

import serial
import time

# Configuration
PORT = 'COM5'
BAUD_RATE = 5500000

def main():
    print("UART Connection Test")
    print("=" * 50)
    
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
            write_timeout=1,
            rtscts=True
        )
        print(f"Connected to {PORT} at {BAUD_RATE} baud")
    except Exception as e:
        print(f"ERROR: Could not open serial port: {e}")
        input("\nPress Enter to exit...")
        return
    
    time.sleep(0.1)
    ser.reset_input_buffer()
    
    # Test 1: Send a simple RGF read message
    # Format: {R<addr>} - 6 bytes for short read
    # But our classifier expects 16 bytes for MSG_START_BURST_RD
    # Let's send a minimal message and see if we get any echo or response
    
    print("\n--- Test 1: Send raw bytes and check for echo ---")
    test_bytes = bytes([0x7B, 0x52, 0x00, 0x00, 0x00, 0x7D])  # {R...}
    print(f"Sending: {test_bytes.hex()}")
    
    for b in test_bytes:
        ser.write(bytes([b]))
        time.sleep(0.001)
    ser.flush()
    
    time.sleep(0.5)
    response = ser.read(100)
    if response:
        print(f"Received {len(response)} bytes: {response.hex()}")
        print(f"ASCII: {''.join(chr(b) if 32 <= b < 127 else '.' for b in response)}")
    else:
        print("No response received")
    
    # Test 2: Check CTS signal
    print(f"\n--- Test 2: Hardware flow control ---")
    print(f"CTS (Clear To Send): {ser.cts}")
    print(f"DSR (Data Set Ready): {ser.dsr}")
    
    # Test 3: Send START_BURST_RD and wait longer
    print("\n--- Test 3: Send MSG_START_BURST_RD ---")
    msg = bytes([
        0x7B,  # '{'
        0x52,  # 'R'
        0x00,  # addr
        0x00, 0x00,  # offset
        0x2C,  # ','
        0x48,  # 'H'
        0x00, 0x01, 0x00,  # height = 256
        0x2C,  # ','
        0x57,  # 'W'
        0x00, 0x01, 0x00,  # width = 256
        0x7D   # '}'
    ])
    print(f"Sending: {msg.hex()}")
    
    for b in msg:
        # Wait for CTS if available
        timeout_count = 0
        while not ser.cts and timeout_count < 1000:
            timeout_count += 1
            time.sleep(0.0001)
        ser.write(bytes([b]))
    ser.flush()
    
    print("Waiting 3 seconds for response...")
    time.sleep(3)
    
    response = ser.read(1000)
    if response:
        print(f"Received {len(response)} bytes!")
        # Print first 100 bytes in hex
        for i in range(0, min(len(response), 100), 16):
            chunk = response[i:i+16]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            print(f"  {i:04X}: {hex_str:<48} {ascii_str}")
    else:
        print("No response - FPGA is not transmitting")
        print("\nPossible causes:")
        print("  1. FPGA not programmed with updated bitstream")
        print("  2. No image stored in SRAM (run send_burst_image.py first)")
        print("  3. RX path not working (message not received)")
        print("  4. TX path not connected properly")
    
    ser.close()
    print("\nTest complete.")
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
