"""
test_tx_loopback.py - Test TX path by sending RGF read and checking response

This tests if the FPGA RGF read path works:
1. Send MSG_RGF_READ: {R<addr>} - 6 bytes
2. Check if FPGA responds with read data

This doesn't require image data in SRAM.
"""

import serial
import time

PORT = 'COM5'
BAUD_RATE = 5500000

def main():
    print("TX Path Test - RGF Read")
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
    
    # MSG_RGF_READ format: {R<addr><off_h><off_l>} - but for short format it's 6 bytes
    # Actually checking the MAC - short read is {R<addr>} = 6 bytes
    # Let me check what the MAC expects...
    
    # The MAC checks msg_ended_5 for short read. So:
    # byte0 = '{', byte1='R', byte2-4 = addr/offset, byte5 = '}'
    
    print("\n--- Test: Send MSG_RGF_READ ---")
    # Format: {R<addr><off_h><off_l>} where byte5 is '}'
    msg_rgf_read = bytes([
        0x7B,  # '{'
        0x52,  # 'R'
        0x00,  # addr
        0x00,  # offset_h
        0x00,  # offset_l
        0x7D   # '}'
    ])
    print(f"Sending: {msg_rgf_read.hex()}")
    print(f"Format: {{R<addr>}} - should trigger RGF read")
    
    for b in msg_rgf_read:
        timeout_count = 0
        while not ser.cts and timeout_count < 10000:
            timeout_count += 1
            time.sleep(0.00001)
        if timeout_count >= 10000:
            print(f"WARNING: CTS timeout on byte 0x{b:02X}")
        ser.write(bytes([b]))
    ser.flush()
    
    print("\nWaiting 2 seconds for response...")
    time.sleep(2)
    
    response = ser.read(100)
    if response:
        print(f"\nReceived {len(response)} bytes!")
        for i in range(0, len(response), 16):
            chunk = response[i:i+16]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            print(f"  {i:04X}: {hex_str:<48} {ascii_str}")
        
        # Try to parse as RGF read response
        if len(response) >= 16 and response[0] == 0x7B:
            print("\n  Looks like a valid response frame!")
    else:
        print("\nNo response received")
        print("\nThis could mean:")
        print("  1. RGF Manager is not responding")
        print("  2. TX MAC/PHY not working")
        print("  3. Message not recognized by classifier")
        print("  4. Baud rate mismatch")
    
    print(f"\nCTS status: {ser.cts}")
    
    ser.close()
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
