#!/usr/bin/env python3
"""
Test TX Trigger - Send MSG_START_BURST_RD and check if FPGA responds.
"""
import serial
import time

# Configuration
PORT = 'COM5'
BAUD_RATE = 5500000

def build_burst_rd_msg():
    """Build MSG_START_BURST_RD: {R<addr><off_h><off_l>,H<h2><h1><h0>,W<w2><w1><w0>}"""
    msg = bytearray(16)
    msg[0] = 0x7B   # '{'
    msg[1] = 0x52   # 'R'
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
    print("TX Trigger Test - MSG_START_BURST_RD")
    print("=" * 50)
    
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
            rtscts=False,
            dsrdtr=False,
            xonxoff=False
        )
        print(f"Connected to {PORT} at {BAUD_RATE} baud")
    except Exception as e:
        print(f"ERROR: {e}")
        input("\nPress Enter to exit...")
        return
    
    time.sleep(0.1)
    
    # Build and send test message
    msg = build_burst_rd_msg()
    print(f"\nSending MSG_START_BURST_RD:")
    print(f"  Hex: {msg.hex()}")
    print(f"  Decoded: {{R addr=0, H=256, W=256}}")
    
    print("\n>>> Watch LEDs on FPGA:")
    print("    LED 1: Should pulse (seq_tx_trigger)")
    print("    LED 2: Should turn ON (seq_tx_active_latch)")
    print("    LED 3: Should turn ON (TX sequencer frame active)")
    
    # Send message slowly
    print("\nSending bytes one-by-one (10ms delay)...")
    for i, byte in enumerate(msg):
        ser.write(bytes([byte]))
        print(f"  Byte {i:2d}: 0x{byte:02X}")
        time.sleep(0.01)
    
    ser.flush()
    
    print("\n>>> Message sent! Waiting 2s for response...")
    
    # Wait for response
    time.sleep(2)
    
    # Check for any received data
    received_count = 0
    received_data = bytearray()
    
    while ser.in_waiting > 0:
        chunk = ser.read(min(ser.in_waiting, 1024))
        received_data.extend(chunk)
        received_count += len(chunk)
    
    if received_count > 0:
        print(f"\n>>> SUCCESS! Received {received_count} bytes!")
        print(f"First 64 bytes: {received_data[:64].hex()}")
        
        # Look for message start
        open_count = received_data.count(0x7B)
        print(f"Found {open_count} '{{' characters (message starts)")
    else:
        print("\n>>> No data received!")
        print("\nCheck these LEDs after sending:")
        print("  LED 0: Should be ON (image in SRAM)")
        print("  LED 1: Should have pulsed (trigger detected)")
        print("  LED 2: Should be ON (TX sequencer latched)")
        print("  LED 3: Should be ON (TX active)")
        print("  LED 4: Should pulse (packet ready)")
        print("  LED 5: Should pulse (composer TX request)")
        print("  LED 6: Should pulse (TX MAC active)")
        print("  LED 7: Should pulse (TX PHY busy)")
    
    ser.close()
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
