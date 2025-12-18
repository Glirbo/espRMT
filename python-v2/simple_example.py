#!/usr/bin/env python3
"""
Simple example: Send a single position command to ESP32

Motor Configuration:
- 4000 steps/revolution (no microstepping)
- 1:50 gear ratio
- Max angular velocity: 30°/sec
"""

import serial
import struct
import time


class SimulatedSerial:
    """Simulated serial for testing without hardware"""
    def __init__(self):
        print("SIMULATION MODE - No hardware connected\n")

    def write(self, data):
        if len(data) == 29:
            timestamp = struct.unpack('<I', data[0:4])[0]
            angles = struct.unpack('<ffffff', data[4:28])
            checksum = data[28]
            print(f"[SIM] t={timestamp:5d}ms  angles=[{angles[0]:6.2f}°, {angles[1]:6.2f}°, {angles[2]:6.2f}°, {angles[3]:6.2f}°, {angles[4]:6.2f}°, {angles[5]:6.2f}°]  cksum=0x{checksum:02X}")
        return len(data)

    def close(self):
        print("\n[SIM] Done")


def send_position(ser, timestamp_ms, angles):
    """Send binary position command to ESP32"""
    # Pack: uint32 + 6 floats (little-endian)
    data = struct.pack('<I ffffff', timestamp_ms,
                      angles[0], angles[1], angles[2],
                      angles[3], angles[4], angles[5])

    # Calculate XOR checksum
    checksum = 0
    for byte in data:
        checksum ^= byte

    # Send 29-byte packet
    ser.write(data + bytes([checksum]))


def main():
    # Configure serial port
    BAUDRATE = 115200

    # Auto-detect ESP32 port (try both S3 and WROOM)
    ports_to_try = ['/dev/ttyACM0', '/dev/ttyUSB0']

    ser = None
    for port in ports_to_try:
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=1)
            time.sleep(2)  # Wait for ESP32 reset
            print(f"✓ Connected to {port}\n")
            break
        except (serial.SerialException, FileNotFoundError):
            if len(ports_to_try) > 1:
                print(f"  {port} not available, trying next port...")
            continue

    # Fall back to simulation if no port available
    if ser is None:
        ser = SimulatedSerial()

    # Send a simple command - all 6 motors move to same angle
    send_position(ser, timestamp_ms=0, angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1)

    send_position(ser, timestamp_ms=1000, angles=[45.0, 45.0, 45.0, 45.0, 45.0, 45.0])
    time.sleep(1)

    send_position(ser, timestamp_ms=2000, angles=[90.0, 90.0, 90.0, 90.0, 90.0, 90.0])

    ser.close()
    print("Done!")


if __name__ == '__main__':
    main()
