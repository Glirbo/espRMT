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
        if len(data) == 17:
            timestamp = struct.unpack('<I', data[0:4])[0]
            angles = struct.unpack('<fff', data[4:16])
            checksum = data[16]
            print(f"[SIM] t={timestamp:5d}ms  angles=[{angles[0]:6.2f}°, {angles[1]:6.2f}°, {angles[2]:6.2f}°]  cksum=0x{checksum:02X}")
        return len(data)

    def close(self):
        print("\n[SIM] Done")


def send_position(ser, timestamp_ms, angles):
    """Send binary position command to ESP32"""
    # Pack: uint32 + 3 floats (little-endian)
    data = struct.pack('<I fff', timestamp_ms, angles[0], angles[1], angles[2])

    # Calculate XOR checksum
    checksum = 0
    for byte in data:
        checksum ^= byte

    # Send 17-byte packet
    ser.write(data + bytes([checksum]))


def main():
    # Configure serial port
    PORT = '/dev/ttyACM0'  # ESP32-S3 native USB (use /dev/ttyUSB0 for UART adapter)
    BAUDRATE = 115200

    # Try to connect to ESP32, fall back to simulation
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for ESP32 reset
        print(f"Connected to {PORT}\n")
    except (serial.SerialException, FileNotFoundError):
        ser = SimulatedSerial()

    # Send a simple command
    send_position(ser, timestamp_ms=0, angles=[0.0, 45.0, 30.0])
    time.sleep(1)

    send_position(ser, timestamp_ms=1000, angles=[0.0, 60.0, 20.0])
    time.sleep(1)

    send_position(ser, timestamp_ms=2000, angles=[0.0, 75.0, 10.0])

    ser.close()
    print("Done!")


if __name__ == '__main__':
    main()
