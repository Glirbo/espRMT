#!/usr/bin/env python3
"""
Simple script to send a fixed number of pulses to a specific motor.

Usage:
    python send_pulses.py

Then enter:
- Motor number (0-5)
- Number of pulses/steps to send
- Direction (forward/backward)
"""

import serial
import struct
import time
import sys

# Motor configuration (from ESP32-WROOM-V2 firmware)
# Motors 0 and 5: 1:10 gearing = 111.11 steps/degree
# Motors 1-4: 1:50 gearing = 555.56 steps/degree
STEPS_PER_DEGREE = [
    111.11,  # Motor 0 (1:10)
    555.56,  # Motor 1 (1:50)
    555.56,  # Motor 2 (1:50)
    555.56,  # Motor 3 (1:50)
    555.56,  # Motor 4 (1:50)
    111.11   # Motor 5 (1:10)
]

# Serial port configuration - will try these ports in order
PORTS = [
    '/dev/ttyACM0',  # ESP32-S3 native USB
    '/dev/ttyUSB0',  # ESP32 with USB-UART adapter
    'COM3',          # Windows (change number as needed)
]
BAUDRATE = 115200

def steps_to_degrees(steps, motor_index):
    """Convert steps to degrees for a specific motor"""
    return steps / STEPS_PER_DEGREE[motor_index]

def degrees_to_steps(degrees, motor_index):
    """Convert degrees to steps for a specific motor"""
    return degrees * STEPS_PER_DEGREE[motor_index]

def send_position(ser, angles):
    """Send position command to ESP32"""
    timestamp_ms = int(time.time() * 1000) % (2**32)  # Wrap at 32-bit

    # Pack data: timestamp + 6 floats (little-endian)
    data = struct.pack('<I 6f', timestamp_ms, *angles)

    # Calculate checksum (XOR of all bytes)
    checksum = 0
    for byte in data:
        checksum ^= byte

    # Send packet
    packet = data + bytes([checksum])
    ser.write(packet)

    return timestamp_ms

def query_position(ser):
    """Query current position from ESP32"""
    # Send position query command
    ser.write(b'POS')
    time.sleep(0.1)

    # Read response
    line = ser.readline().decode('ascii', errors='ignore').strip()

    if line.startswith('POS:'):
        # Parse: "POS:j1,j2,j3,j4,j5,j6"
        angles_str = line[4:].split(',')
        angles = [float(a) for a in angles_str]
        return angles

    return None

def main():
    print("="*60)
    print("Send Fixed Pulses to Motor")
    print("="*60)

    # Connect to ESP32 - try each port in order
    ser = None
    connected_port = None

    for port in PORTS:
        try:
            print(f"Trying {port}...")
            ser = serial.Serial(port, BAUDRATE, timeout=1)
            time.sleep(2)  # Wait for connection
            connected_port = port
            print(f"✓ Connected to {port} @ {BAUDRATE} baud")
            break
        except Exception as e:
            print(f"✗ {port}: {e}")
            continue

    if ser is None:
        print("\nFailed to connect to any port.")
        print("Ports tried:", ", ".join(PORTS))
        print("\nTroubleshooting:")
        print("- Check USB cable is connected")
        print("- Verify ESP32 is powered on")
        print("- On Linux, check: ls /dev/tty* | grep -E 'ACM|USB'")
        print("- On Windows, check Device Manager for COM port")
        print("\nSimulation mode not available for this script.")
        return

    # Query current position
    print("\nQuerying current motor positions...")
    current_angles = query_position(ser)

    if current_angles:
        print("Current positions:")
        for i, angle in enumerate(current_angles):
            steps = degrees_to_steps(angle, i)
            print(f"  Motor {i}: {angle:7.2f}° ({steps:8.0f} steps)")
    else:
        print("Warning: Could not query position, assuming all at 0°")
        current_angles = [0.0] * 6

    # Get user input
    print("\n" + "="*60)
    print("Enter motor and pulse information:")
    print("="*60)

    try:
        motor_num = int(input("Motor number (0-5): "))
        if motor_num < 0 or motor_num > 5:
            print("Error: Motor number must be 0-5")
            return

        steps = int(input("Number of pulses/steps: "))
        if steps < 0:
            print("Error: Number of steps must be positive")
            return

        direction = input("Direction (f=forward, b=backward) [f]: ").strip().lower()
        if direction == 'b' or direction == 'backward':
            steps = -steps

    except ValueError as e:
        print(f"Error: Invalid input - {e}")
        return
    except KeyboardInterrupt:
        print("\n\nCancelled by user")
        return

    # Calculate target angle
    angle_change = steps_to_degrees(steps, motor_num)
    target_angle = current_angles[motor_num] + angle_change

    # Calculate movement time (at 60°/sec max velocity)
    max_velocity = 60.0  # degrees/sec
    duration_sec = abs(angle_change) / max_velocity
    duration_sec = max(duration_sec, 1.0)  # Minimum 1 second

    # Create target angles array (only change the selected motor)
    target_angles = current_angles.copy()
    target_angles[motor_num] = target_angle

    # Display movement info
    print("\n" + "="*60)
    print("Movement Summary:")
    print("="*60)
    print(f"Motor:          {motor_num}")
    print(f"Pulses/Steps:   {abs(steps)} ({'forward' if steps >= 0 else 'backward'})")
    print(f"Gear ratio:     1:{int(STEPS_PER_DEGREE[motor_num] * 360 / 4000)}")
    print(f"Angle change:   {angle_change:+.2f}°")
    print(f"Current angle:  {current_angles[motor_num]:.2f}°")
    print(f"Target angle:   {target_angle:.2f}°")
    print(f"Duration:       {duration_sec:.1f} seconds")
    print("="*60)

    # Safety confirmation
    response = input("\nExecute movement? (yes/no): ").strip().lower()
    if response != 'yes':
        print("Movement cancelled")
        ser.close()
        return

    # 3-second countdown
    print("\nStarting in 3 seconds (Ctrl+C to abort)...")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)

    print(f"\nExecuting movement over 6.0 seconds...")
    
    # Configuration for smooth movement
    TOTAL_DURATION = 6.0  # Fixed 6 seconds as requested
    INTERVAL = 0.05       # 50ms interval
    num_steps = int(TOTAL_DURATION / INTERVAL)
    
    start_time = time.time()
    
    for i in range(1, num_steps + 1):
        # Calculate how far through the movement we are (0.0 to 1.0)
        progress = i / num_steps
        
        # Calculate current intermediate angles
        current_step_angles = []
        for motor_idx in range(6):
            start_ang = current_angles[motor_idx]
            end_ang = target_angles[motor_idx]
            # Linear interpolation: start + (total_change * progress)
            interp_angle = start_ang + (end_ang - start_ang) * progress
            current_step_angles.append(interp_angle)
        
        # Send the intermediate position
        send_position(ser, current_step_angles)
        
        # Maintain the 50ms timing
        elapsed = time.time() - start_time
        expected_time = i * INTERVAL
        sleep_time = expected_time - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print(f"✓ Sequence complete.")

    # Wait for completion
    print(f"Waiting {duration_sec:.1f} seconds for movement to complete...")
    time.sleep(duration_sec + 0.5)

    # Query final position
    print("\nQuerying final position...")
    final_angles = query_position(ser)

    if final_angles:
        print(f"Motor {motor_num}: {final_angles[motor_num]:.2f}° (target was {target_angle:.2f}°)")
        error = final_angles[motor_num] - target_angle
        print(f"Position error: {error:+.2f}°")
    else:
        print("Could not verify final position")

    # Close connection
    ser.close()
    print("\n✓ Done!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
