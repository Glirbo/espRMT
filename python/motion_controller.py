#!/usr/bin/env python3
"""
Robotic Arm Motion Controller
Sends binary position updates to ESP32 via UART
"""

import serial
import struct
import time
import math


class SimulatedSerial:
    """Simulated serial port for testing without hardware"""

    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        print(f"\n{'='*60}")
        print(f"SIMULATION MODE - No hardware connected")
        print(f"Simulating serial port: {port} @ {baudrate} baud")
        print(f"{'='*60}\n")

    def write(self, data):
        """Simulate writing data - print hex representation"""
        # Decode the binary packet
        if len(data) == 17:
            timestamp = struct.unpack('<I', data[0:4])[0]
            angles = struct.unpack('<fff', data[4:16])
            checksum = data[16]

            print(f"[SIM TX] t={timestamp:5d}ms  angles=[{angles[0]:6.2f}°, {angles[1]:6.2f}°, {angles[2]:6.2f}°]  cksum=0x{checksum:02X}")
        else:
            print(f"[SIM TX] {len(data)} bytes: {data.hex()}")
        return len(data)

    def close(self):
        """Close simulated serial port"""
        print("\n[SIM] Closing simulated serial port")


class MotionController:
    """Controls robot arm motion via serial communication with ESP32"""

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, simulate_if_unavailable=True):
        """
        Initialize motion controller

        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows)
            baudrate: Serial baud rate (115200 or 921600 recommended)
            simulate_if_unavailable: If True, use simulation mode when port unavailable
        """
        self.simulation_mode = False

        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for ESP32 to reset after serial connection
            print(f"Connected to {port} at {baudrate} baud")
        except (serial.SerialException, FileNotFoundError) as e:
            if simulate_if_unavailable:
                self.simulation_mode = True
                self.ser = SimulatedSerial(port, baudrate)
            else:
                raise

    def send_position(self, timestamp_ms, angles):
        """
        Send position command to ESP32

        Args:
            timestamp_ms: Timestamp in milliseconds (uint32)
            angles: List of 3 joint angles in degrees [J1, J2, J3]
        """
        # Pack data: little-endian uint32 + 3 floats
        data = struct.pack('<I fff', timestamp_ms, angles[0], angles[1], angles[2])

        # Calculate checksum (XOR of all bytes)
        checksum = 0
        for byte in data:
            checksum ^= byte

        # Send data + checksum (17 bytes total)
        packet = data + bytes([checksum])
        self.ser.write(packet)

    def get_current_position(self):
        """
        Query current position from ESP32

        Returns:
            List of 3 joint angles [J1, J2, J3] or None if failed
        """
        if self.simulation_mode:
            print("[SIM] Position query not available in simulation mode")
            return None

        # Send position request
        self.ser.write(b'P')

        # Read response (format: "POS:j1,j2,j3\n")
        try:
            response = self.ser.readline().decode('utf-8').strip()
            if response.startswith('POS:'):
                angles_str = response[4:]  # Remove "POS:" prefix
                angles = [float(x) for x in angles_str.split(',')]
                if len(angles) == 3:
                    return angles
        except Exception as e:
            print(f"Error reading position: {e}")

        return None

    def execute_trajectory(self, trajectory, update_rate_hz=20):
        """
        Execute a trajectory (list of timestamped positions)

        Args:
            trajectory: List of (timestamp_ms, [angles]) tuples
            update_rate_hz: Rate to send updates (default 20 Hz = 50ms)
        """
        update_period = 1.0 / update_rate_hz
        start_time = time.time()

        for timestamp_ms, angles in trajectory:
            # Send position
            self.send_position(timestamp_ms, angles)

            # Wait until next update time
            target_time = start_time + (timestamp_ms / 1000.0)
            sleep_time = target_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

        print(f"Trajectory complete ({len(trajectory)} points)")

    def close(self):
        """Close serial connection"""
        if self.simulation_mode:
            print(f"\n{'='*60}")
            print(f"SIMULATION COMPLETE")
            print(f"{'='*60}\n")
        self.ser.close()


def validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0):
    """
    Validate that trajectory respects velocity limits

    Args:
        trajectory: List of (timestamp_ms, [angles]) tuples
        max_velocity_deg_per_sec: Maximum allowed angular velocity

    Returns:
        True if valid, prints warnings for violations
    """
    valid = True

    for i in range(1, len(trajectory)):
        t_prev, angles_prev = trajectory[i-1]
        t_curr, angles_curr = trajectory[i]

        dt_sec = (t_curr - t_prev) / 1000.0
        if dt_sec <= 0:
            continue

        for j in range(len(angles_prev)):
            angle_delta = abs(angles_curr[j] - angles_prev[j])
            velocity = angle_delta / dt_sec

            if velocity > max_velocity_deg_per_sec:
                print(f"⚠ WARNING: Joint {j} velocity {velocity:.1f}°/s exceeds limit {max_velocity_deg_per_sec}°/s")
                print(f"  at t={t_curr}ms: {angles_prev[j]:.2f}° → {angles_curr[j]:.2f}° in {dt_sec*1000:.0f}ms")
                valid = False

    return valid


def generate_test_trajectory():
    """
    Generate a simple test trajectory
    Moves from [0, 0, 0] to [0, 45, 30] over 3 seconds

    Returns:
        List of (timestamp_ms, [angles]) tuples
    """
    trajectory = []

    # Start and end positions
    start_angles = [0.0, 0.0, 0.0]
    end_angles = [0.0, 45.0, 30.0]

    # Duration and update rate
    duration_ms = 3000  # 3 seconds (needed for S-curve to stay within 30°/s limit)
    update_period_ms = 50  # 20 Hz

    # Generate trapezoidal velocity profile
    num_points = duration_ms // update_period_ms + 1

    for i in range(num_points):
        t = i * update_period_ms
        # Normalized time (0 to 1)
        t_norm = t / duration_ms

        # Simple S-curve interpolation (ease-in-out)
        # This gives smoother motion than linear interpolation
        if t_norm < 0.5:
            # Acceleration phase
            progress = 2 * t_norm * t_norm
        else:
            # Deceleration phase
            progress = 1 - 2 * (1 - t_norm) * (1 - t_norm)

        # Interpolate angles
        angles = [
            start_angles[j] + progress * (end_angles[j] - start_angles[j])
            for j in range(3)
        ]

        trajectory.append((t, angles))

    return trajectory


def generate_circular_trajectory(center_angles, radius_deg, duration_ms=5000, update_period_ms=50):
    """
    Generate a circular motion trajectory in joint space
    Joint 2 and 3 move in a circle, Joint 1 stays constant

    Args:
        center_angles: Center position [J1, J2, J3]
        radius_deg: Radius of circle in degrees
        duration_ms: Duration of complete circle
        update_period_ms: Update period in milliseconds

    Returns:
        List of (timestamp_ms, [angles]) tuples
    """
    trajectory = []
    num_points = duration_ms // update_period_ms

    for i in range(num_points + 1):
        t = i * update_period_ms
        angle = 2 * math.pi * (t / duration_ms)  # 0 to 2π

        # Circular motion in Joint 2-3 plane
        j1 = center_angles[0]
        j2 = center_angles[1] + radius_deg * math.cos(angle)
        j3 = center_angles[2] + radius_deg * math.sin(angle)

        trajectory.append((t, [j1, j2, j3]))

    return trajectory


def main():
    """Main function - demonstrates usage"""

    # Configuration
    PORT = '/dev/ttyACM0'  # ESP32-S3 native USB (use /dev/ttyUSB0 for UART adapter)
    BAUDRATE = 115200

    try:
        # Connect to ESP32 (or simulate if not available)
        controller = MotionController(PORT, BAUDRATE, simulate_if_unavailable=True)

        # Query current position from ESP32
        if not controller.simulation_mode:
            print("\n=== Querying Current Position ===")
            current_pos = controller.get_current_position()
            if current_pos:
                print(f"ESP32 reports current position: [{current_pos[0]:.2f}°, {current_pos[1]:.2f}°, {current_pos[2]:.2f}°]")
            else:
                print("Failed to query position from ESP32")
            time.sleep(1)

        print("\n" + "="*60)
        print("IMPORTANT: Position motors at [0°, 0°, 0°] before starting!")
        print("Or press Ctrl+C now if motors are not in a safe position")
        print("="*60)
        time.sleep(3)  # Give user time to abort if needed

        print("\n=== Test 1: Simple Linear Move ===")
        print("Moving from [0°, 0°, 0°] to [0°, 45°, 30°] over 3 seconds")
        trajectory = generate_test_trajectory()
        validate_trajectory_velocity(trajectory)
        controller.execute_trajectory(trajectory)

        # Query position after move
        if not controller.simulation_mode:
            time.sleep(0.5)  # Brief pause for motion to complete
            pos = controller.get_current_position()
            if pos:
                print(f"Position after move: [{pos[0]:.2f}°, {pos[1]:.2f}°, {pos[2]:.2f}°]")

        # Wait between moves (5 seconds)
        print("\nWaiting 5 seconds before next test...")
        time.sleep(5)

        print("\n=== Test 2: Circular Motion ===")
        print("Circular motion starting at [0°, 45°, 30°] with 15° radius")
        print("Center: [0°, 30°, 30°], Duration: 5 seconds")
        trajectory = generate_circular_trajectory(
            center_angles=[0.0, 30.0, 30.0],
            radius_deg=15.0,
            duration_ms=5000
        )
        validate_trajectory_velocity(trajectory)
        controller.execute_trajectory(trajectory)

        # Query position after circular motion
        if not controller.simulation_mode:
            time.sleep(0.5)  # Brief pause for motion to complete
            pos = controller.get_current_position()
            if pos:
                print(f"Position after circular motion: [{pos[0]:.2f}°, {pos[1]:.2f}°, {pos[2]:.2f}°]")

        # Close connection
        controller.close()
        print("\nDone!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        controller.close()


if __name__ == '__main__':
    main()
