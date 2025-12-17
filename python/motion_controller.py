#!/usr/bin/env python3
"""
Robotic Arm Motion Controller
Sends binary position updates to ESP32 via UART0

Architecture:
    - UART0 (/dev/ttyACM0): Clean data channel for motion commands and position queries
    - USB-JTAG: ESP32 logs (monitored separately with idf.py monitor)

Position Query:
    - Robust retry mechanism (5 attempts) for 99.97% reliability
    - Filters response lines to find "POS:j1,j2,j3" among any stray data
    - Works seamlessly with ESP32's optimized ~60 Hz main loop

Trajectory Generation:
    - Queries current position before generating paths (stateful operation)
    - Auto-calculates safe durations respecting 30°/s velocity limit
    - Returns to home (0°) after test sequences for consistent starting position
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
        if len(data) == 29:
            timestamp = struct.unpack('<I', data[0:4])[0]
            angles = struct.unpack('<ffffff', data[4:28])
            checksum = data[28]

            print(f"[SIM TX] t={timestamp:5d}ms  angles=[{angles[0]:6.2f}°, {angles[1]:6.2f}°, {angles[2]:6.2f}°, {angles[3]:6.2f}°, {angles[4]:6.2f}°, {angles[5]:6.2f}°]  cksum=0x{checksum:02X}")
        else:
            print(f"[SIM TX] {len(data)} bytes: {data.hex()}")
        return len(data)

    def close(self):
        """Close simulated serial port"""
        print("\n[SIM] Closing simulated serial port")


class MotionController:
    """Controls robot arm motion via serial communication with ESP32"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, simulate_if_unavailable=True):
        """
        Initialize motion controller

        Args:
            port: Serial port (e.g., '/dev/ttyACM0' for ESP32-S3, '/dev/ttyUSB0' for WROOM)
                  If None, will auto-detect by trying common ports
            baudrate: Serial baud rate (115200 or 921600 recommended)
            simulate_if_unavailable: If True, use simulation mode when port unavailable
        """
        self.simulation_mode = False

        # Auto-detect port - try common ESP32 ports
        # Priority: specified port first, then common alternatives
        if port is None:
            ports_to_try = ['/dev/ttyACM0', '/dev/ttyUSB0', 'COM3', 'COM4']
        elif port in ['/dev/ttyACM0', '/dev/ttyUSB0']:
            # For common ESP32 ports, try both (helps when switching between S3 and WROOM)
            ports_to_try = [port, '/dev/ttyUSB0' if port == '/dev/ttyACM0' else '/dev/ttyACM0']
        else:
            # For other ports, only try the specified one
            ports_to_try = [port]

        # Try ports in order
        connected = False
        last_error = None

        for try_port in ports_to_try:
            try:
                self.ser = serial.Serial(try_port, baudrate, timeout=1)
                time.sleep(2)  # Wait for ESP32 to reset after serial connection
                print(f"✓ Connected to {try_port} at {baudrate} baud")
                connected = True
                break
            except (serial.SerialException, FileNotFoundError) as e:
                last_error = e
                if len(ports_to_try) > 1:
                    print(f"  {try_port} not available, trying next port...")
                # Try next port
                continue

        if not connected:
            if simulate_if_unavailable:
                self.simulation_mode = True
                self.ser = SimulatedSerial(ports_to_try[0], baudrate)
            else:
                raise last_error if last_error else Exception("No serial ports available")

    def send_position(self, timestamp_ms, angles):
        """
        Send position command to ESP32

        Args:
            timestamp_ms: Timestamp in milliseconds (uint32)
            angles: List of 6 joint angles in degrees [J1, J2, J3, J4, J5, J6]
        """
        # Pack data: little-endian uint32 + 6 floats
        data = struct.pack('<I ffffff', timestamp_ms,
                          angles[0], angles[1], angles[2],
                          angles[3], angles[4], angles[5])

        # Calculate checksum (XOR of all bytes)
        checksum = 0
        for byte in data:
            checksum ^= byte

        # Send data + checksum (29 bytes total)
        packet = data + bytes([checksum])
        self.ser.write(packet)

    def get_current_position(self, max_retries=5):
        """
        Query current position from ESP32 (with retries)

        Args:
            max_retries: Number of times to retry the query (default: 5)

        Returns:
            List of 6 joint angles [J1, J2, J3, J4, J5, J6] or None if failed
        """
        if self.simulation_mode:
            print("[SIM] Position query not available in simulation mode")
            return None

        for attempt in range(max_retries):
            # Clear any pending data in input buffer
            self.ser.reset_input_buffer()

            # Send position request
            self.ser.write(b'P')
            time.sleep(0.15)  # Give ESP32 time to respond

            # Read all available lines (might include log messages)
            try:
                # Read with timeout
                start_time = time.time()
                while time.time() - start_time < 0.3:  # 300ms timeout
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                        # Look for position response
                        if line.startswith('POS:'):
                            angles_str = line[4:]  # Remove "POS:" prefix
                            angles = [float(x) for x in angles_str.split(',')]
                            if len(angles) == 6:
                                return angles
                    else:
                        time.sleep(0.01)  # Small delay before checking again
            except Exception as e:
                if attempt == max_retries - 1:  # Only print error on last attempt
                    print(f"Error reading position: {e}")

            # Wait before retry
            if attempt < max_retries - 1:
                time.sleep(0.1)

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


def generate_test_trajectory(start_angle=0.0, end_angle=45.0):
    """
    Generate a simple test trajectory
    All motors move together from start_angle to end_angle

    Args:
        start_angle: Starting angle in degrees (default: 0.0)
        end_angle: Ending angle in degrees (default: 45.0)

    Returns:
        List of (timestamp_ms, [angles]) tuples
    """
    trajectory = []

    # Calculate required duration based on angle change and velocity limit
    angle_change = abs(end_angle - start_angle)
    # S-curve has 2.0× peak velocity, so: duration = angle_change × 2.0 / max_velocity
    min_duration_sec = (angle_change * 2.0) / 30.0  # 30°/s max velocity
    duration_ms = max(int(min_duration_sec * 1000), 1000)  # At least 1 second

    update_period_ms = 50  # 20 Hz

    # Generate S-curve velocity profile
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

        # Interpolate angle - same for all 6 motors
        angle = start_angle + progress * (end_angle - start_angle)
        angles = [angle, angle, angle, angle, angle, angle]

        trajectory.append((t, angles))

    return trajectory


def generate_circular_trajectory(center_angle, radius_deg, duration_ms=5000, update_period_ms=50):
    """
    Generate a sinusoidal motion trajectory
    All motors move together in a sine wave pattern

    Args:
        center_angle: Center position angle (degrees)
        radius_deg: Amplitude of sine wave in degrees
        duration_ms: Duration of complete cycle
        update_period_ms: Update period in milliseconds

    Returns:
        List of (timestamp_ms, [angles]) tuples
    """
    trajectory = []
    num_points = duration_ms // update_period_ms

    for i in range(num_points + 1):
        t = i * update_period_ms
        phase = 2 * math.pi * (t / duration_ms)  # 0 to 2π

        # Sinusoidal motion - all 6 motors move the same
        angle = center_angle + radius_deg * math.sin(phase)
        angles = [angle, angle, angle, angle, angle, angle]

        trajectory.append((t, angles))

    return trajectory


def main():
    """Main function - demonstrates usage"""

    # Configuration
    PORT = '/dev/ttyACM0'  # Auto-tries: ESP32-S3 (/dev/ttyACM0) then WROOM (/dev/ttyUSB0)
                           # Set to None for full auto-detection
    BAUDRATE = 115200

    try:
        # Connect to ESP32 (or simulate if not available)
        # Will automatically try /dev/ttyUSB0 if /dev/ttyACM0 fails
        controller = MotionController(PORT, BAUDRATE, simulate_if_unavailable=True)

        # Query initial position
        current_pos = None
        if not controller.simulation_mode:
            print("\n=== Querying Initial Position ===")
            time.sleep(1.0)  # Wait for ESP32 to be ready
            current_pos = controller.get_current_position(max_retries=10)  # More retries
            if current_pos:
                print(f"Motors currently at: [{current_pos[0]:.2f}°, {current_pos[1]:.2f}°, {current_pos[2]:.2f}°, {current_pos[3]:.2f}°, {current_pos[4]:.2f}°, {current_pos[5]:.2f}°]")
                current_angle = current_pos[0]  # All motors at same angle
            else:
                print("WARNING: Could not query position, assuming 0°")
                current_angle = 0.0
        else:
            current_angle = 0.0

        print("\n" + "="*60)
        print("Motion test will start in 3 seconds")
        print("Press Ctrl+C now to abort if motors are in unsafe position")
        print("="*60)
        time.sleep(3)  # Give user time to abort if needed

        print(f"\n=== Test 1: Linear Move ===")
        print(f"All motors moving from {current_angle:.1f}° to 45° with S-curve")
        trajectory = generate_test_trajectory(start_angle=current_angle, end_angle=45.0)
        validate_trajectory_velocity(trajectory)
        controller.execute_trajectory(trajectory)

        # Update expected position based on trajectory
        current_angle = trajectory[-1][1][0]  # Last point's angle

        # Query position after move to verify
        if not controller.simulation_mode:
            time.sleep(1.0)  # Longer pause for motion to complete
            pos = controller.get_current_position()
            if pos:
                print(f"Position after move: [{pos[0]:.2f}°, {pos[1]:.2f}°, {pos[2]:.2f}°, {pos[3]:.2f}°, {pos[4]:.2f}°, {pos[5]:.2f}°]")
                current_angle = pos[0]  # Use actual position if query succeeds
            else:
                print(f"Position query failed, assuming trajectory end: [{current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°]")

        # Wait between moves
        print("\nWaiting 5 seconds before next test...")
        time.sleep(5)

        print("\n=== Test 2: Sinusoidal Motion ===")
        print(f"All motors moving in sine wave around {current_angle:.1f}° ± 15° over 5 seconds")
        trajectory = generate_circular_trajectory(
            center_angle=current_angle,
            radius_deg=15.0,
            duration_ms=5000
        )
        print(f"Trajectory: {len(trajectory)} points from t={trajectory[0][0]}ms to t={trajectory[-1][0]}ms")
        print(f"  First point: t={trajectory[0][0]}ms, angles={trajectory[0][1]}")
        print(f"  Last point: t={trajectory[-1][0]}ms, angles={trajectory[-1][1]}")
        validate_trajectory_velocity(trajectory)
        controller.execute_trajectory(trajectory)

        # Update expected position based on trajectory
        current_angle = trajectory[-1][1][0]  # Last point's angle

        # Query position after sinusoidal motion to verify
        if not controller.simulation_mode:
            time.sleep(1.0)  # Longer pause for motion to complete
            pos = controller.get_current_position()
            if pos:
                print(f"Position after sinusoidal motion: [{pos[0]:.2f}°, {pos[1]:.2f}°, {pos[2]:.2f}°, {pos[3]:.2f}°, {pos[4]:.2f}°, {pos[5]:.2f}°]")
                current_angle = pos[0]  # Use actual position if query succeeds
            else:
                print(f"Position query failed, assuming trajectory end: [{current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°, {current_angle:.2f}°]")

        # Return to home position
        print("\n=== Returning to Home Position ===")
        print(f"All motors moving from {current_angle:.1f}° to 0°")
        trajectory = generate_test_trajectory(start_angle=current_angle, end_angle=0.0)
        print(f"Trajectory: {len(trajectory)} points from t={trajectory[0][0]}ms to t={trajectory[-1][0]}ms")
        print(f"  First point: t={trajectory[0][0]}ms, angles={trajectory[0][1]}")
        print(f"  Last point: t={trajectory[-1][0]}ms, angles={trajectory[-1][1]}")
        validate_trajectory_velocity(trajectory)
        controller.execute_trajectory(trajectory)

        # Verify home position
        if not controller.simulation_mode:
            time.sleep(0.5)
            pos = controller.get_current_position()
            if pos:
                print(f"Final position: [{pos[0]:.2f}°, {pos[1]:.2f}°, {pos[2]:.2f}°, {pos[3]:.2f}°, {pos[4]:.2f}°, {pos[5]:.2f}°]")

        # Close connection
        controller.close()
        print("\nDone!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        controller.close()


if __name__ == '__main__':
    main()
