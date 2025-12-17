#!/usr/bin/env python3
"""
Motor Test Sequences
Four specific test patterns for all 6 motors

Test 1: 0° → 100° → -100° → 0° (auto-calculated safe duration)
Test 2: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0° (auto-calculated safe duration)
Test 3: Same as Test 2 but at 0.5× speed (slower/smoother)
Test 4: Same as Test 2 but at 0.1× speed (very slow/smooth)

SPEED CONTROL:
  Speed multiplier scales velocity limits:
  - 1.0 = Full speed (30°/sec max, default)
  - 0.5 = Half speed (15°/sec max, safer/smoother)
  - 2.0 = Double speed (60°/sec max, aggressive, may exceed motor limits)

  Higher multiplier = faster motion = shorter duration
  Lower multiplier = slower motion = longer duration
"""

import time
from motion_controller import MotionController, validate_trajectory_velocity

# ============================================================================
# SPEED CONFIGURATION - Adjust these values to change motion speed
# ============================================================================
SPEED_MULTIPLIER_TEST1 = 1.0   # Test 1 speed (1.0 = 30°/sec max)
SPEED_MULTIPLIER_TEST2 = 1.0   # Test 2 speed (1.0 = 30°/sec max)
SPEED_MULTIPLIER_TEST3 = 0.5   # Test 3 speed (0.5 = 15°/sec max, slower/smoother)
SPEED_MULTIPLIER_TEST4 = 0.1   # Test 4 speed (0.1 = 3°/sec max, very slow/smooth)

MAX_VELOCITY_BASE = 30.0  # Base velocity limit in degrees/second
# ============================================================================


def s_curve_interpolate(t_norm):
    """
    S-curve interpolation (ease-in-out)

    Args:
        t_norm: Normalized time [0, 1]

    Returns:
        Progress value [0, 1] with smooth acceleration/deceleration
    """
    if t_norm < 0.5:
        return 2 * t_norm * t_norm
    else:
        return 1 - 2 * (1 - t_norm) * (1 - t_norm)


def generate_trajectory_segment(start_angle, end_angle, start_time_ms, duration_ms,
                                update_period_ms=50, num_motors=6):
    """
    Generate trajectory segment from start_angle to end_angle with S-curve interpolation

    Args:
        start_angle: Starting angle in degrees
        end_angle: Ending angle in degrees
        start_time_ms: Start timestamp in milliseconds
        duration_ms: Segment duration in milliseconds
        update_period_ms: Time between updates (50ms = 20Hz)
        num_motors: Number of motors (all move together)

    Returns:
        List of (timestamp_ms, [angles]) tuples
    """
    trajectory = []
    num_points = max(2, int(duration_ms / update_period_ms) + 1)

    for i in range(num_points):
        t_ms = start_time_ms + int(i * duration_ms / (num_points - 1))
        t_norm = i / (num_points - 1)

        # S-curve interpolation
        progress = s_curve_interpolate(t_norm)
        angle = start_angle + progress * (end_angle - start_angle)

        # All motors move together
        angles = [angle] * num_motors
        trajectory.append((t_ms, angles))

    return trajectory


def calculate_segment_duration(angle_change, max_velocity):
    """
    Calculate minimum safe duration for a segment with S-curve interpolation

    Args:
        angle_change: Angular change in degrees
        max_velocity: Maximum velocity in degrees/second

    Returns:
        Duration in milliseconds (rounded to 50ms intervals)
    """
    # For S-curve: peak_velocity = angle_change × 2.0 / duration
    # So: duration = angle_change × 2.0 / max_velocity
    min_duration_sec = (angle_change * 2.0) / max_velocity
    duration_ms = int(min_duration_sec * 1000)

    # Round up to nearest 50ms (update period)
    duration_ms = ((duration_ms + 49) // 50) * 50

    return duration_ms


def generate_test1_trajectory(speed_multiplier=1.0):
    """
    Test 1: 0° → 100° → -100° → 0°
    Auto-calculated duration respecting velocity limit

    Args:
        speed_multiplier: Speed multiplier (1.0 = 30°/sec, 0.5 = 15°/sec, 2.0 = 60°/sec)
    """
    max_velocity = MAX_VELOCITY_BASE * speed_multiplier

    print(f"\n{'='*60}")
    print(f"Test 1: 0° → 100° → -100° → 0° (auto-timed)")
    print(f"  Speed multiplier: {speed_multiplier:.2f}× (max velocity: {max_velocity:.1f}°/sec)")
    print(f"{'='*60}")

    waypoints = [0, 100, -100, 0]
    trajectory = []
    current_time_ms = 0

    for i in range(len(waypoints) - 1):
        start_angle = waypoints[i]
        end_angle = waypoints[i + 1]
        angle_change = abs(end_angle - start_angle)

        # Calculate minimum safe duration
        duration_ms = calculate_segment_duration(angle_change, max_velocity)

        # Calculate peak velocity for verification
        peak_velocity = (angle_change * 2.0) / (duration_ms / 1000.0)

        print(f"  Segment {i+1}: {start_angle:6.1f}° → {end_angle:6.1f}° "
              f"(Δ={angle_change:6.1f}°, {duration_ms:5d}ms, peak: {peak_velocity:.1f}°/s)")

        segment = generate_trajectory_segment(start_angle, end_angle,
                                             current_time_ms, duration_ms)

        # Add all points except the last (to avoid duplicates)
        trajectory.extend(segment[:-1])
        current_time_ms = segment[-1][0]

    # Add final point
    trajectory.append((current_time_ms, [waypoints[-1]] * 6))

    total_duration = trajectory[-1][0] / 1000.0
    print(f"  Total duration: {total_duration:.2f} seconds")
    print(f"  Total points: {len(trajectory)}")

    return trajectory


def generate_test2_trajectory(speed_multiplier=1.0):
    """
    Test 2: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0°
    Auto-calculated duration respecting velocity limit

    Args:
        speed_multiplier: Speed multiplier (1.0 = 30°/sec, 0.5 = 15°/sec, 2.0 = 60°/sec)
    """
    max_velocity = MAX_VELOCITY_BASE * speed_multiplier

    print(f"\n{'='*60}")
    print(f"Test 2: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0° (auto-timed)")
    print(f"  Speed multiplier: {speed_multiplier:.2f}× (max velocity: {max_velocity:.1f}°/sec)")
    print(f"{'='*60}")

    waypoints = [0, 15, -15, 10, -10, 5, -5, 0]
    trajectory = []
    current_time_ms = 0

    for i in range(len(waypoints) - 1):
        start_angle = waypoints[i]
        end_angle = waypoints[i + 1]
        angle_change = abs(end_angle - start_angle)

        # Calculate minimum safe duration
        duration_ms = calculate_segment_duration(angle_change, max_velocity)

        # Calculate peak velocity for verification
        peak_velocity = (angle_change * 2.0) / (duration_ms / 1000.0)

        print(f"  Segment {i+1}: {start_angle:6.1f}° → {end_angle:6.1f}° "
              f"(Δ={angle_change:6.1f}°, {duration_ms:5d}ms, peak: {peak_velocity:.1f}°/s)")

        segment = generate_trajectory_segment(start_angle, end_angle,
                                             current_time_ms, duration_ms)

        # Add all points except the last (to avoid duplicates)
        trajectory.extend(segment[:-1])
        current_time_ms = segment[-1][0]

    # Add final point
    trajectory.append((current_time_ms, [waypoints[-1]] * 6))

    total_duration = trajectory[-1][0] / 1000.0
    print(f"  Total duration: {total_duration:.2f} seconds")
    print(f"  Total points: {len(trajectory)}")

    return trajectory


def generate_test3_trajectory(speed_multiplier=1.0):
    """
    Test 3: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0°
    Auto-calculated duration respecting velocity limit (same path as Test 2, different speed)

    Args:
        speed_multiplier: Speed multiplier (1.0 = 30°/sec, 0.5 = 15°/sec, 2.0 = 60°/sec)
    """
    max_velocity = MAX_VELOCITY_BASE * speed_multiplier

    print(f"\n{'='*60}")
    print(f"Test 3: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0° (auto-timed)")
    print(f"  Speed multiplier: {speed_multiplier:.2f}× (max velocity: {max_velocity:.1f}°/sec)")
    print(f"{'='*60}")

    waypoints = [0, 15, -15, 10, -10, 5, -5, 0]
    trajectory = []
    current_time_ms = 0

    for i in range(len(waypoints) - 1):
        start_angle = waypoints[i]
        end_angle = waypoints[i + 1]
        angle_change = abs(end_angle - start_angle)

        # Calculate minimum safe duration
        duration_ms = calculate_segment_duration(angle_change, max_velocity)

        # Calculate peak velocity for verification
        peak_velocity = (angle_change * 2.0) / (duration_ms / 1000.0)

        print(f"  Segment {i+1}: {start_angle:6.1f}° → {end_angle:6.1f}° "
              f"(Δ={angle_change:6.1f}°, {duration_ms:5d}ms, peak: {peak_velocity:.1f}°/s)")

        segment = generate_trajectory_segment(start_angle, end_angle,
                                             current_time_ms, duration_ms)

        # Add all points except the last (to avoid duplicates)
        trajectory.extend(segment[:-1])
        current_time_ms = segment[-1][0]

    # Add final point
    trajectory.append((current_time_ms, [waypoints[-1]] * 6))

    total_duration = trajectory[-1][0] / 1000.0
    print(f"  Total duration: {total_duration:.2f} seconds")
    print(f"  Total points: {len(trajectory)}")

    return trajectory


def generate_test4_trajectory(speed_multiplier=1.0):
    """
    Test 4: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0°
    Auto-calculated duration respecting velocity limit (same path as Test 2/3, very slow)

    Args:
        speed_multiplier: Speed multiplier (1.0 = 30°/sec, 0.5 = 15°/sec, 0.1 = 3°/sec)
    """
    max_velocity = MAX_VELOCITY_BASE * speed_multiplier

    print(f"\n{'='*60}")
    print(f"Test 4: 0° → 15° → -15° → 10° → -10° → 5° → -5° → 0° (auto-timed)")
    print(f"  Speed multiplier: {speed_multiplier:.2f}× (max velocity: {max_velocity:.1f}°/sec)")
    print(f"{'='*60}")

    waypoints = [0, 15, -15, 10, -10, 5, -5, 0]
    trajectory = []
    current_time_ms = 0

    for i in range(len(waypoints) - 1):
        start_angle = waypoints[i]
        end_angle = waypoints[i + 1]
        angle_change = abs(end_angle - start_angle)

        # Calculate minimum safe duration
        duration_ms = calculate_segment_duration(angle_change, max_velocity)

        # Calculate peak velocity for verification
        peak_velocity = (angle_change * 2.0) / (duration_ms / 1000.0)

        print(f"  Segment {i+1}: {start_angle:6.1f}° → {end_angle:6.1f}° "
              f"(Δ={angle_change:6.1f}°, {duration_ms:5d}ms, peak: {peak_velocity:.1f}°/s)")

        segment = generate_trajectory_segment(start_angle, end_angle,
                                             current_time_ms, duration_ms)

        # Add all points except the last (to avoid duplicates)
        trajectory.extend(segment[:-1])
        current_time_ms = segment[-1][0]

    # Add final point
    trajectory.append((current_time_ms, [waypoints[-1]] * 6))

    total_duration = trajectory[-1][0] / 1000.0
    print(f"  Total duration: {total_duration:.2f} seconds")
    print(f"  Total points: {len(trajectory)}")

    return trajectory


def main():
    """Main test sequence execution"""

    print(f"\n{'='*60}")
    print("Motor Test Sequences")
    print("Tests all 6 motors with synchronized motion patterns")
    print(f"{'='*60}")

    # Connect to ESP32
    print("\nConnecting to ESP32...")
    controller = MotionController('/dev/ttyACM0', 115200, simulate_if_unavailable=True)

    if not controller.simulation_mode:
        # Query current position
        print("\nQuerying current motor position...")
        time.sleep(1.0)
        current_angles = controller.get_current_position(max_retries=10)

        if current_angles:
            print(f"Current angles: {[f'{a:.2f}°' for a in current_angles]}")

            # Check if already at home position
            max_deviation = max(abs(a) for a in current_angles)
            if max_deviation > 5.0:
                print(f"\n⚠ WARNING: Motors not at home position!")
                print(f"  Maximum deviation: {max_deviation:.1f}°")
                print(f"  These tests assume starting from [0, 0, 0, 0, 0, 0]")
                print(f"  Please return motors to home position first")

                response = input("\nContinue anyway? (yes/no): ")
                if response.lower() != 'yes':
                    print("Aborting tests")
                    controller.close()
                    return
        else:
            print("Warning: Could not query position, assuming home [0, 0, 0, 0, 0, 0]")

        # Safety warning
        print("\n" + "="*60)
        print("Tests will start in 3 seconds")
        print("Press Ctrl+C now to abort if motors are in unsafe position")
        print("="*60)
        time.sleep(3)

    # Test 1: Auto-timed sequence
    print("\n\n" + "="*60)
    print("Starting TEST 1")
    print("="*60)
    trajectory1 = generate_test1_trajectory(speed_multiplier=SPEED_MULTIPLIER_TEST1)

    expected_max_vel_1 = MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST1
    print("\nValidating velocity limits...")
    valid1 = validate_trajectory_velocity(trajectory1, max_velocity_deg_per_sec=expected_max_vel_1)
    if valid1:
        print("✓ Test 1 trajectory is within velocity limits")
    else:
        print("⚠ WARNING: Test 1 has velocity violations!")

    print("\nExecuting Test 1...")
    controller.execute_trajectory(trajectory1)
    print("✓ Test 1 complete")

    # Wait between tests
    print("\nWaiting 2 seconds before Test 2...")
    time.sleep(2)

    # Test 2: Auto-timed sequence
    print("\n\n" + "="*60)
    print("Starting TEST 2")
    print("="*60)
    trajectory2 = generate_test2_trajectory(speed_multiplier=SPEED_MULTIPLIER_TEST2)

    expected_max_vel_2 = MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST2
    print("\nValidating velocity limits...")
    valid2 = validate_trajectory_velocity(trajectory2, max_velocity_deg_per_sec=expected_max_vel_2)
    if valid2:
        print("✓ Test 2 trajectory is within velocity limits")
    else:
        print("⚠ WARNING: Test 2 has velocity violations!")
        print("  This test is designed to be aggressive and may exceed limits")

    print("\nExecuting Test 2...")
    controller.execute_trajectory(trajectory2)
    print("✓ Test 2 complete")

    # Wait between tests
    print("\nWaiting 2 seconds before Test 3...")
    time.sleep(2)

    # Test 3: Auto-timed sequence (slower speed)
    print("\n\n" + "="*60)
    print("Starting TEST 3")
    print("="*60)
    trajectory3 = generate_test3_trajectory(speed_multiplier=SPEED_MULTIPLIER_TEST3)

    expected_max_vel_3 = MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST3
    print("\nValidating velocity limits...")
    valid3 = validate_trajectory_velocity(trajectory3, max_velocity_deg_per_sec=expected_max_vel_3)
    if valid3:
        print("✓ Test 3 trajectory is within velocity limits")
    else:
        print("⚠ WARNING: Test 3 has velocity violations!")

    print("\nExecuting Test 3...")
    controller.execute_trajectory(trajectory3)
    print("✓ Test 3 complete")

    # Wait between tests
    print("\nWaiting 2 seconds before Test 4...")
    time.sleep(2)

    # Test 4: Very slow auto-timed sequence
    print("\n\n" + "="*60)
    print("Starting TEST 4")
    print("="*60)
    trajectory4 = generate_test4_trajectory(speed_multiplier=SPEED_MULTIPLIER_TEST4)

    expected_max_vel_4 = MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST4
    print("\nValidating velocity limits...")
    valid4 = validate_trajectory_velocity(trajectory4, max_velocity_deg_per_sec=expected_max_vel_4)
    if valid4:
        print("✓ Test 4 trajectory is within velocity limits")
    else:
        print("⚠ WARNING: Test 4 has velocity violations!")

    print("\nExecuting Test 4...")
    controller.execute_trajectory(trajectory4)
    print("✓ Test 4 complete")

    # Summary
    print("\n\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Test 1 (speed {SPEED_MULTIPLIER_TEST1:.2f}×, {trajectory1[-1][0]/1000:.1f}s):  {'✓ PASS' if valid1 else '✗ VELOCITY VIOLATIONS'}")
    print(f"Test 2 (speed {SPEED_MULTIPLIER_TEST2:.2f}×, {trajectory2[-1][0]/1000:.1f}s):  {'✓ PASS' if valid2 else '✗ VELOCITY VIOLATIONS'}")
    print(f"Test 3 (speed {SPEED_MULTIPLIER_TEST3:.2f}×, {trajectory3[-1][0]/1000:.1f}s):  {'✓ PASS' if valid3 else '✗ VELOCITY VIOLATIONS'}")
    print(f"Test 4 (speed {SPEED_MULTIPLIER_TEST4:.2f}×, {trajectory4[-1][0]/1000:.1f}s):  {'✓ PASS' if valid4 else '✗ VELOCITY VIOLATIONS'}")
    print("="*60)
    print("\nSpeed Configuration:")
    print(f"  Base max velocity: {MAX_VELOCITY_BASE:.1f}°/sec")
    print(f"  Test 1 max velocity: {MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST1:.1f}°/sec")
    print(f"  Test 2 max velocity: {MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST2:.1f}°/sec")
    print(f"  Test 3 max velocity: {MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST3:.1f}°/sec")
    print(f"  Test 4 max velocity: {MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST4:.1f}°/sec")
    print("="*60)

    # Close connection
    controller.close()
    print("\nAll tests complete!")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
