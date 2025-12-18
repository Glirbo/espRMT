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
SPEED_MULTIPLIER_TEST1 = 1.0   # Test 1 speed (1.0 = 60°/sec max)
SPEED_MULTIPLIER_TEST2 = 1.0   # Test 2 speed (1.0 = 60°/sec max)
SPEED_MULTIPLIER_TEST3 = 0.5   # Test 3 speed (0.5 = 30°/sec max, slower/smoother)
SPEED_MULTIPLIER_TEST4 = 0.1   # Test 4 speed (0.1 = 6°/sec max, very slow/smooth)

MAX_VELOCITY_BASE = 60.0  # Base velocity limit in degrees/second (V2 firmware)
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


def display_menu():
    """Display the test selection menu"""
    print(f"\n{'='*60}")
    print("Motor Test Sequences - Menu")
    print(f"{'='*60}")
    print("1. Test 1 - All Motors to 45° (S-curve)")
    print(f"   Speed: {SPEED_MULTIPLIER_TEST1:.1f}× ({MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST1:.1f}°/sec max)")
    print("\n2. Test 2 - Sinusoidal Motion (±15°)")
    print(f"   Speed: {SPEED_MULTIPLIER_TEST2:.1f}× ({MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST2:.1f}°/sec max)")
    print("\n3. Test 3 - Sequential Motor Test")
    print(f"   Speed: {SPEED_MULTIPLIER_TEST3:.1f}× ({MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST3:.1f}°/sec max)")
    print("\n4. Test 4 - Slow Precision Test")
    print(f"   Speed: {SPEED_MULTIPLIER_TEST4:.1f}× ({MAX_VELOCITY_BASE * SPEED_MULTIPLIER_TEST4:.1f}°/sec max)")
    print("\n5. Run ALL Tests (sequential)")
    print("\n0. Exit")
    print(f"{'='*60}")


def run_single_test(controller, test_number):
    """Run a single test by number"""

    test_map = {
        1: (generate_test1_trajectory, SPEED_MULTIPLIER_TEST1, "Test 1 - All Motors to 45°"),
        2: (generate_test2_trajectory, SPEED_MULTIPLIER_TEST2, "Test 2 - Sinusoidal Motion"),
        3: (generate_test3_trajectory, SPEED_MULTIPLIER_TEST3, "Test 3 - Sequential Motors"),
        4: (generate_test4_trajectory, SPEED_MULTIPLIER_TEST4, "Test 4 - Slow Precision"),
    }

    if test_number not in test_map:
        print(f"Invalid test number: {test_number}")
        return False

    generator_func, speed_mult, test_name = test_map[test_number]

    print(f"\n{'='*60}")
    print(f"Starting {test_name}")
    print(f"{'='*60}")

    # Generate trajectory
    trajectory = generator_func(speed_multiplier=speed_mult)

    # Validate
    expected_max_vel = MAX_VELOCITY_BASE * speed_mult
    print(f"\nValidating velocity limits (max {expected_max_vel:.1f}°/sec)...")
    valid = validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=expected_max_vel)

    if valid:
        print("✓ Trajectory is within velocity limits")
    else:
        print("⚠ WARNING: Trajectory has velocity violations!")

    # Execute
    print(f"\nExecuting {test_name}...")
    controller.execute_trajectory(trajectory)
    print(f"✓ {test_name} complete")

    # Summary
    duration_sec = trajectory[-1][0] / 1000.0
    print(f"\nTest Summary:")
    print(f"  Duration: {duration_sec:.1f}s")
    print(f"  Status: {'✓ PASS' if valid else '✗ VELOCITY VIOLATIONS'}")

    return valid


def run_all_tests(controller):
    """Run all tests sequentially"""

    print(f"\n{'='*60}")
    print("Running ALL Tests")
    print(f"{'='*60}")

    results = {}

    for test_num in [1, 2, 3, 4]:
        if test_num > 1:
            print(f"\nWaiting 2 seconds before next test...")
            time.sleep(2)

        results[test_num] = run_single_test(controller, test_num)

    # Final summary
    print(f"\n\n{'='*60}")
    print("ALL TESTS SUMMARY")
    print(f"{'='*60}")
    print(f"Test 1 (speed {SPEED_MULTIPLIER_TEST1:.2f}×): {'✓ PASS' if results[1] else '✗ VELOCITY VIOLATIONS'}")
    print(f"Test 2 (speed {SPEED_MULTIPLIER_TEST2:.2f}×): {'✓ PASS' if results[2] else '✗ VELOCITY VIOLATIONS'}")
    print(f"Test 3 (speed {SPEED_MULTIPLIER_TEST3:.2f}×): {'✓ PASS' if results[3] else '✗ VELOCITY VIOLATIONS'}")
    print(f"Test 4 (speed {SPEED_MULTIPLIER_TEST4:.2f}×): {'✓ PASS' if results[4] else '✗ VELOCITY VIOLATIONS'}")
    print(f"{'='*60}")


def check_home_position(controller):
    """Check if motors are at home position"""

    if controller.simulation_mode:
        return True

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
                return False
    else:
        print("Warning: Could not query position, assuming home [0, 0, 0, 0, 0, 0]")

    # Safety warning
    print("\n" + "="*60)
    print("Test will start in 3 seconds")
    print("Press Ctrl+C now to abort if motors are in unsafe position")
    print("="*60)
    time.sleep(3)

    return True


def main():
    """Main test sequence execution with menu"""

    print(f"\n{'='*60}")
    print("Motor Test Sequences")
    print("Tests all 6 motors with synchronized motion patterns")
    print(f"{'='*60}")

    # Connect to ESP32
    print("\nConnecting to ESP32...")
    controller = MotionController('/dev/ttyACM0', 115200, simulate_if_unavailable=True)

    # Main menu loop
    while True:
        display_menu()

        try:
            choice = input("\nEnter your choice (0-5): ").strip()

            if choice == '0':
                print("\nExiting...")
                break

            elif choice == '5':
                # Run all tests
                if not check_home_position(controller):
                    controller.close()
                    return

                run_all_tests(controller)

                # Ask if user wants to continue
                response = input("\n\nRun another test? (yes/no): ").strip().lower()
                if response != 'yes':
                    break

            elif choice in ['1', '2', '3', '4']:
                test_num = int(choice)

                # Check home position before running test
                if not check_home_position(controller):
                    controller.close()
                    return

                run_single_test(controller, test_num)

                # Ask if user wants to continue
                response = input("\n\nRun another test? (yes/no): ").strip().lower()
                if response != 'yes':
                    break

            else:
                print("Invalid choice. Please enter a number between 0 and 5.")

        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            break

    # Close connection
    controller.close()
    print("\nGoodbye!")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
