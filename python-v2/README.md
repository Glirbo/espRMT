# Python V2 - Updated for ESP32-WROOM-V2 Firmware

This folder contains Python scripts updated to work with the ESP32-WROOM-V2 firmware, which features per-motor gear ratios and 60°/sec maximum velocity.

## Changes from V1

### **Increased Velocity Limit**
All scripts have been updated from 30°/sec to **60°/sec** maximum angular velocity:

- `motion_controller.py`: Default validation changed to 60°/sec
- `motor_test_sequences.py`: MAX_VELOCITY_BASE = 60.0
- `inverse_kinematics_controller.py`: All planners use 60°/sec
- `inverse_kinematics_controller_gui.py`: GUI planner uses 60°/sec

### **Per-Motor Gear Ratios**
The V2 firmware supports different gear ratios per motor:
- **Motors 0 & 5**: 1:10 gearing (111.11 steps/degree)
- **Motors 1-4**: 1:50 gearing (555.56 steps/degree)

**Note:** The Python scripts don't need to be aware of gear ratios - the ESP32 firmware handles all gear ratio calculations internally. Python sends angles in degrees, and ESP32 converts to steps using the correct per-motor ratio.

## Files Included

### Core Motion Control
- **motion_controller.py** - Main motion controller class
  - Binary protocol implementation
  - Position query with retry mechanism
  - Trajectory execution
  - Velocity validation (60°/sec)

### Test Scripts
- **motor_test_sequences.py** - Interactive test menu with 4 test patterns
  - **Interactive menu system** - Select individual tests or run all
  - 4 different test patterns with speed multipliers
  - Updated for 60°/sec base velocity
  - Speed multipliers: 1.0x (60°/s), 0.5x (30°/s), 0.1x (6°/s)
  - Safety checks before each test run

- **simple_example.py** - Minimal example (~70 lines)
  - Good starting point for learning the API

- **send_pulses.py** - Direct step/pulse control
  - Send a fixed number of pulses to a specific motor
  - Interactive prompts for motor, steps, and direction
  - Perfect for testing individual motors and calibration
  - Automatically converts steps to angles based on gear ratios
  - Auto-detects port (tries /dev/ttyACM0, /dev/ttyUSB0, COM3)

### Inverse Kinematics
- **inverse_kinematics_controller.py** - Full IK system
  - 6-DOF robot kinematics
  - Cartesian path planning
  - Joint-space and Cartesian trajectories
  - Updated to 60°/sec

- **inverse_kinematics_controller_gui.py** - Interactive GUI
  - Visual control of robot arm
  - Real-time position updates
  - Cartesian and joint-space control
  - Updated to 60°/sec

### Dependencies
- **requirements.txt** - Python package dependencies
  - pyserial

## Quick Start

### 1. Install Dependencies
```bash
cd python-v2
pip install -r requirements.txt
```

### 2. Connect Hardware
Ensure ESP32-WROOM-V2 firmware is flashed and connected via USB.

### 3. Run Test Sequences

**Interactive Test Menu (Recommended):**
```bash
python motor_test_sequences.py
```

The script now presents an interactive menu:
```
============================================================
Motor Test Sequences - Menu
============================================================
1. Test 1 - All Motors to 45° (S-curve)
   Speed: 1.0× (60°/sec max)

2. Test 2 - Sinusoidal Motion (±15°)
   Speed: 1.0× (60°/sec max)

3. Test 3 - Sequential Motor Test
   Speed: 0.5× (30°/sec max)

4. Test 4 - Slow Precision Test
   Speed: 0.1× (6°/sec max)

5. Run ALL Tests (sequential)

0. Exit
============================================================
Enter your choice (0-5):
```

**Other Scripts:**
```bash
# Send fixed pulses to a specific motor (great for testing/calibration)
python send_pulses.py

# Simple example (minimal code)
python simple_example.py

# IK controller (requires DH parameters configuration)
python inverse_kinematics_controller.py

# Interactive GUI
python inverse_kinematics_controller_gui.py
```

## Motor Configuration Reference

The V2 firmware uses these motor parameters:

| Motor | Gearing | Steps/Rev | Steps/° | Max Steps/s @ 60°/s |
|-------|---------|-----------|---------|---------------------|
| 0     | 1:10    | 4000      | 111.11  | 6,667               |
| 1     | 1:50    | 4000      | 555.56  | 33,333              |
| 2     | 1:50    | 4000      | 555.56  | 33,333              |
| 3     | 1:50    | 4000      | 555.56  | 33,333              |
| 4     | 1:50    | 4000      | 555.56  | 33,333              |
| 5     | 1:10    | 4000      | 111.11  | 6,667               |

**Python scripts don't need to know these values!** Just send angles in degrees, and the firmware handles conversion.

## Speed Multipliers (motor_test_sequences.py)

The test sequences use speed multipliers to test different velocities:

```python
SPEED_MULTIPLIER_TEST1 = 1.0   # 60°/sec (full speed)
SPEED_MULTIPLIER_TEST2 = 1.0   # 60°/sec (full speed)
SPEED_MULTIPLIER_TEST3 = 0.5   # 30°/sec (smooth)
SPEED_MULTIPLIER_TEST4 = 0.1   # 6°/sec (very slow/precise)
```

Adjust these to customize test speeds for your application.

## Interactive Menu Features

The `motor_test_sequences.py` script provides an easy-to-use menu interface:

**Menu Options:**
- **Options 1-4**: Run individual tests independently
- **Option 5**: Run all 4 tests sequentially with summary
- **Option 0**: Exit cleanly

**Safety Features:**
- Home position check before each test
- 3-second countdown before motion starts
- Ctrl+C interrupt handling at any time
- Option to run another test or exit after completion

**Typical Workflow:**
1. Script connects to ESP32 (or simulation mode if unavailable)
2. Menu displays with speed information for each test
3. Select a test number (1-5) or 0 to exit
4. System checks motor home position and warns if needed
5. 3-second safety countdown
6. Test executes with progress updates
7. Test summary shows duration and pass/fail status
8. Prompt to run another test or exit

**Benefits:**
- Test individual sequences during development
- Quickly repeat specific tests for debugging
- Skip tests you don't need
- See velocity limits for each test before running

## Velocity Validation

All scripts include automatic velocity validation:

```python
from motion_controller import validate_trajectory_velocity

# Generate trajectory
trajectory = generate_trajectory()

# Validate (60°/sec is now the default)
valid = validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=60.0)

if valid:
    controller.execute_trajectory(trajectory)
else:
    print("ERROR: Trajectory exceeds velocity limits!")
```

The validator checks each segment and warns if any joint exceeds the limit.

## S-Curve Trajectory Considerations

S-curve interpolation has a **2.0× velocity multiplier at peak**. When calculating durations:

```python
# For S-curve trajectories
angle_change = 45.0  # degrees
max_velocity = 60.0  # degrees/sec

# Duration must account for 2x peak
min_duration = (angle_change * 2.0) / max_velocity
# = (45 * 2.0) / 60 = 1.5 seconds minimum

duration_ms = int(min_duration * 1000)  # Convert to milliseconds
```

The motion_controller automatically calculates safe durations for you.

## Serial Port Configuration

Update the serial port in scripts to match your system:

**Linux:**
```python
PORT = '/dev/ttyACM0'  # ESP32-S3 native USB
PORT = '/dev/ttyUSB0'  # ESP32 with external USB-UART
```

**Windows:**
```python
PORT = 'COM3'  # Check Device Manager for correct port
```

**macOS:**
```python
PORT = '/dev/cu.usbserial-XXXX'  # Check ls /dev/cu.*
```

## Position Query Feature

The V2 firmware supports querying current motor positions:

```python
controller = MotionController('/dev/ttyACM0', 115200)

# Query current position (returns [j0, j1, j2, j3, j4, j5] or None)
angles = controller.get_current_position()

if angles:
    print(f"Current position: {angles}")
else:
    print("Position query failed - using trajectory endpoint")
```

The controller has automatic fallback - if queries fail, it tracks expected position from trajectory endpoints.

## Troubleshooting

### Velocity Warnings
If you see "Motor X velocity limited" warnings:
- Trajectory is trying to exceed 60°/sec
- Reduce angle changes or increase duration
- Use `validate_trajectory_velocity()` before executing

### Position Query Failures
If position queries frequently fail:
- Check USB cable quality
- Ensure ESP32 logging verbosity is set to WARNING (not DEBUG)
- Python has automatic retry with fallback, so occasional failures are OK

### Motors Move Incorrectly
- Verify correct ESP32-WROOM-V2 firmware is flashed
- Check motor wiring (STEP/DIR pins)
- Ensure motors are manually positioned at [0,0,0,0,0,0] on startup
- System uses open-loop position tracking (no encoders)

### Build Errors
If you get import errors:
```bash
pip install --upgrade -r requirements.txt
```

## Migration from V1

If you're upgrading from V1 Python scripts:

1. ✅ **Automatic**: Velocity validation functions already updated
2. ✅ **Automatic**: Test sequences use new 60°/sec base
3. ✅ **Automatic**: IK planners use 60°/sec
4. ⚠️ **Manual**: Update any custom trajectories you created
   - Change hardcoded 30.0 → 60.0 in your custom code
   - Re-validate trajectory durations

## Example: Custom Trajectory

```python
#!/usr/bin/env python3
from motion_controller import MotionController, validate_trajectory_velocity
import time

# Connect to ESP32
controller = MotionController('/dev/ttyACM0', 115200)

# Get current position
current = controller.get_current_position()
if not current:
    current = [0, 0, 0, 0, 0, 0]  # Fallback

# Create custom trajectory
trajectory = []
duration_ms = 3000  # 3 seconds
update_period_ms = 50  # 20 Hz

for i in range(0, duration_ms + update_period_ms, update_period_ms):
    # Move motors 0 and 5 (1:10 gearing) through larger angles
    # Move motors 1-4 (1:50 gearing) through smaller angles
    angles = [
        current[0] + 30.0 * (i / duration_ms),  # Motor 0: 30° total (1:10)
        current[1] + 10.0 * (i / duration_ms),  # Motor 1: 10° total (1:50)
        current[2] + 10.0 * (i / duration_ms),  # Motor 2: 10° total (1:50)
        current[3] + 10.0 * (i / duration_ms),  # Motor 3: 10° total (1:50)
        current[4] + 10.0 * (i / duration_ms),  # Motor 4: 10° total (1:50)
        current[5] + 30.0 * (i / duration_ms),  # Motor 5: 30° total (1:10)
    ]
    trajectory.append((i, angles))

# Validate before executing
if validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=60.0):
    print("Executing trajectory...")
    controller.execute_trajectory(trajectory)
    time.sleep(duration_ms / 1000.0 + 1.0)
    print("Done!")
else:
    print("ERROR: Trajectory validation failed!")

controller.close()
```

## Performance Notes

**Update Rate:**
- ESP32 processes commands at ~60Hz (16ms cycle time)
- Python sends at 20Hz (50ms intervals)
- This gives ESP32 plenty of headroom for smooth motion

**Buffer Capacity:**
- Motors 1-4 (1:50): 5.5° max per 50ms command
- Motors 0,5 (1:10): 27.6° max per 50ms command
- At 60°/sec: Max 3.0° per 50ms needed
- Safety margins: 1.84× (high-gear) and 9.2× (low-gear)

## Contributing

When creating new scripts:
1. Use `validate_trajectory_velocity()` with 60.0 default
2. Calculate S-curve durations with 2.0× multiplier
3. Test at reduced speeds first (0.5× or 0.1× multiplier)
4. Document any custom DH parameters or kinematics

## License

Same as main project license.
