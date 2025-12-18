# Python Motion Controller

Python scripts for controlling the robotic arm via serial communication.

## Features

- **Automatic Simulation Mode**: Scripts automatically detect when no hardware is connected and fall back to simulation mode
- **Binary Protocol**: Efficient 17-byte packets with checksums
- **Position Query**: Query current motor positions from ESP32 with `get_current_position()`
- **Trajectory Generation**: Built-in functions for linear and circular motion with S-curve interpolation
- **Velocity Validation**: Automatic checking that trajectories respect 30°/sec limit
- **Easy to Use**: Simple API for sending position commands

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Full Motion Controller

```bash
python3 motion_controller.py
```

**Features:**
- Complete trajectory generation
- Test trajectories (linear, circular)
- Automatic simulation mode if no ESP32 connected

### Simple Example

```bash
python3 simple_example.py
```

**Features:**
- Minimal ~60 lines of code
- Good for learning the protocol
- Shows basic packet structure

### Motor Test Sequences (Interactive Menu)

```bash
python3 motor_test_sequences.py
```

**Interactive menu for testing motor sequences:**

```
============================================================
Motor Test Sequences - Menu
============================================================
1. Test 1 - All Motors to 45° (S-curve)
   Speed: 1.0× (30°/sec max)

2. Test 2 - Sinusoidal Motion (±15°)
   Speed: 1.0× (30°/sec max)

3. Test 3 - Sequential Motor Test
   Speed: 0.5× (15°/sec max)

4. Test 4 - Slow Precision Test
   Speed: 0.1× (3°/sec max)

5. Run ALL Tests (sequential)

0. Exit
============================================================
```

**Features:**
- Interactive menu - select individual tests or run all
- 4 different test patterns with speed multipliers
- Safety checks (home position verification)
- 3-second countdown before motion starts
- Option to repeat tests or exit after completion
- Velocity validation for each test
- Test summary with pass/fail status

**Menu Options:**
- **1-4**: Run individual tests independently
- **5**: Run all 4 tests sequentially
- **0**: Exit cleanly

## Simulation Mode

Both scripts automatically enter simulation mode when:
- Serial port doesn't exist
- ESP32 not connected
- Port access denied

**Example simulation output:**
```
============================================================
SIMULATION MODE - No hardware connected
Simulating serial port: /dev/ttyACM0 @ 115200 baud
============================================================

[SIM TX] t=    0ms  angles=[  0.00°,  45.00°,  30.00°]  cksum=0xC7
[SIM TX] t=   50ms  angles=[  0.00°,  45.15°,  29.90°]  cksum=0xE9
[SIM TX] t=  100ms  angles=[  0.00°,  45.60°,  29.60°]  cksum=0xBC
...
```

This lets you:
- Test trajectory generation without hardware
- Verify packet encoding
- Debug motion planning
- Learn the system before connecting real hardware

## Disable Simulation Mode

To force an error if hardware is not available:

```python
# In motion_controller.py
controller = MotionController(PORT, BAUDRATE, simulate_if_unavailable=False)
```

## Configuration

Edit the scripts to change:
- **Port**: `/dev/ttyACM0` (ESP32-S3 native USB) or `/dev/ttyUSB0` (UART adapter) on Linux, `COM3` on Windows
- **Baud rate**: 115200 or 921600
- **Update rate**: 20 Hz (50ms intervals) in `execute_trajectory()`

## Packet Format

Binary packet (17 bytes):
```
[0-3]    uint32_t  Timestamp (milliseconds)
[4-7]    float     Joint 1 angle (degrees)
[8-11]   float     Joint 2 angle (degrees)
[12-15]  float     Joint 3 angle (degrees)
[16]     uint8_t   XOR checksum
```

All multi-byte values are **little-endian**.

## Querying Current Position

```python
from motion_controller import MotionController

controller = MotionController('/dev/ttyACM0', 115200)

# Query current position from ESP32
angles = controller.get_current_position()
if angles:
    print(f"Current position: J1={angles[0]:.2f}°, J2={angles[1]:.2f}°, J3={angles[2]:.2f}°")
else:
    print("Failed to query position")

# Verify position after a move
controller.send_position(0, [0, 45, 30])
time.sleep(1)
final_pos = controller.get_current_position()
print(f"Final position: {final_pos}")

controller.close()
```

**Important Notes:**
- Position query returns ESP32's internal tracking (step counting), NOT actual encoder feedback
- Only works when connected to real hardware (not in simulation mode)
- ESP32 must be running firmware with position query support (send 'P' or '?' command)

## Creating Custom Trajectories

```python
from motion_controller import MotionController, validate_trajectory_velocity

controller = MotionController('/dev/ttyACM0', 115200)  # ESP32-S3 USB port

# Create custom trajectory
trajectory = []
for t in range(0, 1001, 50):  # 0 to 1000ms, every 50ms
    # Your motion planning here
    angle1 = ...  # Calculate angle 1
    angle2 = ...  # Calculate angle 2
    angle3 = ...  # Calculate angle 3
    trajectory.append((t, [angle1, angle2, angle3]))

# IMPORTANT: Validate velocity limits before executing
if validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0):
    print("Trajectory valid - executing")
    controller.execute_trajectory(trajectory, update_rate_hz=20)
else:
    print("ERROR: Trajectory exceeds velocity limits!")

controller.close()
```

**Velocity Limit Considerations:**
- Maximum angular velocity: **30°/sec** (hardware limit)
- S-curve interpolation has **2.0× peak velocity multiplier**
- For 45° move: need minimum 3 seconds (45° × 2.0 / 30°/s = 3.0s)
- Always use `validate_trajectory_velocity()` to check before execution

## Examples

See `motion_controller.py` for:
- `generate_test_trajectory()` - Linear motion from [0, 0, 0] to [0, 45, 30] over 3 seconds with S-curve
- `generate_circular_trajectory()` - Circular motion in joint space
- `validate_trajectory_velocity()` - Checks if trajectory respects velocity limits

**Current Test Sequence:**
1. **Position query** - Queries ESP32 for current position on startup
2. **Warning message** - 3-second countdown (press Ctrl+C to abort if motors not at [0, 0, 0])
3. **Test 1: Linear move** - [0, 0, 0] → [0, 45, 30] over 3 seconds
4. **Position verification** - Queries position after move
5. **5-second pause**
6. **Test 2: Circular motion** - Starts at [0, 45, 30], 15° radius, 5 seconds
7. **Position verification** - Queries final position

## Troubleshooting

### "No module named 'serial'"
```bash
pip install pyserial
```

### Permission Denied (Linux)
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### "Failed to query position from ESP32"

**Causes:**
1. ESP32 firmware doesn't have position query support (older version)
2. Serial communication issue
3. ESP32 not ready yet

**Solutions:**
```bash
# 1. Rebuild ESP32 firmware with latest code
cd ../esp32-idf
idf.py build flash

# 2. Check ESP32 logs for position responses
idf.py monitor
# Look for: "Position query: sent [0.00°, 0.00°, 0.00°]"

# 3. Test position query manually
echo "P" > /dev/ttyACM0
# Should see: "POS:0.00,0.00,0.00" in monitor
```

### Velocity Warnings in ESP32 Logs

**Example:**
```
W (1234) RMTArm: Motor 2: Requested 37.6°/s exceeds max 30.0°/s!
```

**Cause:** Trajectory violates 30°/sec velocity limit (often due to S-curve peak)

**Solution:**
```python
# Increase trajectory duration
duration_ms = (max_angle_change * 2.0) / 30.0 * 1000  # S-curve needs 2× factor

# Or use validate_trajectory_velocity() before executing
if not validate_trajectory_velocity(trajectory):
    print("Adjust trajectory duration!")
```

### Different Port
Check your port:
```bash
# Linux - Find available ports
ls /dev/ttyUSB* /dev/ttyACM*

# ESP32-S3 with native USB typically shows as /dev/ttyACM0
# ESP32 with USB-to-UART chip typically shows as /dev/ttyUSB0

# Windows
# Check Device Manager for COM port number
```

Then update `PORT` in the script.

**Common ports:**
- ESP32-S3 native USB: `/dev/ttyACM0` (Linux), `COMx` (Windows)
- ESP32 with UART chip: `/dev/ttyUSB0` (Linux), `COMx` (Windows)
