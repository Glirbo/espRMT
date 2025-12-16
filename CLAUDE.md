# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A robotic arm motion control system using Python for motion planning and ESP32 for real-time stepper motor control via the RMT (Remote Control Transceiver) hardware module.

**Key Architecture:**
- **Python (PC)**: High-level motion planning, trajectory generation, inverse kinematics
- **ESP32**: Real-time pulse generation, angle-to-step conversion, hardware-timed motor control
- **Communication**: Binary protocol (17-byte packets) over UART at 115200 baud

## Current Motor Configuration (CRITICAL)

The system is configured with the following motor parameters:

```c
Steps per revolution: 4000 (direct, NO microstepping)
Gear ratio:          1:50
Steps per degree:    555.56
Max velocity:        30°/sec (HARD LIMIT - enforced by both Python and ESP32)
Max step rate:       16,667 steps/sec
Min pulse period:    60μs
```

**When modifying trajectories or motion planning:**
- ALWAYS validate that trajectories respect the 30°/sec velocity limit
- Use `validate_trajectory_velocity()` function in Python before executing
- At 50ms updates (20Hz), max angular change is 1.5° per update
- Buffer size (16,384 items) supports ~29.5° moves in 50ms
- **S-curve interpolation has 2.0× velocity multiplier at peak** - account for this when calculating durations
- For S-curve: `peak_velocity = angle_change × 2.0 / duration` must be ≤ 30°/sec

## Build Commands

### ESP32 Firmware

```bash
# Setup ESP-IDF environment (required once per terminal session)
. $HOME/esp/esp-idf/export.sh

# Navigate to firmware directory
cd esp32-idf

# Build only
idf.py build

# Build and flash to ESP32
idf.py -p /dev/ttyACM0 flash  # ESP32-S3 native USB
# OR
idf.py -p /dev/ttyUSB0 flash  # ESP32 with UART adapter

# Build, flash, and monitor serial output
idf.py -p /dev/ttyACM0 flash monitor

# Monitor only (after flashing)
idf.py -p /dev/ttyACM0 monitor

# Exit monitor: Ctrl+]
```

### Python Controller

```bash
cd python

# Install dependencies (first time only)
pip install -r requirements.txt

# Run full motion controller with test trajectories
python3 motion_controller.py

# Run simple example (minimal code for learning)
python3 simple_example.py
```

**Simulation mode:** Both Python scripts automatically detect when hardware is unavailable and fall back to simulation mode, printing packets to console instead of sending to serial.

## System Architecture

### Motion Planning Pipeline

The system implements a 9-step pipeline from Cartesian coordinates to motor pulses:

1. **Interpolate Cartesian Path** - Create waypoints in XYZ space
2. **Inverse Kinematics** - Convert each XYZ point to joint angles (CRITICAL: IK on EVERY waypoint to ensure straight lines)
3. **Calculate Joint Distances** - Determine maximum joint movement
4. **Determine Timing** - Set total move duration based on velocity limits
5. **Generate Velocity Profile** - Create trapezoidal/triangular acceleration profile
6. **Generate Setpoints** - Timestamped angle targets at update rate (20Hz = 50ms)
7. **Binary Encoding** - Pack into 17-byte packets and stream to ESP32
8. **ESP32 Processing** - Receive, validate checksum, convert angles to steps
9. **RMT Pulse Generation** - Hardware-timed STEP pulses to motor drivers

### Why This Architecture?

**Python handles motion planning because:**
- Complex trajectory calculations (IK, interpolation, profiling)
- Easy visualization and debugging
- Access to scientific libraries (NumPy, SciPy)
- Multi-axis coordination is straightforward

**ESP32 handles pulse generation because:**
- RMT module provides hardware-timed pulses (no CPU jitter)
- Microsecond precision
- 8 independent channels for multi-axis control
- Non-blocking operation

### Binary Protocol

**Motion Command (17-byte packet):**
```
Bytes 0-3:   Timestamp (uint32_t, little-endian) - milliseconds
Bytes 4-7:   Joint 1 angle (float, little-endian) - degrees
Bytes 8-11:  Joint 2 angle (float, little-endian) - degrees
Bytes 12-15: Joint 3 angle (float, little-endian) - degrees
Byte 16:     Checksum (XOR of all previous 16 bytes)
```

All values are little-endian. The checksum is a simple XOR of all data bytes.

**Position Query Protocol (ASCII):**
- **Request**: Send single byte `'P'` (0x50) or `'?'` (0x3F)
- **Response**: ASCII string `"POS:j1,j2,j3\n"` (e.g., `"POS:0.00,45.00,30.00\n"`)
- Use `controller.get_current_position()` in Python to query current motor positions
- Returns list `[j1, j2, j3]` or `None` if failed
- Position values are the ESP32's internal tracking (not actual encoder feedback)

## Critical Implementation Details

### Position Tracking and Homing

**IMPORTANT**: The system uses **open-loop position tracking** (no encoders):
- ESP32 assumes motors start at `[0°, 0°, 0°]` on power-up
- Position is tracked by counting steps sent, NOT actual motor position
- If motors are moved manually or lose steps, tracking becomes incorrect
- **ALWAYS manually position motors at [0°, 0°, 0°] before powering on**

**Position Query Feature:**
```python
# Query what ESP32 thinks the position is
angles = controller.get_current_position()
if angles:
    print(f"ESP32 reports: {angles}")  # e.g., [0.0, 45.0, 30.0]
```

**Recommended for production:**
- Add limit switches for homing
- Implement homing routine on startup
- Consider adding encoders for closed-loop control

**Current test sequences:**
- Test 1: Linear move from [0, 0, 0] → [0, 45, 30] over 3 seconds
- Test 2: Circular motion starting at [0, 45, 30] with 15° radius over 5 seconds
- 3-second warning displayed before motion begins (Ctrl+C to abort)

### ESP32 RMT Module

The RMT (Remote Control Transceiver) is a hardware peripheral that generates precise pulse sequences:

**Configuration:**
- Clock divider: 80 (80MHz / 80 = 1MHz, so 1 tick = 1μs)
- Pulse width: 5μs (configurable based on driver requirements)
- Each RMT item defines: `{level0, duration0, level1, duration1}`
- Pre-allocated buffers to avoid malloc() in real-time path

**Pre-allocated Buffers (CRITICAL):**
The system uses static buffers to eliminate malloc() overhead:
```c
#define MAX_RMT_ITEMS 16384
static rmt_item32_t rmt_items_buffer[NUM_MOTORS][MAX_RMT_ITEMS];

// Pending packet buffer (for protocol coexistence)
static uint8_t pending_packet[PACKET_SIZE];
static int pending_packet_len = 0;
```
This costs ~192KB RAM but ensures consistent 20Hz timing. DO NOT use malloc/free in the motion control path.

**Pending Packet Buffer System:**
The system handles both 1-byte position queries and 17-byte motion commands:
- `check_position_request()` reads 1 byte first
- If it's 'P' or '?': sends position response
- If not: stores byte in `pending_packet` buffer
- `receive_command()` uses pending byte as first byte of motion command
- This prevents packet loss when protocols coexist

**Velocity Limiting:**
The ESP32 enforces minimum pulse period to prevent exceeding 30°/sec:
```c
#define MIN_PERIOD_US 60  // Calculated from MAX_VELOCITY * STEPS_PER_DEGREE
```

### Coordinated Multi-Axis Motion

All joints must start and stop together. The system achieves this by:
1. Python generates synchronized setpoints (all joints move proportionally)
2. ESP32 receives single command with all 3 joint angles
3. RMT channels run in parallel (hardware-synchronized)
4. All joints complete their moves within the same 50ms window

### GPIO Pin Assignments

**ESP32-S3 (current default):**
- Motor 0: STEP=GPIO1, DIR=GPIO4
- Motor 1: STEP=GPIO2, DIR=GPIO5
- Motor 2: STEP=GPIO3, DIR=GPIO6

**Classic ESP32:**
- Motor 0: STEP=GPIO25, DIR=GPIO32
- Motor 1: STEP=GPIO26, DIR=GPIO33
- Motor 2: STEP=GPIO27, DIR=GPIO14

Defined in `esp32-idf/main/main.c` lines 52-60.

## Important Constraints

### Inverse Kinematics Must Run on Every Waypoint

**CRITICAL:** Never interpolate in joint space for straight-line moves.

❌ **WRONG:**
```python
# This creates a curved path in XYZ space!
start_angles = [0, 45, 30]
end_angles = [0, 75, 10]
interpolate_angles(start_angles, end_angles)  # CURVED PATH
```

✅ **CORRECT:**
```python
# Interpolate in Cartesian space, then IK each point
for xyz in interpolate_xyz(start_xyz, end_xyz):
    angles = inverse_kinematics(xyz)  # Straight line!
    waypoints.append(angles)
```

This is why the system performs IK on every interpolated waypoint.

### Velocity Validation

Always validate trajectories before execution:
```python
trajectory = generate_test_trajectory()
validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0)
controller.execute_trajectory(trajectory)
```

The validator checks each segment and warns if any joint exceeds limits.

### Update Rate vs. Trajectory Smoothness

- Update rate: 20Hz (50ms intervals) - **DO NOT CHANGE** without recalculating buffers
- Each update can move maximum 1.5° at 30°/sec
- For smooth motion, ensure trajectory generator respects this

## File Structure

```
RMTArm/
├── README.md              # Comprehensive system documentation with 9-step pipeline
├── MONITORING_GUIDE.md    # How to monitor ESP32 logs while running Python controller
├── python/
│   ├── motion_controller.py  # Main controller with trajectory generation
│   ├── simple_example.py     # Minimal example (~70 lines)
│   ├── README.md             # Python-specific documentation
│   └── requirements.txt      # Python dependencies (pyserial)
└── esp32-idf/
    ├── main/
    │   ├── main.c            # ESP32 firmware (RMT control, UART, motion processing)
    │   └── CMakeLists.txt
    ├── CMakeLists.txt
    └── sdkconfig.defaults    # ESP32 configuration (baud rate, logging, RMT settings)
```

## Common Development Tasks

### Adding a New Trajectory Type

1. Create generator function in `python/motion_controller.py`:
```python
def generate_my_trajectory():
    trajectory = []
    # Generate (timestamp_ms, [j1, j2, j3]) tuples
    # ... your logic here ...
    return trajectory
```

2. **CRITICAL**: Calculate duration accounting for S-curve peak velocity:
```python
# For S-curve interpolation (2.0× multiplier at peak)
max_angle_change = 45.0  # degrees
required_duration = (max_angle_change * 2.0) / 30.0  # seconds
duration_ms = int(required_duration * 1000)

# Example: 45° move needs 3 seconds (45 × 2.0 / 30 = 3.0s)
```

3. Validate velocity limits:
```python
trajectory = generate_my_trajectory()
valid = validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0)
if not valid:
    print("ERROR: Trajectory exceeds velocity limits!")
    return
```

4. Execute:
```python
controller.execute_trajectory(trajectory)
```

### Querying Motor Position

```python
# Connect to ESP32
controller = MotionController('/dev/ttyACM0', 115200)

# Query current position anytime
angles = controller.get_current_position()
if angles:
    print(f"Motors at: J1={angles[0]:.2f}°, J2={angles[1]:.2f}°, J3={angles[2]:.2f}°")
else:
    print("Position query failed - check connection")

# Verify position after move
controller.send_position(0, [0, 45, 30])
time.sleep(1)
final_pos = controller.get_current_position()
# Compare final_pos to expected [0, 45, 30]
```

### Changing Motor Configuration

If you need to change motor parameters (steps/rev, gear ratio, etc.):

1. Update `esp32-idf/main/main.c` lines 35-40:
```c
#define STEPS_PER_REV   4000.0f  // Your motor's steps
#define GEAR_RATIO      50.0f    // Your gearing
#define STEPS_PER_DEGREE ((STEPS_PER_REV * GEAR_RATIO) / 360.0f)
```

2. Recalculate derived values:
   - `MIN_PERIOD_US` = 1,000,000 / (MAX_VELOCITY × STEPS_PER_DEGREE)
   - `MAX_RMT_ITEMS` buffer size (current: 16,384 items ≈ 29.5° at current config)

3. Rebuild and reflash ESP32

4. Update comments showing calculations

### Monitoring ESP32 While Running Python

See `MONITORING_GUIDE.md` for 5 different solutions. **Recommended for development:**

**Solution 1: Dual UART (best)** - Use external USB-UART adapter on GPIO17/18 for logs
**Solution 2: Sequential** - Monitor startup, exit, then run Python

The port conflict occurs because both `idf.py monitor` and Python need exclusive access to the serial port.

### Debugging Timing Issues

If you see timing inconsistencies (not 20Hz):

1. Check ESP32 logs for velocity warnings
2. Verify Python is sending at correct rate
3. Check for malloc/free in ESP32 motion path (should use pre-allocated buffers)
4. Monitor statistics (printed every 500 commands)

Current system achieves consistent 20Hz with pre-allocated buffers.

## Testing Strategy

1. **Test in simulation first**: Both Python scripts auto-detect missing hardware
2. **Test simple moves**: Use `simple_example.py` to verify communication
3. **Test trajectories**: Use `motion_controller.py` test sequences
4. **Validate velocity**: Check ESP32 logs for velocity limit warnings
5. **Monitor timing**: Ensure consistent 20Hz update rate

## Key Lessons Learned (Historical Context)

### Performance Issues (Resolved)
- **malloc() overhead**: Original code used malloc/free for RMT items, causing 18ms delays. Switched to pre-allocated static buffers for consistent 20Hz timing.
- **Statistics overhead**: Large log boxes (17 lines) caused 202ms delays. Reduced to single-line compact stats every 500 commands.
- **Buffer sizing**: Started at 512 items (insufficient), increased to 16,384 to handle large moves without clamping.

### Configuration Changes
- **Microstepping removed**: Changed from calculated microstepping (200 × 16) to direct steps/rev (4000) for clarity and accuracy.
- **Motor configuration**: Now uses 4000 steps/rev with 1:50 gear ratio = 555.56 steps/degree.

### Protocol Issues (Resolved)
- **Position query packet loss**: Initial implementation of `check_position_request()` consumed motion command bytes without processing them, causing packet loss. Fixed with pending packet buffer system.
- **Protocol coexistence**: Successfully implemented dual protocol (1-byte ASCII position queries + 17-byte binary motion commands) without interference.

### Velocity Limiting
- **S-curve peak velocity**: S-curve interpolation has 2.0× velocity multiplier at midpoint. Must calculate duration as: `duration ≥ (angle_change × 2.0) / max_velocity`.
- **Example**: 45° move with 30°/sec limit requires minimum 3 seconds with S-curve (not 1.5 seconds).
- **Validation**: Always use `validate_trajectory_velocity()` before execution to catch violations.

### Position Tracking
- **Open-loop limitation**: System has no encoders - position is tracked by counting steps. Motors must be manually positioned at [0, 0, 0] on startup.
- **Position query feature**: Added ability to query ESP32's internal position tracking with 'P' command for debugging and verification.

## Documentation

- `README.md`: Complete system documentation with detailed 9-step pipeline example
- `python/README.md`: Python-specific usage and API documentation
- `MONITORING_GUIDE.md`: Solutions for monitoring ESP32 while running Python controller
- See README.md for comprehensive RMT deep dive with timing diagrams

## ESP-IDF Version

This project requires ESP-IDF v5.0 or later. The code uses:
- `esp_rom_delay_us()` (not deprecated `ets_delay_us()`)
- Modern UART API
- Standard RMT driver API
