# Robotic Arm Motion Control System

A Python-based motion planning system for a robotic arm using ESP32 for stepper motor control via RMT (Remote Control) module.

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│  PYTHON (PC)                                            │
│  - Cartesian path interpolation                         │
│  - Inverse kinematics                                   │
│  - Trapezoidal velocity profile generation              │
│  - Timestamped setpoint generation                      │
└─────────────────┬───────────────────────────────────────┘
                  │ USB/UART/WiFi
                  │ {time, [angles]}
                  ▼
┌─────────────────────────────────────────────────────────┐
│  ESP32                                                  │
│  - Receive angle setpoints                              │
│  - Convert angles to steps                              │
│  - Generate pulses via RMT module                       │
└─────────────────┬───────────────────────────────────────┘
                  │ STEP/DIR signals
                  ▼
┌─────────────────────────────────────────────────────────┐
│  STEPPER MOTOR DRIVERS                                  │
│  - Drive stepper motors                                 │
└─────────────────────────────────────────────────────────┘
```

## Motion Planning Pipeline

```
XYZ Start & End Points
         ↓
┌────────────────────────────────┐
│ STEP 1: Interpolate Path       │
│ Create waypoints in XYZ space  │
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 2: Inverse Kinematics     │
│ Convert each XYZ to angles     │
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 3: Calculate Distances    │
│ Find max joint movement        │
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 4: Determine Timing       │
│ Set total move duration        │
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 5: Velocity Profile       │
│ Create trapezoidal profile     │
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 6: Generate Setpoints     │
│ Timestamped angle targets      │
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 7: Send to ESP32          │
│ Stream setpoints at update rate│
└────────────────────────────────┘
         ↓
┌────────────────────────────────┐
│ STEP 8: RMT Pulse Generation   │
│ ESP32 generates STEP pulses    │
└────────────────────────────────┘
```

---

## Detailed Step-by-Step Example

### Given:
- **Start Position (XYZ)**: (0, 40, 50) mm
- **End Position (XYZ)**: (0, -40, 30) mm
- **Goal**: Move in a straight line
- **Robot**: 3-axis robotic arm

---

### STEP 1: Interpolate the Cartesian Path

Create waypoints along a straight line between start and end points.

**Calculate total Cartesian distance:**
```
dx = 0 - 0 = 0 mm
dy = -40 - 40 = -80 mm
dz = 30 - 50 = -20 mm

distance = √(dx² + dy² + dz²)
distance = √(0² + 80² + 20²)
distance = √6800 ≈ 82.46 mm
```

**Choose interpolation spacing:** 20 mm (adjust based on required path accuracy)

**Calculate number of waypoints:**
```
segments = 82.46 / 20 ≈ 4
waypoints = 5 (including start and end)
```

**Generate interpolated XYZ points:**
```
t = 0.00 → Point 0: (0.0,   40.0,  50.0)  [Start]
t = 0.25 → Point 1: (0.0,   20.0,  45.0)
t = 0.50 → Point 2: (0.0,    0.0,  40.0)  [Midpoint]
t = 0.75 → Point 3: (0.0,  -20.0,  35.0)
t = 1.00 → Point 4: (0.0,  -40.0,  30.0)  [End]
```

Where `t` is the normalized parameter (0 to 1) along the path.

**Formula:**
```python
xyz(t) = start_xyz + t * (end_xyz - start_xyz)
```

---

### STEP 2: Inverse Kinematics (IK) on Each Waypoint

Convert each XYZ waypoint to joint angles using your IK solver.

**Run IK for each point:**
```
XYZ (0, 40, 50)   → IK() → Joint angles: [0°,  45°, 30°]
XYZ (0, 20, 45)   → IK() → Joint angles: [0°,  52°, 25°]
XYZ (0, 0, 40)    → IK() → Joint angles: [0°,  60°, 20°]
XYZ (0, -20, 35)  → IK() → Joint angles: [0°,  68°, 15°]
XYZ (0, -40, 30)  → IK() → Joint angles: [0°,  75°, 10°]
```

**Critical: Why IK on every point?**
- Interpolating in joint space does NOT give a straight line in XYZ space
- Must interpolate in XYZ, then convert to joint angles
- This ensures the end effector follows a straight Cartesian path

**IK Validation:**
For each IK solution, check:
1. IK solver succeeded (solution exists)
2. All angles within joint limits
3. No self-collisions
4. Solution is consistent with previous configuration (avoid flips)

If any check fails → path is not feasible, abort.

---

### STEP 3: Calculate Joint Movement Distances

Determine how far each joint must travel for the entire move.

**Calculate per-joint distances:**
```
Joint 1: |0° - 0°|   = 0°
Joint 2: |75° - 45°| = 30°
Joint 3: |10° - 30°| = 20°
```

**Identify the master axis:**
```
Max distance = 30° (Joint 2)
```

The joint with the maximum travel determines the move timing to ensure all joints start and stop together (coordinated motion).

---

### STEP 4: Determine Overall Move Timing

**Option A: Time-based (Cartesian velocity)**
```
Desired Cartesian speed: 50 mm/s
Total time = 82.46 mm / 50 mm/s = 1.65 seconds
```

**Option B: Joint-based (respecting joint limits)**
```
Max joint velocity: 180°/s
Minimum time = 30° / 180°/s = 0.167 seconds
```

**Recommendation:** Choose a duration that:
- Respects maximum joint velocities and accelerations
- Provides smooth motion
- Balances speed and accuracy

**For this example, use: 1.0 second**

---

### STEP 5: Create Trapezoidal Velocity Profile

A trapezoidal profile has three phases:
1. **Acceleration** - ramp up from 0 to max velocity
2. **Constant velocity** - cruise phase
3. **Deceleration** - ramp down to 0

**Define motion constraints:**
```
Total distance: 30° (Joint 2, the master)
Max velocity: 180°/s
Max acceleration: 360°/s²
Total time: 1.0 second
```

**Calculate acceleration time:**
```
t_accel = v_max / acceleration
t_accel = 180 / 360 = 0.5 seconds
```

**Calculate distance during acceleration:**
```
d_accel = 0.5 × acceleration × t_accel²
d_accel = 0.5 × 360 × 0.5² = 45°
```

**Calculate distance during deceleration:**
```
d_decel = 45° (same as acceleration)
```

**Calculate cruise distance:**
```
d_cruise = total_distance - d_accel - d_decel
d_cruise = 30 - 45 - 45 = -60° (negative!)
```

**Result: Triangular profile** (distance too short to reach max velocity)

Recalculate for triangular profile:
```
v_peak = √(distance × acceleration)
v_peak = √(30 × 360) ≈ 103.9°/s

t_accel = v_peak / acceleration ≈ 0.289s
t_total = 2 × t_accel ≈ 0.578s
```

**Scale to desired 1.0 second duration:**
```
Reduce acceleration to stretch the move:
New acceleration ≈ 120°/s²
New peak velocity ≈ 60°/s
Acceleration time: 0.5s
Deceleration time: 0.5s
```

**Velocity profile visualization:**
```
Velocity (°/s)
  ↑
 60 |      /\
    |     /  \
    |    /    \
    |   /      \
  0 |__/________\___→ Time (s)
    0   0.5    1.0
```

---

### STEP 6: Generate Timestamped Setpoints

Create discrete setpoints at regular intervals (e.g., every 50ms for 20 Hz update rate).

**For each timestamp, calculate position along the velocity profile:**

```
Time (s) | Phase | Velocity (°/s) | Joint 2 Pos (°) | Joint 3 Pos (°) | Joint 1 Pos (°)
---------|-------|----------------|-----------------|-----------------|----------------
0.00     | Accel | 0.0            | 45.0            | 30.0            | 0.0
0.05     | Accel | 6.0            | 45.2            | 29.9            | 0.0
0.10     | Accel | 12.0           | 45.6            | 29.7            | 0.0
0.15     | Accel | 18.0           | 46.4            | 29.6            | 0.0
0.20     | Accel | 24.0           | 47.4            | 29.3            | 0.0
0.25     | Accel | 30.0           | 48.8            | 29.0            | 0.0
0.30     | Accel | 36.0           | 50.4            | 28.6            | 0.0
0.35     | Accel | 42.0           | 52.4            | 28.1            | 0.0
0.40     | Accel | 48.0           | 54.6            | 27.6            | 0.0
0.45     | Accel | 54.0           | 57.2            | 27.1            | 0.0
0.50     | Peak  | 60.0           | 60.0            | 20.0            | 0.0  ← Midpoint
0.55     | Decel | 54.0           | 62.8            | 17.9            | 0.0
0.60     | Decel | 48.0           | 65.4            | 16.4            | 0.0
0.65     | Decel | 42.0           | 67.6            | 15.2            | 0.0
0.70     | Decel | 36.0           | 69.6            | 14.1            | 0.0
0.75     | Decel | 30.0           | 71.2            | 13.3            | 0.0
0.80     | Decel | 24.0           | 72.6            | 12.7            | 0.0
0.85     | Decel | 18.0           | 73.6            | 12.1            | 0.0
0.90     | Decel | 12.0           | 74.4            | 11.7            | 0.0
0.95     | Decel | 6.0            | 74.8            | 11.3            | 0.0
1.00     | Stop  | 0.0            | 75.0            | 10.0            | 0.0  ← End
```

**Position profile visualization:**
```
Position (°)
  ↑
 75 |           ___--
    |       __--
 60 |     _/
    |   _/
 45 |__/___________→ Time (s)
    0   0.5    1.0
```

**Scaling for all joints:**
Each joint is scaled proportionally based on its total distance:
```python
joint_pos(t) = start_angle + (profile_progress(t) × total_joint_distance)
```

---

### STEP 7: Send Setpoints to ESP32

Python streams setpoints to ESP32 at the update rate (50ms intervals).

#### Binary Protocol (Recommended)

**Data packet structure:**
```c
struct MotionCommand {
    uint32_t timestamp_ms;      // 4 bytes - Time in milliseconds
    float joint_angles[3];      // 12 bytes - Target angles in degrees
    uint8_t checksum;           // 1 byte - Simple XOR checksum
} __attribute__((packed));      // Total: 17 bytes
```

**Packet layout:**
```
Byte 0-3:   Timestamp (uint32_t, little-endian)
Byte 4-7:   Joint 1 angle (float, little-endian)
Byte 8-11:  Joint 2 angle (float, little-endian)
Byte 12-15: Joint 3 angle (float, little-endian)
Byte 16:    Checksum (XOR of all previous bytes)
```

**Example packets sent:**
```
Timestamp: 0ms,   Joints: [0.0°, 45.0°, 30.0°]
Timestamp: 50ms,  Joints: [0.0°, 45.6°, 29.7°]
Timestamp: 100ms, Joints: [0.0°, 46.8°, 29.0°]
...
```

**Python sending:**
```python
import struct

def send_position(ser, timestamp_ms, angles):
    # Pack data: little-endian uint32 + 3 floats
    data = struct.pack('<I fff', timestamp_ms, angles[0], angles[1], angles[2])

    # Calculate checksum (XOR of all bytes)
    checksum = 0
    for byte in data:
        checksum ^= byte

    # Send data + checksum
    ser.write(data + bytes([checksum]))
```

**Communication considerations:**
- **Baud rate:** 115200 or higher (921600 recommended for USB)
- **Flow control:** Optional, but recommended for reliability
- **Buffering:** ESP32 should buffer 2-3 commands to handle jitter
- **Acknowledgment:** Optional - ESP32 can echo timestamp when complete
- **Error handling:** Checksum validation, timeout detection

---

### STEP 8: ESP32 Receives and Parses Data

ESP32 receives binary packets via UART and validates them.

**Receive packet:**
```c
#define PACKET_SIZE 17
uint8_t rx_buffer[PACKET_SIZE];

typedef struct {
    uint32_t timestamp_ms;
    float joint_angles[3];
    uint8_t checksum;
} __attribute__((packed)) MotionCommand;

bool receive_command(MotionCommand* cmd) {
    if (uart_read_bytes(UART_NUM_0, rx_buffer, PACKET_SIZE, 100 / portTICK_PERIOD_MS) == PACKET_SIZE) {

        // Verify checksum
        uint8_t calc_checksum = 0;
        for (int i = 0; i < PACKET_SIZE - 1; i++) {
            calc_checksum ^= rx_buffer[i];
        }

        if (calc_checksum != rx_buffer[PACKET_SIZE - 1]) {
            return false;  // Checksum error
        }

        // Copy data to command structure
        memcpy(cmd, rx_buffer, sizeof(MotionCommand));
        return true;
    }
    return false;
}
```

**Calculate step requirements:**
```c
// Motor configuration constants
#define STEPS_PER_REV 200.0f
#define MICROSTEPS 16.0f
#define GEAR_RATIO 10.0f
#define STEPS_PER_DEGREE ((STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0f)
// = 88.89 steps/degree

// Current positions
float current_angles[3] = {0.0, 0.0, 0.0};
int32_t current_steps[3] = {0, 0, 0};

void process_command(MotionCommand* cmd) {
    for (int i = 0; i < 3; i++) {
        // Calculate angle change
        float angle_delta = cmd->joint_angles[i] - current_angles[i];

        // Convert to steps
        int32_t steps_delta = (int32_t)roundf(angle_delta * STEPS_PER_DEGREE);

        // Determine direction
        bool direction = (steps_delta >= 0);

        // Calculate time available (until next update)
        uint32_t time_ms = 50;  // 50ms between updates

        // Move motor
        move_motor_rmt(i, abs(steps_delta), direction, time_ms);

        // Update current position
        current_angles[i] = cmd->joint_angles[i];
        current_steps[i] += steps_delta;
    }
}
```

---

### STEP 9: RMT Pulse Generation

The RMT (Remote Control Transceiver) module generates precise STEP pulses in hardware.

#### RMT Overview

**Key features:**
- 8 independent channels (control 8 motors)
- Hardware-based timing (no CPU jitter)
- Microsecond precision
- Non-blocking operation
- Configurable clock divider

**How RMT works:**
RMT generates sequences of pulse patterns. Each pattern (item) defines:
- **Level0**: First logic level (HIGH/LOW)
- **Duration0**: Duration in ticks
- **Level1**: Second logic level
- **Duration1**: Duration in ticks

**One stepper pulse:**
```
     ┌─────┐
HIGH │     │
     │     │
LOW  ┘     └─────
     ←─┬─→←──┬──→
      5μs   (period-5μs)
```

This is one RMT item: `{level0: HIGH, duration0: 5μs, level1: LOW, duration1: remaining}`

#### RMT Configuration

```c
#include "driver/rmt.h"
#include "driver/gpio.h"

#define RMT_TICK_PER_US 1  // 1 tick = 1 microsecond

// Pin definitions
#define STEP_PIN_0  GPIO_NUM_25
#define STEP_PIN_1  GPIO_NUM_26
#define STEP_PIN_2  GPIO_NUM_27
#define DIR_PIN_0   GPIO_NUM_32
#define DIR_PIN_1   GPIO_NUM_33
#define DIR_PIN_2   GPIO_NUM_14

const gpio_num_t step_pins[] = {STEP_PIN_0, STEP_PIN_1, STEP_PIN_2};
const gpio_num_t dir_pins[] = {DIR_PIN_0, DIR_PIN_1, DIR_PIN_2};

void setup_rmt(void) {
    for (int i = 0; i < 3; i++) {
        // Configure direction pins
        gpio_set_direction(dir_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(dir_pins[i], 0);

        // Configure RMT channel
        rmt_config_t config = {
            .rmt_mode = RMT_MODE_TX,
            .channel = (rmt_channel_t)i,
            .gpio_num = step_pins[i],
            .clk_div = 80,  // 80MHz / 80 = 1MHz (1 tick = 1μs)
            .mem_block_num = 1,
            .tx_config = {
                .carrier_en = false,
                .loop_en = false,
                .idle_level = RMT_IDLE_LEVEL_LOW,
                .idle_output_en = true,
            }
        };

        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    }
}
```

**Clock divider calculation:**
- ESP32 APB clock: 80 MHz
- Clock divider: 80
- RMT tick frequency: 80MHz / 80 = 1 MHz
- 1 tick = 1 microsecond

#### Generating Pulses

```c
#define PULSE_WIDTH_US 5  // Pulse width (check your driver datasheet)

void move_motor_rmt(int motor_index, uint32_t steps, bool direction, uint32_t time_ms) {
    if (steps == 0) return;

    // Set direction pin
    gpio_set_level(dir_pins[motor_index], direction ? 1 : 0);

    // Small delay for direction setup time (most drivers need 200ns-1μs)
    ets_delay_us(1);

    // Calculate pulse period (microseconds)
    uint32_t period_us = (time_ms * 1000) / steps;

    // Ensure period is achievable (RMT duration is 15-bit max: 32767)
    if (period_us > 30000) period_us = 30000;
    if (period_us < 10) period_us = 10;  // Minimum safe period

    // Build RMT items
    rmt_item32_t* items = (rmt_item32_t*)malloc(steps * sizeof(rmt_item32_t));

    for (uint32_t i = 0; i < steps; i++) {
        items[i].level0 = 1;                           // HIGH
        items[i].duration0 = PULSE_WIDTH_US;          // 5μs pulse
        items[i].level1 = 0;                          // LOW
        items[i].duration1 = period_us - PULSE_WIDTH_US;  // Remaining time
    }

    // Send to RMT (non-blocking)
    rmt_write_items((rmt_channel_t)motor_index, items, steps, false);

    free(items);
}
```

#### Timing Diagram: Complete System Flow

```
Python              UART               ESP32              RMT              Driver
  │                   │                  │                  │                 │
  │─ Binary packet ──→│                  │                  │                 │
  │  (17 bytes)       │─────────────────→│                  │                 │
  │                   │                  │                  │                 │
  │                   │                  │ Parse & validate │                 │
  │                   │                  │ Calculate steps  │                 │
  │                   │                  │ Δangle = 0.6°    │                 │
  │                   │                  │ steps = 53       │                 │
  │                   │                  │ period = 943μs   │                 │
  │                   │                  │                  │                 │
  │                   │                  │ Set DIR pin ────────────────────→ │
  │                   │                  │                  │                 │
  │                   │                  │─ RMT items ─────→│                 │
  │                   │                  │  (53 pulses)     │                 │
  │                   │                  │                  │                 │
  │                   │                  │                  │─ STEP pulse ──→│
  │                   │                  │                  │  (5μs HIGH)    │
  │                   │                  │                  │                 │
  │                   │                  │                  │─ 938μs LOW ───→│
  │                   │                  │                  │                 │
  │                   │                  │                  │─ STEP pulse ──→│
  │                   │                  │                  │                 │
  │                   │                  │                  │   (repeat 53x) │
  │                   │                  │                  │                 │
  │                   │                  │← Done ───────────│                 │
  │                   │                  │                  │                 │
  │                   │← ACK (optional) ─│                  │                 │
  │← ACK ─────────────│                  │                  │                 │
  │                   │                  │                  │                 │
```

#### Advanced: Streaming for Long Moves

For moves with thousands of steps, use memory-efficient streaming:

```c
#define RMT_BUFFER_SIZE 64

void move_motor_streaming(int motor_index, uint32_t total_steps, uint32_t period_us) {
    rmt_item32_t items[RMT_BUFFER_SIZE];

    // Pre-fill buffer with identical pulse patterns
    for (int i = 0; i < RMT_BUFFER_SIZE; i++) {
        items[i].level0 = 1;
        items[i].duration0 = PULSE_WIDTH_US;
        items[i].level1 = 0;
        items[i].duration1 = period_us - PULSE_WIDTH_US;
    }

    uint32_t steps_sent = 0;
    while (steps_sent < total_steps) {
        uint32_t chunk_size = (total_steps - steps_sent > RMT_BUFFER_SIZE)
                              ? RMT_BUFFER_SIZE
                              : (total_steps - steps_sent);

        // Wait for previous chunk to finish
        rmt_wait_tx_done((rmt_channel_t)motor_index, portMAX_DELAY);

        // Send next chunk
        rmt_write_items((rmt_channel_t)motor_index, items, chunk_size, false);

        steps_sent += chunk_size;
    }
}
```

#### Key RMT Considerations

**1. Timing Limits:**
```
Min pulse width:     1-5μs (check driver datasheet)
Max pulse period:    32.767ms (15-bit duration @ 1μs ticks)
Min practical period: ~10μs (100kHz max pulse rate)
```

**2. Memory Constraints:**
```
Total RMT memory: 512 x 32-bit items (shared across 8 channels)
Default per channel: 64 items
Expandable to: 128 items (mem_block_num = 2)
```

**3. Direction Setup Time:**
Most stepper drivers require 200ns-1μs between DIR change and first STEP pulse.
```c
gpio_set_level(dir_pin, direction);
ets_delay_us(1);  // Wait 1μs
// Now safe to send STEP pulses
```

**4. Simultaneous Multi-Axis Motion:**
```c
// Start all motors "simultaneously" (within microseconds of each other)
for (int i = 0; i < 3; i++) {
    move_motor_rmt(i, steps[i], directions[i], time_ms);
}
// All RMT channels run in parallel (hardware-timed)
```

---

## Key Design Decisions

### Why Python Does Motion Planning (Option 1)

**Advantages:**
- More computational power for complex trajectory planning
- Easy to visualize and debug (plot graphs, log data)
- Can use scientific libraries (NumPy, SciPy)
- Sophisticated path planning (obstacle avoidance, optimization)
- All joints coordinated easily

**Disadvantages:**
- Requires reliable communication
- Communication latency must be handled
- Less autonomous (ESP32 depends on PC)

**Alternative (Option 2):** ESP32 does motion profiling
- Python sends: target angles + velocity/acceleration limits
- ESP32 calculates profile in real-time
- More autonomous, but harder to coordinate multi-axis moves

---

## Important IK Considerations

### Multiple Solutions
Most robot arms have multiple IK solutions for the same XYZ position:
- Elbow up / elbow down
- Wrist flipped / not flipped
- Different shoulder configurations

**Solution:** Always choose the configuration closest to the current joint angles to avoid sudden jumps.

### IK May Fail
Some points may be:
- **Out of reach** (beyond workspace)
- **In singularity** (infinite solutions or no solution)
- **Causing self-collision**
- **Violating joint limits**

**Validation checklist:**
```python
for xyz_point in interpolated_path:
    angles = inverse_kinematics(xyz_point)

    if angles is None:
        # IK failed
        return "Path not feasible"

    if not within_joint_limits(angles):
        return "Joint limits exceeded"

    if self_collision(angles):
        return "Self-collision detected"

    if config_flip(angles, previous_angles):
        return "Configuration discontinuity"

    waypoints.append(angles)
```

### Interpolation Density

**Trade-off:** More points = straighter path + more computation

| Spacing | Points (for 82mm path) | Path Quality | IK Calls |
|---------|------------------------|--------------|----------|
| 50mm    | 2                      | May curve    | 2        |
| 20mm    | 5                      | Good         | 5        |
| 10mm    | 9                      | Very good    | 9        |
| 5mm     | 17                     | Excellent    | 17       |
| 1mm     | 83                     | Overkill     | 83       |

**Recommendation:** Start with 5-10mm, adjust based on testing.

---

## Motion Profile Types

### Trapezoidal Profile
```
Velocity
    /‾‾‾\
   /     \
  /       \
 /         \
─────────────
Accel│Cruise│Decel
```
- Constant acceleration
- Simple to calculate
- Smooth position, discontinuous acceleration
- Good for most applications

### Triangular Profile
```
Velocity
    /\
   /  \
  /    \
 /      \
──────────
Accel│Decel
```
- Short moves (don't reach max velocity)
- No cruise phase
- Still smooth

### S-Curve Profile (Advanced)
```
Velocity
   _/‾\_
 _/     \_
/         \
```
- Smooth acceleration (jerk-limited)
- Better for high-speed moves
- More complex calculation
- Reduces vibration

---

## Update Rate Selection

| Rate  | Period | Use Case                        | CPU Load  |
|-------|--------|---------------------------------|-----------|
| 10 Hz | 100ms  | Slow moves, low precision       | Very low  |
| 20 Hz | 50ms   | General purpose (recommended)   | Low       |
| 50 Hz | 20ms   | High precision                  | Medium    |
| 100Hz | 10ms   | Very high precision             | High      |
| 200Hz | 5ms    | Extreme precision, fast moves   | Very high |

**Recommendation:** 20-50 Hz for most applications

---

## Implementation Checklist

- [ ] Implement forward kinematics
- [ ] Implement inverse kinematics with solution selection
- [ ] Create Cartesian path interpolation
- [ ] Implement trapezoidal velocity profile generator
- [ ] Create setpoint generation with configurable update rate
- [ ] Implement communication protocol (Python ↔ ESP32)
- [ ] ESP32: Angle to step conversion
- [ ] ESP32: RMT pulse generation
- [ ] Add joint limit checking
- [ ] Add workspace validation
- [ ] Implement error handling and recovery
- [ ] Add visualization/debugging tools
- [ ] Test with simple moves
- [ ] Test with complex paths
- [ ] Tune acceleration/velocity limits

---

## System Parameters

### Motion Limits (Example - adjust for your robot)
```
Max joint velocities:    [180°/s, 180°/s, 180°/s]
Max joint accelerations: [360°/s², 360°/s², 360°/s²]
Joint limits (min):      [-180°, -90°, -90°]
Joint limits (max):      [180°, 90°, 90°]
```

### Motor Configuration
```
Steps per revolution: 200 (1.8° stepper)
Microstepping:        16
Gear ratio:          10:1
Total steps/degree:  (200 × 16 × 10) / 360 = 88.89
```

### Communication
```
Update rate:         20 Hz (50ms)
Baud rate:          115200
Protocol:           Binary or JSON
```

---

## Implementation Files

This repository includes complete working implementations:

### Python Motion Controller

Located in `python/motion_controller.py`

**Features:**
- Binary protocol implementation with checksums
- Trajectory generation and execution
- Example test trajectories (linear moves, circular motion)
- Easy-to-use API for sending position commands

**Usage:**
```bash
cd python
pip install -r requirements.txt
python motion_controller.py
```

**Configuration:**
Edit the `PORT` and `BAUDRATE` variables in `main()` to match your setup.

### ESP32-IDF Firmware

Located in `esp32-idf/`

**Features:**
- Binary packet reception with checksum validation
- RMT-based stepper pulse generation
- Multi-axis coordinated motion
- Non-blocking hardware-timed pulses
- Detailed logging and statistics

**Building:**
```bash
cd esp32-idf
. $HOME/esp/esp-idf/export.sh  # Setup ESP-IDF environment
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

See `esp32-idf/README.md` for detailed setup and configuration instructions.

---

## Quick Start Guide

### 1. Hardware Setup

Connect stepper motors to drivers (STEP/DIR interface):

**For ESP32-S3 (default):**
- Motor 0: STEP → GPIO1, DIR → GPIO4
- Motor 1: STEP → GPIO2, DIR → GPIO5
- Motor 2: STEP → GPIO3, DIR → GPIO6

**For classic ESP32:**
- Motor 0: STEP → GPIO25, DIR → GPIO32
- Motor 1: STEP → GPIO26, DIR → GPIO33
- Motor 2: STEP → GPIO27, DIR → GPIO14

*Note: Pins can be changed in `esp32-idf/main/main.c`*

### 2. Flash ESP32

```bash
cd esp32-idf
idf.py -p /dev/ttyUSB0 flash monitor
```

### 3. Run Python Controller

In a new terminal:
```bash
cd python
python motion_controller.py
```

You should see the motors move through the test trajectories!

---

## Next Steps

1. **Implement kinematics** - Forward and inverse for your specific robot
2. **Test IK** - Verify solutions for your workspace
3. **Build path planner** - Implement interpolation and profiling
4. **Integration** - Connect kinematics to motion controller
5. **Tuning** - Optimize velocities, accelerations, update rate
6. **Advanced features** - Add limit switches, homing, error recovery

---

## References

- Trapezoidal motion profile: Standard motion control technique
- Inverse kinematics: Robot-specific (implement based on your arm's geometry)
- ESP32 RMT: [ESP-IDF RMT Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
- Stepper control: Step/Direction interface standard

---

## License

[Add your license here]

## Author

[Add your name/info here]
