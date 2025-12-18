# ESP32-WROOM V2 - Per-Motor Gear Ratio Support

This is an enhanced version of the ESP32-WROOM firmware with per-motor gear ratio configuration and increased maximum velocity.

## Changes from V1

### 1. Per-Motor Gear Ratios
Instead of a single gear ratio for all motors, V2 supports individual gear ratios per motor:

```c
// Motor configuration
Motor 0: 1:10 gearing → 111.11 steps/degree
Motor 1: 1:50 gearing → 555.56 steps/degree
Motor 2: 1:50 gearing → 555.56 steps/degree
Motor 3: 1:50 gearing → 555.56 steps/degree
Motor 4: 1:50 gearing → 555.56 steps/degree
Motor 5: 1:10 gearing → 111.11 steps/degree
```

**Why per-motor ratios?**
- Base/shoulder joints (0,5) typically need less gearing for faster rotation
- Elbow/wrist joints (1-4) need higher gearing for precision and torque
- Matches typical robotic arm mechanical design

### 2. Increased Maximum Velocity
- **V1**: 30°/sec maximum angular velocity
- **V2**: 60°/sec maximum angular velocity

This doubles the speed capability while maintaining:
- Hardware pulse timing: 30μs minimum period (was 60μs)
- Step rate for 1:50 motors: 33,333 steps/sec (was 16,667)
- Step rate for 1:10 motors: 6,667 steps/sec
- RMT module easily handles these rates

### 3. Updated Buffer Capacity Calculations

The 3,072-item buffer now accommodates different motor types:

**High-gearing motors (1-4) with 1:50:**
- 555.56 steps/degree
- At 60°/sec: 3.0° per 50ms update = 1,667 steps needed
- Buffer capacity: 5.5° max (3,072 steps)
- Safety margin: 1.84× ✓

**Low-gearing motors (0,5) with 1:10:**
- 111.11 steps/degree
- At 60°/sec: 3.0° per 50ms update = 333 steps needed
- Buffer capacity: 27.6° max (3,072 steps)
- Safety margin: 9.2× ✓✓

Both motor types have adequate buffer capacity for 60°/sec operation.

## Motor Configuration

To customize gear ratios for your robot, edit `main/main.c` lines 58-75:

```c
// Per-motor gear ratios
static const float motor_gear_ratios[NUM_MOTORS] = {
    10.0f,  // Motor 0
    50.0f,  // Motor 1
    50.0f,  // Motor 2
    50.0f,  // Motor 3
    50.0f,  // Motor 4
    10.0f   // Motor 5
};

// Per-motor steps per degree (STEPS_PER_REV * gear_ratio / 360)
static const float steps_per_degree[NUM_MOTORS] = {
    111.11f,  // Motor 0: (4000 * 10) / 360
    555.56f,  // Motor 1: (4000 * 50) / 360
    555.56f,  // Motor 2: (4000 * 50) / 360
    555.56f,  // Motor 3: (4000 * 50) / 360
    555.56f,  // Motor 4: (4000 * 50) / 360
    111.11f   // Motor 5: (4000 * 10) / 360
};
```

**Important:** When changing gear ratios:
1. Update both `motor_gear_ratios[]` and `steps_per_degree[]` arrays
2. Recalculate `MIN_PERIOD_US` based on the highest steps/degree (line 82)
3. Verify buffer capacity is adequate for your configuration

## Technical Details

### Velocity Limiting
The system enforces a 30μs minimum pulse period globally, calculated from the motor with the highest step rate:
```
MIN_PERIOD_US = 1,000,000μs / (60°/s × 555.56 steps/°) = 30μs
```

This ensures all motors respect the 60°/sec velocity limit, even though low-gearing motors could theoretically move faster.

### Per-Motor Step Rate Calculations
The firmware automatically uses the correct `steps_per_degree[i]` for each motor:
- Angle-to-steps conversion: `steps = angle × steps_per_degree[motor_index]`
- Velocity validation: `velocity = step_rate / steps_per_degree[motor_index]`

No manual step rate adjustments needed - it's automatic!

## Building and Flashing

```bash
# Setup ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Navigate to project directory
cd esp32-idf-wroom-v2

# Build
idf.py build

# Flash to ESP32-WROOM
idf.py -p /dev/ttyUSB0 flash monitor
```

## Startup Output

When the firmware boots, you'll see detailed per-motor configuration:

```
ESP32-WROOM ROBOTIC ARM CONTROLLER v2.0
Per-Motor Gearing │ 60°/sec Max Velocity

Motor Configuration (V2 - Per-Motor Gearing):
  Steps per revolution: 4000 (no microstepping)
  Per-motor gear ratios and steps/degree:
    Motor 0: 1:10 gearing │ 111.11 steps/° │ max 6667 steps/s @ 60°/s
    Motor 1: 1:50 gearing │ 555.56 steps/° │ max 33333 steps/s @ 60°/s
    Motor 2: 1:50 gearing │ 555.56 steps/° │ max 33333 steps/s @ 60°/s
    Motor 3: 1:50 gearing │ 555.56 steps/° │ max 33333 steps/s @ 60°/s
    Motor 4: 1:50 gearing │ 555.56 steps/° │ max 33333 steps/s @ 60°/s
    Motor 5: 1:10 gearing │ 111.11 steps/° │ max 6667 steps/s @ 60°/s

Motion Limits:
  Max angular velocity: 60.0°/sec (all motors)
  Max step rate:        33,333 steps/sec (motors 1-4)
  Max step rate:        6,667 steps/sec (motors 0,5)
  Min pulse period:     30μs (based on highest step rate)
  Buffer size:          3072 items per motor
    Motors 1-4 (1:50): 5.5° max in 50ms
    Motors 0,5 (1:10): 27.6° max in 50ms
```

## Python Controller Compatibility

**Use the `python-v2` folder** - All scripts have been updated for 60°/sec:

### Updated Scripts in python-v2/:
✅ **motor_test_sequences.py** - Interactive menu system
  - Select individual tests (1-4) or run all tests (5)
  - Updated for 60°/sec base velocity
  - Safety checks and home position verification
  - Repeat testing capability

✅ **motion_controller.py** - Core library updated to 60°/sec
✅ **inverse_kinematics_controller.py** - IK system updated
✅ **inverse_kinematics_controller_gui.py** - GUI updated
✅ **simple_example.py** - Compatible with V2 firmware

### Quick Start:
```bash
cd python-v2
pip install -r requirements.txt
python motor_test_sequences.py  # Interactive menu (recommended)
```

### If Using Original python/ Folder:
You'll need to manually update velocity limits from 30°/sec to 60°/sec:
1. `motion_controller.py`: Lines 211, 261
2. `motor_test_sequences.py`: Line 32
3. All `validate_trajectory_velocity()` calls

## Hardware Considerations

Before using 60°/sec in production:

1. **Stepper driver capability**: Verify drivers can handle 33kHz step rate
2. **Motor torque**: Check motors maintain adequate torque at higher speeds
3. **Power supply**: Ensure sufficient current for faster operation
4. **Mechanical resonance**: Test for vibration at higher speeds
5. **Acceleration limits**: May need to adjust ramp rates

Start testing at 30°/sec and gradually increase to 60°/sec while monitoring performance.

## Migration from V1

If you're upgrading from V1:

1. **Firmware**: Flash this V2 firmware to your ESP32
2. **Python**: Update velocity limits as described above
3. **Testing**: Start with 30°/sec to verify operation, then increase to 60°/sec
4. **Calibration**: Verify motor directions and home positions haven't changed

## License

Same as main project license.
