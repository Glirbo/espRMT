# 6-Motor Configuration Summary

This document summarizes the changes made to support 6 motors instead of 3.

## Hardware Changes

### GPIO Pin Assignments (ESP32-WROOM)

| Motor | STEP Pin | DIR Pin | RMT Channel |
|-------|----------|---------|-------------|
| 0     | GPIO25   | GPIO32  | CH0         |
| 1     | GPIO26   | GPIO33  | CH1         |
| 2     | GPIO27   | GPIO14  | CH2         |
| 3     | GPIO12   | GPIO16  | CH3         |
| 4     | GPIO13   | GPIO17  | CH4         |
| 5     | GPIO15   | GPIO18  | CH5         |

**Additional GPIO:**
- GPIO21: 3.3V power output (always HIGH, max 40mA)

## Communication Protocol Changes

### Binary Packet Format

**Old (3 motors): 17 bytes**
```
[4 bytes timestamp][12 bytes: 3×float][1 byte checksum]
```

**New (6 motors): 29 bytes**
```
[4 bytes timestamp][24 bytes: 6×float][1 byte checksum]
```

### Position Query Response

**Old format (3 motors):**
```
POS:j1,j2,j3\n
```

**New format (6 motors):**
```
POS:j1,j2,j3,j4,j5,j6\n
```

**Important:** Position query logging has been disabled on ESP32-WROOM to prevent UART interference. The response is sent as clean ASCII data with no log prefix, ensuring reliable parsing by the Python controller.

## Memory Usage

### RMT Buffer Allocation

**Configuration:**
- Buffer size per motor: 3,072 items (reduced from 16,384 for ESP32-WROOM)
- Buffer capacity: ~5.5° per 50ms update
- Total motors: 6

**Memory calculation:**
```
6 motors × 3,072 items × 4 bytes/item = 73,728 bytes (~74KB)
```

**ESP32-WROOM SRAM:**
- Total available: 520KB
- RMT buffers: ~74KB
- Remaining: ~446KB (for system, stack, heap, networking disabled)

### Comparison to 3-Motor Version

| Metric | 3 Motors (S3) | 6 Motors (WROOM) |
|--------|---------------|-------------------|
| Packet size | 17 bytes | 29 bytes |
| RMT buffer/motor | 16,384 items | 3,072 items |
| Total RMT memory | ~192KB | ~74KB |
| Buffer capacity | ~29.5° | ~5.5° |
| Number of channels | 3 | 6 |

**Note:** Buffer size was reduced for WROOM due to limited SRAM, but 5.5° per update is sufficient for smooth motion at 30°/sec (max 1.5° per 50ms update).

## Software Changes

### ESP32 Firmware (main.c)

**Modified constants:**
```c
#define NUM_MOTORS      6
#define PACKET_SIZE     29  // was 17
```

**Added GPIO definitions:**
```c
#define STEP_PIN_3      GPIO_NUM_12
#define STEP_PIN_4      GPIO_NUM_13
#define STEP_PIN_5      GPIO_NUM_15
#define DIR_PIN_3       GPIO_NUM_16
#define DIR_PIN_4       GPIO_NUM_17
#define DIR_PIN_5       GPIO_NUM_18
```

**Updated arrays:**
```c
static float current_angles[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static int32_t current_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
```

### Python Controller (motion_controller.py)

**send_position() function:**
```python
# Old: struct.pack('<I fff', ...)
# New:
struct.pack('<I ffffff', timestamp_ms,
           angles[0], angles[1], angles[2],
           angles[3], angles[4], angles[5])
```

**get_current_position():**
- Now expects 6 values from position query
- Returns list of 6 floats instead of 3

**Trajectory generation:**
```python
# Old: angles = [angle, angle, angle]
# New:
angles = [angle, angle, angle, angle, angle, angle]
```

### Python Simple Example (simple_example.py)

**Updated to send 6-motor commands:**
```python
send_position(ser, timestamp_ms=0, angles=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
send_position(ser, timestamp_ms=1000, angles=[45.0, 45.0, 45.0, 45.0, 45.0, 45.0])
```

## Testing the Configuration

### Verify Pin Configuration

After flashing, check the startup banner shows all 6 motors:

```
RMT CONFIGURATION
✓ Motor 0 configured: STEP pin: GPIO25  │  DIR pin: GPIO32  │  RMT: CH0
✓ Motor 1 configured: STEP pin: GPIO26  │  DIR pin: GPIO33  │  RMT: CH1
✓ Motor 2 configured: STEP pin: GPIO27  │  DIR pin: GPIO14  │  RMT: CH2
✓ Motor 3 configured: STEP pin: GPIO12  │  DIR pin: GPIO16  │  RMT: CH3
✓ Motor 4 configured: STEP pin: GPIO13  │  DIR pin: GPIO17  │  RMT: CH4
✓ Motor 5 configured: STEP pin: GPIO15  │  DIR pin: GPIO18  │  RMT: CH5
```

### Test Position Query

Use Python to query position:
```python
from motion_controller import MotionController
controller = MotionController()
pos = controller.get_current_position()
print(pos)  # Should show list of 6 angles
```

Expected output:
```
[0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
```

### Test Motion Commands

Run the simple example:
```bash
cd python
python3 simple_example.py
```

All 6 motors should move together.

## Wiring Guide

### Stepper Driver Connections

For each motor (0-5), connect to stepper driver:
- ESP32 STEP pin → Driver STEP/PULSE input
- ESP32 DIR pin → Driver DIR/DIRECTION input
- ESP32 GND → Driver GND
- Driver 5V/3.3V → ESP32 3.3V (if needed for logic level)

### Power Considerations

- ESP32 GPIO can source max 40mA per pin
- Stepper drivers should have their own power supply
- ESP32 GPIO pins only control driver logic (low current)
- Never power stepper motors directly from ESP32

### GPIO21 Usage (3.3V Output)

- Max current: 40mA
- Suitable for: sensors, pull-ups, logic reference
- NOT suitable for: motor drivers, high-current loads

## Troubleshooting

### Build Errors

If build fails with memory errors:
1. Reduce `MAX_RMT_ITEMS` in main.c (currently 3072)
2. Ensure networking components are excluded in CMakeLists.txt
3. Run `idf.py fullclean` before rebuilding

### Position Query Returns Wrong Number of Values

Check firmware version matches Python controller:
- Firmware should send 6 comma-separated values
- Python expects exactly 6 values
- Both must be updated together

### Position Query Reliability Issues

**ESP32-WROOM specific:**
- Position query logging is **disabled** to prevent UART interference
- Response format is clean: `POS:j1,j2,j3,j4,j5,j6\n`
- If queries still fail, check for other log messages on UART0
- Enable DEBUG logging only when debugging (adds UART traffic)

**To re-enable position query logging (for debugging only):**
```c
// In main.c, send_current_position():
ESP_LOGD(TAG, "Position query: sent [%.2f°, ...]", ...);
```
Then enable DEBUG level logging via `idf.py menuconfig`

### Motors Don't Move

1. Check wiring matches pin assignments above
2. Verify stepper drivers are powered correctly
3. Check ESP32 logs for velocity warnings
4. Use `simple_example.py` to test basic functionality

## Performance Notes

**Maximum Movement per Update:**
- Buffer capacity: 5.5° per motor per 50ms update
- Velocity limit: 30°/sec = 1.5° per 50ms
- Safety margin: 3.6× (plenty of headroom)

**If you need larger moves:**
- Increase `MAX_RMT_ITEMS` (uses more RAM)
- Or split large moves into multiple trajectory points
- Current configuration is optimized for ESP32-WROOM's limited SRAM

## Files Modified

### ESP32 Firmware
- `esp32-idf-wroom/main/main.c` - All motor control logic

### Python Controller
- `python/motion_controller.py` - Main controller with 6-motor support
- `python/simple_example.py` - Simple example updated for 6 motors

### Documentation
- `esp32-idf-wroom/README_WROOM.md` - Updated pin assignments and memory info
- `esp32-idf-wroom/6_MOTOR_CONFIG.md` - This file

## Migration from 3-Motor Setup

If upgrading from 3-motor configuration:

1. **Flash new firmware** to ESP32
2. **Update Python scripts** (both files changed)
3. **Wire 3 additional motors** to new GPIO pins
4. **Test with simple_example.py** first
5. **Verify position queries** return 6 values

Old 3-motor trajectories will NOT work - must update to send 6 angles.
