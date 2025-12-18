# send_pulses.py - Direct Step Control

Simple script for sending a specific number of pulses (steps) to a single motor.

## Quick Start

```bash
python send_pulses.py
```

## Usage

The script will prompt you for:
1. **Motor number** (0-5)
2. **Number of pulses/steps** to send
3. **Direction** (forward or backward)

### Example Session

```
============================================================
Send Fixed Pulses to Motor
============================================================
✓ Connected to /dev/ttyACM0 @ 115200 baud

Querying current motor positions...
Current positions:
  Motor 0:    0.00° (      0 steps)
  Motor 1:    0.00° (      0 steps)
  Motor 2:    0.00° (      0 steps)
  Motor 3:    0.00° (      0 steps)
  Motor 4:    0.00° (      0 steps)
  Motor 5:    0.00° (      0 steps)

============================================================
Enter motor and pulse information:
============================================================
Motor number (0-5): 1
Number of pulses/steps: 40000
Direction (f=forward, b=backward) [f]: f

============================================================
Movement Summary:
============================================================
Motor:          1
Pulses/Steps:   40000 (forward)
Gear ratio:     1:50
Angle change:   +71.99°
Current angle:  0.00°
Target angle:   71.99°
Duration:       1.2 seconds
============================================================

Execute movement? (yes/no): yes

Starting in 3 seconds (Ctrl+C to abort)...
  3...
  2...
  1...

Sending command...
✓ Command sent at t=1234567ms
Waiting 1.2 seconds for movement to complete...

Querying final position...
Motor 1: 71.99° (target was 71.99°)
Position error: +0.00°

✓ Done!
```

## Motor Configuration (V2)

The script automatically accounts for per-motor gear ratios:

| Motor | Gearing | Steps/Degree | Example: 40000 steps |
|-------|---------|--------------|----------------------|
| 0     | 1:10    | 111.11       | 360.00° (1 rotation) |
| 1     | 1:50    | 555.56       | 71.99°               |
| 2     | 1:50    | 555.56       | 71.99°               |
| 3     | 1:50    | 555.56       | 71.99°               |
| 4     | 1:50    | 555.56       | 71.99°               |
| 5     | 1:10    | 111.11       | 360.00° (1 rotation) |

## Use Cases

- **Testing individual motors** during hardware setup
- **Precise step control** for calibration
- **Manual positioning** to specific step counts
- **Debugging motor behavior** without writing code

## Features

✅ **Interactive prompts** - No command-line arguments needed
✅ **Current position query** - Shows where motors are before moving
✅ **Automatic angle calculation** - Converts steps to degrees based on gearing
✅ **Safety confirmation** - Asks "yes/no" before executing
✅ **3-second countdown** - Time to abort with Ctrl+C
✅ **Position verification** - Queries final position after movement
✅ **Error reporting** - Shows position error in degrees

## Configuration

The script automatically tries these ports in order:
1. `/dev/ttyACM0` - ESP32-S3 native USB
2. `/dev/ttyUSB0` - ESP32 with USB-UART adapter
3. `COM3` - Windows (edit as needed)

**Example output:**
```
Trying /dev/ttyACM0...
✗ /dev/ttyACM0: [Errno 2] No such file or directory
Trying /dev/ttyUSB0...
✓ Connected to /dev/ttyUSB0 @ 115200 baud
```

Edit the script to change:
- **PORTS**: List of ports to try (add/remove as needed)
- **BAUDRATE**: Baud rate (default: `115200`)
- **Max velocity**: Movement speed (default: `60°/sec`)

## Tips

**Forward vs Backward:**
- `f` or `forward` = positive rotation
- `b` or `backward` = negative rotation

**Large movements:**
Duration is calculated automatically based on 60°/sec velocity:
- 40000 steps on motor 1 (1:50) = 71.99° = ~1.2 seconds
- 40000 steps on motor 0 (1:10) = 360° = ~6.0 seconds

**Position tracking:**
The script queries current position before moving, so you can:
- Run multiple times to increment position
- Chain movements together
- Always know where you are

**Safety:**
- Motors must be at safe position before large moves
- System uses open-loop control (no encoders)
- Always verify final position matches expected

## Example Commands

**Move motor 1 forward 10000 steps:**
```
Motor number: 1
Number of pulses: 10000
Direction: f
Result: +18.00°
```

**Move motor 0 backward one full rotation (4000 steps with 1:10 gearing = 36°):**
```
Motor number: 0
Number of pulses: 4000
Direction: b
Result: -36.00°
```

**Test motor 3 with small movement (1000 steps):**
```
Motor number: 3
Number of pulses: 1000
Direction: f
Result: +1.80°
```

## Troubleshooting

**"Failed to connect to any port"**
The script automatically tries `/dev/ttyACM0`, `/dev/ttyUSB0`, and `COM3`.

If all fail:
- Check USB cable is connected
- Verify ESP32 is powered on
- On Linux: Run `ls /dev/tty* | grep -E 'ACM|USB'` to find your port
- On Windows: Check Device Manager for COM port number
- Add your port to the `PORTS` list in the script

**"Could not query position"**
- ESP32 firmware must support "POS" position query
- Check USB cable quality
- Try increasing timeout in script

**Motor doesn't move expected distance**
- Verify gear ratio matches your hardware
- Check STEPS_PER_DEGREE configuration
- System uses open-loop control - large errors indicate mechanical issues

## Related Scripts

- **motor_test_sequences.py** - Interactive menu with predefined test patterns
- **motion_controller.py** - Core library with trajectory generation
- **simple_example.py** - Minimal example of basic usage
