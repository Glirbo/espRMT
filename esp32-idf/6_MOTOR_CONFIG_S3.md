# ESP32-S3 6-Motor Configuration

This document summarizes the ESP32-S3 specific configuration for 6 motors.

## GPIO Pin Assignments (ESP32-S3)

| Motor | STEP Pin | DIR Pin | RMT Channel |
|-------|----------|---------|-------------|
| 0     | GPIO1    | GPIO8   | CH0         |
| 1     | GPIO2    | GPIO9   | CH1         |
| 2     | GPIO3    | GPIO10  | CH2         |
| 3     | GPIO5    | GPIO11  | CH3         |
| 4     | GPIO6    | GPIO12  | CH4         |
| 5     | GPIO7    | GPIO13  | CH5         |

**Note:** GPIO4 is skipped (often used for other peripherals on ESP32-S3)

## Key Advantages over WROOM

### Separate USB-JTAG Console

**ESP32-S3 Benefit:**
- UART0: Clean data channel (motion commands + position queries)
- USB-JTAG: Logging output (ESP_LOGI, ESP_LOGW, ESP_LOGE)
- **No UART interference** - logs don't corrupt data packets

**Position query logging:**
```c
ESP_LOGI(TAG, "Position query: sent [%.2f°, ...]", ...);
```
This is **enabled** on S3 (safe due to separate console) but **disabled** on WROOM.

### Memory Configuration

**RMT Buffer Configuration:**
- Buffer size per motor: 3,072 items (reduced to fit in SRAM with networking stack)
- Total: 6 motors × 3,072 items × 4 bytes = **~74KB**
- ESP32-S3 has 512KB SRAM (similar to WROOM's 520KB)

**Buffer capacity:**
- **Maximum move per update**: ~5.5° in 50ms
- **Safety margin**: 3.6× (max needed is 1.5° per update at 30°/s)
- **What this means**: You cannot move a single motor more than 5.5° in one 50ms command
- **Why it's sufficient**: At 30°/sec max velocity, you only need 1.5° per update
- **Protection**: Python validator and ESP32 buffer clamp prevent overflows

**Example operation at max velocity:**
```
t=0ms:   Motor at 0°
t=50ms:  Motor at 1.5°   ✅ Fits in buffer (3.6× safety margin)
t=100ms: Motor at 3.0°   ✅ Smooth motion continues
t=150ms: Motor at 4.5°   ✅ No buffer overflow
```

**Edge case (prevented by system):**
```
t=0ms:   Motor at 0°
t=50ms:  Motor at 10°   ❌ Would require 200°/sec (exceeds limits)
                        → Python validator rejects trajectory
                        → ESP32 clamps to 5.5° if received
```

## Communication Protocol

**Binary Packet: 29 bytes**
```
[4 bytes timestamp][24 bytes: 6×float][1 byte checksum]
```

**Position Query Response:**
```
POS:j1,j2,j3,j4,j5,j6\n
```

**Logging Output (visible on USB-JTAG):**
```
I (12345) RMTArm: Position query: sent [0.00°, 0.00°, 0.00°, 0.00°, 0.00°, 0.00°]
```
This appears on USB-JTAG console, NOT on UART0, so it doesn't interfere with data.

## Build Commands

```bash
# Setup ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Navigate to S3 firmware directory
cd esp32-idf

# Build
idf.py build

# Flash to ESP32-S3 (native USB)
idf.py -p /dev/ttyACM0 flash

# Monitor logs on USB-JTAG while Python uses UART0
idf.py -p /dev/ttyACM0 monitor
```

## Monitoring Both Channels Simultaneously

**This is the BIG advantage of ESP32-S3:**

**Terminal 1: Monitor ESP32 logs (USB-JTAG)**
```bash
idf.py -p /dev/ttyACM0 monitor
```

**Terminal 2: Run Python controller (UART0)**
```bash
cd ../python
python3 motion_controller.py
```

Both can run **at the same time** without conflict!

## Python Controller Usage

No changes needed - same code works for both S3 and WROOM:

```python
from motion_controller import MotionController

# Auto-detects /dev/ttyACM0 (S3) or /dev/ttyUSB0 (WROOM)
controller = MotionController()

# Or specify explicitly
controller = MotionController('/dev/ttyACM0', 115200)
```

## Comparison: S3 vs WROOM

| Feature | ESP32-S3 | ESP32-WROOM |
|---------|----------|-------------|
| Motors | 6 | 6 |
| STEP pins | GPIO1-3,5-7 | GPIO12-13,15,25-27 |
| DIR pins | GPIO8-13 | GPIO14,16-18,32-33 |
| UART channels | UART0 + USB-JTAG | UART0 only |
| Position logging | Enabled (safe) | Disabled (interference) |
| RMT buffer/motor | 3,072 items | 3,072 items |
| Total RMT memory | ~74KB | ~74KB |
| Buffer capacity | ~5.5° | ~5.5° |
| SRAM available | 512KB | 520KB |
| Debug logging | Safe anytime | Causes interference |
| Monitor + Python | Simultaneous | Sequential only |
| Port | /dev/ttyACM0 | /dev/ttyUSB0 |

## Wiring Guide

### ESP32-S3 DevKit Connections

For each motor (0-5), connect to stepper driver:

**Motor 0:**
- ESP32-S3 GPIO1 → Driver STEP
- ESP32-S3 GPIO8 → Driver DIR

**Motor 1:**
- ESP32-S3 GPIO2 → Driver STEP
- ESP32-S3 GPIO9 → Driver DIR

**Motor 2:**
- ESP32-S3 GPIO3 → Driver STEP
- ESP32-S3 GPIO10 → Driver DIR

**Motor 3:**
- ESP32-S3 GPIO5 → Driver STEP
- ESP32-S3 GPIO11 → Driver DIR

**Motor 4:**
- ESP32-S3 GPIO6 → Driver STEP
- ESP32-S3 GPIO12 → Driver DIR

**Motor 5:**
- ESP32-S3 GPIO7 → Driver STEP
- ESP32-S3 GPIO13 → Driver DIR

**Power:**
- ESP32 GND → Drivers GND (common ground)
- Stepper drivers should have separate power supply
- ESP32 pins only control logic (low current)

## Testing

### Verify Startup

After flashing, you should see on USB-JTAG console:

```
╔════════════════════════════════════════════════════════╗
║                                                        ║
║       ESP32 ROBOTIC ARM CONTROLLER v1.0                ║
║                  (6 MOTORS)                            ║
║                                                        ║
╚════════════════════════════════════════════════════════╝

RMT CONFIGURATION
✓ Motor 0 configured: STEP pin: GPIO1  │  DIR pin: GPIO8   │  RMT: CH0
✓ Motor 1 configured: STEP pin: GPIO2  │  DIR pin: GPIO9   │  RMT: CH1
✓ Motor 2 configured: STEP pin: GPIO3  │  DIR pin: GPIO10  │  RMT: CH2
✓ Motor 3 configured: STEP pin: GPIO5  │  DIR pin: GPIO11  │  RMT: CH3
✓ Motor 4 configured: STEP pin: GPIO6  │  DIR pin: GPIO12  │  RMT: CH4
✓ Motor 5 configured: STEP pin: GPIO7  │  DIR pin: GPIO13  │  RMT: CH5

SYSTEM READY
```

### Test with Python

```bash
cd python
python3 simple_example.py
```

All 6 motors should move together.

### Verify Position Queries

In Python:
```python
from motion_controller import MotionController
controller = MotionController('/dev/ttyACM0')
pos = controller.get_current_position()
print(pos)  # Should show [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

On USB-JTAG console, you'll see:
```
I (12345) RMTArm: Position query: sent [0.00°, 0.00°, 0.00°, 0.00°, 0.00°, 0.00°]
```

## Troubleshooting

### Port Not Found (/dev/ttyACM0)

**Symptom:** Python can't connect to /dev/ttyACM0

**Solution:**
1. Check if ESP32-S3 is connected via USB
2. Verify it appears in `ls /dev/tty*`
3. Try unplugging and replugging USB cable
4. Check if you have correct permissions: `sudo chmod 666 /dev/ttyACM0`

### Two Serial Ports Appear

**Symptom:** Both /dev/ttyACM0 and /dev/ttyACM1 appear

**Explanation:**
- ESP32-S3 can expose multiple USB endpoints
- Use the one for UART0 (typically the first one: /dev/ttyACM0)
- The second one might be USB-JTAG console

### Monitor Shows Logs But Python Fails

**Symptom:** idf.py monitor shows logs, but Python can't connect

**Cause:** Both trying to use same port

**Solution:**
- Stop `idf.py monitor` (Ctrl+])
- Run Python script
- Or use USB-JTAG for monitoring (different port)

## Advanced: Debug Logging

Since S3 has separate console, you can enable DEBUG logging without affecting data:

```bash
idf.py menuconfig
# → Component config
# → Log output
# → Default log verbosity
# → Debug
```

This will show detailed per-command information on USB-JTAG console while Python continues working normally on UART0.

## Summary

**ESP32-S3 is ideal for development:**
- ✅ Separate logging channel (no UART interference)
- ✅ More memory (larger RMT buffers)
- ✅ Monitor and control simultaneously
- ✅ Debug logging safe anytime
- ✅ Native USB (no external adapter needed)

**ESP32-WROOM for production:**
- ✅ Lower cost
- ✅ Smaller form factor
- ✅ Still fully functional with optimized logging
- ⚠️ Requires external USB-UART adapter
- ⚠️ Can't monitor and control simultaneously
