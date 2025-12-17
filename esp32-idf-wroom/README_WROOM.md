# ESP32-WROOM Edition

This is the ESP32-WROOM-specific version of the RMT Arm Controller, adapted from the main ESP32-S3 version.

## Key Differences from ESP32-S3 Version

### Hardware Architecture

**GPIO Pinout (6 Motors):**
- Motor 0: STEP=GPIO25, DIR=GPIO32
- Motor 1: STEP=GPIO26, DIR=GPIO33
- Motor 2: STEP=GPIO27, DIR=GPIO14
- Motor 3: STEP=GPIO12, DIR=GPIO16
- Motor 4: STEP=GPIO13, DIR=GPIO17
- Motor 5: STEP=GPIO15, DIR=GPIO18
- **GPIO21: Always HIGH (3.3V power output)** - for external power requirements

**Communication:**
- UART0 only (no separate USB-JTAG like S3)
- Logging and data share the same UART channel
- Connect to `/dev/ttyUSB0` (typically via CH340/CP210x USB-UART adapter)
- **Position query logging disabled** to prevent UART interference with data packets

### Binary Size Optimizations

The ESP32-WROOM has more limited flash compared to S3, so several optimizations are enabled:

1. **Compiler optimization for size** (`CONFIG_COMPILER_OPTIMIZATION_SIZE=y`)
2. **Wi-Fi disabled** - not needed for this application (~500KB saved)
3. **Bluetooth disabled** - not needed for this application (~400KB saved)
4. **Nano newlib** - smaller printf implementation
5. **Single app partition** - simplified partition table
6. **Assertions disabled** - reduces code size

Expected binary size reduction: **~1MB smaller** than S3 version

### Memory Considerations

- Total SRAM: 520KB (vs 512KB internal + 8MB PSRAM on S3)
- RMT buffers use ~74KB (6 motors × 3072 items × 4 bytes)
- Remaining ~446KB for stack, heap, and system

**Important:** This configuration works but leaves limited headroom. If you need to:
- Add more features → reduce `MAX_RMT_ITEMS` in main.c
- Control more motors → reduce buffer size per motor
- Enable Wi-Fi → will need significant memory optimization

## Build Instructions

### Setup ESP-IDF Environment

```bash
# Setup ESP-IDF environment (required once per terminal session)
. $HOME/esp/esp-idf/export.sh

# Navigate to WROOM firmware directory
cd esp32-idf-wroom
```

### Build and Flash

```bash
# Build only
idf.py build

# Build and flash to ESP32-WROOM (typically /dev/ttyUSB0)
idf.py -p /dev/ttyUSB0 flash

# Build, flash, and monitor serial output
idf.py -p /dev/ttyUSB0 flash monitor

# Monitor only (after flashing)
idf.py -p /dev/ttyUSB0 monitor

# Exit monitor: Ctrl+]
```

### Clean Build (if needed)

```bash
# Full clean rebuild (recommended after major config changes)
idf.py fullclean
idf.py build
```

## Python Controller Usage

The Python controller works identically with both ESP32-S3 and ESP32-WROOM. Simply change the serial port:

```python
# For WROOM (typically USB-UART adapter)
controller = MotionController('/dev/ttyUSB0', 115200)

# For S3 (native USB)
controller = MotionController('/dev/ttyACM0', 115200)
```

**Auto-detection:** Both scripts now automatically try `/dev/ttyACM0` first, then fall back to `/dev/ttyUSB0`, so you can use the same code for both boards without modification.

## GPIO21 as 3.3V Power Source

GPIO21 is configured to always output HIGH (3.3V) for external power requirements.

**Specifications:**
- Output voltage: 3.3V (typical)
- Maximum current: 40mA per GPIO pin (as per ESP32 datasheet)
- Total current for all GPIOs: 200mA maximum

**Important Warnings:**
⚠️ **Do not exceed 40mA** on GPIO21 to avoid damaging the ESP32
⚠️ **Do not use for motor drivers** - insufficient current capacity
✓ **Suitable for:** Small sensors, LED indicators, pull-up resistors, logic level signals

**Example uses:**
- Powering small I2C/SPI sensors (typically <10mA)
- Pull-up resistors for communication lines
- Logic level reference voltage
- Small LED with current-limiting resistor

## Monitoring and Debugging

### Shared UART Consideration

Unlike the ESP32-S3 which has separate USB-JTAG for logging, the WROOM outputs both data and logs to UART0. To minimize interference:

1. **Position query logging disabled** - Clean responses with no log messages
2. **Command logging at DEBUG level** - Per-command details hidden by default
3. **Only errors and warnings shown** - Critical issues still visible
4. **Periodic statistics** - Status updates every 500 commands

**Clean position query response:**
```
POS:0.00,0.00,0.00,0.00,0.00,0.00
```

**For debugging:** Enable debug logs via `idf.py menuconfig` → Log output → Debug level

### Serial Monitoring

You cannot monitor and run Python controller simultaneously. Use one of these approaches:

**Option 1: Monitor startup, then run Python**
```bash
# Terminal 1: Monitor ESP32 startup
idf.py -p /dev/ttyUSB0 monitor
# Press Ctrl+] to exit after startup

# Terminal 2: Run Python controller
cd ../python
python3 motion_controller.py
```

**Option 2: Monitor after Python session**
```bash
# Terminal 1: Run Python controller
python3 motion_controller.py
# Exit when done

# Terminal 2: Monitor to see final state
idf.py -p /dev/ttyUSB0 monitor
```

## Verifying Installation

After flashing, you should see this startup banner:

```
╔════════════════════════════════════════════════════════╗
║                                                        ║
║   ESP32-WROOM ROBOTIC ARM CONTROLLER v1.0              ║
║                  (6 MOTORS)                            ║
║                                                        ║
╚════════════════════════════════════════════════════════╝

Motor Configuration:
  Steps per revolution: 4000 (no microstepping)
  Gear ratio:           50:1
  Steps per degree:     555.56

GPIO CONFIGURATION
✓ GPIO21 configured as 3.3V power output (always HIGH)

RMT CONFIGURATION
✓ Motor 0 configured:
  STEP pin: GPIO25  │  DIR pin: GPIO32  │  RMT: CH0
✓ Motor 1 configured:
  STEP pin: GPIO26  │  DIR pin: GPIO33  │  RMT: CH1
✓ Motor 2 configured:
  STEP pin: GPIO27  │  DIR pin: GPIO14  │  RMT: CH2
✓ Motor 3 configured:
  STEP pin: GPIO12  │  DIR pin: GPIO16  │  RMT: CH3
✓ Motor 4 configured:
  STEP pin: GPIO13  │  DIR pin: GPIO17  │  RMT: CH4
✓ Motor 5 configured:
  STEP pin: GPIO15  │  DIR pin: GPIO18  │  RMT: CH5

SYSTEM READY
```

## Binary Size Verification

After building, check the binary size:

```bash
idf.py size

# Should show something like:
# Total sizes:
# DRAM .data size:   ~15KB
# DRAM .bss  size:   ~200KB (RMT buffers)
# Used static IRAM:  ~80KB
# Flash code:        ~250KB (down from ~700KB on unoptimized S3)
# Flash rodata:      ~80KB
```

## Troubleshooting

### Build fails with "target not matching"

```bash
# Clean and reconfigure for ESP32
idf.py fullclean
idf.py set-target esp32
idf.py build
```

### Out of memory errors

Reduce `MAX_RMT_ITEMS` in main.c:
```c
// Reduce from 16384 to smaller value
#define MAX_RMT_ITEMS 8192  // Supports ~14.7° moves in 50ms
```

### Wi-Fi/Bluetooth errors during build

These should be disabled by default. If you see errors, verify:
```bash
idf.py menuconfig
# Component config → ESP32-specific → Bluetooth → [disable]
# Component config → Wi-Fi → [disable]
```

## Notes

1. **No USB-JTAG:** Unlike S3, WROOM requires external USB-UART adapter
2. **Shared UART:** Logs and data both use UART0 (some interference expected)
3. **Memory tight:** 192KB RMT buffers leave ~300KB for everything else
4. **Flash savings:** Binary is ~1MB smaller than unoptimized S3 build
5. **GPIO21 power:** Remember 40mA limit, suitable only for low-power devices

## Reverting to ESP32-S3

If you need to switch back to ESP32-S3:

```bash
cd ../esp32-idf
idf.py -p /dev/ttyACM0 flash monitor
```

The original S3 version has:
- Separate USB-JTAG logging (cleaner position queries)
- More SRAM available (512KB + 8MB PSRAM)
- Native USB support (no adapter needed)
