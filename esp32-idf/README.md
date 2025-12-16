# ESP32-IDF Robotic Arm Controller

ESP-IDF firmware for controlling stepper motors via RMT module.

## Requirements

- ESP-IDF v5.0 or later
- ESP32 development board (ESP32, ESP32-S3, ESP32-C3, etc.)
- Stepper motor drivers (STEP/DIR interface)

**Note:** Default pin configuration is for ESP32-S3. See "Hardware Connections" below for other variants.

## Hardware Connections

### Default Pin Configuration (ESP32-S3)

| Motor | STEP Pin | DIR Pin | RMT Channel |
|-------|----------|---------|-------------|
| 0     | GPIO 1   | GPIO 4  | Channel 0   |
| 1     | GPIO 2   | GPIO 5  | Channel 1   |
| 2     | GPIO 3   | GPIO 6  | Channel 2   |

**UART:** Uses default UART0 (USB serial via GPIO 43/44 on ESP32-S3)

### Alternative Pins

If the above pins conflict with your board layout:
- **STEP pins:** GPIO 8, 9, 10
- **DIR pins:** GPIO 11, 12, 13

### ESP32-S3 GPIO Notes

**Safe GPIO pins:** 1-18, 21, 38-48

**Avoid:**
- GPIO 0: Boot button (strapping pin)
- GPIO 19-20: USB D-/D+ (if using native USB)
- GPIO 26-37: May be connected to flash/PSRAM on some modules
- GPIO 43-44: UART0 TX/RX (for USB-Serial communication)

**Note for original ESP32:** If using classic ESP32 (not S3), use GPIO 25-27 for STEP and 32-33, 14 for DIR.

### Motor Driver Connections

For each motor:
```
ESP32 STEP pin → Driver STEP/PUL input
ESP32 DIR pin  → Driver DIR input
GND            → Driver GND
```

**Important:** Ensure common ground between ESP32 and motor drivers.

## Building and Flashing

### Setup ESP-IDF Environment

```bash
# Navigate to project directory
cd esp32-idf

# Set up ESP-IDF environment (do this each terminal session)
. $HOME/esp/esp-idf/export.sh
```

### Configure

```bash
# Optional: Configure project settings
idf.py menuconfig

# You can modify:
# - Pin assignments in main/main.c
# - Motor configuration (steps/rev, microstepping, gear ratio)
# - UART baud rate
```

### Build

```bash
idf.py build
```

### Flash

```bash
# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Or combine flash + monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

**Note:** Replace `/dev/ttyUSB0` with your actual serial port (e.g., `COM3` on Windows).

## Configuration

### Modify Pin Assignments

Edit `main/main.c` to match your hardware:

```c
// GPIO Pin Definitions (ESP32-S3 defaults)
#define STEP_PIN_0      GPIO_NUM_1   // Change these to match your wiring
#define STEP_PIN_1      GPIO_NUM_2
#define STEP_PIN_2      GPIO_NUM_3
#define DIR_PIN_0       GPIO_NUM_4
#define DIR_PIN_1       GPIO_NUM_5
#define DIR_PIN_2       GPIO_NUM_6
```

**For classic ESP32, use:**
```c
#define STEP_PIN_0      GPIO_NUM_25
#define STEP_PIN_1      GPIO_NUM_26
#define STEP_PIN_2      GPIO_NUM_27
#define DIR_PIN_0       GPIO_NUM_32
#define DIR_PIN_1       GPIO_NUM_33
#define DIR_PIN_2       GPIO_NUM_14
```

### Modify Motor Configuration

Edit `main/main.c`:

```c
// Motor Configuration
#define STEPS_PER_REV   200.0f   // Steps per revolution (1.8° = 200)
#define MICROSTEPS      16.0f    // Microstepping setting
#define GEAR_RATIO      10.0f    // Gear reduction ratio
```

### Modify UART Baud Rate

Edit `main/main.c`:

```c
#define UART_BAUD_RATE  115200  // Or 921600 for higher speed
```

## Protocol

The ESP32 receives binary packets via UART:

### Packet Format (17 bytes)

| Bytes | Type     | Description                  |
|-------|----------|------------------------------|
| 0-3   | uint32_t | Timestamp in milliseconds    |
| 4-7   | float    | Joint 1 angle (degrees)      |
| 8-11  | float    | Joint 2 angle (degrees)      |
| 12-15 | float    | Joint 3 angle (degrees)      |
| 16    | uint8_t  | Checksum (XOR of bytes 0-15) |

**Byte order:** Little-endian

## Testing

1. Flash the firmware to ESP32
2. Run the Python motion controller:
   ```bash
   cd ../python
   python3 motion_controller.py
   ```

## Monitoring

View real-time logs:
```bash
idf.py -p /dev/ttyUSB0 monitor
```

Press `Ctrl+]` to exit monitor.

## Troubleshooting

### Build Errors

```bash
# Clean build
idf.py fullclean
idf.py build
```

### Flash Errors

```bash
# Check port permissions (Linux)
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect

# Erase flash and reflash
idf.py -p /dev/ttyUSB0 erase-flash
idf.py -p /dev/ttyUSB0 flash
```

### Motors Not Moving

1. Check wiring (STEP, DIR, GND)
2. Verify motor driver is powered
3. Check pin definitions match your wiring
4. Monitor serial output for error messages
5. Verify stepper driver enable pin (if present)

### Checksum Errors

1. Check baud rate matches Python script
2. Ensure good USB cable
3. Try lower baud rate (115200)

## License

[Add your license]
