# Project Structure

```
RMTArm/
├── README.md                      # Main documentation with complete theory
├── PROJECT_STRUCTURE.md           # This file
│
├── python/                        # Python motion controller
│   ├── motion_controller.py       # Full motion controller with trajectories
│   ├── simple_example.py          # Minimal example for testing
│   └── requirements.txt           # Python dependencies (pyserial)
│
└── esp32-idf/                     # ESP-IDF firmware project
    ├── CMakeLists.txt             # Top-level build configuration
    ├── sdkconfig.defaults         # Default ESP32 settings
    ├── README.md                  # ESP32 build and flash instructions
    │
    └── main/                      # Main application code
        ├── CMakeLists.txt         # Component build configuration
        └── main.c                 # ESP32 firmware (UART + RMT control)
```

## File Descriptions

### Documentation

- **README.md** - Complete system documentation including:
  - System architecture
  - Motion planning pipeline (9 detailed steps)
  - Binary protocol specification
  - RMT configuration and usage
  - Velocity profile generation
  - IK considerations
  - Quick start guide

- **PROJECT_STRUCTURE.md** - This file, describes project organization

### Python Controller (`python/`)

- **motion_controller.py** - Production-ready motion controller
  - `MotionController` class for serial communication
  - Binary packet encoding with checksums
  - Trajectory generation functions
  - Example test trajectories (linear, circular)
  - Complete error handling

- **simple_example.py** - Minimal example for learning
  - Bare-bones position sending
  - Good starting point for understanding the protocol
  - ~40 lines of code

- **requirements.txt** - Python package dependencies
  - Only requires: `pyserial>=3.5`

### ESP32 Firmware (`esp32-idf/`)

- **main/main.c** - Complete firmware implementation (~300 lines)
  - UART receiver with checksum validation
  - RMT pulse generator for 3 motors
  - Angle-to-steps conversion
  - FreeRTOS task structure
  - Comprehensive logging

- **CMakeLists.txt** - ESP-IDF build system files
  - Top-level and component configuration
  - Standard ESP-IDF project structure

- **sdkconfig.defaults** - ESP32 configuration
  - UART baud rate
  - FreeRTOS settings
  - RMT settings
  - Logging levels

- **README.md** - ESP32-specific documentation
  - Hardware connections
  - Build instructions
  - Flash instructions
  - Pin configuration
  - Troubleshooting

## Workflow

### Development Workflow

1. **Plan motion** (Python)
   - Interpolate XYZ path
   - Run inverse kinematics
   - Generate velocity profile
   - Create timestamped setpoints

2. **Send commands** (Python)
   - Encode as binary packets
   - Send via UART to ESP32

3. **Execute motion** (ESP32)
   - Receive and validate packets
   - Convert angles to steps
   - Generate RMT pulses
   - Control stepper motors

### Getting Started

1. Read `README.md` for complete theory
2. Build and flash ESP32 firmware (see `esp32-idf/README.md`)
3. Run Python controller (see `python/motion_controller.py`)
4. Start with `python/simple_example.py` to understand basics

## Dependencies

### Python Side
- Python 3.7+
- pyserial (for UART communication)

### ESP32 Side
- ESP-IDF v5.0+ (full framework)
- ESP32 hardware (any variant with RMT)
- Stepper motor drivers (STEP/DIR interface)

## Communication Protocol

**Binary packet format (17 bytes):**
```
[timestamp:4] [angle1:4] [angle2:4] [angle3:4] [checksum:1]
```

- All values little-endian
- Timestamp: uint32_t milliseconds
- Angles: float degrees
- Checksum: XOR of all previous bytes

## Hardware Requirements

- ESP32 development board
- 3x stepper motors
- 3x stepper motor drivers (STEP/DIR interface)
  - Examples: A4988, DRV8825, TB6600, etc.
- Power supply for motors (match to your motors)
- USB cable for ESP32 programming/communication

## Key Features

✓ **Binary protocol** - Efficient, reliable communication
✓ **Checksum validation** - Detect transmission errors
✓ **RMT hardware timing** - Precise, jitter-free pulses
✓ **Multi-axis coordination** - All motors move simultaneously
✓ **Non-blocking operation** - CPU free during motion
✓ **Configurable** - Easy to adjust pins, motor settings
✓ **Well documented** - Complete theory and examples

## Next Steps

1. Implement inverse kinematics for your specific robot
2. Add forward kinematics for verification
3. Implement path planning with obstacle avoidance
4. Add limit switches and homing routines
5. Implement error recovery and safety features
6. Add closed-loop control with encoders (optional)
