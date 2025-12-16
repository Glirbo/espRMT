# ESP32 Monitoring Guide

How to monitor ESP32 logs while running Python motion controller.

## The Problem

Both `idf.py monitor` and the Python script need exclusive access to the serial port. You can't open the same port (`/dev/ttyACM0`) with two programs simultaneously.

## Solutions

### ✅ Solution 1: Use Two UART Ports (Best for Development)

Split communication and logging onto separate UARTs.

**Hardware Setup:**
```
ESP32-S3          USB-to-UART Adapter
GPIO 17 (TX1) --> RX
GPIO 18 (RX1) --> TX
GND           --> GND
```

**Configuration:**

1. Edit `esp32-idf/sdkconfig.defaults`:
   ```
   # Use UART1 for logging
   CONFIG_ESP_CONSOLE_UART_NUM=1
   CONFIG_ESP_CONSOLE_UART_TX_GPIO=17
   CONFIG_ESP_CONSOLE_UART_RX_GPIO=18
   ```

2. Rebuild and flash:
   ```bash
   cd esp32-idf
   idf.py build flash
   ```

3. Monitor logs on the USB-UART adapter:
   ```bash
   # Terminal 1 - Monitor logs
   screen /dev/ttyUSB0 115200
   # or
   idf.py monitor -p /dev/ttyUSB0
   ```

4. Run Python controller on native USB:
   ```bash
   # Terminal 2 - Control robot
   cd python
   python3 motion_controller.py
   ```

**Advantages:**
- ✅ Real-time logging while controlling
- ✅ Full debug output
- ✅ No interference between channels

**Disadvantages:**
- ❌ Requires USB-to-UART adapter (~$5)
- ❌ Extra wiring

---

### ✅ Solution 2: Sequential Monitoring (Simplest, No Extra Hardware)

Monitor ESP32 first, then run Python script.

**Usage:**
```bash
# Terminal 1 - Start monitoring FIRST
cd esp32-idf
idf.py monitor

# Wait for ESP32 to boot and show "System ready"
# Then Ctrl+] to exit monitor

# Terminal 2 - Now run Python
cd python
python3 motion_controller.py
```

**Advantages:**
- ✅ No extra hardware needed
- ✅ Can verify ESP32 booted correctly
- ✅ Simple setup

**Disadvantages:**
- ❌ Can't see logs while Python is running
- ❌ Must restart to see logs again

---

### ✅ Solution 3: Add Logging to Python Script

The ESP32 can send acknowledgments back to Python for monitoring.

**Add to ESP32** (`main.c`):

```c
void process_command(MotionCommand *cmd)
{
    ESP_LOGI(TAG, "Command @ %" PRIu32 " ms: [%.2f°, %.2f°, %.2f°]",
             cmd->timestamp_ms,
             cmd->joint_angles[0],
             cmd->joint_angles[1],
             cmd->joint_angles[2]);

    for (int i = 0; i < NUM_MOTORS; i++) {
        // ... existing code ...
    }

    // Send acknowledgment back to Python (optional)
    char ack[32];
    snprintf(ack, sizeof(ack), "ACK:%lu\n", cmd->timestamp_ms);
    uart_write_bytes(UART_NUM_0, ack, strlen(ack));
}
```

**Add to Python** (`motion_controller.py`):

```python
def send_position(self, timestamp_ms, angles):
    """Send position command to ESP32"""
    # ... existing send code ...

    # Read acknowledgment (if not in simulation mode)
    if not self.simulation_mode:
        try:
            response = self.ser.readline().decode('utf-8').strip()
            if response:
                print(f"  ESP32: {response}")
        except:
            pass
```

**Advantages:**
- ✅ No extra hardware
- ✅ See ESP32 responses in Python terminal
- ✅ Good for production use

**Disadvantages:**
- ❌ Limited logging (only what ESP32 sends back)
- ❌ Not suitable for debugging

---

### ✅ Solution 4: WiFi Logging (Advanced)

ESP32 sends logs over WiFi/UDP to your computer.

**ESP32 Setup:**
```c
// In main.c, add WiFi and UDP logging
// Send logs to UDP port on PC
```

**PC receives logs:**
```bash
nc -ul 8888
```

**Advantages:**
- ✅ No port conflicts
- ✅ Can log from anywhere on network
- ✅ Multiple devices can receive logs

**Disadvantages:**
- ❌ Requires WiFi setup
- ❌ More complex configuration
- ❌ Adds latency

---

### ✅ Solution 5: Use ESP-IDF's Built-in JTAG (ESP32-S3 Only)

ESP32-S3 has built-in USB JTAG for debugging.

**Setup:**
```bash
# Terminal 1 - OpenOCD with JTAG
openocd -f board/esp32s3-builtin.cfg

# Terminal 2 - View logs
telnet localhost 4444

# Terminal 3 - Run Python
python3 motion_controller.py
```

**Advantages:**
- ✅ Professional debugging
- ✅ Breakpoints, variable inspection
- ✅ Logs separate from data channel

**Disadvantages:**
- ❌ Complex setup
- ❌ Requires OpenOCD knowledge
- ❌ ESP32-S3 specific

---

## Recommended Workflows

### For Development (Debugging)

**Best: Dual UART (Solution 1)**
- Connect USB-to-UART adapter to GPIO 17/18
- Monitor logs on adapter port
- Control via native USB

**Alternative: Sequential (Solution 2)**
- Monitor ESP32 startup
- Exit monitor
- Run Python controller
- Re-monitor if issues occur

### For Production/Testing

**Best: Python-based monitoring (Solution 3)**
- ESP32 sends status updates
- Python displays them
- Single terminal needed

### Quick Testing

**Use simulation mode:**
```bash
# Test without ESP32 connected
python3 motion_controller.py
# Uses automatic simulation mode
```

---

## Quick Reference

| Method | Extra Hardware | Real-time Logs | Complexity |
|--------|----------------|----------------|------------|
| Dual UART | USB-UART (~$5) | ✅ Yes | Low |
| Sequential | None | ❌ No | Very Low |
| Python ACKs | None | ⚠️ Limited | Low |
| WiFi | WiFi network | ✅ Yes | Medium |
| JTAG | None (S3 only) | ✅ Yes | High |

---

## Commands Cheat Sheet

### Dual UART Setup
```bash
# Terminal 1 - Logs on USB-UART adapter
screen /dev/ttyUSB0 115200

# Terminal 2 - Control on native USB
python3 motion_controller.py
```

### Sequential Monitoring
```bash
# Terminal 1
idf.py monitor              # Watch startup
# Press Ctrl+] to exit
cd ../python
python3 motion_controller.py   # Run controller
```

### Check Available Ports
```bash
ls -l /dev/tty{ACM,USB}*
```

### Kill Process Using Port
```bash
# If port is stuck
fuser -k /dev/ttyACM0
```

---

## Troubleshooting

### "Port is busy"
```bash
# Find what's using the port
lsof /dev/ttyACM0

# Kill it
sudo fuser -k /dev/ttyACM0
```

### "Permission denied"
```bash
# Add yourself to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Can't see logs
```bash
# Check log level in sdkconfig.defaults
CONFIG_LOG_DEFAULT_LEVEL_INFO=y

# Rebuild
idf.py build flash
```

---

## Example: Dual Terminal Setup

**Terminal 1 (Logs):**
```
$ screen /dev/ttyUSB0 115200

I (328) RMTArm: === ESP32 Robotic Arm Controller ===
I (329) RMTArm: Motor config: 88.89 steps/degree
I (330) RMTArm: Setting up RMT for 3 motors
I (335) RMTArm: Motor 0: STEP=GPIO1, DIR=GPIO4, RMT_CH=0
I (340) RMTArm: Motor 1: STEP=GPIO2, DIR=GPIO5, RMT_CH=1
I (345) RMTArm: Motor 2: STEP=GPIO3, DIR=GPIO6, RMT_CH=2
I (350) RMTArm: System ready - waiting for commands...
I (1234) RMTArm: Command @ 0 ms: [0.00°, 45.00°, 30.00°]
I (1285) RMTArm: Command @ 50 ms: [0.00°, 45.15°, 29.90°]
```

**Terminal 2 (Control):**
```
$ python3 motion_controller.py

Connected to /dev/ttyACM0 at 115200 baud

=== Test 1: Simple Linear Move ===
Moving from [0°, 45°, 30°] to [0°, 75°, 10°] over 1 second
Trajectory complete (21 points)
```

Perfect! Now you can see what the ESP32 is doing while controlling it! 🎉
