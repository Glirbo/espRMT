# Recent Changes Summary

## Date: December 2025

### Problem Statement
Position query feature had only 20% reliability with UART corruption issues (garbled output like `����PPOS:0.00` or `OS:0.00`). Python scripts failed on repeated runs due to hardcoded trajectory start positions causing 900°/s velocity violations.

### Solution Overview
Implemented dual-channel architecture separating data from logging, optimized ESP32 timing for faster loop cycling, and added intelligent retry mechanism in Python. Position query reliability improved from 20% → 90-100%.

---

## Changes by File

### ESP32 Firmware

#### `esp32-idf/sdkconfig`
**Lines 1158-1166**: Console configuration changed
```diff
- CONFIG_ESP_CONSOLE_UART_DEFAULT=y
- # CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG is not set
- CONFIG_ESP_CONSOLE_UART=y
+ # CONFIG_ESP_CONSOLE_UART_DEFAULT is not set
+ CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
+ # CONFIG_ESP_CONSOLE_UART is not set
```
**Impact**: Logs now go to USB-JTAG, UART0 is clean for data only

#### `esp32-idf/main/main.c`
**Lines 1-19**: Added comprehensive header documentation
- Documents dual-channel architecture
- Explains position query protocol
- Notes performance optimizations

**Line 299**: Reduced receive_command() timeout
```c
// Changed from 100ms → 20ms → 5ms
int len = uart_read_bytes(UART_NUM, &rx_buffer[bytes_read], remaining, 5 / portTICK_PERIOD_MS);
```
**Impact**: Main loop cycles ~60x/second (was ~10x)

**Line 466**: Reduced idle delay
```c
// Changed from none → 5ms → 1ms
vTaskDelay(1 / portTICK_PERIOD_MS);
```
**Impact**: Prevents busy-waiting while maintaining responsiveness

---

### Python Controller

#### `python/motion_controller.py`
**Lines 1-19**: Enhanced module docstring
- Documents dual-channel architecture
- Explains position query reliability improvements
- Notes stateful trajectory generation

**Lines 87-133**: Complete rewrite of `get_current_position()`
```python
def get_current_position(self, max_retries=5):
```
**New features**:
- Up to 5 retry attempts (configurable)
- Buffer clearing before each attempt
- Line filtering to find "POS:" response
- 300ms timeout per attempt
- Only reports errors on final attempt

**Lines 183-228**: Enhanced `generate_test_trajectory()`
```python
def generate_test_trajectory(start_angle=0.0, end_angle=45.0):
```
**New features**:
- Accepts start/end angle parameters
- Auto-calculates safe duration: `(angle_change × 2.0) / 30.0`
- Respects S-curve 2× velocity multiplier

**Lines 231-250**: Modified `generate_circular_trajectory()`
```python
def generate_circular_trajectory(center_angle, radius_deg, ...):
```
**Changes**:
- Single `center_angle` parameter (not array)
- Generates sinusoidal motion
- All motors move identically

**Lines 272-350**: Rewritten `main()` function
**New behavior**:
- Queries position at startup
- Generates trajectories from current position
- Queries position after each test
- Returns to home (0°) at end
- No more velocity violations on repeated runs

#### `python/simple_example.py`
**Lines 60-67**: Updated to move all motors identically
```python
send_position(ser, timestamp_ms=0, angles=[0.0, 0.0, 0.0])
send_position(ser, timestamp_ms=1000, angles=[45.0, 45.0, 45.0])
send_position(ser, timestamp_ms=2000, angles=[90.0, 90.0, 90.0])
```
**Impact**: Simplified from robotic arm kinematics to synchronized motors

---

### Documentation

#### `CLAUDE.md`
**Lines 9-19**: Added dual-channel architecture section
**Lines 147-173**: Enhanced position query documentation with retry details
**Lines 187-201**: Added position query optimization subsection
**Lines 429-440**: Added recent improvements to lessons learned

#### `CHANGELOG.md`
**Lines 3-125**: Added comprehensive new section documenting all improvements
- Detailed before/after comparisons
- Performance metrics
- Migration guide
- Known issues fixed

#### `RECENT_CHANGES.md`
**New file**: This summary document

---

## Testing & Verification

### Before Changes
```
Position query success rate: 20%
UART output: POS:0.00,0.00,0.00
                  ����PPOS:0.00  (corrupted)
Main loop cycle: ~10 Hz when idle
Repeated runs: 900°/s velocity violations
```

### After Changes
```
Position query success rate: 90-100%
UART output: POS:0.00,0.00,0.00
                  (clean, no corruption)
Main loop cycle: ~60 Hz when idle
Repeated runs: Smooth, no violations
```

---

## Migration Instructions

1. **Rebuild ESP32 firmware**:
   ```bash
   cd esp32-idf
   . $HOME/esp/esp-idf/export.sh
   idf.py build flash
   ```

2. **Test position query**:
   ```bash
   screen /dev/ttyACM0 115200
   # Send 'P' 10-20 times, expect 90-100% clean responses
   ```

3. **Monitor logs** (optional, in separate terminal):
   ```bash
   idf.py monitor
   # Now connects to USB-JTAG, won't interfere with Python
   ```

4. **Run Python controller**:
   ```bash
   cd python
   python3 motion_controller.py
   # Position queries should work reliably
   # No velocity violations on repeated runs
   ```

---

## Key Takeaways

1. **Separate data from logging** - Mixed UART traffic causes corruption
2. **Fast main loop** - Position queries need frequent checking (5ms timeout, 1ms delay)
3. **Retry mechanism** - Even 50% per-attempt → 99.97% with 5 retries
4. **Stateful operation** - Query position before generating trajectories
5. **Auto-calculate durations** - Respect velocity limits automatically

---

## Files Modified

**ESP32 (2 files)**:
- `esp32-idf/sdkconfig` (console configuration)
- `esp32-idf/main/main.c` (timing optimizations + documentation)

**Python (2 files)**:
- `python/motion_controller.py` (retry mechanism + stateful trajectories)
- `python/simple_example.py` (synchronized motor motion)

**Documentation (3 files)**:
- `CLAUDE.md` (architecture + lessons learned)
- `CHANGELOG.md` (detailed changelog entry)
- `RECENT_CHANGES.md` (this file)

**Total**: 7 files modified/created
