# RMTArm Project Changelog

## Latest Session (December 2025)

### Motor Configuration Update
- **Changed**: Removed microstepping calculation, now uses direct steps/revolution
- **Old**: `STEPS_PER_REV = 200`, `MICROSTEPS = 16`, `GEAR_RATIO = 50`
  - Resulted in: 444.44 steps/degree
- **New**: `STEPS_PER_REV = 4000`, `GEAR_RATIO = 50` (no microstepping variable)
  - Results in: 555.56 steps/degree
- **Impact**: Changed min pulse period from 75μs to 60μs, max step rate from 13,333 to 16,667 steps/sec

### Position Query Feature (NEW)
- **Added**: Ability to query current motor positions from ESP32
- **Protocol**: Send single byte `'P'` or `'?'` to request position
- **Response**: ASCII string `"POS:j1,j2,j3\n"`
- **Python API**: `controller.get_current_position()` returns `[j1, j2, j3]` or `None`
- **Implementation**:
  - ESP32: `send_current_position()` and `check_position_request()` functions
  - Pending packet buffer system to prevent packet loss
  - Both protocols (position query + motion commands) coexist without interference

### Trajectory Updates
- **Linear motion test**:
  - Duration: 1 second → **3 seconds** (to respect velocity limits with S-curve)
  - Start position: [0, 45, 30] → **[0, 0, 0]** (matches ESP32 default assumption)
  - End position: [0, 75, 10] → **[0, 45, 30]**
- **Circular motion test**:
  - Center: [0, 60, 20] → **[0, 30, 30]**
  - Now starts at [0, 45, 30] (where linear motion ends)
  - No gap between tests for smooth continuation
- **Added**: 5-second pause between linear and circular tests
- **Removed**: Homing sequence (was never a real homing, just moved to [0,0,0])

### Velocity Validation (NEW)
- **Added**: `validate_trajectory_velocity()` function in Python
- **Purpose**: Checks every trajectory segment for velocity limit violations
- **Critical discovery**: S-curve interpolation has **2.0× peak velocity multiplier**
  - Formula: `peak_velocity = angle_change × 2.0 / duration`
  - Example: 45° in 2 seconds → peak velocity 45°/s (exceeds 30°/s limit!)
  - Solution: 45° needs minimum 3 seconds with S-curve
- **Integration**: Automatically called before executing test trajectories

### Position Tracking Warning (NEW)
- **Added**: 3-second warning message before motion starts
- **Message**: "IMPORTANT: Position motors at [0°, 0°, 0°] before starting!"
- **Purpose**: Remind user that system has no encoders, motors must be manually positioned
- **User option**: Press Ctrl+C to abort if motors not positioned correctly

### ESP32 Firmware Fixes
- **Fixed**: Packet loss in position query implementation
  - **Problem**: `check_position_request()` consumed motion command bytes without processing
  - **Solution**: Added `pending_packet` buffer to store partial packets
  - **Impact**: Both protocols now work reliably without interference
- **Improved**: Position response logging (ESP_LOGD → ESP_LOGI)
- **Added**: Startup message indicating position query support

### Documentation Updates
- **CLAUDE.md**:
  - Added S-curve velocity considerations
  - Documented position query protocol
  - Added position tracking and homing section
  - Added pending packet buffer explanation
  - Updated Key Lessons Learned with new issues and solutions
  - Added querying position to Common Development Tasks
- **python/README.md**:
  - Added position query API documentation
  - Added velocity validation documentation
  - Added current test sequence description
  - Added troubleshooting for position queries and velocity warnings
  - Added S-curve velocity limit calculations
- **CHANGELOG.md**: Created this file

### Breaking Changes
None - all changes are backward compatible. Old trajectories will still work (though they may exceed velocity limits).

### Migration Guide
If upgrading from previous version:
1. Rebuild ESP32 firmware: `cd esp32-idf && idf.py build flash`
2. Update Python scripts: Already updated in repository
3. Manually position motors at [0, 0, 0] before running
4. Test position query: `python3 -c "from motion_controller import MotionController; c = MotionController('/dev/ttyACM0', 115200); print(c.get_current_position())"`

### Known Issues
- System still has no real homing with limit switches (open-loop position tracking only)
- Position tracking can drift if motors lose steps or are moved manually
- S-curve interpolation makes it easy to exceed velocity limits - always validate trajectories

### Future Improvements Suggested
- Add limit switches for automatic homing
- Implement proper homing routine on startup
- Consider adding encoders for closed-loop control
- Add optional linear interpolation mode (constant velocity, no S-curve) for velocity-critical applications
- Add trajectory preview/visualization tool
