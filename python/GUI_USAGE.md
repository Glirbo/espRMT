# Inverse Kinematics Controller GUI - Usage Guide

## Overview

The GUI provides an interactive interface for controlling the 6-DOF robotic arm using inverse kinematics. You can input target poses in Cartesian space (X, Y, Z, Roll, Pitch, Yaw) and the system will automatically calculate joint angles and execute the motion.

## Running the GUI

```bash
cd /home/mikkel/git/espRMT/python
python3 inverse_kinematics_controller_gui.py
```

## GUI Layout

### 1. Connection Panel (Top)
- **Port**: Serial port for ESP32 (default: `/dev/ttyACM0`)
- **Connect**: Connect to the robot
- **Disconnect**: Disconnect from the robot
- **Status**: Shows connection state (Disconnected/Connected/Simulation Mode)

### 2. Current Pose Panel (Left)
Displays the robot's current end-effector position and orientation:
- **Position**: X, Y, Z in millimeters
- **Orientation**: Roll, Pitch, Yaw in degrees
- **Query Position**: Button to request current position from ESP32

### 3. Target Pose Panel (Right)
Input fields for desired end-effector position and orientation:
- **Position**: X, Y, Z in millimeters
- **Orientation**: Roll, Pitch, Yaw in degrees
- **Use Current as Target**: Copies current pose to target fields

### 4. Motion Control Panel (Middle)
Control buttons for executing motions:
- **Move to Target (Position Only)**: Moves to target XYZ, ignoring orientation (3-DOF IK)
- **Move to Target (Full Pose)**: Moves to target with orientation control (6-DOF IK)
- **Go Home**: Returns robot to home position [0,0,0,0,0,0]
- **STOP**: Emergency stop (completes current segment, then stops)

### 5. Log Panel (Bottom)
Shows timestamped messages about:
- Connection events
- Motion planning progress
- Trajectory execution status
- Errors and warnings
- Clear Log button to reset

## Typical Workflow

### First-Time Setup
1. Ensure motors are manually positioned at [0,0,0,0,0,0] (home position)
2. Connect ESP32 to computer via USB
3. Launch GUI: `python3 inverse_kinematics_controller_gui.py`

### Basic Operation
1. **Connect to Robot**:
   - Verify port (usually `/dev/ttyACM0` for ESP32-S3)
   - Click "Connect"
   - Wait for "Connected successfully!" message
   - GUI will automatically query current position

2. **Set Target Pose**:
   - Method 1: Manually type X, Y, Z (and Roll, Pitch, Yaw if needed)
   - Method 2: Click "Use Current as Target", then modify values

3. **Execute Motion**:
   - For position-only moves: Click "Move to Target (Position Only)"
   - For full pose moves: Click "Move to Target (Full Pose)"
   - Watch log for planning and execution progress

4. **Monitor Progress**:
   - Log shows trajectory details (points, duration)
   - After motion completes, current pose updates automatically
   - Use "Query Position" to manually refresh

5. **Return Home**:
   - Click "Go Home" to return to [0,0,0,0,0,0]

## Features

### Position-Only Mode (3-DOF)
- Controls only XYZ position
- Orientation is free (determined by IK solver)
- Faster and more reliable for simple moves
- Good for: pick-and-place, point-to-point moves

### Full Pose Mode (6-DOF)
- Controls position AND orientation
- Useful when end-effector angle matters
- Example: keeping tool perpendicular to surface

### Trajectory Generation
- Automatically interpolates straight lines in Cartesian space
- Runs inverse kinematics on every waypoint (5mm spacing)
- Calculates safe velocities (30°/sec max)
- Uses S-curve interpolation for smooth motion

### Safety Features
- Velocity validation before execution
- Joint limit enforcement
- Simulation mode if hardware unavailable
- Position fallback if ESP32 query fails

## Example Moves

### Move 20mm in X Direction
1. Click "Use Current as Target"
2. Add 20 to the X value
3. Click "Move to Target (Position Only)"

### Rotate 45° Around Z Axis
1. Click "Use Current as Target"
2. Add 45 to the Yaw value
3. Click "Move to Target (Full Pose)"

### Circle Pattern (Manual)
From home position [104, 0, 75] mm:
1. Move to [104, 20, 75]
2. Move to [84, 0, 75]
3. Move to [104, -20, 75]
4. Move to [124, 0, 75]
5. Return to [104, 0, 75]

## Simulation Mode

If ESP32 hardware is not detected, the GUI automatically enters simulation mode:
- All motion planning still works
- Binary packets are printed to console instead of being sent
- Useful for testing trajectories without hardware
- Warning shown in connection status

## Troubleshooting

### "Failed to connect"
- Check USB cable connection
- Verify port name: `ls /dev/ttyACM* /dev/ttyUSB*`
- Ensure no other programs are using the serial port
- Try `sudo usermod -a -G dialout $USER` (logout/login after)

### "Position query failed"
- Normal if ESP32 is busy executing motion
- GUI uses fallback (trajectory endpoint)
- Wait 1 second after motion, then click "Query Position"

### "Trajectory exceeds velocity limits"
- Move is too fast for hardware
- Try smaller movements or let system auto-calculate
- Check max_velocity_deg_per_sec setting (30°/sec default)

### "IK failed for waypoint"
- Target position is unreachable
- Check if target is within workspace
- Verify joint limits aren't too restrictive
- Try position-only mode instead of full pose

### GUI Freezes During Motion
- Long trajectories block UI temporarily
- Motion runs in background thread
- GUI will become responsive when done
- This is normal behavior

## Tips

1. **Start Small**: Test with small movements (5-10mm) before large ones
2. **Query Often**: Click "Query Position" before starting new motions
3. **Use Position-Only**: When orientation doesn't matter, position-only is more reliable
4. **Monitor Logs**: Watch for velocity warnings and errors
5. **Home Regularly**: Return to home position periodically to reset tracking

## Technical Details

- **Update Rate**: 20Hz (50ms between commands)
- **Max Velocity**: 30°/sec per joint
- **Waypoint Spacing**: 5mm (configurable in code)
- **IK Method**: Damped Least Squares (Levenberg-Marquardt)
- **Interpolation**: S-curve (smooth acceleration/deceleration)

## Known Limitations

- No encoder feedback (open-loop position tracking)
- STOP button completes current segment (not instant)
- Position queries have ~10% failure rate (uses fallback)
- Long trajectories may take time to compute
- No obstacle avoidance
- No workspace boundary visualization

## Advanced Usage

### Custom DH Parameters
Edit `dh_params` in the GUI code (lines 29-36) to match your robot configuration.

### Custom Joint Limits
Edit `joint_limits` in the GUI code (lines 38-45).

### Change Velocity Limit
Modify `max_velocity_deg_per_sec` in line 51.

### Change Waypoint Spacing
In motion methods, adjust `spacing_mm` parameter (default: 5.0mm).

## See Also

- `inverse_kinematics_controller.py`: Core IK implementation
- `motion_controller.py`: ESP32 communication and trajectory execution
- `CLAUDE.md`: Full system architecture documentation
- `README.md`: Hardware setup and build instructions
