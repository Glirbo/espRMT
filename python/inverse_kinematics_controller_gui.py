#!/usr/bin/env python3
"""
GUI for 6-DOF Inverse Kinematics Controller

Features:
- Interactive input for target pose (X, Y, Z, Roll, Pitch, Yaw)
- Real-time display of current pose
- Position-only (3-DOF) or full pose (6-DOF) control
- Connection management with ESP32
- Visual feedback and logging
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import numpy as np
import time
import threading
from inverse_kinematics_controller import RobotKinematics, CartesianMotionPlanner
from motion_controller import MotionController, validate_trajectory_velocity


class IKControllerGUI:
    """GUI for Inverse Kinematics Controller"""

    def __init__(self, root):
        """Initialize GUI"""
        self.root = root
        self.root.title("6-DOF Inverse Kinematics Controller")
        self.root.geometry("900x750")

        # Robot configuration (same as demo)
        self.dh_params = [
            [0,     0,   25, 0],    # Joint 1: Base rotation
            [90,    0,   0,  0],    # Joint 2: Shoulder
            [0,     32,  0,  0],    # Joint 3: Elbow
            [0,     23,  13, 0],    # Joint 4: Wrist 1
            [90,    0,   13, 0],    # Joint 5: Wrist 2
            [90,    0,   11, 0],    # Joint 6: Wrist 3
        ]

        self.joint_limits = [
            [-170, 170],    # Joint 1
            [-120, 120],    # Joint 2
            [-150, 150],    # Joint 3
            [-180, 180],    # Joint 4
            [-120, 120],    # Joint 5
            [-180, 180],    # Joint 6
        ]

        # Initialize robot kinematics
        self.robot = RobotKinematics(self.dh_params, self.joint_limits)
        self.planner = CartesianMotionPlanner(self.robot, max_velocity_deg_per_sec=30.0)

        # Controller state
        self.controller = None
        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.is_connected = False
        self.is_moving = False

        # Create GUI
        self.create_widgets()

        # Initial display update
        self.update_current_display()

    def create_widgets(self):
        """Create all GUI widgets"""

        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_entry = ttk.Entry(conn_frame, width=20)
        self.port_entry.insert(0, "/dev/ttyACM0")
        self.port_entry.grid(row=0, column=1, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=2, padx=5)

        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=3, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)

        # Current Pose Frame
        current_frame = ttk.LabelFrame(self.root, text="Current Pose", padding=10)
        current_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")

        ttk.Label(current_frame, text="Position (mm):").grid(row=0, column=0, columnspan=2, sticky="w")

        ttk.Label(current_frame, text="X:").grid(row=1, column=0, sticky="e")
        self.curr_x_label = ttk.Label(current_frame, text="0.00", width=10, relief="sunken")
        self.curr_x_label.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(current_frame, text="Y:").grid(row=2, column=0, sticky="e")
        self.curr_y_label = ttk.Label(current_frame, text="0.00", width=10, relief="sunken")
        self.curr_y_label.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(current_frame, text="Z:").grid(row=3, column=0, sticky="e")
        self.curr_z_label = ttk.Label(current_frame, text="0.00", width=10, relief="sunken")
        self.curr_z_label.grid(row=3, column=1, padx=5, pady=2)

        ttk.Separator(current_frame, orient="horizontal").grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)

        ttk.Label(current_frame, text="Orientation (deg):").grid(row=5, column=0, columnspan=2, sticky="w")

        ttk.Label(current_frame, text="Roll:").grid(row=6, column=0, sticky="e")
        self.curr_roll_label = ttk.Label(current_frame, text="0.00", width=10, relief="sunken")
        self.curr_roll_label.grid(row=6, column=1, padx=5, pady=2)

        ttk.Label(current_frame, text="Pitch:").grid(row=7, column=0, sticky="e")
        self.curr_pitch_label = ttk.Label(current_frame, text="0.00", width=10, relief="sunken")
        self.curr_pitch_label.grid(row=7, column=1, padx=5, pady=2)

        ttk.Label(current_frame, text="Yaw:").grid(row=8, column=0, sticky="e")
        self.curr_yaw_label = ttk.Label(current_frame, text="0.00", width=10, relief="sunken")
        self.curr_yaw_label.grid(row=8, column=1, padx=5, pady=2)

        ttk.Button(current_frame, text="Query Position", command=self.query_position).grid(row=9, column=0, columnspan=2, pady=10)

        # Target Pose Frame
        target_frame = ttk.LabelFrame(self.root, text="Target Pose", padding=10)
        target_frame.grid(row=1, column=1, padx=10, pady=5, sticky="nsew")

        ttk.Label(target_frame, text="Position (mm):").grid(row=0, column=0, columnspan=2, sticky="w")

        ttk.Label(target_frame, text="X:").grid(row=1, column=0, sticky="e")
        self.target_x_entry = ttk.Entry(target_frame, width=12)
        self.target_x_entry.insert(0, "0.0")
        self.target_x_entry.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(target_frame, text="Y:").grid(row=2, column=0, sticky="e")
        self.target_y_entry = ttk.Entry(target_frame, width=12)
        self.target_y_entry.insert(0, "0.0")
        self.target_y_entry.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(target_frame, text="Z:").grid(row=3, column=0, sticky="e")
        self.target_z_entry = ttk.Entry(target_frame, width=12)
        self.target_z_entry.insert(0, "0.0")
        self.target_z_entry.grid(row=3, column=1, padx=5, pady=2)

        ttk.Separator(target_frame, orient="horizontal").grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)

        ttk.Label(target_frame, text="Orientation (deg):").grid(row=5, column=0, columnspan=2, sticky="w")

        ttk.Label(target_frame, text="Roll:").grid(row=6, column=0, sticky="e")
        self.target_roll_entry = ttk.Entry(target_frame, width=12)
        self.target_roll_entry.insert(0, "0.0")
        self.target_roll_entry.grid(row=6, column=1, padx=5, pady=2)

        ttk.Label(target_frame, text="Pitch:").grid(row=7, column=0, sticky="e")
        self.target_pitch_entry = ttk.Entry(target_frame, width=12)
        self.target_pitch_entry.insert(0, "0.0")
        self.target_pitch_entry.grid(row=7, column=1, padx=5, pady=2)

        ttk.Label(target_frame, text="Yaw:").grid(row=8, column=0, sticky="e")
        self.target_yaw_entry = ttk.Entry(target_frame, width=12)
        self.target_yaw_entry.insert(0, "0.0")
        self.target_yaw_entry.grid(row=8, column=1, padx=5, pady=2)

        ttk.Button(target_frame, text="Use Current as Target", command=self.use_current_as_target).grid(row=9, column=0, columnspan=2, pady=5)

        # Control Buttons Frame
        control_frame = ttk.LabelFrame(self.root, text="Motion Control", padding=10)
        control_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        self.move_pos_btn = ttk.Button(control_frame, text="Move to Target (Position Only)",
                                       command=lambda: self.move_to_target(position_only=True),
                                       state="disabled")
        self.move_pos_btn.grid(row=0, column=0, padx=5, pady=5)

        self.move_pose_btn = ttk.Button(control_frame, text="Move to Target (Full Pose)",
                                        command=lambda: self.move_to_target(position_only=False),
                                        state="disabled")
        self.move_pose_btn.grid(row=0, column=1, padx=5, pady=5)

        self.home_btn = ttk.Button(control_frame, text="Go Home [0,0,0,0,0,0]",
                                   command=self.go_home,
                                   state="disabled")
        self.home_btn.grid(row=0, column=2, padx=5, pady=5)

        self.stop_btn = ttk.Button(control_frame, text="STOP",
                                   command=self.stop_motion,
                                   state="disabled")
        self.stop_btn.grid(row=0, column=3, padx=5, pady=5)

        # Log Frame
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")

        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, width=100, state="disabled")
        self.log_text.pack(fill="both", expand=True)

        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).pack(pady=5)

        # Configure grid weights
        self.root.grid_rowconfigure(3, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        # Initial log
        self.log("GUI initialized. Connect to robot to begin.")

    def log(self, message):
        """Add message to log window"""
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")

    def clear_log(self):
        """Clear log window"""
        self.log_text.config(state="normal")
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state="disabled")

    def update_current_display(self):
        """Update current pose display"""
        pos = self.robot.get_position(self.current_angles)
        pose = self.robot.get_pose(self.current_angles)

        self.curr_x_label.config(text=f"{pos[0]:.2f}")
        self.curr_y_label.config(text=f"{pos[1]:.2f}")
        self.curr_z_label.config(text=f"{pos[2]:.2f}")

        self.curr_roll_label.config(text=f"{np.rad2deg(pose[3]):.2f}")
        self.curr_pitch_label.config(text=f"{np.rad2deg(pose[4]):.2f}")
        self.curr_yaw_label.config(text=f"{np.rad2deg(pose[5]):.2f}")

    def use_current_as_target(self):
        """Copy current pose to target fields"""
        pos = self.robot.get_position(self.current_angles)
        pose = self.robot.get_pose(self.current_angles)

        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, f"{pos[0]:.2f}")

        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, f"{pos[1]:.2f}")

        self.target_z_entry.delete(0, tk.END)
        self.target_z_entry.insert(0, f"{pos[2]:.2f}")

        self.target_roll_entry.delete(0, tk.END)
        self.target_roll_entry.insert(0, f"{np.rad2deg(pose[3]):.2f}")

        self.target_pitch_entry.delete(0, tk.END)
        self.target_pitch_entry.insert(0, f"{np.rad2deg(pose[4]):.2f}")

        self.target_yaw_entry.delete(0, tk.END)
        self.target_yaw_entry.insert(0, f"{np.rad2deg(pose[5]):.2f}")

        self.log("Copied current pose to target fields")

    def connect(self):
        """Connect to ESP32"""
        port = self.port_entry.get()
        self.log(f"Connecting to {port}...")

        try:
            self.controller = MotionController(port, 115200, simulate_if_unavailable=True)

            if self.controller.simulation_mode:
                self.log("WARNING: Running in simulation mode (hardware not detected)")
                self.status_label.config(text="Status: Simulation Mode", foreground="orange")
            else:
                self.log("Connected successfully!")
                self.status_label.config(text="Status: Connected", foreground="green")

                # Query initial position
                time.sleep(1.0)
                angles = self.controller.get_current_position(max_retries=10)
                if angles:
                    self.current_angles = angles
                    self.log(f"Current joint angles: {[f'{a:.2f}' for a in angles]}")
                    self.update_current_display()
                else:
                    self.log("Could not query position, assuming home [0,0,0,0,0,0]")

            self.is_connected = True
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.move_pos_btn.config(state="normal")
            self.move_pose_btn.config(state="normal")
            self.home_btn.config(state="normal")

        except Exception as e:
            self.log(f"ERROR: Failed to connect: {e}")
            self.status_label.config(text="Status: Error", foreground="red")

    def disconnect(self):
        """Disconnect from ESP32"""
        if self.controller:
            self.controller.close()
            self.controller = None

        self.is_connected = False
        self.status_label.config(text="Status: Disconnected", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.move_pos_btn.config(state="disabled")
        self.move_pose_btn.config(state="disabled")
        self.home_btn.config(state="disabled")
        self.log("Disconnected")

    def query_position(self):
        """Query current position from ESP32"""
        if not self.is_connected or not self.controller:
            self.log("ERROR: Not connected")
            return

        if self.controller.simulation_mode:
            self.log("Simulation mode - position tracking from trajectory endpoints")
            return

        self.log("Querying position...")
        angles = self.controller.get_current_position(max_retries=10)

        if angles:
            self.current_angles = angles
            self.log(f"Joint angles: {[f'{a:.2f}' for a in angles]}")
            self.update_current_display()

            pos = self.robot.get_position(angles)
            self.log(f"End-effector: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f} mm")
        else:
            self.log("ERROR: Position query failed")

    def get_target_pose(self, position_only=False):
        """Get target pose from entry fields"""
        try:
            x = float(self.target_x_entry.get())
            y = float(self.target_y_entry.get())
            z = float(self.target_z_entry.get())

            if position_only:
                return np.array([x, y, z])
            else:
                roll = np.deg2rad(float(self.target_roll_entry.get()))
                pitch = np.deg2rad(float(self.target_pitch_entry.get()))
                yaw = np.deg2rad(float(self.target_yaw_entry.get()))
                return np.array([x, y, z, roll, pitch, yaw])

        except ValueError as e:
            self.log(f"ERROR: Invalid target values: {e}")
            return None

    def move_to_target(self, position_only=True):
        """Move to target pose (threaded)"""
        if not self.is_connected or self.is_moving:
            return

        target = self.get_target_pose(position_only)
        if target is None:
            return

        # Run in background thread
        thread = threading.Thread(target=self._move_to_target_thread, args=(target, position_only))
        thread.daemon = True
        thread.start()

    def _move_to_target_thread(self, target, position_only):
        """Background thread for motion execution"""
        self.is_moving = True
        self.stop_btn.config(state="normal")
        self.move_pos_btn.config(state="disabled")
        self.move_pose_btn.config(state="disabled")

        try:
            # Get current pose
            current_pos = self.robot.get_position(self.current_angles)

            if position_only:
                self.log(f"Planning move to: X={target[0]:.1f}, Y={target[1]:.1f}, Z={target[2]:.1f} mm (position-only)")
                trajectory = self.planner.move_linear(current_pos, target,
                                                     initial_angles=self.current_angles,
                                                     spacing_mm=5.0, verbose=False)
            else:
                current_pose = self.robot.get_pose(self.current_angles)
                self.log(f"Planning move to: X={target[0]:.1f}, Y={target[1]:.1f}, Z={target[2]:.1f} mm")
                self.log(f"  Orientation: R={np.rad2deg(target[3]):.1f}°, P={np.rad2deg(target[4]):.1f}°, Y={np.rad2deg(target[5]):.1f}°")
                trajectory = self.planner.move_linear_pose(current_pose, target,
                                                          initial_angles=self.current_angles,
                                                          spacing_mm=5.0, verbose=False)

            if trajectory is None:
                self.log("ERROR: Failed to generate trajectory")
                return

            self.log(f"Trajectory: {len(trajectory)} points, {trajectory[-1][0]/1000:.2f}s duration")

            # Validate velocity
            valid = validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0)
            if not valid:
                self.log("WARNING: Trajectory exceeds velocity limits!")

            # Execute
            self.log("Executing trajectory...")
            self.controller.execute_trajectory(trajectory)

            # Update current angles from trajectory endpoint
            self.current_angles = trajectory[-1][1]

            # Query actual position if not in simulation
            if not self.controller.simulation_mode:
                time.sleep(1.0)
                angles = self.controller.get_current_position(max_retries=5)
                if angles:
                    self.current_angles = angles
                    self.log("Position updated from ESP32")
                else:
                    self.log("Using trajectory endpoint for position")

            self.update_current_display()
            self.log("Move complete!")

        except Exception as e:
            self.log(f"ERROR: {e}")
            import traceback
            self.log(traceback.format_exc())

        finally:
            self.is_moving = False
            self.stop_btn.config(state="disabled")
            self.move_pos_btn.config(state="normal")
            self.move_pose_btn.config(state="normal")

    def go_home(self):
        """Return to home position [0,0,0,0,0,0]"""
        if not self.is_connected or self.is_moving:
            return

        self.log("Returning to home position...")

        # Set all target fields to zero
        for entry in [self.target_x_entry, self.target_y_entry, self.target_z_entry,
                     self.target_roll_entry, self.target_pitch_entry, self.target_yaw_entry]:
            entry.delete(0, tk.END)
            entry.insert(0, "0.0")

        # Calculate home position in Cartesian space
        home_angles = [0, 0, 0, 0, 0, 0]
        home_pos = self.robot.get_position(home_angles)

        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, f"{home_pos[0]:.2f}")
        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, f"{home_pos[1]:.2f}")
        self.target_z_entry.delete(0, tk.END)
        self.target_z_entry.insert(0, f"{home_pos[2]:.2f}")

        # Move to home
        self.move_to_target(position_only=True)

    def stop_motion(self):
        """Emergency stop (note: cannot stop mid-trajectory currently)"""
        self.log("STOP requested (will complete current segment)")
        self.is_moving = False


def main():
    """Main entry point"""
    root = tk.Tk()
    app = IKControllerGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
