#!/usr/bin/env python3
"""
6-DOF Inverse Kinematics Controller using Damped Least Squares Method

Features:
- Forward kinematics using DH parameters
- Numerical Jacobian calculation
- Damped Least Squares (Levenberg-Marquardt) IK solver
- Joint angle limits enforcement
- Smooth Cartesian trajectory generation
- Integration with ESP32 motion controller

DH Parameter Convention (Modified DH):
    Each row: [alpha, a, d, theta_offset]
    - alpha: Link twist (rotation about x_{i-1})
    - a: Link length (translation along x_{i-1})
    - d: Link offset (translation along z_i)
    - theta_offset: Joint angle offset
"""

import numpy as np
import time
from motion_controller import MotionController, validate_trajectory_velocity


class RobotKinematics:
    """6-DOF Robot Arm Kinematics using DH Parameters"""

    def __init__(self, dh_params, joint_limits):
        """
        Initialize robot kinematics

        Args:
            dh_params: List of [alpha, a, d, theta_offset] for each joint
            joint_limits: List of [min_angle, max_angle] for each joint (degrees)
        """
        self.dh_params = np.array(dh_params, dtype=np.float64)
        self.joint_limits = np.array(joint_limits, dtype=np.float64)
        self.num_joints = len(dh_params)

        # Convert alpha and theta_offset from degrees to radians in DH params
        self.dh_params_rad = self.dh_params.copy()
        self.dh_params_rad[:, 0] = np.deg2rad(self.dh_params[:, 0])  # alpha
        self.dh_params_rad[:, 3] = np.deg2rad(self.dh_params[:, 3])  # theta_offset

        print(f"Robot initialized with {self.num_joints} joints")
        print(f"DH Parameters (alpha°, a, d, theta_offset°):")
        for i, params in enumerate(self.dh_params):
            print(f"  Joint {i+1}: {params}")
        print(f"Joint Limits (degrees):")
        for i, limits in enumerate(self.joint_limits):
            print(f"  Joint {i+1}: [{limits[0]:6.1f}, {limits[1]:6.1f}]")

    def dh_transform(self, alpha, a, d, theta):
        """
        Calculate DH transformation matrix (Modified DH Convention)

        Args:
            alpha: Link twist (radians)
            a: Link length
            d: Link offset
            theta: Joint angle (radians)

        Returns:
            4x4 homogeneous transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Modified DH convention
        T = np.array([
            [ct,    -st,    0,      a],
            [st*ca, ct*ca,  -sa,    -sa*d],
            [st*sa, ct*sa,  ca,     ca*d],
            [0,     0,      0,      1]
        ])

        return T

    def forward_kinematics(self, joint_angles_deg):
        """
        Calculate end-effector pose from joint angles

        Args:
            joint_angles_deg: List/array of 6 joint angles in degrees

        Returns:
            4x4 transformation matrix (position and orientation)
        """
        joint_angles_rad = np.deg2rad(joint_angles_deg)

        # Start with identity matrix
        T = np.eye(4)

        # Multiply transformations for each joint
        for i in range(self.num_joints):
            alpha, a, d, theta_offset = self.dh_params_rad[i]
            theta = joint_angles_rad[i] + theta_offset

            T_i = self.dh_transform(alpha, a, d, theta)
            T = T @ T_i

        return T

    def get_position(self, joint_angles_deg):
        """
        Get end-effector XYZ position from joint angles

        Args:
            joint_angles_deg: List/array of 6 joint angles in degrees

        Returns:
            [x, y, z] position in mm
        """
        T = self.forward_kinematics(joint_angles_deg)
        return T[:3, 3]

    def get_pose(self, joint_angles_deg):
        """
        Get end-effector pose (position + orientation) from joint angles

        Args:
            joint_angles_deg: List/array of 6 joint angles in degrees

        Returns:
            [x, y, z, roll, pitch, yaw] in mm and radians
        """
        T = self.forward_kinematics(joint_angles_deg)
        pos = T[:3, 3]
        rpy = self.rotation_matrix_to_rpy(T[:3, :3])
        return np.concatenate([pos, rpy])

    def rotation_matrix_to_rpy(self, R):
        """
        Convert rotation matrix to roll-pitch-yaw angles (ZYX convention)

        Args:
            R: 3x3 rotation matrix

        Returns:
            [roll, pitch, yaw] in radians
        """
        # ZYX Euler angles (common robotics convention)
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))

        # Handle gimbal lock
        if np.abs(np.cos(pitch)) > 1e-6:
            yaw = np.arctan2(R[1, 0], R[0, 0])
            roll = np.arctan2(R[2, 1], R[2, 2])
        else:
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0

        return np.array([roll, pitch, yaw])

    def rpy_to_rotation_matrix(self, roll, pitch, yaw):
        """
        Convert roll-pitch-yaw angles to rotation matrix (ZYX convention)

        Args:
            roll, pitch, yaw: Rotation angles in radians

        Returns:
            3x3 rotation matrix
        """
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        # ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])

        return R

    def compute_jacobian(self, joint_angles_deg, epsilon=1e-5, position_only=False):
        """
        Compute numerical Jacobian matrix (full 6x6 or position-only 3x6)

        Args:
            joint_angles_deg: Current joint angles in degrees
            epsilon: Small perturbation for numerical differentiation
            position_only: If True, return 3x6 position Jacobian only

        Returns:
            6x6 or 3x6 Jacobian matrix
        """
        if position_only:
            J = np.zeros((3, self.num_joints))
            pos_current = self.get_position(joint_angles_deg)

            # Numerical differentiation for each joint
            for i in range(self.num_joints):
                angles_perturbed = np.array(joint_angles_deg, dtype=np.float64)
                angles_perturbed[i] += epsilon

                pos_perturbed = self.get_position(angles_perturbed)
                J[:, i] = (pos_perturbed - pos_current) / epsilon

            return J
        else:
            # Full 6×6 Jacobian (position + orientation)
            J = np.zeros((6, self.num_joints))
            pose_current = self.get_pose(joint_angles_deg)

            # Numerical differentiation for each joint
            for i in range(self.num_joints):
                angles_perturbed = np.array(joint_angles_deg, dtype=np.float64)
                angles_perturbed[i] += epsilon

                pose_perturbed = self.get_pose(angles_perturbed)

                # Position derivatives
                J[:3, i] = (pose_perturbed[:3] - pose_current[:3]) / epsilon

                # Orientation derivatives (handle angle wrapping)
                orientation_delta = pose_perturbed[3:] - pose_current[3:]
                # Wrap angles to [-π, π]
                orientation_delta = np.arctan2(np.sin(orientation_delta), np.cos(orientation_delta))
                J[3:, i] = orientation_delta / epsilon

            return J

    def enforce_joint_limits(self, joint_angles_deg):
        """
        Clamp joint angles to limits

        Args:
            joint_angles_deg: Array of joint angles in degrees

        Returns:
            Clamped joint angles
        """
        clamped = np.clip(joint_angles_deg,
                         self.joint_limits[:, 0],
                         self.joint_limits[:, 1])
        return clamped

    def inverse_kinematics(self, target, initial_guess=None,
                          max_iterations=100, tolerance_pos=0.1, tolerance_ori=0.01,
                          damping=0.01, position_only=False, verbose=False):
        """
        Solve inverse kinematics using Damped Least Squares (Levenberg-Marquardt)

        Args:
            target: Target [x, y, z] (mm) for position-only, or
                    [x, y, z, roll, pitch, yaw] (mm, rad) for full pose
            initial_guess: Initial joint angles (degrees), None for zeros
            max_iterations: Maximum iterations
            tolerance_pos: Position error tolerance in mm
            tolerance_ori: Orientation error tolerance in radians
            damping: Damping factor (lambda) for numerical stability
            position_only: If True, only control position (3-DOF), ignore orientation
            verbose: Print iteration details

        Returns:
            joint_angles (degrees) or None if failed
        """
        # Initialize with guess or zeros
        if initial_guess is None:
            q = np.zeros(self.num_joints)
        else:
            q = np.array(initial_guess, dtype=np.float64)

        target = np.array(target, dtype=np.float64)

        # Determine if position-only or full pose IK
        if position_only or len(target) == 3:
            position_only = True
            target_dim = 3
        else:
            target_dim = 6

        for iteration in range(max_iterations):
            # Current pose or position
            if position_only:
                current = self.get_position(q)
                error = target - current
                error_norm = np.linalg.norm(error)
                converged = error_norm < tolerance_pos
            else:
                current = self.get_pose(q)
                error_pos = target[:3] - current[:3]
                error_ori = target[3:] - current[3:]
                # Wrap orientation error to [-π, π]
                error_ori = np.arctan2(np.sin(error_ori), np.cos(error_ori))
                error = np.concatenate([error_pos, error_ori])
                error_norm_pos = np.linalg.norm(error_pos)
                error_norm_ori = np.linalg.norm(error_ori)
                converged = (error_norm_pos < tolerance_pos) and (error_norm_ori < tolerance_ori)

            if verbose and iteration % 10 == 0:
                if position_only:
                    print(f"  Iteration {iteration}: pos_error = {error_norm:.3f} mm")
                else:
                    print(f"  Iteration {iteration}: pos_error = {error_norm_pos:.3f} mm, ori_error = {error_norm_ori:.4f} rad")

            # Check convergence
            if converged:
                if verbose:
                    if position_only:
                        print(f"  ✓ Converged in {iteration} iterations, error = {error_norm:.4f} mm")
                    else:
                        print(f"  ✓ Converged in {iteration} iterations")
                        print(f"    pos_error = {error_norm_pos:.4f} mm, ori_error = {error_norm_ori:.6f} rad")
                return self.enforce_joint_limits(q)

            # Compute Jacobian
            J = self.compute_jacobian(q, position_only=position_only)

            # Damped Least Squares (Levenberg-Marquardt)
            # Δq = (J^T J + λI)^-1 J^T Δx
            JTJ = J.T @ J
            lambda_I = damping * np.eye(self.num_joints)
            delta_q = np.linalg.solve(JTJ + lambda_I, J.T @ error)

            # Update joint angles (with step size limiting for stability)
            max_step_deg = 5.0  # Max 5 degrees per iteration
            step_scale = min(1.0, max_step_deg / (np.linalg.norm(delta_q) + 1e-10))
            q += delta_q * step_scale

            # Enforce joint limits
            q = self.enforce_joint_limits(q)

        if verbose:
            if position_only:
                print(f"  ✗ Failed to converge after {max_iterations} iterations, error = {error_norm:.4f} mm")
            else:
                print(f"  ✗ Failed to converge after {max_iterations} iterations")
                print(f"    pos_error = {error_norm_pos:.4f} mm, ori_error = {error_norm_ori:.6f} rad")

        # Return best solution even if not fully converged
        return self.enforce_joint_limits(q)


class CartesianMotionPlanner:
    """Generate smooth Cartesian trajectories"""

    def __init__(self, robot_kinematics, max_velocity_deg_per_sec=30.0):
        """
        Initialize motion planner

        Args:
            robot_kinematics: RobotKinematics instance
            max_velocity_deg_per_sec: Maximum joint velocity limit
        """
        self.robot = robot_kinematics
        self.max_velocity = max_velocity_deg_per_sec

    def interpolate_cartesian(self, start_xyz, end_xyz, spacing_mm=5.0):
        """
        Interpolate straight line in Cartesian space (position only)

        Args:
            start_xyz: Start position [x, y, z] in mm
            end_xyz: End position [x, y, z] in mm
            spacing_mm: Distance between waypoints

        Returns:
            List of [x, y, z] waypoints
        """
        start = np.array(start_xyz)
        end = np.array(end_xyz)

        distance = np.linalg.norm(end - start)
        num_points = max(2, int(distance / spacing_mm) + 1)

        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start + t * (end - start)
            waypoints.append(point)

        return waypoints

    def interpolate_pose(self, start_pose, end_pose, spacing_mm=5.0):
        """
        Interpolate in Cartesian space with both position and orientation

        Args:
            start_pose: Start [x, y, z, roll, pitch, yaw] (mm, rad)
            end_pose: End [x, y, z, roll, pitch, yaw] (mm, rad)
            spacing_mm: Distance between waypoints

        Returns:
            List of [x, y, z, roll, pitch, yaw] waypoints
        """
        start = np.array(start_pose)
        end = np.array(end_pose)

        # Calculate distance based on position only
        distance = np.linalg.norm(end[:3] - start[:3])
        num_points = max(2, int(distance / spacing_mm) + 1)

        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)

            # Linear interpolation for position
            pos = start[:3] + t * (end[:3] - start[:3])

            # SLERP-like interpolation for orientation
            # Wrap angle differences to [-π, π]
            ori_delta = end[3:] - start[3:]
            ori_delta = np.arctan2(np.sin(ori_delta), np.cos(ori_delta))
            ori = start[3:] + t * ori_delta

            pose = np.concatenate([pos, ori])
            waypoints.append(pose)

        return waypoints

    def cartesian_to_joint_trajectory(self, waypoints, initial_angles=None,
                                     update_period_ms=50, position_only=None, verbose=False):
        """
        Convert Cartesian waypoints to joint space trajectory with timing

        Args:
            waypoints: List of [x, y, z] or [x, y, z, roll, pitch, yaw]
            initial_angles: Starting joint angles (degrees), None for zeros
            update_period_ms: Time between updates (50ms = 20Hz)
            position_only: Force position-only mode, auto-detect if None
            verbose: Print IK details

        Returns:
            List of (timestamp_ms, [angles]) tuples
        """
        trajectory = []
        current_angles = initial_angles if initial_angles is not None else np.zeros(6)

        # Auto-detect if position-only based on waypoint dimensions
        if position_only is None:
            position_only = len(waypoints[0]) == 3

        if verbose:
            mode = "position-only" if position_only else "full pose"
            print(f"\nSolving {mode} IK for {len(waypoints)} waypoints...")

        # Solve IK for each waypoint
        joint_waypoints = []
        for i, target in enumerate(waypoints):
            if verbose:
                if position_only:
                    print(f"Waypoint {i+1}/{len(waypoints)}: pos={target}")
                else:
                    print(f"Waypoint {i+1}/{len(waypoints)}: pos={target[:3]}, rpy={np.rad2deg(target[3:])}°")

            # Use previous solution as initial guess for continuity
            angles = self.robot.inverse_kinematics(
                target,
                initial_guess=current_angles,
                position_only=position_only,
                verbose=verbose
            )

            if angles is None:
                print(f"  ✗ IK failed for waypoint {i+1}")
                return None

            joint_waypoints.append(angles)
            current_angles = angles

        # Calculate timing based on maximum joint movement
        if verbose:
            print("\nCalculating trajectory timing...")

        # Calculate duration needed for each segment
        timestamps = [0]  # Start at t=0
        current_time = 0

        for i in range(1, len(joint_waypoints)):
            prev_angles = joint_waypoints[i-1]
            curr_angles = joint_waypoints[i]

            # Find maximum angular change across all joints
            angle_deltas = np.abs(curr_angles - prev_angles)
            max_delta = np.max(angle_deltas)

            # Calculate minimum time needed for this segment
            # Account for S-curve 2× velocity multiplier
            min_time_sec = (max_delta * 2.0) / self.max_velocity

            # Round up to nearest update period
            num_updates = max(1, int(np.ceil(min_time_sec * 1000 / update_period_ms)))
            segment_duration_ms = num_updates * update_period_ms

            current_time += segment_duration_ms
            timestamps.append(current_time)

        # Interpolate joint angles between waypoints with S-curve
        trajectory = []
        for i in range(len(joint_waypoints) - 1):
            start_angles = joint_waypoints[i]
            end_angles = joint_waypoints[i+1]
            start_time = timestamps[i]
            end_time = timestamps[i+1]
            duration = end_time - start_time

            # Generate intermediate points with S-curve interpolation
            num_points = duration // update_period_ms + 1
            for j in range(num_points):
                t_ms = start_time + j * update_period_ms
                t_norm = (t_ms - start_time) / duration

                # S-curve interpolation (ease-in-out)
                if t_norm < 0.5:
                    progress = 2 * t_norm * t_norm
                else:
                    progress = 1 - 2 * (1 - t_norm) * (1 - t_norm)

                # Interpolate angles
                angles = start_angles + progress * (end_angles - start_angles)
                trajectory.append((int(t_ms), angles.tolist()))

        # Add final point
        trajectory.append((int(timestamps[-1]), joint_waypoints[-1].tolist()))

        if verbose:
            print(f"\n✓ Generated trajectory with {len(trajectory)} points over {timestamps[-1]/1000:.2f} seconds")

        return trajectory

    def move_linear(self, start_xyz, end_xyz, initial_angles=None, spacing_mm=5.0, verbose=True):
        """
        Generate trajectory for linear move in Cartesian space (position only)

        Args:
            start_xyz: Start position [x, y, z]
            end_xyz: End position [x, y, z]
            initial_angles: Starting joint angles (degrees)
            spacing_mm: Waypoint spacing
            verbose: Print progress

        Returns:
            Trajectory or None if failed
        """
        if verbose:
            print(f"\n{'='*60}")
            print(f"Planning linear move (position only):")
            print(f"  Start: [{start_xyz[0]:.1f}, {start_xyz[1]:.1f}, {start_xyz[2]:.1f}] mm")
            print(f"  End:   [{end_xyz[0]:.1f}, {end_xyz[1]:.1f}, {end_xyz[2]:.1f}] mm")
            distance = np.linalg.norm(np.array(end_xyz) - np.array(start_xyz))
            print(f"  Distance: {distance:.1f} mm")
            print(f"{'='*60}")

        # Interpolate Cartesian path
        waypoints = self.interpolate_cartesian(start_xyz, end_xyz, spacing_mm)

        # Convert to joint trajectory (position-only)
        trajectory = self.cartesian_to_joint_trajectory(waypoints, initial_angles,
                                                       position_only=True, verbose=verbose)

        return trajectory

    def move_linear_pose(self, start_pose, end_pose, initial_angles=None, spacing_mm=5.0, verbose=True):
        """
        Generate trajectory for linear move with orientation control

        Args:
            start_pose: Start [x, y, z, roll, pitch, yaw] (mm, rad)
            end_pose: End [x, y, z, roll, pitch, yaw] (mm, rad)
            initial_angles: Starting joint angles (degrees)
            spacing_mm: Waypoint spacing
            verbose: Print progress

        Returns:
            Trajectory or None if failed
        """
        if verbose:
            print(f"\n{'='*60}")
            print(f"Planning linear move (full pose):")
            print(f"  Start pos: [{start_pose[0]:.1f}, {start_pose[1]:.1f}, {start_pose[2]:.1f}] mm")
            print(f"  Start ori: [{np.rad2deg(start_pose[3]):.1f}°, {np.rad2deg(start_pose[4]):.1f}°, {np.rad2deg(start_pose[5]):.1f}°]")
            print(f"  End pos:   [{end_pose[0]:.1f}, {end_pose[1]:.1f}, {end_pose[2]:.1f}] mm")
            print(f"  End ori:   [{np.rad2deg(end_pose[3]):.1f}°, {np.rad2deg(end_pose[4]):.1f}°, {np.rad2deg(end_pose[5]):.1f}°]")
            distance = np.linalg.norm(np.array(end_pose[:3]) - np.array(start_pose[:3]))
            print(f"  Distance: {distance:.1f} mm")
            print(f"{'='*60}")

        # Interpolate Cartesian path with orientation
        waypoints = self.interpolate_pose(start_pose, end_pose, spacing_mm)

        # Convert to joint trajectory (full pose)
        trajectory = self.cartesian_to_joint_trajectory(waypoints, initial_angles,
                                                       position_only=False, verbose=verbose)

        return trajectory


def demo_inverse_kinematics():
    """Demonstration of inverse kinematics controller"""

    # DH Parameters: [alpha, a, d, theta_offset]
    # Format: [link_twist, link_length, link_offset, joint_offset]
    dh_params = [
        [0,     0,   25, 0],    # Joint 1: Base rotation
        [90,    0,   0,  0],    # Joint 2: Shoulder
        [0,     32,  0,  0],    # Joint 3: Elbow
        [0,     23,  13, 0],    # Joint 4: Wrist 1
        [90,    0,   13, 0],    # Joint 5: Wrist 2
        [90,    0,   11, 0],    # Joint 6: Wrist 3 (end-effector)
    ]

    # Joint limits (degrees): [min, max]
    joint_limits = [
        [-170, 170],    # Joint 1
        [-120, 120],    # Joint 2
        [-150, 150],    # Joint 3
        [-180, 180],    # Joint 4
        [-120, 120],    # Joint 5
        [-180, 180],    # Joint 6
    ]

    # Initialize robot kinematics
    robot = RobotKinematics(dh_params, joint_limits)

    # Initialize motion planner
    planner = CartesianMotionPlanner(robot, max_velocity_deg_per_sec=30.0)

    # Test forward kinematics
    print(f"\n{'='*60}")
    print("Testing Forward Kinematics")
    print(f"{'='*60}")
    test_angles = [0, 0, 0, 0, 0, 0]
    print(f"Joint angles: {test_angles}")
    pos = robot.get_position(test_angles)
    print(f"End-effector position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] mm")

    # Test inverse kinematics - Position only
    print(f"\n{'='*60}")
    print("Testing Inverse Kinematics - Position Only (3-DOF)")
    print(f"{'='*60}")
    target_xyz = pos  # Use FK result as IK target
    print(f"Target position: [{target_xyz[0]:.2f}, {target_xyz[1]:.2f}, {target_xyz[2]:.2f}] mm")
    solved_angles = robot.inverse_kinematics(target_xyz, position_only=True, verbose=True)
    if solved_angles is not None:
        print(f"Solved angles: {[f'{a:.2f}°' for a in solved_angles]}")
        achieved_pos = robot.get_position(solved_angles)
        error = np.linalg.norm(achieved_pos - target_xyz)
        print(f"Position error: {error:.4f} mm")

    # Test inverse kinematics - Full pose
    print(f"\n{'='*60}")
    print("Testing Inverse Kinematics - Full Pose (6-DOF)")
    print(f"{'='*60}")
    target_pose = robot.get_pose(test_angles)  # Use FK result as IK target
    print(f"Target position: [{target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f}] mm")
    print(f"Target orientation: [{np.rad2deg(target_pose[3]):.2f}°, {np.rad2deg(target_pose[4]):.2f}°, {np.rad2deg(target_pose[5]):.2f}°]")
    solved_angles = robot.inverse_kinematics(target_pose, position_only=False, verbose=True)
    if solved_angles is not None:
        print(f"Solved angles: {[f'{a:.2f}°' for a in solved_angles]}")
        achieved_pose = robot.get_pose(solved_angles)
        error_pos = np.linalg.norm(achieved_pose[:3] - target_pose[:3])
        error_ori = np.linalg.norm(achieved_pose[3:] - target_pose[3:])
        print(f"Position error: {error_pos:.4f} mm")
        print(f"Orientation error: {error_ori:.6f} rad ({np.rad2deg(error_ori):.4f}°)")

    # Connect to ESP32
    print(f"\n{'='*60}")
    print("Connecting to ESP32")
    print(f"{'='*60}")
    controller = MotionController('/dev/ttyACM0', 115200, simulate_if_unavailable=True)

    # Query current position
    current_angles = None
    if not controller.simulation_mode:
        print("\nQuerying current motor position...")
        time.sleep(1.0)
        current_angles = controller.get_current_position(max_retries=10)
        if current_angles:
            print(f"Current angles: {[f'{a:.2f}°' for a in current_angles]}")
            current_xyz = robot.get_position(current_angles)
            print(f"Current XYZ: [{current_xyz[0]:.1f}, {current_xyz[1]:.1f}, {current_xyz[2]:.1f}] mm")
        else:
            print("Warning: Could not query position, using home position")
            current_angles = [0, 0, 0, 0, 0, 0]
    else:
        current_angles = [0, 0, 0, 0, 0, 0]

    # Calculate current end-effector position
    current_xyz = robot.get_position(current_angles)

    print("\n" + "="*60)
    print("Cartesian motion test will start in 3 seconds")
    print("Press Ctrl+C now to abort if motors are in unsafe position")
    print("="*60)
    time.sleep(3)

    # Demo 1: Position-only Cartesian move
    print(f"\n{'='*60}")
    print("Demo 1: Position-Only Cartesian Move (3-DOF)")
    print(f"{'='*60}")
    target_xyz = current_xyz + np.array([20.0, 0, 0])  # Move 20mm in X

    trajectory = planner.move_linear(
        current_xyz,
        target_xyz,
        initial_angles=current_angles,
        spacing_mm=5.0,
        verbose=True
    )

    if trajectory:
        print(f"\n✓ Trajectory generated successfully")
        print(f"  Points: {len(trajectory)}")
        print(f"  Duration: {trajectory[-1][0]/1000:.2f} seconds")

        # Validate velocity limits
        print("\nValidating velocity limits...")
        valid = validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0)
        if valid:
            print("✓ Trajectory is within velocity limits")
        else:
            print("⚠ Warning: Velocity violations detected!")

        # Execute trajectory
        print("\nExecuting trajectory...")
        controller.execute_trajectory(trajectory)

        # Query final position
        if not controller.simulation_mode:
            time.sleep(1.0)
            final_angles = controller.get_current_position()
            if final_angles:
                final_xyz = robot.get_position(final_angles)
                print(f"\nFinal position: [{final_xyz[0]:.1f}, {final_xyz[1]:.1f}, {final_xyz[2]:.1f}] mm")
                target_error = np.linalg.norm(final_xyz - target_xyz)
                print(f"Target error: {target_error:.2f} mm")
                # Update current state
                current_angles = final_angles
                current_xyz = final_xyz
    else:
        print("✗ Failed to generate trajectory")

    # Wait before next demo
    print("\nWaiting 3 seconds before next demo...")
    time.sleep(3)

    # Demo 2: Full pose Cartesian move (with orientation control)
    print(f"\n{'='*60}")
    print("Demo 2: Full Pose Cartesian Move (6-DOF)")
    print(f"{'='*60}")

    # Get current full pose
    current_pose = robot.get_pose(current_angles)
    print(f"Current pose:")
    print(f"  Position: [{current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f}] mm")
    print(f"  Orientation: [{np.rad2deg(current_pose[3]):.1f}°, {np.rad2deg(current_pose[4]):.1f}°, {np.rad2deg(current_pose[5]):.1f}°]")

    # Move back to original position with a 45° rotation in yaw
    target_pose = current_pose.copy()
    target_pose[0] -= 20.0  # Move back 20mm in X
    target_pose[5] += np.deg2rad(45.0)  # Rotate 45° around Z axis (yaw)

    trajectory = planner.move_linear_pose(
        current_pose,
        target_pose,
        initial_angles=current_angles,
        spacing_mm=5.0,
        verbose=True
    )

    if trajectory:
        print(f"\n✓ Trajectory generated successfully")
        print(f"  Points: {len(trajectory)}")
        print(f"  Duration: {trajectory[-1][0]/1000:.2f} seconds")

        # Validate velocity limits
        print("\nValidating velocity limits...")
        valid = validate_trajectory_velocity(trajectory, max_velocity_deg_per_sec=30.0)
        if valid:
            print("✓ Trajectory is within velocity limits")
        else:
            print("⚠ Warning: Velocity violations detected!")

        # Execute trajectory
        print("\nExecuting trajectory...")
        controller.execute_trajectory(trajectory)

        # Query final position
        if not controller.simulation_mode:
            time.sleep(1.0)
            final_angles = controller.get_current_position()
            if final_angles:
                final_pose = robot.get_pose(final_angles)
                print(f"\nFinal pose:")
                print(f"  Position: [{final_pose[0]:.1f}, {final_pose[1]:.1f}, {final_pose[2]:.1f}] mm")
                print(f"  Orientation: [{np.rad2deg(final_pose[3]):.1f}°, {np.rad2deg(final_pose[4]):.1f}°, {np.rad2deg(final_pose[5]):.1f}°]")
                pos_error = np.linalg.norm(final_pose[:3] - target_pose[:3])
                ori_error = np.linalg.norm(final_pose[3:] - target_pose[3:])
                print(f"Position error: {pos_error:.2f} mm")
                print(f"Orientation error: {ori_error:.4f} rad ({np.rad2deg(ori_error):.2f}°)")
    else:
        print("✗ Failed to generate trajectory")

    # Close controller
    controller.close()
    print("\nDemo complete!")


if __name__ == '__main__':
    try:
        demo_inverse_kinematics()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
