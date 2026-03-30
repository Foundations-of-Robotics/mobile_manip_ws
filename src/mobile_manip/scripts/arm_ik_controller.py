#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
import numpy as np


class ArmIKController(Node):
    def __init__(self):
        super().__init__('arm_ik_controller')
        
        # Declare parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('pose_topic', '/mobile_manip/arm_command_node/ee_pose_cmd')
        self.declare_parameter('trajectory_topic', '/mobile_manip/arm_0_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('joint_state_topic', '/mobile_manip/platform/joint_states')
        self.declare_parameter('end_effector_frame', 'tool_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('trajectory_time', 5.0)
        self.declare_parameter('max_joint_speed', 0.35)
        self.declare_parameter('min_trajectory_time', 0.5)
        
        # Get parameters
        urdf_path = self.get_parameter('urdf_path').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.ee_frame = self.get_parameter('end_effector_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.trajectory_time = self.get_parameter('trajectory_time').value
        self.max_joint_speed = float(self.get_parameter('max_joint_speed').value)
        self.min_trajectory_time = float(self.get_parameter('min_trajectory_time').value)
        
        # Load robot model with pinocchio
        if not urdf_path:
            self.get_logger().error("URDF path not provided!")
            return
            
        try:
            self.get_logger().info(f"Loading URDF from: {urdf_path}")
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.get_logger().info(f"Successfully loaded model with {self.model.njoints} joints")
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {str(e)}")
            return
        
        # Get end effector frame ID
        # retrieve frame ID(s); handle duplicates
        matches = [(idx,f) for idx,f in enumerate(self.model.frames) if f.name == self.ee_frame]
        if not matches:
            self.get_logger().error(f"Frame '{self.ee_frame}' not found in model!")
            return
        if len(matches) > 1:
            idx,f = matches[0]
            self.get_logger().warn(
                f"Several frames named '{self.ee_frame}' found; using first (id={idx}, type={f.type})" )
        self.ee_frame_id = matches[0][0]
        
        # Get joint names (excluding the free flyer)
        self.joint_names = []
        for i in range(1, self.model.njoints):
            joint = self.model.joints[i]
            if joint.nv == 1:  # Only single DOF joints
                # pinocchio JointModel uses shortname() to get the joint's name
                self.joint_names.append(joint.shortname())
        
        self.get_logger().info(f"Loaded {len(self.joint_names)} joints: {self.joint_names}")
        
        # Subscribe to target pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )
        
        # Publisher for joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic,
            10
        )
        
        # Subscribe to current joint states for dynamic duration scaling.
        self.current_joint_positions = {}
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )
        
        # IK solver parameters
        self.max_iterations = 1000
        self.tolerance = 1e-3
        self.ik_restarts = 6
        
        # Controller-specific joint order expected on joint_trajectory topic.
        self.controller_joint_names = [
            'arm_0_joint_1',
            'arm_0_joint_2',
            'arm_0_joint_3',
            'arm_0_joint_4',
            'arm_0_joint_5',
            'arm_0_joint_6',
        ]

        # Approximate reachable workspace limits for END_EFFECTOR frame (meters)
        self.workspace_limits = {
            'x': (-0.629, 0.634),
            'y': (-0.634, 0.633),
            'z': (-0.368, 0.877),
            'radius': (0.0, 0.634),     # sqrt(x^2 + y^2)
            'distance': (0.0, 0.878),   # sqrt(x^2 + y^2 + z^2)
        }
        
        self.get_logger().info(f"Subscribed to {self.pose_topic}")
        self.get_logger().info(f"Subscribed to {self.joint_state_topic}")
        self.get_logger().info(f"Publishing to {self.trajectory_topic}")
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint positions by name."""
        useful_joints = set(self.controller_joint_names)
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            joint_name = msg.name[i]
            if joint_name in useful_joints:
                self.current_joint_positions[joint_name] = msg.position[i]
    
    def pose_callback(self, msg: PoseStamped):
        """Callback for target pose subscription"""
        try:
            # Extract pose
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            x, y, z = position
            radius = float(np.linalg.norm(position[:2]))
            distance = float(np.linalg.norm(position))
            
            # Workspace guard: skip IK if target is outside known reachable bounds
            violations = []
            x_min, x_max = self.workspace_limits['x']
            y_min, y_max = self.workspace_limits['y']
            z_min, z_max = self.workspace_limits['z']
            r_min, r_max = self.workspace_limits['radius']
            d_min, d_max = self.workspace_limits['distance']
            
            if not (x_min <= x <= x_max):
                violations.append(f"x={x:.3f} not in [{x_min:.3f}, {x_max:.3f}]")
            if not (y_min <= y <= y_max):
                violations.append(f"y={y:.3f} not in [{y_min:.3f}, {y_max:.3f}]")
            if not (z_min <= z <= z_max):
                violations.append(f"z={z:.3f} not in [{z_min:.3f}, {z_max:.3f}]")
            if not (r_min <= radius <= r_max):
                violations.append(f"radius={radius:.3f} not in [{r_min:.3f}, {r_max:.3f}]")
            if not (d_min <= distance <= d_max):
                violations.append(f"distance={distance:.3f} not in [{d_min:.3f}, {d_max:.3f}]")
            
            if violations:
                self.get_logger().warn(
                    "Target pose outside reachable workspace; skipping IK. " + "; ".join(violations)
                )
                return

            orientation = np.array([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ])
            
            # Create target transformation matrix
            target_R = pin.Quaternion(orientation).toRotationMatrix()
            target_T = pin.SE3(target_R, position)
            
            # Solve IK
            q_sol = self.solve_ik(target_T)
            
            if q_sol is not None:
                # Publish trajectory
                self.publish_trajectory(q_sol)
                self.get_logger().info(f"IK solution found: {q_sol}")
            else:
                self.get_logger().warn("IK solver failed to find solution")
                
        except Exception as e:
            self.get_logger().error(f"Error in pose callback: {str(e)}")
    
    def solve_ik(self, target_T, q_init=None):
        """Solve inverse kinematics using pinocchio (position only)"""
        target_pos = target_T.translation
        q_lower = self.model.lowerPositionLimit
        q_upper = self.model.upperPositionLimit
        
        best_q = None
        best_err = float('inf')

        def ee_pos(q_cfg):
            pin.forwardKinematics(self.model, self.data, q_cfg)
            pin.updateFramePlacements(self.model, self.data)
            return self.data.oMf[self.ee_frame_id].translation.copy()

        # Multi-start helps avoid local minima/singular starts.
        for attempt in range(self.ik_restarts):
            if attempt == 0 and q_init is not None:
                q = q_init.copy()
            elif attempt == 1:
                q = pin.neutral(self.model)
            else:
                q = np.random.uniform(q_lower, q_upper)

            damping = 1e-3
            eps = 1e-6

            for _ in range(self.max_iterations):
                current_pos = ee_pos(q)
                err_pos = target_pos - current_pos
                pos_err = np.linalg.norm(err_pos)
                if pos_err < best_err:
                    best_err = pos_err
                    best_q = q.copy()

                if pos_err < self.tolerance:
                    return q

                # Numerical position Jacobian d(position)/dq to avoid frame-order ambiguity.
                Jpos = np.zeros((3, self.model.nv))
                for j in range(self.model.nv):
                    q_pert = q.copy()
                    q_pert[j] += eps
                    q_pert = np.clip(q_pert, q_lower, q_upper)
                    p_pert = ee_pos(q_pert)
                    Jpos[:, j] = (p_pert - current_pos) / eps

                # Damped least squares: dq = J^T (J J^T + λI)^-1 * e
                H = Jpos @ Jpos.T + damping * np.eye(3)
                dq = Jpos.T @ np.linalg.solve(H, err_pos)
                
                # Backtracking line search for stable decrease of position error.
                step = 1.0
                improved = False
                for _ in range(8):
                    q_try = pin.integrate(self.model, q, dq * step)
                    q_try = np.clip(q_try, q_lower, q_upper)
                    err_try = np.linalg.norm(target_pos - ee_pos(q_try))
                    if err_try < pos_err:
                        q = q_try
                        improved = True
                        break
                    step *= 0.5
                
                # If no improving step found, restart with a new seed.
                if not improved:
                    break

        self.get_logger().warn(
            f"IK did not converge after {self.ik_restarts} restarts x {self.max_iterations} iterations; "
            f"best position error={best_err:.4f} m"
        )
        return None
    
    def publish_trajectory(self, q_solution):
        """Publish joint trajectory command"""
        q_arm = np.asarray(q_solution, dtype=float).reshape(-1)
        n_ctrl = len(self.controller_joint_names)
        if q_arm.shape[0] < n_ctrl:
            self.get_logger().error(
                f"IK solution has {q_arm.shape[0]} joints, expected at least {n_ctrl}"
            )
            return
        
        # Keep IK joint order unchanged and publish exactly controller joint count.
        ordered_positions = q_arm[:n_ctrl].tolist()
        
        # Create trajectory message
        traj = JointTrajectory()
        #traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = self.base_frame
        traj.joint_names = self.controller_joint_names
        
        # Duration proportional to max commanded joint distance from current state.
        duration = float(self.trajectory_time)
        if self.max_joint_speed > 0.0:
            missing = [jn for jn in traj.joint_names if jn not in self.current_joint_positions]
            if missing:
                self.get_logger().warn(
                    f"Missing current joint states for {missing}; using fixed trajectory_time={duration:.3f}s"
                )
            else:
                max_delta = max(
                    abs(ordered_positions[i] - self.current_joint_positions[jn])
                    for i, jn in enumerate(traj.joint_names)
                )
                duration = max(self.min_trajectory_time, max_delta / self.max_joint_speed)
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = ordered_positions
        point.velocities = [0.0] * len(ordered_positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points = [point]
        
        # Publish
        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ArmIKController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
