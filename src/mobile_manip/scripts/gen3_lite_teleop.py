#!/usr/bin/env python3

# Original code by Alexandre Lapointe
# Adapted for Gen3lite Lite by Ali Imran

import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ros2_control / controllers
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration

# Trajectory message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Gripper action
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


class PS4ArmTwistTeleop(Node):
    def __init__(self):
        super().__init__('ps4_arm_twist_teleop')

        # --- Topics ---
        self.declare_parameter('twist_topic', '/twist_controller/commands')
        self.declare_parameter('joy_topic', '/mobile_manip/joy_teleop/joy')
        self.declare_parameter('trajectory_topic', '/joint_trajectory_controller/joint_trajectory')

        # --- Axes (keep your base mapping; adjust via params) ---
        self.declare_parameter('axis_lx', 1)
        self.declare_parameter('axis_ly', 0)
        self.declare_parameter('axis_rx', 4)
        self.declare_parameter('axis_r2', 5)

        # --- Buttons ---
        self.declare_parameter('btn_deadman', 5)           # R1
        self.declare_parameter('btn_toggle_select', 7)     # Select/Share (set per your /joy)
        self.declare_parameter('btn_home', 0)              # Cross (X) to send trajectory
        # New: PS4 Square (open) and Circle (close); common mapping: X=0, O=1, △=2, □=3
        self.declare_parameter('btn_open', 3)              # Triangle
        self.declare_parameter('btn_close', 1)             # Circle
        self.declare_parameter('btn_retract', 2)           # Square

        # --- Controller names & CM service namespace ---
        self.declare_parameter('twist_controller_name', 'twist_controller')
        self.declare_parameter('trajectory_controller_name', 'joint_trajectory_controller')
        self.declare_parameter('controller_manager_ns', '/controller_manager')

        # --- Scaling ---
        self.declare_parameter('scale_vx', 0.25)
        self.declare_parameter('scale_vy', 0.25)
        self.declare_parameter('scale_vz', 0.25)

        # --- Misc behavior ---
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('publish_zero_when_released', True)
        self.declare_parameter('invert_lx', True)
        self.declare_parameter('invert_ly', True)
        self.declare_parameter('invert_rx', True)
        self.declare_parameter('publish_rate_hz', 60.0)

        # --- Trajectory params ---
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ])
        self.declare_parameter('home_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('retract_positions', [0.0, 0.37, 2.622, -1.53, -0.698, -1.518])
        self.declare_parameter('home_time_sec', 3)

        # --- Gripper action params ---
        self.declare_parameter('gripper_action_name', '/gen3_lite_2f_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_pos', 0.1) 
        self.declare_parameter('gripper_close_pos', 0.8)  
        self.declare_parameter('gripper_max_effort', 100.0) 

        # --- Get params ---
        self.twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value

        self.axis_lx = self.get_parameter('axis_lx').value
        self.axis_ly = self.get_parameter('axis_ly').value
        self.axis_rx = self.get_parameter('axis_rx').value
        self.axis_r2 = self.get_parameter('axis_r2').value

        self.btn_deadman = self.get_parameter('btn_deadman').value
        self.btn_toggle_select = self.get_parameter('btn_toggle_select').value
        self.btn_home = self.get_parameter('btn_home').value
        self.btn_open = self.get_parameter('btn_open').value
        self.btn_close = self.get_parameter('btn_close').value
        self.btn_retract = self.get_parameter('btn_retract').value

        self.twist_controller_name = self.get_parameter('twist_controller_name').get_parameter_value().string_value
        self.trajectory_controller_name = self.get_parameter('trajectory_controller_name').get_parameter_value().string_value
        self.controller_manager_ns = self.get_parameter('controller_manager_ns').get_parameter_value().string_value

        self.scale_vx = float(self.get_parameter('scale_vx').value)
        self.scale_vy = float(self.get_parameter('scale_vy').value)
        self.scale_vz = float(self.get_parameter('scale_vz').value)

        self.deadzone = float(self.get_parameter('deadzone').value)
        self.publish_zero_when_released = bool(self.get_parameter('publish_zero_when_released').value)

        self.invert_lx = bool(self.get_parameter('invert_lx').value)
        self.invert_ly = bool(self.get_parameter('invert_ly').value)
        self.invert_rx = bool(self.get_parameter('invert_rx').value)

        self.joint_names: List[str] = list(self.get_parameter('joint_names').value)
        self.home_positions: List[float] = [float(v) for v in list(self.get_parameter('home_positions').value)]
        self.retract_positions: List[float] = [float(v) for v in list(self.get_parameter('retract_positions').value)]
        self.home_time_sec = int(self.get_parameter('home_time_sec').value)

        self.gripper_action_name = self.get_parameter('gripper_action_name').get_parameter_value().string_value
        self.gripper_open_pos = float(self.get_parameter('gripper_open_pos').value)
        self.gripper_close_pos = float(self.get_parameter('gripper_close_pos').value)
        self.gripper_max_effort = float(self.get_parameter('gripper_max_effort').value)

        rate = float(self.get_parameter('publish_rate_hz').value)

        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        twist_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subs/Pubs
        self.sub = self.create_subscription(Joy, joy_topic, self.joy_cb, joy_qos)
        self.pub = self.create_publisher(Twist, self.twist_topic, twist_qos)
        self.traj_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

        # Controller Manager client
        self.switch_srv_name = f'{self.controller_manager_ns}/switch_controller'
        self.switch_cli = self.create_client(SwitchController, self.switch_srv_name)

        # Gripper Action client
        self.gripper_client: ActionClient = ActionClient(self, GripperCommand, self.gripper_action_name)

        # State
        self.last_cmd: Twist = Twist()
        self.deadman_active: bool = False
        self.have_joy: bool = False

        self._prev_toggle_btn = 0
        self._prev_home_btn = 0
        self._prev_retract_btn = 0 
        self._prev_open_btn = 0
        self._prev_close_btn = 0
        self._twist_active = False 

        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        # Send retract trajectory on startup (with a small delay to ensure controller is ready)
        self.startup_timer = self.create_timer(5.0, self._startup_retract)

        self.get_logger().info(
            f"PS4ArmTwistTeleop started. Joy: {joy_topic} | Twist: {self.twist_topic} | "
            f"Traj: {self.trajectory_topic} | CM: {self.switch_srv_name} | GripperAction: {self.gripper_action_name}"
        )

    def _startup_retract(self):
        """Called once on startup to send the arm to retract position."""
        self.startup_timer.cancel()  # Only run once
        self.get_logger().info("Startup: Sending arm to retract position...")
        self.send_retract_trajectory()

    @staticmethod
    def _apply_deadzone(x: float, dz: float) -> float:
        if abs(x) < dz:
            return 0.0
        return (abs(x) - dz) / (1.0 - dz) * (1 if x > 0 else -1)

    def joy_cb(self, msg: Joy):
        self.have_joy = True

        # Helpers
        def get_axis(idx: int) -> float:
            if 0 <= idx < len(msg.axes):
                return msg.axes[idx]
            return 0.0

        def get_btn(idx: int) -> int:
            if 0 <= idx < len(msg.buttons):
                return msg.buttons[idx]
            return 0

        # --- Toggle on Select (edge) ---
        toggle_now = get_btn(self.btn_toggle_select)
        
        if toggle_now == 1 and self._prev_toggle_btn == 0:
            self.toggle_controllers()
        self._prev_toggle_btn = toggle_now

        # --- Home trajectory on Cross (edge) and retract on Square (edge) when trajectory controller active ---
        home_now = get_btn(self.btn_home)
        if (home_now == 1 and self._prev_home_btn == 0) and (not self._twist_active):
            self.send_home_trajectory()
        self._prev_home_btn = home_now

        retract_now = get_btn(self.btn_retract)
        if (retract_now == 1 and self._prev_retract_btn == 0) and (not self._twist_active):
            self.send_retract_trajectory()
        self._prev_retract_btn = retract_now

        # --- Gripper on Triangle (open) and Circle (close) (edge) ---
        open_now = get_btn(self.btn_open)
        if open_now == 1 and self._prev_open_btn == 0:
            self.send_gripper_command(self.gripper_open_pos)
        self._prev_open_btn = open_now

        close_now = get_btn(self.btn_close)
        if close_now == 1 and self._prev_close_btn == 0:
            self.send_gripper_command(self.gripper_close_pos)
        self._prev_close_btn = close_now

        # --- Normal teleop (for twist controller) ---
        deadman = get_btn(self.btn_deadman) == 1

        lx = self._apply_deadzone(get_axis(self.axis_lx), self.deadzone)  # -> vy
        ly = self._apply_deadzone(get_axis(self.axis_ly), self.deadzone)  # -> vx
        rx = self._apply_deadzone(get_axis(self.axis_rx), self.deadzone)  # -> vz
        r2_axis = get_axis(self.axis_r2)
        if self.invert_lx:
            lx = -lx
        if self.invert_ly:
            ly = -ly
        if self.invert_rx:
            rx = -rx

        vx = ly * self.scale_vx
        vy = lx * self.scale_vy
        vz = rx * self.scale_vz

        self.deadman_active = deadman

        cmd = Twist()
        if r2_axis == -1 and self._twist_active:
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.linear.z = vz  
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0

        self.last_cmd = cmd

    def publish_cmd(self):
        if not self.have_joy:
            return
        if self.deadman_active or self.publish_zero_when_released:
            self.pub.publish(self.last_cmd)

    def toggle_controllers(self):
        """Toggle between twist_controller and joint_trajectory_controller."""
        if self._twist_active:
            # Safety: zero twist before switching away from twist
            try:
                self.pub.publish(Twist())
            except Exception:
                pass

        req = SwitchController.Request()
        if self._twist_active:
            req.activate_controllers = [self.trajectory_controller_name]
            req.deactivate_controllers = [self.twist_controller_name]
        else:
            req.activate_controllers = [self.twist_controller_name]
            req.deactivate_controllers = [self.trajectory_controller_name]

        req.strictness = 1  # STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=0, nanosec=0)

        if not self.switch_cli.service_is_ready():
            self.get_logger().warn(f"Waiting for {self.switch_srv_name} ...")
            if not self.switch_cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().error(f"{self.switch_srv_name} not available. Toggle aborted.")
                return

        future = self.switch_cli.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
            except Exception as e:
                self.get_logger().error(f"SwitchController call failed: {e}")
                return
            if resp.ok:
                self._twist_active = not self._twist_active
                active = (self.twist_controller_name if self._twist_active
                          else self.trajectory_controller_name)
                self.get_logger().info(f"Switched OK. Active: {active}")
            else:
                self.get_logger().error("SwitchController response: ok = False")

        future.add_done_callback(_done_cb)

    def send_home_trajectory(self):
        """Publish a one-shot JointTrajectory to the trajectory controller."""
        if len(self.joint_names) != len(self.home_positions):
            self.get_logger().error(
                f"joint_names ({len(self.joint_names)}) and home_positions ({len(self.home_positions)}) length mismatch."
            )
            return

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(self.home_positions)
        pt.time_from_start = Duration(sec=int(self.home_time_sec), nanosec=0)
        traj.points = [pt]

        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"Sent home trajectory ({self.home_time_sec}s) to {self.trajectory_topic}"
        )

    def send_retract_trajectory(self):
        """Publish a one-shot JointTrajectory to retract the arm."""
        if len(self.joint_names) != len(self.retract_positions):
            self.get_logger().error(
                f"joint_names ({len(self.joint_names)}) and retract_positions ({len(self.retract_positions)}) length mismatch."
            )
            return

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(self.retract_positions)
        pt.time_from_start = Duration(sec=int(self.home_time_sec), nanosec=0)
        traj.points = [pt]

        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"Sent retract trajectory ({self.home_time_sec}s) to {self.trajectory_topic}"
        )

    # -------- Gripper helpers --------
    def send_gripper_command(self, position: float):
        """Send a GripperCommand goal (async)."""
        # Try an immediate non-blocking check first (avoids long waits in teleop loop)
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(f"Gripper action server {self.gripper_action_name} not ready")
            # Try a short wait once to reduce spam if it just started
            self.gripper_client.wait_for_server(timeout_sec=0.5)
            if not self.gripper_client.server_is_ready():
                self.get_logger().error("Gripper action still not ready. Skipping command.")
                return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.gripper_max_effort)

        self.get_logger().info(f"Gripper goal -> position={goal.command.position:.4f}, "
                               f"max_effort={goal.command.max_effort}")

        send_future = self.gripper_client.send_goal_async(goal, feedback_callback=self._gripper_feedback_cb)
        send_future.add_done_callback(self._gripper_goal_response_cb)

    def _gripper_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_cb)

    def _gripper_feedback_cb(self, feedback_msg):
        # feedback_msg.feedback is control_msgs/action/GripperCommand_Feedback
        # Often empty for this action; keep for future extensions/logging.
        pass

    def _gripper_result_cb(self, future):
        try:
            result = future.result().result  # control_msgs/GripperCommand_Result
            # Many drivers leave this mostly empty; log success anyway.
            self.get_logger().info("Gripper goal finished")
        except Exception as e:
            self.get_logger().error(f"Gripper result error: {e}")


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = PS4ArmTwistTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist()) 
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
