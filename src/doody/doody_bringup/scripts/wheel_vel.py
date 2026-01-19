#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# /home/ws/src/doody/doody_bringup/scripts/wheel_vel.py


class WheelVelNode(Node):
    def __init__(self):
        super().__init__("wheel_vel_node")

        # parameters
        self.declare_parameter("joint_states_topic", "platform/wheels/cmd")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("left_wheel_name", "front_left_wheel_joint")
        self.declare_parameter("right_wheel_name", "front_right_wheel_joint")
        self.declare_parameter("wheel_radius", 0.0984/2)  # meters
        self.declare_parameter("wheel_separation", 0.3765)  # meters (distance between whw/cmeels)

        js_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.left_name = self.get_parameter("left_wheel_name").get_parameter_value().string_value
        self.right_name = self.get_parameter("right_wheel_name").get_parameter_value().string_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(JointState, js_topic, self.joint_state_cb, 10)

        self.get_logger().info(
            f"Listening to '{js_topic}', publishing Twist to '{cmd_topic}'. "
            f"Left='{self.left_name}', Right='{self.right_name}', "
            f"r={self.wheel_radius}, sep={self.wheel_separation}"
        )

    def joint_state_cb(self, msg: JointState):
        # Find indices of the two wheel joints
        try:
            idx_l = msg.name.index(self.left_name)
            idx_r = msg.name.index(self.right_name)
        except ValueError:
            # one or both joints not present in this message
            self.get_logger().debug("Wheel joint names not found in JointState message.")
            return

        # Get angular velocities (rad/s). JointState.velocity may be empty or shorter than names.
        try:
            omega_l = msg.velocity[idx_l]
            omega_r = msg.velocity[idx_r]
        except (IndexError, TypeError):
            self.get_logger().warn("JointState.velocity does not contain expected entries.")
            return

        # Compute robot linear and angular velocities
        # v = r * (omega_l + omega_r) / 2
        # omega_z = r * (omega_r - omega_l) / wheel_separation
        v_linear = self.wheel_radius * 0.5 * (omega_l + omega_r)
        v_angular_z = self.wheel_radius * (omega_r - omega_l) / max(self.wheel_separation, 1e-6)

        twist = Twist()
        twist.linear.x = float(v_linear)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(v_angular_z)

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()