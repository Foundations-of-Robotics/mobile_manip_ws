#!/usr/bin/env python3
 
import math
import random
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

#!/usr/bin/env python3
# /home/ws/src/doody/doody_bringup/scripts/t265_odom.py
# ROS2 Humble node: subscribe to geometry_msgs/Pose and publish nav_msgs/Odometry



class PoseToOdom(Node):
    def __init__(self):
        super().__init__('t265_odom_publisher')

        # configurable parameters
        self.declare_parameter('input_topic', 'sensors/gt') #directly from gazebo is delayed and scale ?!? to implement in Jazzy
        #self.declare_parameter('input_topic', 'platform/odom')
        self.declare_parameter('output_topic', 'sensors/uwb/pose')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'map')
        # standard deviation for per-field gaussian noise applied to pose
        # small default so behavior changes only slightly; adjust via ros2 param
        self.declare_parameter('position_noise_std', 0.08)
        self.declare_parameter('orientation_noise_std', 0.001)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 1)
        self.vel_pub = self.create_publisher(Odometry, 'sensors/uwb/vel', 1)
        self.pose_sub = self.create_subscription(Pose, input_topic, self.pose_callback, 1)
        
        # storage for latest received pose and computed velocity
        self.latest_pose = None
        self._prev_pose = None
        self._prev_pose_time_ns = None
        self.latest_vel = None

        # publish rate (Hz) and timer to publish pose+velocity regularly
        self.declare_parameter('publish_rate', 50.0)
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        # create timer that calls run() at the specified frequency
        self.create_timer(1.0 / max(1e-6, self.publish_rate), self.run)

        self.get_logger().info(f'Subscribing to: {input_topic} -> publishing odom on: {output_topic}')
        # retrieved numeric value for noise standard deviation
        self.position_noise_std = float(self.get_parameter('position_noise_std').get_parameter_value().double_value)
        self.orientation_noise_std = float(self.get_parameter('orientation_noise_std').get_parameter_value().double_value)

    def pose_callback(self, pose_msg: Pose):
        # store latest pose and compute velocity by differencing with previous pose
        now_ns = self.get_clock().now().nanoseconds

        # on first message, initialize storage and zero velocity
        if self._prev_pose is None:
            self._prev_pose = deepcopy(pose_msg)
            self._prev_pose_time_ns = now_ns
            self.latest_vel = Twist()
            self.latest_pose = deepcopy(pose_msg)
            return

        dt = (now_ns - self._prev_pose_time_ns) / 1e9
        if dt <= 0.0:
            # avoid divide-by-zero; keep previous velocity
            pass
        else:
            vx = (pose_msg.position.x - self._prev_pose.position.x) / dt
            vy = (pose_msg.position.y - self._prev_pose.position.y) / dt
            vz = (pose_msg.position.z - self._prev_pose.position.z) / dt
            vel = Twist()
            vel.linear.x = vx
            vel.linear.y = vy
            vel.linear.z = vz
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.latest_vel = vel

        # update stored pose and timestamp for next delta
        self._prev_pose = deepcopy(pose_msg)
        self._prev_pose_time_ns = now_ns
        self.latest_pose = deepcopy(pose_msg)

    def run(self):
        # publish the most recently stored pose (with noise) and odometry containing velocity
        if self.latest_pose is None:
            return

        pose_out = PoseWithCovarianceStamped()
        pose_out.header.stamp = self.get_clock().now().to_msg()
        pose_out.header.frame_id = self.base_frame

        # copy pose (don't mutate stored message) and apply gaussian noise
        pose_out.pose.pose = deepcopy(self.latest_pose)

        std = self.position_noise_std
        if std and std > 0.0:
            pose_out.pose.pose.position.x += random.gauss(0.0, std)
            pose_out.pose.pose.position.y += random.gauss(0.0, std)
            pose_out.pose.pose.position.z += random.gauss(0.0, std)

        std = self.orientation_noise_std
        if std and std > 0.0:
            q = pose_out.pose.pose.orientation
            q.x += random.gauss(0.0, std)
            q.y += random.gauss(0.0, std)
            q.z += random.gauss(0.0, std)
            q.w += random.gauss(0.0, std)
            mag = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if mag > 0.0:
                q.x /= mag
                q.y /= mag
                q.z /= mag
                q.w /= mag

        pose_out.pose.covariance = [0.0] * 36
        self.pose_pub.publish(pose_out)

        # publish odometry message containing velocity
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        # place the (noisy) pose into odom.pose
        odom.pose.pose = deepcopy(pose_out.pose.pose)
        # attach last-computed velocity (or zero if not available)
        odom.twist.twist = deepcopy(self.latest_vel) if self.latest_vel is not None else Twist()
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        self.vel_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()