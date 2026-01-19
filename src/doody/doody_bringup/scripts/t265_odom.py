#!/usr/bin/env python3
 
import math
import random
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

#!/usr/bin/env python3
# /home/ws/src/doody/doody_bringup/scripts/t265_odom.py
# ROS2 Humble node: subscribe to geometry_msgs/Pose and publish nav_msgs/Odometry



class PoseToOdom(Node):
    def __init__(self):
        super().__init__('t265_odom_publisher')

        # configurable parameters
        #self.declare_parameter('input_topic', 'sensors/gt') #directly from gazebo is delayed and scale ?!? to implement in Jazzy
        self.declare_parameter('input_topic', 'platform/odom')
        self.declare_parameter('output_topic', 'sensors/t265/pose/sample')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # standard deviation for per-field gaussian noise applied to pose
        # small default so behavior changes only slightly; adjust via ros2 param
        self.declare_parameter('position_noise_std', 0.005)
        self.declare_parameter('orientation_noise_std', 0.0001)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_pub = self.create_publisher(Odometry, output_topic, 1)
        self.pose_sub = self.create_subscription(Odometry, input_topic, self.pose_callback, 1)
        
        self.initialised = False
        self.offset = Pose()

        self.get_logger().info(f'Subscribing to: {input_topic} -> publishing odom on: {output_topic}')
        # retrieved numeric value for noise standard deviation
        self.position_noise_std = float(self.get_parameter('position_noise_std').get_parameter_value().double_value)
        self.orientation_noise_std = float(self.get_parameter('orientation_noise_std').get_parameter_value().double_value)

    def pose_callback(self, pose_msg: Odometry):
        if not self.initialised:
            self.offset = deepcopy(pose_msg.pose.pose)
            self.initialised = True
        
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # copy pose (don't mutate incoming message) and apply gaussian noise
        #odom.pose.pose = deepcopy(pose_msg)
        odom = deepcopy(pose_msg)

        std = self.position_noise_std
        if std and std > 0.0:
            # add gaussian noise to position
            odom.pose.pose.position.x = (odom.pose.pose.position.x- self.offset.position.x)+random.gauss(0.0, std)
            odom.pose.pose.position.y = (odom.pose.pose.position.y- self.offset.position.y)+random.gauss(0.0, std)
            odom.pose.pose.position.z = (odom.pose.pose.position.z- self.offset.position.z)+random.gauss(0.0, std)

        std = self.orientation_noise_std
        if std and std > 0.0:
            # add gaussian noise to quaternion components
            q = odom.pose.pose.orientation
            q.x = (q.x-self.offset.orientation.x)+random.gauss(0.0, std)
            q.y = (q.y-self.offset.orientation.y)+random.gauss(0.0, std)
            q.z = (q.z-self.offset.orientation.z)+random.gauss(0.0, std)
            q.w = (q.w-self.offset.orientation.w)+random.gauss(0.0, std)

            # renormalize quaternion to unit length to avoid invalid rotations
            mag = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if mag > 0.0:
                q.x /= mag
                q.y /= mag
                q.z /= mag
                q.w /= mag

        # default covariances (zeros); adjust if needed
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36

        # no velocity information available from Pose alone -> zeros
        odom.twist.twist = Twist()

        self.odom_pub.publish(odom)


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