#!/usr/bin/env python3

import re

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from mobile_manip.msg import PoseStampedArray
import time


class TagPosePublisher(Node):
    def __init__(self):
        super().__init__('tag_pose_publisher')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('array_topic', 'tag_pose')
        self.tf_topic = self.get_parameter('tf_topic').value
        self.array_topic = self.get_parameter('array_topic').value

        self.sub = self.create_subscription(
            TFMessage,
            self.tf_topic,
            self.tf_callback,
            10)

        self.pub_array = self.create_publisher(PoseStampedArray, self.array_topic, 10)
        self.get_logger().info(
            f"subscribing to {self.tf_topic}, publishing arrays on {self.array_topic}")
        # precompile regex for tag frame names (tag followed by digits)
        self.tag_re = re.compile(r"^tag\d+$")
        # storage for last seen tags: {tag_id: (PoseStamped, timestamp, source_frame)}
        self._tag_map = {}
        self.frame_id = ''
        # timer for periodic publication
        self._timer = self.create_timer(0.1, self._timer_callback)  # 10 Hz

    def tf_callback(self, msg: TFMessage):
        # update stored poses for every recognized tag
        now = time.time()
        for t in msg.transforms:
            if self.tag_re.match(t.child_frame_id):
                self.frame_id = t.header.frame_id
                ps = PoseStamped()
                ps.header = t.header
                ps.header.frame_id = t.child_frame_id
                ps.pose.position.x = t.transform.translation.x
                ps.pose.position.y = t.transform.translation.y
                ps.pose.position.z = t.transform.translation.z
                ps.pose.orientation.x = t.transform.rotation.x
                ps.pose.orientation.y = t.transform.rotation.y
                ps.pose.orientation.z = t.transform.rotation.z
                ps.pose.orientation.w = t.transform.rotation.w
                # store with timestamp and source frame (camera)
                self._tag_map[t.child_frame_id] = (ps, now, t.header.frame_id)

    def _timer_callback(self):
        # remove tags not seen for more than 5 seconds
        cutoff = time.time() - 5.0
        to_delete = [tid for tid,(ps,ts,frame) in self._tag_map.items() if ts < cutoff]
        for tid in to_delete:
            del self._tag_map[tid]
        msg = PoseStampedArray()
        if self._tag_map:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.poses = [ps for ps,_,_ in self._tag_map.values()]
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.poses = []
        self.pub_array.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
