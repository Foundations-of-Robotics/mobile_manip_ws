#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class FusedOdomTfNode : public rclcpp::Node
{
public:
  FusedOdomTfNode()
  : Node("fused_odom_tf"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    this->declare_parameter<std::string>("fused_odom_topic", "/mobile_manip/sensors/fused_odometry");
    fused_odom_topic_ = this->get_parameter("fused_odom_topic").as_string();
    lookup_parent_frame_ = "odom";
    lookup_child_frame_ = "base_link";
    output_parent_frame_ = "odom";
    output_child_frame_ = "map";

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(timer_interface);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      fused_odom_topic_,
      10,
      std::bind(&FusedOdomTfNode::fused_odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Node ready. fused_odom=%s, tf=%s->%s",
      fused_odom_topic_.c_str(),
      output_parent_frame_.c_str(),
      output_child_frame_.c_str());
  }

private:
  void fused_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped odom_to_base_link;
    try {
      odom_to_base_link = tf_buffer_.lookupTransform(
        lookup_parent_frame_,
        lookup_child_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN(this->get_logger(), "Lookup TF odom->base_link failed: %s", exc.what());
      return;
    }

    tf2::Transform tf_odom_base;
    tf2::Transform tf_map_base;
    tf2::fromMsg(odom_to_base_link.transform, tf_odom_base);

    geometry_msgs::msg::Transform map_to_base_msg;
    map_to_base_msg.translation.x = msg->pose.pose.position.x;
    map_to_base_msg.translation.y = msg->pose.pose.position.y;
    map_to_base_msg.translation.z = msg->pose.pose.position.z;
    map_to_base_msg.rotation = msg->pose.pose.orientation;
    tf2::fromMsg(map_to_base_msg, tf_map_base);

    const tf2::Transform tf_odom_map = tf_odom_base * tf_map_base.inverse();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = output_parent_frame_;
    tf_msg.child_frame_id = output_child_frame_;
    tf_msg.transform = tf2::toMsg(tf_odom_map);
    tf_broadcaster_.sendTransform(tf_msg);
  }

  std::string fused_odom_topic_;
  std::string lookup_parent_frame_;
  std::string lookup_child_frame_;
  std::string output_parent_frame_;
  std::string output_child_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusedOdomTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
