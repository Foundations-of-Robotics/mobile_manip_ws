#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

class UwbPoseNode : public rclcpp::Node
{
public:
  UwbPoseNode()
  : Node("uwb_pose_publisher"),
    rng_(std::random_device{}())
  {
    this->declare_parameter<std::string>("input_topic", "sensors/gt");
    this->declare_parameter<std::string>("output_topic", "sensors/uwb/pose");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "map");
    this->declare_parameter<double>("position_noise_std", 0.08);
    this->declare_parameter<double>("orientation_noise_std", 0.001);
    this->declare_parameter<double>("publish_rate", 50.0);

    const auto input_topic = this->get_parameter("input_topic").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    position_noise_std_ = this->get_parameter("position_noise_std").as_double();
    orientation_noise_std_ = this->get_parameter("orientation_noise_std").as_double();
    publish_rate_ = std::max(this->get_parameter("publish_rate").as_double(), 1e-6);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(output_topic, sensor_qos);
    vel_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sensors/uwb/vel", sensor_qos);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      input_topic,
      sensor_qos,
      std::bind(&UwbPoseNode::pose_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&UwbPoseNode::run, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to: %s -> publishing pose on: %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  double gaussian(double stddev)
  {
    if (stddev <= 0.0) {
      return 0.0;
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(rng_);
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    const auto now = this->get_clock()->now();

    if (!have_prev_pose_) {
      prev_pose_ = *msg;
      prev_pose_time_ = now;
      latest_vel_ = geometry_msgs::msg::Twist();
      latest_pose_ = *msg;
      have_prev_pose_ = true;
      have_latest_pose_ = true;
      return;
    }

    const double dt = (now - prev_pose_time_).seconds();
    if (dt > 0.0) {
      latest_vel_.linear.x = (msg->position.x - prev_pose_.position.x) / dt;
      latest_vel_.linear.y = (msg->position.y - prev_pose_.position.y) / dt;
      latest_vel_.linear.z = (msg->position.z - prev_pose_.position.z) / dt;
      latest_vel_.angular.x = 0.0;
      latest_vel_.angular.y = 0.0;
      latest_vel_.angular.z = 0.0;
    }

    prev_pose_ = *msg;
    prev_pose_time_ = now;
    latest_pose_ = *msg;
    have_latest_pose_ = true;
  }

  void run()
  {
    if (!have_latest_pose_) {
      return;
    }

    const auto stamp = this->get_clock()->now();

    geometry_msgs::msg::PoseWithCovarianceStamped pose_out;
    pose_out.header.stamp = stamp;
    pose_out.header.frame_id = base_frame_;
    pose_out.pose.pose = latest_pose_;

    if (position_noise_std_ > 0.0) {
      pose_out.pose.pose.position.x += gaussian(position_noise_std_);
      pose_out.pose.pose.position.y += gaussian(position_noise_std_);
      pose_out.pose.pose.position.z += gaussian(position_noise_std_);
    }

    if (orientation_noise_std_ > 0.0) {
      auto & q = pose_out.pose.pose.orientation;
      q.x += gaussian(orientation_noise_std_);
      q.y += gaussian(orientation_noise_std_);
      q.z += gaussian(orientation_noise_std_);
      q.w += gaussian(orientation_noise_std_);

      const double magnitude = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (magnitude > 0.0) {
        q.x /= magnitude;
        q.y /= magnitude;
        q.z /= magnitude;
        q.w /= magnitude;
      }
    }

    pose_pub_->publish(pose_out);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose_out.pose.pose;
    odom.twist.twist = latest_vel_;
    vel_pub_->publish(odom);
  }

  std::string odom_frame_;
  std::string base_frame_;
  double position_noise_std_{0.0};
  double orientation_noise_std_{0.0};
  double publish_rate_{50.0};

  bool have_prev_pose_{false};
  bool have_latest_pose_{false};
  geometry_msgs::msg::Pose latest_pose_;
  geometry_msgs::msg::Pose prev_pose_;
  geometry_msgs::msg::Twist latest_vel_;
  rclcpp::Time prev_pose_time_{0, 0, RCL_ROS_TIME};
  std::mt19937 rng_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
