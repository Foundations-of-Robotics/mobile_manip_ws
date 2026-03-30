#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

class PoseToOdom : public rclcpp::Node
{
public:
  PoseToOdom()
  : Node("t265_odom_publisher"),
    rng_(std::random_device{}())
  {
    this->declare_parameter<std::string>("input_topic", "sensors/gt");
    this->declare_parameter<std::string>("output_topic", "sensors/t265/pose/sample");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "t265_link");
    this->declare_parameter<double>("position_noise_std", 0.005);
    this->declare_parameter<double>("orientation_noise_std", 0.0001);
    this->declare_parameter<double>("publish_rate", 20.0);

    const auto input_topic = this->get_parameter("input_topic").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    position_noise_std_ = this->get_parameter("position_noise_std").as_double();
    orientation_noise_std_ = this->get_parameter("orientation_noise_std").as_double();
    publish_rate_ = std::max(this->get_parameter("publish_rate").as_double(), 1e-6);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, sensor_qos);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      input_topic,
      sensor_qos,
      std::bind(&PoseToOdom::pose_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&PoseToOdom::run, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to: %s -> publishing odom on: %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  struct RollPitchYaw
  {
    double roll;
    double pitch;
    double yaw;
  };

  static double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double heading_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static RollPitchYaw rpy_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    constexpr double kHalfPi = 1.5707963267948966;
    RollPitchYaw rpy{};

    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    rpy.roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0) {
      rpy.pitch = std::copysign(kHalfPi, sinp);
    } else {
      rpy.pitch = std::asin(sinp);
    }

    rpy.yaw = heading_from_quaternion(q);
    return rpy;
  }

  static geometry_msgs::msg::Quaternion quaternion_from_rpy(
    double roll,
    double pitch,
    double yaw)
  {
    geometry_msgs::msg::Quaternion q;

    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }

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

    if (!initialized_) {
      offset_ = *msg;
      initialized_ = true;
    }

    if (!have_prev_pose_) {
      prev_pose_ = *msg;
      prev_yaw_ = heading_from_quaternion(msg->orientation);
      prev_pose_time_ = now;
      latest_vel_ = geometry_msgs::msg::Twist();
      latest_pose_ = *msg;
      have_prev_pose_ = true;
      have_latest_pose_ = true;
      return;
    }

    const double dt = (now - prev_pose_time_).seconds();
    if (dt > 0.0) {
      const double yaw = heading_from_quaternion(msg->orientation);
      latest_vel_.linear.x = (msg->position.x - prev_pose_.position.x) / dt;
      latest_vel_.linear.y = (msg->position.y - prev_pose_.position.y) / dt;
      latest_vel_.linear.z = (msg->position.z - prev_pose_.position.z) / dt;
      latest_vel_.angular.x = 0.0;
      latest_vel_.angular.y = 0.0;
      latest_vel_.angular.z = normalize_angle(yaw - prev_yaw_) / dt;
      prev_yaw_ = yaw;
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

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = latest_pose_;

    if (position_noise_std_ > 0.0) {
      odom.pose.pose.position.x =
        (odom.pose.pose.position.x - offset_.position.x) + gaussian(position_noise_std_);
      odom.pose.pose.position.y =
        (odom.pose.pose.position.y - offset_.position.y) + gaussian(position_noise_std_);
      odom.pose.pose.position.z = 0.0;
    }

    {
      const auto rpy = rpy_from_quaternion(odom.pose.pose.orientation);
      const double yaw_offset = heading_from_quaternion(offset_.orientation);
      const double yaw_noise =
        orientation_noise_std_ > 0.0 ? gaussian(orientation_noise_std_) : 0.0;
      odom.pose.pose.orientation = quaternion_from_rpy(
        rpy.roll,
        rpy.pitch,
        normalize_angle(rpy.yaw - yaw_offset + yaw_noise));
    }

    odom.twist.twist = latest_vel_;
    odom_pub_->publish(odom);
  }

  std::string odom_frame_;
  std::string base_frame_;
  double position_noise_std_{0.0};
  double orientation_noise_std_{0.0};
  double publish_rate_{20.0};

  bool initialized_{false};
  bool have_prev_pose_{false};
  bool have_latest_pose_{false};
  geometry_msgs::msg::Pose offset_;
  geometry_msgs::msg::Pose latest_pose_;
  geometry_msgs::msg::Pose prev_pose_;
  geometry_msgs::msg::Twist latest_vel_;
  double prev_yaw_{0.0};
  rclcpp::Time prev_pose_time_{0, 0, RCL_ROS_TIME};
  std::mt19937 rng_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseToOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
