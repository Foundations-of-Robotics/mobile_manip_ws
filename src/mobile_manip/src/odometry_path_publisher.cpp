#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

class OdometryPathPublisherNode : public rclcpp::Node
{
public:
  OdometryPathPublisherNode()
  : Node("odometry_path_publisher")
  {
    this->declare_parameter<double>("scan_period_s", 2.0);
    this->declare_parameter<int>("path_buffer_size", 1000);
    this->declare_parameter<std::string>("default_frame_id", "map");

    scan_period_s_ = std::max(this->get_parameter("scan_period_s").as_double(), 0.1);
    const auto path_buffer_size = this->get_parameter("path_buffer_size").as_int();
    path_buffer_size_ = static_cast<std::size_t>(std::max<int64_t>(path_buffer_size, 1));
    default_frame_id_ = this->get_parameter("default_frame_id").as_string();

    scan_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(scan_period_s_),
      std::bind(&OdometryPathPublisherNode::discover_pose_topics, this));
    discover_pose_topics();

    RCLCPP_INFO(
      this->get_logger(),
      "Node ready. Dynamic discovery of Odometry, Pose, PoseStamped and "
      "PoseWithCovarianceStamped topics enabled.");
  }

private:
  struct SourceData
  {
    std::string topic_type;
    std::deque<geometry_msgs::msg::PoseStamped> buffer;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher;
    rclcpp::SubscriptionBase::SharedPtr subscription;
  };

  void discover_pose_topics()
  {
    for (const auto & [topic_name, topic_types] : this->get_topic_names_and_types()) {
      if (sources_.count(topic_name) > 0U) {
        continue;
      }

      const auto topic_type = get_supported_topic_type(topic_types);
      if (topic_type.empty()) {
        continue;
      }

      SourceData data;
      data.topic_type = topic_type;
      data.publisher = this->create_publisher<nav_msgs::msg::Path>(
        topic_name + "_path",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

      if (topic_type == kOdometryType) {
        data.subscription = this->create_subscription<nav_msgs::msg::Odometry>(
          topic_name,
          rclcpp::SensorDataQoS().keep_last(1),
          [this, topic_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->handle_odometry(topic_name, *msg);
          });
      } else if (topic_type == kPoseStampedType) {
        data.subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic_name,
          rclcpp::SensorDataQoS().keep_last(1),
          [this, topic_name](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->handle_pose_stamped(topic_name, *msg);
          });
      } else if (topic_type == kPoseType) {
        data.subscription = this->create_subscription<geometry_msgs::msg::Pose>(
          topic_name,
          rclcpp::SensorDataQoS().keep_last(1),
          [this, topic_name](const geometry_msgs::msg::Pose::SharedPtr msg) {
            this->handle_pose(topic_name, *msg);
          });
      } else if (topic_type == kPoseWithCovarianceStampedType) {
        data.subscription =
          this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic_name,
          rclcpp::SensorDataQoS().keep_last(1),
          [this, topic_name](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            this->handle_pose_with_covariance_stamped(topic_name, *msg);
          });
      } else {
        continue;
      }

      sources_.emplace(topic_name, std::move(data));
      RCLCPP_INFO(
        this->get_logger(),
        "Subscription created: %s (%s) -> publication %s_path",
        topic_name.c_str(),
        topic_type.c_str(),
        topic_name.c_str());
    }
  }

  std::string get_supported_topic_type(const std::vector<std::string> & topic_types) const
  {
    for (const auto & topic_type : topic_types) {
      if (
        topic_type == kOdometryType ||
        topic_type == kPoseType ||
        topic_type == kPoseStampedType ||
        topic_type == kPoseWithCovarianceStampedType)
      {
        return topic_type;
      }
    }
    return "";
  }

  void handle_odometry(const std::string & topic_name, const nav_msgs::msg::Odometry & msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    append_and_publish(topic_name, pose);
  }

  void handle_pose_stamped(
    const std::string & topic_name,
    const geometry_msgs::msg::PoseStamped & msg)
  {
    append_and_publish(topic_name, msg);
  }

  void handle_pose(const std::string & topic_name, const geometry_msgs::msg::Pose & msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = default_frame_id_;
    pose.pose = msg;
    append_and_publish(topic_name, pose);
  }

  void handle_pose_with_covariance_stamped(
    const std::string & topic_name,
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    append_and_publish(topic_name, pose);
  }

  void append_and_publish(const std::string & topic_name, const geometry_msgs::msg::PoseStamped & pose)
  {
    auto it = sources_.find(topic_name);
    if (it == sources_.end()) {
      return;
    }

    auto & buffer = it->second.buffer;
    if (buffer.size() >= path_buffer_size_) {
      buffer.pop_front();
    }
    buffer.push_back(pose);

    if (it->second.publisher->get_subscription_count() == 0U) {
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = pose.header.stamp;
    path_msg.header.frame_id = default_frame_id_;
    path_msg.poses.reserve(buffer.size());
    path_msg.poses.assign(buffer.begin(), buffer.end());
    for (auto & buffered_pose : path_msg.poses) {
      buffered_pose.header.frame_id = default_frame_id_;
    }
    it->second.publisher->publish(path_msg);
  }

  static constexpr const char * kOdometryType = "nav_msgs/msg/Odometry";
  static constexpr const char * kPoseType = "geometry_msgs/msg/Pose";
  static constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";
  static constexpr const char * kPoseWithCovarianceStampedType =
    "geometry_msgs/msg/PoseWithCovarianceStamped";

  double scan_period_s_{2.0};
  std::size_t path_buffer_size_{1000};
  std::string default_frame_id_{"map"};
  std::unordered_map<std::string, SourceData> sources_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryPathPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
