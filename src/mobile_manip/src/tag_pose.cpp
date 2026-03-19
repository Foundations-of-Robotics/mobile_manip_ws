#include <chrono>
#include <optional>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_manip/msg/pose_stamped_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class TagPosePublisher : public rclcpp::Node
{
public:
  TagPosePublisher()
  : Node("tag_pose_publisher"),
    tag_regex_("^(tag\\d+|tag[^:]+:\\d+)$")
  {
    this->declare_parameter<std::string>("tf_topic", "/tf");
    this->declare_parameter<std::string>("array_topic", "tag_pose");
    tf_topic_ = this->get_parameter("tf_topic").as_string();
    array_topic_ = this->get_parameter("array_topic").as_string();

    sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic_,
      10,
      std::bind(&TagPosePublisher::tf_callback, this, std::placeholders::_1));

    pub_array_ = this->create_publisher<mobile_manip::msg::PoseStampedArray>(array_topic_, 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TagPosePublisher::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "subscribing to %s, publishing arrays on %s",
      tf_topic_.c_str(),
      array_topic_.c_str());
  }

private:
  struct StoredTag
  {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time seen_time;
    std::string source_frame;
  };

  std::optional<std::string> normalize_tag_frame_id(const std::string & child_frame_id) const
  {
    std::smatch match;
    if (std::regex_match(child_frame_id, match, std::regex("^tag(\\d+)$"))) {
      return child_frame_id;
    }
    if (std::regex_match(child_frame_id, match, std::regex("^tag[^:]+:(\\d+)$"))) {
      return std::string("tag") + match[1].str();
    }
    return std::nullopt;
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    const auto now = this->get_clock()->now();
    for (const auto & transform : msg->transforms) {
      if (!std::regex_match(transform.child_frame_id, tag_regex_)) {
        continue;
      }

      const auto normalized_frame_id = normalize_tag_frame_id(transform.child_frame_id);
      if (!normalized_frame_id.has_value()) {
        continue;
      }

      frame_id_ = transform.header.frame_id;

      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.header.frame_id = *normalized_frame_id;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      tag_map_[*normalized_frame_id] = StoredTag{pose, now, transform.header.frame_id};
    }
  }

  void timer_callback()
  {
    const auto now = this->get_clock()->now();
    const auto cutoff = now - rclcpp::Duration::from_seconds(5.0);

    for (auto it = tag_map_.begin(); it != tag_map_.end();) {
      if (it->second.seen_time < cutoff) {
        it = tag_map_.erase(it);
      } else {
        ++it;
      }
    }

    mobile_manip::msg::PoseStampedArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;
    msg.poses.reserve(tag_map_.size());

    for (const auto & [tag_id, stored_tag] : tag_map_) {
      (void)tag_id;
      msg.poses.push_back(stored_tag.pose);
    }

    pub_array_->publish(msg);
  }

  std::string tf_topic_;
  std::string array_topic_;
  std::string frame_id_;
  std::regex tag_regex_;
  std::unordered_map<std::string, StoredTag> tag_map_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<mobile_manip::msg::PoseStampedArray>::SharedPtr pub_array_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TagPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
