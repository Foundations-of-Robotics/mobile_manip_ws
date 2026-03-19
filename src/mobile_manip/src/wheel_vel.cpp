#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class WheelVelNode : public rclcpp::Node
{
public:
  WheelVelNode()
  : Node("wheel_vel_node")
  {
    this->declare_parameter<std::string>("joint_states_topic", "platform/wheels/cmd");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("left_wheel_name", "front_left_wheel_joint");
    this->declare_parameter<std::string>("right_wheel_name", "front_right_wheel_joint");
    this->declare_parameter<double>("wheel_radius", 0.0984 / 2.0);
    this->declare_parameter<double>("wheel_separation", 0.3765);

    const auto joint_states_topic = this->get_parameter("joint_states_topic").as_string();
    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    left_name_ = this->get_parameter("left_wheel_name").as_string();
    right_name_ = this->get_parameter("right_wheel_name").as_string();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = std::max(this->get_parameter("wheel_separation").as_double(), 1e-6);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, sensor_qos);
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic,
      sensor_qos,
      std::bind(&WheelVelNode::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Listening to '%s', publishing Twist to '%s'. Left='%s', Right='%s', r=%f, sep=%f",
      joint_states_topic.c_str(),
      cmd_vel_topic.c_str(),
      left_name_.c_str(),
      right_name_.c_str(),
      wheel_radius_,
      wheel_separation_);
  }

private:
  static constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

  void update_joint_indices(const std::vector<std::string> & names)
  {
    left_index_ = kInvalidIndex;
    right_index_ = kInvalidIndex;

    for (std::size_t i = 0; i < names.size(); ++i) {
      if (names[i] == left_name_) {
        left_index_ = i;
      } else if (names[i] == right_name_) {
        right_index_ = i;
      }
    }
  }

  bool indices_are_valid(const sensor_msgs::msg::JointState & msg) const
  {
    if (left_index_ == kInvalidIndex || right_index_ == kInvalidIndex) {
      return false;
    }
    if (left_index_ >= msg.name.size() || right_index_ >= msg.name.size()) {
      return false;
    }
    return msg.name[left_index_] == left_name_ && msg.name[right_index_] == right_name_;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!indices_are_valid(*msg)) {
      update_joint_indices(msg->name);
      if (!indices_are_valid(*msg)) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Wheel joint names not found in JointState message.");
        return;
      }
    }

    if (left_index_ >= msg->velocity.size() || right_index_ >= msg->velocity.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "JointState.velocity does not contain expected entries.");
      return;
    }

    const double omega_l = msg->velocity[left_index_];
    const double omega_r = msg->velocity[right_index_];
    const double v_linear = wheel_radius_ * 0.5 * (omega_l + omega_r);
    const double v_angular_z = wheel_radius_ * (omega_r - omega_l) / wheel_separation_;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = v_linear;
    twist.angular.z = v_angular_z;
    pub_->publish(twist);
  }

  std::string left_name_;
  std::string right_name_;
  double wheel_radius_{0.0};
  double wheel_separation_{1.0};
  std::size_t left_index_{kInvalidIndex};
  std::size_t right_index_{kInvalidIndex};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
