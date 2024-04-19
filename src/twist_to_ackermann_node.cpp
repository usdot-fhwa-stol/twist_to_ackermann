// Copyright 2024 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace twist_to_ackermann
{
auto yaw_rate_to_steering_angle(double speed, double yaw_rate, double wheel_base)
{
  if (std::abs(speed) < 0.1 || std::abs(yaw_rate) < 0.1) {
    return 0.0;
  }

  return -std::atan(wheel_base / (speed / yaw_rate));
}

class TwistToAckermannNode : public rclcpp::Node
{
public:
  TwistToAckermannNode() : TwistToAckermannNode(rclcpp::NodeOptions{}) {}

  explicit TwistToAckermannNode(const rclcpp::NodeOptions & options)
  : Node("twist_to_ackermann_node"),
    twist_subscription_{create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1,
      [this](const geometry_msgs::msg::Twist & msg) {
        ackermann_publisher_->publish(to_ackermann_msg(msg));
      })},
    ackermann_publisher_{
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann", 1)}
  {
    declare_parameter("wheel_base", 1.0);
  }

  auto to_ackermann_msg(const geometry_msgs::msg::Twist & twist_msg)
    -> ackermann_msgs::msg::AckermannDriveStamped
  {
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    ackermann_msg.header.stamp = get_clock()->now();
    ackermann_msg.header.frame_id = "rear_axle_link";

    ackermann_msg.drive.speed = twist_msg.linear.x;
    ackermann_msg.drive.steering_angle = yaw_rate_to_steering_angle(
      twist_msg.linear.x, twist_msg.angular.z, get_parameter("wheel_base").as_double());

    return ackermann_msg;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_{nullptr};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_{
    nullptr};
};

}  // namespace twist_to_ackermann

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);

  auto node{std::make_shared<twist_to_ackermann::TwistToAckermannNode>(rclcpp::NodeOptions{})};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}