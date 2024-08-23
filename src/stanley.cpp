// Copyright (c) 2024 by Smart Rollerz e.V.
// All rights reserved.

#include "controllers/stanley.hpp"

StanleyControllerNode::StanleyControllerNode()
: Node("stanley_controller")
{
  // Variables
  k_ = 0.5;
  // Initialize subscribers
  refpose_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/controller/ref_pose", 1,
      std::bind(&StanleyControllerNode::refposeCallback, this,
                std::placeholders::_1));

  velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/targetVelocity", 1,
      std::bind(&StanleyControllerNode::velocityCallback, this,
                std::placeholders::_1));

  // Initialize publisher
  target_stering_angle_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/targetSteeringAngle", 1);
}

void StanleyControllerNode::refposeCallback(
  const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  refpose_ = *msg;
  auto target = computeControlCommand();
  std_msgs::msg::Float64 target_steer_angle;

  // Check if the target is available
  if (!target) {
    return;
  }

  // Publish the target steering angle
  target_steer_angle.data = target.value_or(0.0);
  target_stering_angle_publisher_->publish(target_steer_angle);
}

void StanleyControllerNode::velocityCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  velocity_ = msg->data;
}

std::optional<double> StanleyControllerNode::computeControlCommand()
{
  if (!refpose_ || !velocity_) {
    return std::nullopt;
  }

  // Compute the cross-track error
  double cross_track_error = sqrt(pow(refpose_->x - current_pose_.x, 2) +
                                  pow(refpose_->y - current_pose_.y, 2));

  // Compute the heading error
  double heading_error = std::atan2(refpose_->y, refpose_->x) - refpose_->z;

  // Stanley control law
  double control_steering_angle =
    heading_error +
    std::atan2(k_ * cross_track_error, velocity_.value_or(0.0));

  return control_steering_angle;
}
