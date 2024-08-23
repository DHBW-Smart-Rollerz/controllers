// Copyright (c) 2024 by Smart Rollerz e.V.
// All rights reserved.
#ifndef CONTROLLERS__STANLEY_HPP_
#define CONTROLLERS__STANLEY_HPP_

#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

class StanleyControllerNode : public rclcpp::Node {
public:
  StanleyControllerNode();

  /**
   * @brief Callback function for the current pose subscriber
   *
   * @param msg
   */
  void refposeCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

  /**
   * @brief Callback function for the velocity subscriber.
   *
   * @param msg
   */
  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Compute the control command based on the current pose and reference
   * path.
   *
   * @return std::optional<double>
   */
  std::optional<double> computeControlCommand();

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
    refpose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    target_stering_angle_publisher_;

  std::optional<geometry_msgs::msg::Vector3> refpose_;
  geometry_msgs::msg::Vector3 current_pose_;
  std::optional<double> velocity_;
  double k_;
};
#endif  // CONTROLLERS__STANLEY_HPP_
