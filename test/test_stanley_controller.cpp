// Copyright (c) 2024 by Smart Rollerz e.V.
// All rights reserved.

#include <gtest/gtest.h>

#include <geometry_msgs/msg/vector3.hpp>

#include "controllers/stanley.hpp"

class StanleyControllerTest : public ::testing::Test {
protected:
  void SetUp() override {node_ = std::make_shared<StanleyControllerNode>();}

  std::shared_ptr<StanleyControllerNode> node_;
};

/**
 * @brief Test the initialization of the StanleyControllerNode
 *
 */
TEST_F(StanleyControllerTest, TestInitialization) {
                                                    ASSERT_NE(node_, nullptr);
}

/**
 * @brief Test the pose callback function with no message
 *
 */
TEST_F(StanleyControllerTest, TestComputeControlCommand_NoPoseNoPath) {
  // Ensure no command is published when no pose or path is available
  ASSERT_NO_THROW(node_->computeControlCommand());
}

/**
 * @brief Test the pose callback function with a pose message
 *
 */
TEST_F(StanleyControllerTest, TestComputeControlCommand_WithPoseAndPath) {
  // Setup a simple pose and path
  geometry_msgs::msg::Vector3 refpose;
  refpose.x = 1.0;
  refpose.y = 0.0;
  refpose.z = 0.0;

  node_->refposeCallback(
      std::make_shared<geometry_msgs::msg::Vector3>(refpose));

  std_msgs::msg::Float64 velocity;
  velocity.data = 1.0;
  node_->velocityCallback(std::make_shared<std_msgs::msg::Float64>(velocity));

  ASSERT_NO_THROW(node_->computeControlCommand());
}

/**
 * @brief Test the computeControlCommand function
 *
 */
TEST_F(StanleyControllerTest, TestComputeControlCommand) {
  // Setup a simple pose and path
  geometry_msgs::msg::Vector3 refpose;
  refpose.x = 1.0;
  refpose.y = 0.0;
  refpose.z = 0.0;

  node_->refposeCallback(
      std::make_shared<geometry_msgs::msg::Vector3>(refpose));

  std_msgs::msg::Float64 velocity;
  velocity.data = 1.0;
  node_->velocityCallback(std::make_shared<std_msgs::msg::Float64>(velocity));

  auto steering_angle = node_->computeControlCommand();
  ASSERT_NE(steering_angle, 0.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
