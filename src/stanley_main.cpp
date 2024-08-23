// Copyright (c) 2024 by Smart Rollerz e.V.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "controllers/stanley.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StanleyControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
