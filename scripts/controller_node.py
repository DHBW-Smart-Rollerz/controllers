#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32


from controllers.controller_pid import PIDController


class ControllerNode(Node):

    def __init__(self):
        super().__init__("minimal_publisher")

        self.pid_controller = PIDController(2, 1, 0.5)
        self.current = 0
        self.target = 0

        self.publisher = self.create_publisher(Float32, "topic", 10)
        self.target_subscription = self.create_subscription(
            Float32, "/control/speed/target", self.callback_target, 10
        )
        self.current_subscription = self.create_subscription(
            Float32, "/sensor/speed", self.callback_current, 10
        )

    def callback_target(self, msg):
        self.target = msg.data
        self.get_logger().info('New Target: "%s"' % msg.data)

        value = self.pid_controller.update(self.current, self.target)
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)

    def callback_current(self, msg):
        self.current = msg.data
        self.get_logger().info('New Current Velocity: "%s"' % msg.data)

        value = self.pid_controller.update(self.current, self.target)
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
