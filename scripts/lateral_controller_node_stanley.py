#! /usr/bin/env python3
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32, Int16

from controllers.controller_lqr import angle_with_x_axis, distance_to_point
from controllers.controller_stanley import calculate_steering_angle


class LateralControllerNode_Stanley(Node):
    def __init__(self):
        super().__init__("lateral_controller_node_stanley")

        self.speed = 0
        self.polyom_a = 0
        self.polyom_b = 0
        self.polyom_c = 0
        self.angle_delta = 0
        self.offset = 0

        self.publisher = self.create_publisher(
            Int16, "/control/steering_angle/target", 10
        )

        self.left_polinom_subscription = self.create_subscription(
            Vector3, "/path_planning/target/left", self.callback_, 10
        )

        self.speed_subscription = self.create_subscription(
            Float32, "/controller/speed/target", self.callback_update, 10
        )

    def callback_(self, msg):
        self.a = msg.x
        self.b = msg.y
        self.c = msg.z

        temp_x, self.offset = distance_to_point(self.a, self.b, self.c, 0, 0)
        self.angle_delta = angle_with_x_axis(self.a, self.b, temp_x)

        value = calculate_steering_angle(
            self.speed, self.offset, self.angle_delta, 1, 45
        )

        msg_ = Int16()
        msg_.data = value
        self.publisher.publish(msg_)

    def callback_update(self, msg):
        self.speed = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = LateralControllerNode_Stanley()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
