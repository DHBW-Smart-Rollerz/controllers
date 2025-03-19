import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Vector3

from controllers.controller_lqr import (
    LQRController,
    distance_to_point,
    angle_with_x_axis,
)


class LateralControllerNodeLQR_1(Node):

    def __init__(self):
        super().__init__("lateral_controller_node_lqr_1")

        self.lqr_controller = LQRController()
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
            Float32, "/control/speed/limit", self.callback_update, 10
        )

    def callback_(self, msg):

        self.a = msg.x
        self.b = msg.y
        self.c = msg.z

        temp_x, self.offset = distance_to_point(self.a, self.b, self.c, 0, 0)
        self.angle_delta = angle_with_x_axis(self.a, self.b, temp_x)

        value = self.lqr_controller.get_control_signal(
            self.speed, self.angle_delta, self.offset
        )

        msg_ = Int16()
        msg_.data = value
        self.publisher.publish(msg_)

    def callback_update(self, msg):
        self.speed = msg.data
        self.lqr_controller.compute_gain(self.speed)


def main(args=None):
    rclpy.init(args=args)

    node = LateralControllerNodeLQR_1()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
