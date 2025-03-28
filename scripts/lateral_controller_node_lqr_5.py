#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Vector3

from controllers.controller_lqr_3 import LQRController

import numpy as np


def distance_to_point(a, b, c, x0, y0):
    def distance(x):
        y = a * x**2 + b * x + c
        return np.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    from scipy.optimize import minimize

    result = minimize(distance, 0)
    return result.x[0], result.fun


def angle_with_x_axis(a, b, x0):
    derivative = 2 * a * x0 + b
    return np.degrees(np.arctan(derivative))


def angle_with_y_axis(a, b, x0):
    # Calculate derivative at x0
    derivative = 2 * a * x0 + b

    # Angle with y-axis = π/2 - arctangent of slope
    angle = (np.pi / 2) - np.arctan(derivative)
    return angle


class LateralControllerNodeLQR_5(Node):

    def __init__(self):
        super().__init__("lateral_controller_node_lqr_5")

        self.get_logger().info("LQR Node initialized – Hallo!")

        self.lqr_controller = LQRController()

        self.publisher = self.create_publisher(
            Int16, "/control/steering_angle/target", 10
        )

        self.left_polinom_subscription = self.create_subscription(
            Vector3, "/path_planning/target/left", self.path_callback, 10
        )

        self.left_polinom_subscription = self.create_subscription(
            Float32, "/control/speed/limit", self.limit_callback, 10
        )

    def path_callback(self, msg):
        a = msg.x
        b = msg.y
        c = msg.z

        x_closest, offset = distance_to_point(a, b, c, 0, 0)
        angle_delta = angle_with_x_axis(a, b, x_closest)

        # Krümmung näherungsweise aus 2. Ableitung
        curvature = 2 * a / (1 + (2 * a * x_closest + b) ** 2) ** 1.5
        offset = offset / 1000  # Umrechnung von mm in m
        control = self.lqr_controller.get_control_signal(
            [angle_delta, offset], curvature
        )

        self.get_logger().info(
            f"Berechnetes u (Lenkwinkel): {control}°, delta={angle_delta:.2f}, offset={offset:.3f}, k={curvature:.3f}"
        )
        control = np.rad2deg(control)  # Umrechnung von rad in Grad
        control = int(control)  # Umwandlung in Integer
        control = np.clip(control, -45, 45)  # Begrenzung auf ±45°

        msg_out = Int16()
        msg_out.data = control
        self.publisher.publish(msg_out)

    def limit_callback(self, msg):
        self.lqr_controller.update_parameters(
            self.lqr_controller.Ts,
            msg.data,
            self.lqr_controller.l,
            self.lqr_controller.D,
        )
        self.get_logger().info(f"Speed limit set to: {self.lqr_controller.v} m/s")


def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNodeLQR_5()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
