#! /usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from scipy.optimize import minimize
from std_msgs.msg import Float32, Int16

from controllers.controller_lqr_4 import LQRController


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
    x0 = 0
    derivative = 2 * a * x0 + b
    tmp = np.arctan(derivative)
    # Calculate angle with y-axis in degrees (90° - angle with x-axis)
    angle = np.pi / 2 - tmp
    return angle, tmp


class LateralControllerNode4(Node):
    def __init__(self):
        super().__init__("lateral_controller_node_lqr_integrator")

        self.get_logger().info("LQR Integrator Node initialized")

        self.lqr_controller = LQRController()

        self.publisher = self.create_publisher(
            Int16, "/control/steering_angle/target", 10
        )

        self.path_subscription = self.create_subscription(
            Vector3, "/path_planning/target/left", self.path_callback, 10
        )

        self.speed_subscription = self.create_subscription(
            Float32, "/control/speed/limit", self.limit_callback, 10
        )

    def path_callback(self, msg):
        a = msg.x
        b = msg.y
        c = msg.z

        x_closest, offset = distance_to_point(a, b, c, 0, 0)
        angle_delta, tmp = angle_with_y_axis(a, b, 0)

        # Krümmung aus zweiter Ableitung
        offset = offset / 1000.0  # mm -> m

        curvature = 0
        offset = offset - 0.2
        control = self.lqr_controller.get_control_signal([tmp, offset], y_ref=0.0)

        self.get_logger().info(
            f"steuerung={control:.3f} rad | winkelfehler={tmp:.2f}°, offset={offset:.3f} m, k={curvature:}, temp = {angle_delta:.3f}"
        )

        # Umrechnung und Begrenzung
        control_deg = int(np.clip(np.rad2deg(control), -45, 45))
        control_deg = control_deg * -1

        msg_out = Int16()
        msg_out.data = control_deg
        self.publisher.publish(msg_out)

    def limit_callback(self, msg):
        self.lqr_controller.update_parameters(
            self.lqr_controller.Ts,
            msg.data,
            self.lqr_controller.l,
            self.lqr_controller.D,
        )
        self.get_logger().info(f"Speed updated to: {self.lqr_controller.v:.2f} m/s")


def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNode4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
