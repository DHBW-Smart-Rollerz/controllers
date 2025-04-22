#! /usr/bin/env python3

from time import time

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

    result = minimize(distance, 0)
    return result.x[0], result.fun


def angle_with_y_axis(a, b, x0):
    derivative = 2 * a * x0 + b
    angle_rad = np.arctan(derivative)
    angle_with_y = np.pi / 2 - angle_rad
    return angle_with_y, angle_rad


class LateralControllerNode4(Node):
    def __init__(self):
        super().__init__("lateral_controller_node_lqr_integrator")

        self.get_logger().info("LQR Integrator Node initialized")

        self.lqr_controller = LQRController()
        self.itterator = 0
        self.publisher = self.create_publisher(
            Int16, "/control/steering_angle/target", 10
        )

        self.path_subscription = self.create_subscription(
            Vector3, "/path_planning/target/left", self.path_callback_l, 10
        )
        self.path_subscription = self.create_subscription(
            Vector3, "/path_planning/target/right", self.path_callback_r, 10
        )

        self.speed_subscription = self.create_subscription(
            Float32, "/control/speed/limit", self.limit_callback, 10
        )

        self.speed_subscription = self.create_subscription(
            Float32, "/control/speed/limit", self.limit_callback, 10
        )
        # /state_machine/go_lane

        self.latest_path_l = None
        self.latest_path_r = None
        self.last_path_time = 0.0  # Zeitstempel des letzten Pfads
        self.new_path_available = False

        timer_period = 0.020  # 1 ms → 1000 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_callback_l(self, msg):
        self.latest_path_l = (msg.x, msg.y, msg.z)
        self.last_path_time = time()
        self.new_path_available = True

    def path_callback_r(self, msg):
        self.latest_path_r = (msg.x, msg.y, msg.z)

    def timer_callback(self):
        if self.latest_path_l is None:
            self.get_logger().warn("No path data available.")
            return

        a_l, b_l, c_l = self.latest_path_l
        x_closest, offset_l = distance_to_point(a_l, b_l, c_l, 0, 0)

        a_r, b_r, c_r = self.latest_path_r
        x_closest, offset_r = distance_to_point(a_r, b_r, c_r, 0, 0)

        offset = offset_l  # - offset_r

        angle_with_y, angle_with_x = angle_with_y_axis(a_l, b_l, 0)

        offset = offset / 1000.0  # mm → m
        offset -= 0.10  # ggf. Sensor-Korrektur

        curvature = 0  # Platzhalter

        control = self.lqr_controller.get_control_signal(
            [angle_with_x, offset], y_ref=0.0
        )

        if self.new_path_available:
            self.get_logger().info("🟢 Neue Lane-Daten verwendet.")
        else:
            age = time() - self.last_path_time
            self.get_logger().info(f"🟡 Alte Lane-Daten verwendet ({age:.3f} s alt).")

        self.new_path_available = False

        self.get_logger().info(
            f"Steuerung = {control:.3f} rad | Winkel = {angle_with_x:.2f} rad | "
            f"Offset = {offset:.3f} m | Krümmung = {curvature} | temp = {angle_with_y:.3f}"
        )

        control_deg = int(np.clip(np.rad2deg(control), -30, 30)) * -1

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
