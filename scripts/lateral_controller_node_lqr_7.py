#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from lane_msgs.msg import LaneDetectionResult, Lane

from smarty_utils.enums import Location
from std_msgs.msg import Int16, Float32, String
from geometry_msgs.msg import Vector3

from controllers.controller_lqr_4 import LQRController

# from smarty_utils import location

import numpy as np
from scipy.optimize import minimize
from time import time
from scipy.interpolate import interp1d


def distance_to_point(a, b, c, x0, y0):
    def distance(x):
        y = a * x**2 + b * x + c
        return np.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    result = minimize(distance, 0)
    if c < 0:
        result = -result

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

        # self.path_subscription = self.create_subscription(
        #    Vector3, "/path_planning/target/left", self.path_callback_l, 10
        # )
        # self.path_subscription = self.create_subscription(
        #    Vector3, "/path_planning/target/right", self.path_callback_r, 10
        # )

        self.speed_subscription = self.create_subscription(
            Float32, "/control/speed/limit", self.limit_callback, 10
        )

        self.get_lane = self.create_subscription(
            Float32, "/state_machine/go_lane", self.limit_callback, 10
        )

        self.debug_lane_detection_sub = self.create_subscription(
            LaneDetectionResult, "/lane_detection/lane", self.ld_callback, 10
        )

        self.lane_mode_sub = self.create_subscription(
            String, "/state_machine/goal_lane", self.change_mode, 10
        )

        self.latest_path = None
        self.last_path_time = 0.0  # Zeitstempel des letzten Pfads
        self.new_path_available = False
        self.lane_mode = Location.LEFT
        # self.lane_mode = Location.RIGHT

        timer_period = 0.020  # 1 ms → 1000 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        if self.latest_path is None:
            self.get_logger().warn("No path data available.")
            return

        a, b, c = self.latest_path
        x_closest, offset = distance_to_point(a, b, c, 0, 0)

        angle_with_y, angle_with_x = angle_with_y_axis(a, b, 0)

        offset = offset / 1000.0  # mm → m
        offset -= 0.0  # ggf. Sensor-Korrektur

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

    def extract_points(self, lane: Lane) -> np.ndarray:
        points = [[point.x, point.y, point.z] for point in lane.points]
        points = np.array(points)
        points = points[points[:, 0].argsort()]
        return points

    def extract_lane(self, left: Lane, right: Lane) -> tuple[float, float, float]:
        left_p = self.extract_points(left)
        right_p = self.extract_points(right)

        # Interpolate points to ensure both arrays have the same x coordinates

        # Interpolate left and right lanes
        x_min = max(left_p[:, 0].min(), right_p[:, 0].min())
        x_max = min(left_p[:, 0].max(), right_p[:, 0].max())
        x_common = np.linspace(x_min, x_max, num=100)

        left_interp = interp1d(
            left_p[:, 0], left_p[:, 1], kind="linear", fill_value="extrapolate"
        )
        right_interp = interp1d(
            right_p[:, 0], right_p[:, 1], kind="linear", fill_value="extrapolate"
        )

        y_left = left_interp(x_common)
        y_right = right_interp(x_common)

        # Get the middle points
        x = x_common
        y = (y_left + y_right) / 2

        # Fit a polynomial of degree 2 to the middle points
        coeffs = np.polyfit(x, y, 2)
        return coeffs[0], coeffs[1], coeffs[2]

    def ld_callback(self, msg: LaneDetectionResult):
        if self.lane_mode == Location.LEFT:
            self.latest_path = self.extract_lane(msg.left, msg.center)
            self.last_path_time = time()
            self.new_path_available = True

        if self.lane_mode == Location.RIGHT:
            self.latest_path = self.extract_lane(msg.center, msg.right)
            self.last_path_time = time()
            self.new_path_available = True

    def change_mode(self, msg: String):
        self.lane_mode = Location(msg.data)

    # def path_callback_l(self, msg):
    #    self.latest_path_l = (msg.x, msg.y, msg.z)
    #    self.last_path_time = time()
    #    self.new_path_available = True


def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNode4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
