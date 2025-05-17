#! /usr/bin/env python3

from time import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from lane_msgs.msg import Lane, LaneDetectionResult
from rclpy.node import Node
from scipy.interpolate import interp1d
from scipy.optimize import minimize
from smarty_utils.enums import Location
from std_msgs.msg import Float32, Int16, String

from controllers.controller_lqr_4 import LQRController

# from smarty_utils import location



class LateralControllerNode(SmartyNode):
    """ROS Node for LateralController Node."""

    def __init__(self):
        """Initialize the LateralController Node."""
        super().__init__(
            "path_planning_node",
            "pathplanning",
            node_parameters={
                # Subscriber topics
                "control_speed_limit": "/control/speed/limit",
                "state_machine_go_lane": "/state_machine/go_lane",
                "remote_state_subscriber": "/remoteState",
                "goal_lane_subscriber": "/state_machine/goal_lane",
                "pose_estimation_subscriber": "/pose_estimation/pose",
                "new_image_subscriber": "/lane_detection/new_image",
                # Publisher topics
                "targetSteeringAngle_pub": "/control/steering_angle/target",
                "path_planning_left_publisher": "/path_planning/target/left",
                "path_planning_right_publisher": "/path_planning/target/right",
                "ref_point_publisher": "/path_planning/target/pose",
                "image_debug_publisher": "/path_planning/debug/image",
                # Parameters
                "trj_look_forward": 100,
            },

        # )

        self.get_lane = self.create_subscription(
            Float32, "/state_machine/go_lane", self.limit_callback, 10
        )

        self.debug_lane_detection_sub = self.create_subscription(
            LaneDetectionResult, "/lane_detection/lane", self.ld_callback, 10
        )

        self.lane_mode_sub = self.create_subscription(
            String, "/state_machine/goal_lane", self.change_mode, 10
        )

            subscribed_topics={
                "lane_points_subscriber": (
                    LaneDetectionResult,
                    self.serialized_points,
                    None,
                ),
                "pose_estimation_subscriber": (
                    geometry_msgs.msg.Pose,
                    self.pose_callback,
                    None,
                ),
                "new_image_subscriber": (
                    std_msgs.msg.Header,
                    self.timestamp_callback,
                    None,
                ),
                "image_subscriber": (
                    sensor_msgs.msg.Image,
                    self.debug_image_callback,
                    None,
                ),
                "remote_state_subscriber": (
                    std_msgs.msg.UInt8,
                    self.new_remote_state,
                    None,
                ),
                "goal_lane_subscriber": (
                    std_msgs.msg.String,
                    lambda msg: setattr(self, "_goal_lane", Location(msg.data)),
                    None,
                ),
            },
            published_topics={
                "targetSteeringAngle_pub": (std_msgs.msg.Int16, None),
                "path_planning_left_publisher": (geometry_msgs.msg.Vector3, None),
                "path_planning_right_publisher": (geometry_msgs.msg.Vector3, None),
                "ref_point_publisher": (geometry_msgs.msg.Vector3, None),
                "image_debug_publisher": (sensor_msgs.msg.Image, None),
            },
        )
        self._goal_lane = Location.RIGHT
        self.times = []

        # Setup Framework & Cord Transformation
        self.cv_bridge = cv_bridge.CvBridge()
        self.coord_trans = CoordinateTransform()
        self.myController = PPController(
            self.get_parameter, debug=self._debug, logger=self.get_logger()
        )
        self.drive_point_ruling = None

        # Estimation results
        self._est_data: dict[str, EstimationData] = {}
        self.abs_vec = np.zeros(3)  # x, y, psi
        self.est_vec = np.zeros(3)  # x, y, psi
        self._newest_lane_timestamp = ""

        # Initialize transformation classes for debug image
        if self._debug:
            self._debug_image = None
            self.distortion = Distortion(self.coord_trans._calib)
            self.birds_eyed = Birdseye(self.coord_trans._calib, self.distortion)
            self.debug_timer = self.create_timer(0.02, self.debug_image)

        # Log initialization
        self.get_logger().info(
            f"Path planning Node initialized [debug={self._debug}, trj_look_forward={self.get_parameter('trj_look_forward').value}]"
        )
