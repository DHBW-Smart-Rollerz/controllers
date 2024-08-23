# Copyright (c) 2024 Smart Rollerz e.V.
# All rights reserved.

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    """
    Launch the stanley controller node.

    Returns:
        LaunchDescription: The launch description.
    """
    node = Node(
        package="controllers",
        executable="stanley_controller",
        name="stanley_controller",
        output="screen",
        parameters=[],
    )

    return LaunchDescription([node])
