from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
        }.items(),
    )

    red_stop_camera = Node(
        package="red_stop_camera",
        executable="red_stop_camera",
        name="red_stop_camera",
        output="screen",
    )

    line_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense_line_follower"),
                "launch",
                "line_follow.launch.py",
            )
        )
    )

    motor_node = Node(
        package="arduino_motor",
        executable="motor_node",
        name="motor_node",
        output="screen",
    )

    return LaunchDescription(
        [realsense_launch, red_stop_camera, line_follower, motor_node]
    )
