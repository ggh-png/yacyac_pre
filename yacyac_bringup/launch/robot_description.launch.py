import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    odom_to_base_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "odom",
            "base_link",
        ],
        output="screen",
    )
    base_to_lidar_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0.14",
            "0",
            "0",
            "0",
            "base_link",
            "lidar_link",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            base_to_lidar_publisher,
            odom_to_base_publisher,
        ]
    )
