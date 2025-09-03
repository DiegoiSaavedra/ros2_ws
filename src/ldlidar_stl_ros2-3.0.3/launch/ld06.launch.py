#!/usr/bin/env python3

"""
Launch file for the LD06 LiDAR.

The static transform between ``base_link`` and ``base_laser`` can be disabled
using the ``publish_tf`` launch argument.  This avoids conflicts when another
launch file publishes the same transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_tf = LaunchConfiguration("publish_tf")

    ldlidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD06",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD06"},
            {"topic_name": "scan"},
            {"frame_id": "base_laser"},
            {"port_name": "/dev/ttyUSB0"},
            {"port_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
        ],
    )

    base_link_to_laser_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_ld06",
        arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "base_laser"],
        condition=IfCondition(publish_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_tf", default_value="true"),
            ldlidar_node,
            base_link_to_laser_tf_node,
        ]
    )

