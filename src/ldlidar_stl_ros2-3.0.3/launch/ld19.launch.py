#!/usr/bin/env python3

"""
Launch file for the LD19 LiDAR.

The node optionally publishes a static transform between ``base_link`` and
``base_laser``.  Set the ``publish_tf`` launch argument to ``true`` when the
transform is not provided elsewhere (e.g. in an external bringup launch file).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_tf = LaunchConfiguration("publish_tf")

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD19"},
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

    # Optional base_link to base_laser static transform
    base_link_to_laser_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_ld19",
        arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "base_laser"],
        condition=IfCondition(publish_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_tf", default_value="false"),
            ldlidar_node,
            base_link_to_laser_tf_node,
        ]
    )

