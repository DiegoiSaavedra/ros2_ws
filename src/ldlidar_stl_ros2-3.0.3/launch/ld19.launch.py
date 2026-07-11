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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_tf = LaunchConfiguration("publish_tf")
    scan_min_range = LaunchConfiguration("scan_min_range")
    enable_near_debug = LaunchConfiguration("enable_near_debug")

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {"product_name": "LDLiDAR_LD19"},
            {"topic_name": "scan"},
            {"frame_id": "base_laser"},
            {"port_name": "/dev/ldlidar"},  # symlink estable (regla udev)
            {"port_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
            {"scan_min_range": ParameterValue(scan_min_range, value_type=float)},
            {"enable_near_debug": ParameterValue(enable_near_debug, value_type=bool)},
            {"near_debug_topic": "scan_near_debug"},
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
            DeclareLaunchArgument("scan_min_range", default_value="0.02"),
            DeclareLaunchArgument("enable_near_debug", default_value="false"),
            ldlidar_node,
            base_link_to_laser_tf_node,
        ]
    )
