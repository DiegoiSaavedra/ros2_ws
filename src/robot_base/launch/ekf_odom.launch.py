from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')  # usa el nombre correcto

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[ekf_yaml]
    )

    return LaunchDescription([ekf])
