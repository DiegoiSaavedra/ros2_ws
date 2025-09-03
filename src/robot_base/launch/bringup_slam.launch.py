from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pkg = get_package_share_directory('robot_base')

    base_launch = PathJoinSubstitution([pkg, 'launch', 'base_driver.launch.py'])
    static_tf = PathJoinSubstitution([pkg, 'launch', 'static_tf.launch.py'])
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([
        # Tu driver con YAML de la base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'params_file': os.path.join(pkg, 'config', 'base.yaml')}.items()
        ),
        # TF fijo base_link -> laser
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf),
        ),
        # SLAM Toolbox (modo mapping)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),
        # RViz2 (opcional, arrancarlo ya con un layout básico)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])

