from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_base'),
                         'launch', 'bringup_all.launch.py')
        ),
        launch_arguments={
            'slam_params_file': '/home/diego/ros2_ws2/ros2_ws/config/mapper_params_online_async.yaml',
            'base_link_frame': 'base_link',
            'laser_frame': 'base_laser'
        }.items()
    )

    return LaunchDescription([bringup_all])

