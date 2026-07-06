from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lanzador principal: incluye el bringup completo con sus defaults.
    # Para ver en RViz (mejor desde otra PC): use_rviz:=true
    bringup_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_base'),
                         'launch', 'bringup_all.launch.py')
        )
    )

    return LaunchDescription([bringup_all])
