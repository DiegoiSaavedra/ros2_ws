from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lanza SOLO el driver de la base (util para probar motores/odometria)
    pkg_share = get_package_share_directory('robot_base')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'base.yaml'))

    n = Node(
        package='robot_base',
        executable='base_driver',
        name='base_driver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([params_arg, n])
