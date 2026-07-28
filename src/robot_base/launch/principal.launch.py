from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Subir a 0.6 para conducir con teleop_twist_keyboard; ver bringup_all.
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    declare_cmd_vel_timeout = DeclareLaunchArgument(
        'cmd_vel_timeout', default_value='0.25')

    # localization (por defecto) navega sobre el mapa guardado;
    # slam_mode:=mapping levanta un mapa nuevo.
    slam_mode = LaunchConfiguration('slam_mode')
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode', default_value='localization')

    # Lanzador principal: incluye el bringup completo con sus defaults.
    # Para ver en RViz (mejor desde otra PC): use_rviz:=true
    bringup_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_base'),
                         'launch', 'bringup_all.launch.py')
        ),
        launch_arguments={
            'cmd_vel_timeout': cmd_vel_timeout,
            'slam_mode': slam_mode,
        }.items()
    )

    # Puente para visualizar desde otra PC con Foxglove Studio
    # (app de Windows/Mac/Linux, renderiza en la PC y no carga a la Pi).
    # En Foxglove: Open connection -> ws://192.168.1.30:8765
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}],
    )

    return LaunchDescription(
        [declare_cmd_vel_timeout, declare_slam_mode, bringup_all, foxglove])
