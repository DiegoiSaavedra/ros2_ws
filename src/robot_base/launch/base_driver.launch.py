from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')
    params_arg = DeclareLaunchArgument('params_file', default_value='')

    n = Node(
        package='robot_base',
        executable='base_driver',
        name='base_driver',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baud': LaunchConfiguration('baud')},
            LaunchConfiguration('params_file'),
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        params_arg,
        n,
    ])

