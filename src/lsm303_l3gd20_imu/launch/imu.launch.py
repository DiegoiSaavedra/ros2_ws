from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare('lsm303_l3gd20_imu')

    imu_params = PathJoinSubstitution([pkg, 'config', 'imu_params.yaml'])

    imu_node = Node(
        package='lsm303_l3gd20_imu',
        executable='imu_node',
        name='lsm303_l3gd20_imu',
        output='screen',
        parameters=[imu_params],
        respawn=True,
        respawn_delay=2.0,
    )

    # Posicion medida del sensor: 13.5 cm hacia atras y 7 cm a la
    # izquierda del centro del robot (medido 2026-07-06, aproximado).
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=[
            '--x', '-0.135', '--y', '0.07', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ],
    )

    return LaunchDescription([imu_node, static_tf])

