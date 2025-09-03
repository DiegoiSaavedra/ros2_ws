from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare('lsm303_l3gd20_imu')

    imu_params = PathJoinSubstitution([pkg, 'config', 'imu_params.yaml'])
    madgwick_params = PathJoinSubstitution([pkg, 'config', 'madgwick.yaml'])

    imu_node = Node(
        package='lsm303_l3gd20_imu',
        executable='imu_node',
        name='lsm303_l3gd20_imu',
        output='screen',
        parameters=[imu_params],
        # Si ALGÚN día publicas en relativos, puedes mapear aquí:
        # remappings=[
        #     ('imu/data_raw', '/imu/data_raw'),
        #     ('imu/mag', '/mag'),
        # ],
    )

    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[madgwick_params],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),  # entrada IMU
            ('imu/mag', '/mag'),                # entrada magnetómetro
            ('imu/data', '/imu/data'),          # salida fusionada
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ],
    )

    return LaunchDescription([imu_node, madgwick, static_tf])

