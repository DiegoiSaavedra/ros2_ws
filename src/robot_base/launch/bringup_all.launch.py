from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')

    base_params_file = LaunchConfiguration('base_params_file')
    ekf_params_file  = LaunchConfiguration('ekf_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz         = LaunchConfiguration('use_rviz')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')

    base_link_frame  = LaunchConfiguration('base_link_frame')
    laser_frame      = LaunchConfiguration('laser_frame')
    laser_x          = LaunchConfiguration('laser_x')
    laser_y          = LaunchConfiguration('laser_y')
    laser_z          = LaunchConfiguration('laser_z')
    laser_roll       = LaunchConfiguration('laser_roll')
    laser_pitch      = LaunchConfiguration('laser_pitch')
    laser_yaw        = LaunchConfiguration('laser_yaw')

    declare_args = [
        DeclareLaunchArgument('base_params_file',
            default_value=os.path.join(pkg_share, 'config', 'base.yaml')),
        DeclareLaunchArgument('ekf_params_file',
            default_value=os.path.join(pkg_share, 'config', 'ekf_odom.yaml')),
        DeclareLaunchArgument('slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam_online_async.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('publish_laser_tf', default_value='true'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_roll',  default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw',   default_value='0.0'),
    ]

    base_driver = Node(
        package='robot_base',
        executable='base_driver',
        name='base_driver',
        output='screen',
        parameters=[base_params_file],
        respawn=True, respawn_delay=2.0
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lsm303_l3gd20_imu'), 'launch', 'imu.launch.py')
        )
    )

    imu_cov_fix = Node(
    package='robot_base',
    executable='imu_cov_fix',
    name='imu_cov_fix',
    output='screen',
    # Añadimos los remappings para asegurar que los tópicos son correctos
    remappings=[('imu/data', '/imu/data'), ('imu/data_fixed', '/imu/data_fixed')],
    # Y aquí añadimos los parámetros para poder ajustarlos
    parameters=[{
        # Un valor más PEQUEÑO aquí significa MÁS CONFIANZA en la IMU.
        # Prueba con 0.002. Si quieres más confianza, prueba 0.001 o 0.0005.
        'cov_orientation': [0.002, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.002],
        'cov_velocity':    [0.002, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.002],
        'cov_acceleration':[0.05,  0.0, 0.0, 0.0, 0.05,  0.0, 0.0, 0.0, 0.05],
    }]
)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[ekf_params_file],
        respawn=True, respawn_delay=2.0
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld19.launch.py')
        ),
        launch_arguments={'publish_tf': 'false'}.items()
    )

    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=[laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw,
                   base_link_frame, laser_frame],
        condition=IfCondition(publish_laser_tf)
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )

    ekf_delayed  = TimerAction(period=1.0, actions=[ekf])
    slam_delayed = TimerAction(period=2.0, actions=[slam])
    rviz_delayed = TimerAction(period=3.0, actions=[rviz])

    return LaunchDescription(
        declare_args + [
            base_driver,
            imu_launch,
            imu_cov_fix,
            lidar_launch,
            static_laser_tf,
            ekf_delayed,
            slam_delayed,
            rviz_delayed,
        ]
    )
