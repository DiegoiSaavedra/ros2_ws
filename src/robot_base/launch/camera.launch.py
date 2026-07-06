from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Camara RealSense D435. Se lanza APARTE del bringup principal:
    #   ros2 launch robot_base camera.launch.py
    # Conectarla a un puerto USB 3.0 (los azules) con un buen cable.
    #
    # Perfiles moderados para no saturar la Pi; el pointcloud ya viene
    # alineado a color. Subir a 848x480x30 si sobra CPU.
    rs_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
        }.items()
    )

    # POSICION PROVISORIA (camara al frente del robot, 15 cm de altura):
    # medir y corregir cuando se monte de verdad, igual que se hizo con
    # el IMU y el lidar.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '--x', '0.10', '--y', '0', '--z', '0.15',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ],
    )

    return LaunchDescription([rs_camera, static_tf])
