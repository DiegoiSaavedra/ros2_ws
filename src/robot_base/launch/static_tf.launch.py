from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # x y z roll pitch yaw   parent     child
    # LIDAR a 0.18 m del piso, centrado y sin inclinaciones
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.0', '0.0', '0.18', '0.0', '0.0', '0.0', 'base_link', 'laser']
        )
    ])

