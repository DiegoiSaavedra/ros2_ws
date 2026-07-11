from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Navegacion Nav2 SOBRE EL MAPA VIVO de slam_toolbox (sin AMCL ni
    # map_server: slam_toolbox ya publica /map y el TF map->odom).
    # Uso: con principal.launch.py corriendo,
    #   ros2 launch robot_base nav.launch.py
    # y mandar la meta desde Foxglove (Publish pose -> /goal_pose, frame map)
    # o RViz (boton Nav2 Goal).
    params = os.path.join(get_package_share_directory('robot_base'),
                          'config', 'nav2_params.yaml')

    # La barrera se activa antes que los productores de velocidad. El manager
    # lifecycle la desactiva después de ellos al recorrer la lista al revés.
    nodes = ['collision_monitor', 'controller_server', 'planner_server',
             'behavior_server', 'bt_navigator']

    return LaunchDescription([
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[params]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': nodes}]),
    ])
