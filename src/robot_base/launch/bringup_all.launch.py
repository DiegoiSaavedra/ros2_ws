from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')

    base_params_file = LaunchConfiguration('base_params_file')
    ekf_params_file  = LaunchConfiguration('ekf_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_localization_params_file = LaunchConfiguration(
        'slam_localization_params_file')
    slam_mode        = LaunchConfiguration('slam_mode')
    use_rviz         = LaunchConfiguration('use_rviz')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    scan_min_range    = LaunchConfiguration('scan_min_range')
    enable_near_debug = LaunchConfiguration('enable_near_debug')
    cmd_vel_timeout   = LaunchConfiguration('cmd_vel_timeout')
    fixed_beam_size   = LaunchConfiguration('fixed_beam_size')

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
            default_value=os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')),
        DeclareLaunchArgument('slam_localization_params_file',
            default_value=os.path.join(pkg_share, 'config', 'mapper_params_localization.yaml')),
        # localization (por defecto): se ubica contra el mapa fijo ya grabado y
        # no puede corromperlo. Para levantar un mapa nuevo con teleop:
        #   ros2 launch robot_base principal.launch.py slam_mode:=mapping
        DeclareLaunchArgument('slam_mode', default_value='localization'),
        # RViz consume mucha CPU en la Pi: actívalo con use_rviz:=true
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_laser_tf', default_value='true'),
        # MEDIDO el 29-jul con el robot en medio de la habitacion y el filtro
        # abierto del todo (enable_near_debug:=true publica /scan_near_debug
        # con range_min 0.001): de 505 rayos, CERO caen por debajo de 0.30 m y
        # la distancia minima es 0.47 m. O sea que el lidar NO se ve a si
        # mismo: no hay autogolpes que filtrar.
        #
        # El 0.15 que estuvo aqui el 28-jul se puso creyendo que habia
        # autogolpes del soporte de la camara y los cables a 3-9 cm. Esa
        # premisa era falsa, y el filtro salia carisimo: un retorno por debajo
        # del minimo se vuelve NaN en el driver (demo.cpp) y laser_geometry lo
        # descarta al proyectar, asi que la pared no queda marcada NI limpiada
        # -- simplemente no existe. Con el pasillo a 12-13 cm del centro del
        # lidar (medido con cinta), Nav2 y Collision Monitor quedaban ciegos
        # justo contra la pared que tenian que esquivar.
        #
        # 0.05 deja ~7 cm de margen sobre esos 12-13 cm y aun descarta ruido
        # absurdo por debajo de 5 cm. Si se vuelve a montar la D435 u otra
        # cosa a la altura del lidar (z=0.18), hay que RE-MEDIR con
        # enable_near_debug en vez de volver a subir este numero a ojo.
        DeclareLaunchArgument('scan_min_range', default_value='0.05'),
        DeclareLaunchArgument('enable_near_debug', default_value='false'),
        # El LD19 gira a velocidad variable y entrega 503-505 puntos por vuelta.
        # slam_toolbox fija el tamano con el primer scan y descarta el resto
        # ("contains 504 range readings, expected 505"), perdiendo scans justo
        # en los giros, que es donde el robot se descuadraba del mapa. Con un
        # tamano fijo todos los mensajes son validos. 0 = comportamiento
        # original (tamano variable).
        DeclareLaunchArgument('fixed_beam_size', default_value='505'),
        # Producción (Nav2 publica a 10 Hz): 0.25 s son 2.5 periodos, bien.
        # Para conducir a mano hay que subirlo: teleop_twist_keyboard de Jazzy
        # publica UN mensaje por pulsación (no tiene repeat_rate), asi que con
        # 0.25 s el watchdog corta entre tecla y tecla y el robot va a tirones.
        #   ros2 launch robot_base principal.launch.py cmd_vel_timeout:=0.6
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.25'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        # Debe coincidir con el frame_id que publica el driver del lidar
        DeclareLaunchArgument('laser_frame', default_value='base_laser'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.18'),
        DeclareLaunchArgument('laser_roll',  default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw',   default_value='0.0'),
    ]

    base_driver = Node(
        package='robot_base',
        executable='base_driver',
        name='base_driver',
        output='screen',
        parameters=[
            base_params_file,
            {'cmd_vel_timeout': ParameterValue(
                cmd_vel_timeout, value_type=float)},
        ],
        respawn=True, respawn_delay=2.0
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lsm303_l3gd20_imu'), 'launch', 'imu.launch.py')
        )
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
        launch_arguments={
            'publish_tf': 'false',
            'scan_min_range': scan_min_range,
            'enable_near_debug': enable_near_debug,
            'fixed_beam_size': fixed_beam_size,
        }.items()
    )

    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', laser_x, '--y', laser_y, '--z', laser_z,
                   '--roll', laser_roll, '--pitch', laser_pitch, '--yaw', laser_yaw,
                   '--frame-id', base_link_frame, '--child-frame-id', laser_frame],
        condition=IfCondition(publish_laser_tf)
    )

    slam_pkg = get_package_share_directory('slam_toolbox')
    is_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    is_localization = PythonExpression(["'", slam_mode, "' != 'mapping'"])

    # Construye el mapa (async_slam_toolbox_node).
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(is_mapping)
    )

    # Se localiza contra el mapa guardado (localization_slam_toolbox_node).
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_localization_params_file,
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(is_localization)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', '11.rviz')],
        condition=IfCondition(use_rviz)
    )

    ekf_delayed  = TimerAction(period=1.0, actions=[ekf])
    slam_delayed = TimerAction(
        period=2.0, actions=[slam_mapping, slam_localization])
    rviz_delayed = TimerAction(period=3.0, actions=[rviz])

    return LaunchDescription(
        declare_args + [
            base_driver,
            imu_launch,
            lidar_launch,
            static_laser_tf,
            ekf_delayed,
            slam_delayed,
            rviz_delayed,
        ]
    )
