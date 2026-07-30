from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Camara RealSense D435. Se lanza APARTE del bringup principal:
    #   ros2 launch robot_base camera.launch.py
    # Conectarla a un puerto USB 3.0 (los azules) con un buen cable.
    #
    # Se lanza el nodo directo (sin rs_launch.py) porque en ARM el filtro
    # de pointcloud se llama "pointcloud__neon_" y rs_launch.py solo
    # conoce "pointcloud", asi que el enable nunca llegaba al nodo.
    #
    # Perfiles moderados para no saturar la Pi; el pointcloud ya viene
    # alineado a color. Subir a 848x480x30 si sobra CPU.
    rs_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            # reset de hardware al arrancar: evita "Depth stream start
            # failure" cuando la camara quedo colgada de una sesion previa
            # (agrega ~5 s al arranque)
            'initial_reset': True,
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'pointcloud__neon_.enable': True,
            # 2 = stream de color, para que el pointcloud salga texturizado
            'pointcloud__neon_.stream_filter': 2,
            'align_depth.enable': True,
            'enable_infra1': False,
            'enable_infra2': False,
            # el timestamp global falla en la Pi (set_xu Protocol error
            # en bucle); con esto usa el reloj del sistema y no spamea
            'depth_module.global_time_enabled': False,
            'rgb_camera.global_time_enabled': False,
        }],
    )

    # Scan 2D falso a partir de la imagen de profundidad, para sumar la
    # camara como segunda fuente de obstaculos en Nav2 (ve lo que queda
    # por debajo del plano del lidar). Publica en /camera_scan para no
    # pisar el /scan del lidar. PENDIENTE al montar la camara: ajustar
    # scan_height (filas de la imagen que colapsa al scan) segun la
    # altura/inclinacion final, y agregar /camera_scan como observation
    # source del costmap local en nav2_params.yaml.
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            # crudo: hereda el stamp de la camara, que con
            # global_time_enabled=false queda ~0.4 s corrido del reloj
            # del sistema y los costmaps lo descartan; scan_restamp lo
            # re-estampa y publica el /camera_scan definitivo
            ('scan', '/camera_scan_raw'),
        ],
        parameters=[{
            'output_frame': 'camera_depth_frame',
            # franja angosta (10 px = +-0.6 grados): con el lente a 8 cm,
            # una franja ancha veia el piso cuando el robot cabecea al
            # frenar/retroceder y dejaba obstaculos fantasma pegados en el
            # costmap global ("Start occupied" recurrente, 9-jul). Con
            # +-0.6 grados el piso recien entra en rango con >2.4 grados
            # de cabeceo. Sigue viendo cualquier objeto de >8 cm de alto.
            'scan_height': 10,
            'range_min': 0.3,       # la D435 no mide mas cerca
            # margen extra contra el cabeceo (a 0.12 m/s no hace falta
            # ver obstaculos mas alla de 1.5 m)
            'range_max': 1.5,
            'scan_time': 0.033,
        }],
    )

    # Re-estampa /camera_scan_raw -> /camera_scan con la hora del sistema
    # (ver comentario en el remapping de arriba).
    restamp = Node(
        package='robot_base',
        executable='scan_restamp',
        name='scan_restamp',
        output='screen',
    )

    # Posicion medida con el soporte impreso (9-jul-2026): lente 11 cm
    # adelante del centro del lidar (= base_link) y 8 cm del piso,
    # apuntando horizontal.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '--x', '0.11', '--y', '0', '--z', '0.08',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ],
    )

    return LaunchDescription([rs_camera, depth_to_scan, restamp, static_tf])
