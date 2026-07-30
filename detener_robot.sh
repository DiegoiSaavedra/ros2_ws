#!/bin/bash
# Detiene TODO el robot de forma segura: el launch y cualquier nodo
# que haya quedado huerfano. Uso:  bash ~/ros2_ws2/ros2_ws/detener_robot.sh

NODOS="lib/robot_base/base_driver lsm303_l3gd20_imu/imu_node ldlidar_stl_ros2_node robot_localization/ekf_node async_slam_toolbox_node tf2_ros/static_transform_publisher rviz2"

echo "Deteniendo robot..."

# 1) Ctrl+C al launch para que apague a sus hijos en orden.
# El patron "bin/ros2 launch" solo coincide con el proceso real de
# ros2 launch, no con shells/wrappers que contengan el texto del
# comando (matarlos deja al launch colgado con nodos zombies).
pkill -INT -f "bin/ros2 launch" 2>/dev/null
for i in 1 2 3 4 5 6 7 8; do
    pgrep -f "bin/ros2 launch" >/dev/null || break
    sleep 1
done
# si el launch quedo colgado (visto en la practica), rematarlo
pkill -KILL -f "bin/ros2 launch" 2>/dev/null

# 2) Ctrl+C directo a cualquier nodo huerfano que haya sobrevivido
for n in $NODOS; do pkill -INT -f "$n" 2>/dev/null; done
sleep 2
for n in $NODOS; do pkill -KILL -f "$n" 2>/dev/null; done
sleep 1

# 3) Verificacion
VIVOS=$(pgrep -f "lib/robot_base/base_driver|lsm303_l3gd20_imu/imu_node|ldlidar_stl_ros2_node|robot_localization/ekf_node|async_slam_toolbox_node")
if [ -n "$VIVOS" ]; then
    echo "ATENCION: quedaron procesos vivos (PIDs: $VIVOS)"
    exit 1
fi
echo "Robot detenido por completo."
