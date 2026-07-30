#!/usr/bin/env python3
"""Re-estampa /camera_scan_raw -> /camera_scan con la hora del sistema.

La D435 corre con global_time_enabled=false (el sync de reloj de
librealsense spamea "set_xu Protocol error" en la Pi), y eso deja sus
timestamps ~0.4 s corridos respecto al reloj del sistema. El message
filter de los costmaps de Nav2 nunca encuentra TF para esos stamps y
descarta todos los scans de la camara. Este nodo los re-estampa con
now(); el error de pose resultante (<0.4 s a 0.12 m/s = ~5 cm) es
irrelevante para marcar obstaculos.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanRestamp(Node):
    def __init__(self):
        super().__init__('scan_restamp')
        self.pub = self.create_publisher(LaserScan, '/camera_scan',
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/camera_scan_raw',
                                 self.cb, qos_profile_sensor_data)

    def cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
