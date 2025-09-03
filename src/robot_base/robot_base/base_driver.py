#!/usr/bin/env python3
import math
import serial

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')

        # Par\E1metros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.035)           # m
        self.declare_parameter('base_width', 0.145)             # m
        self.declare_parameter('ticks_per_revolution', 4532)    # ticks
        self.declare_parameter('cmd_timeout', 0.5)              # s
        self.declare_parameter('verbose_serial', False)         # logs crudos

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.base_width = self.get_parameter('base_width').get_parameter_value().double_value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.verbose_serial = self.get_parameter('verbose_serial').get_parameter_value().bool_value

        # Serial
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f"Serial abierto en {self.port} @ {self.baud} baud")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir {self.port}: {e}")

        # Pub/Sub y TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Estado odometr\EDa
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # Timers
        self.timer = self.create_timer(0.02, self.read_serial)     # ~50 Hz
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        omega = float(msg.angular.z)
        L = self.base_width

        v_left = vx - omega * (L / 2.0)
        v_right = vx + omega * (L / 2.0)

        K_PWM = 255.0 / 1.0  # ganancia emp\EDrica
        pwm_left = int(max(min(v_left * K_PWM, 255.0), -255.0))
        pwm_right = int(max(min(v_right * K_PWM, 255.0), -255.0))

        cmd = f"{pwm_left},{pwm_right}\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
                self.last_cmd_time = self.get_clock().now()
            except Exception as e:
                self.get_logger().error(f"Error enviando serial: {e}")
        else:
            self.get_logger().warning("Serial no disponible para enviar cmd_vel")

    def watchdog_check(self):
        # STOP si no llega /cmd_vel en cmd_timeout
        if (self.get_clock().now() - self.last_cmd_time) > Duration(seconds=self.cmd_timeout):
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"0,0\n")
                except Exception as e:
                    self.get_logger().error(f"Error watchdog serial: {e}")

    def read_serial(self):
        if not (self.ser and self.ser.is_open):
            return
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            if self.verbose_serial:
                self.get_logger().debug(f"SER:{line}")

            if line == 'ACK':
                return

            parts = line.split(',')
            if len(parts) != 2:
                return

            ticks_left = int(parts[0])
            ticks_right = int(parts[1])

            dticks_left = ticks_left - self.prev_ticks_left
            dticks_right = ticks_right - self.prev_ticks_right
            self.prev_ticks_left = ticks_left
            self.prev_ticks_right = ticks_right

            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = now

            dist_per_tick = (2.0 * math.pi * self.wheel_radius) / float(self.ticks_per_revolution)
            dist_left = dticks_left * dist_per_tick
            dist_right = dticks_right * dist_per_tick

            d = (dist_left + dist_right) / 2.0
            dtheta = (dist_right - dist_left) / self.base_width

            self.theta += dtheta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            self.x += d * math.cos(self.theta)
            self.y += d * math.sin(self.theta)

            vx = d / dt
            vth = dtheta / dt

            # Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            q = quaternion_from_euler(0.0, 0.0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            odom_msg.pose.covariance = [
                0.02, 0,    0,    0,    0,    0,
                0,    0.02, 0,    0,    0,    0,
                0,    0,    1e6,  0,    0,    0,
                0,    0,    0,    1e6,  0,    0,
                0,    0,    0,    0,    1e6,  0,
                0,    0,    0,    0,    0,    0.05,
            ]

            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = vth
            odom_msg.twist.covariance = [
                0.05, 0,    0,    0,    0,    0,
                0,    0.05, 0,    0,    0,    0,
                0,    0,    1e6,  0,    0,    0,
                0,    0,    0,    1e6,  0,    0,
                0,    0,    0,    0,    1e6,  0,
                0,    0,    0,    0,    0,    0.1,
            ]

            self.odom_pub.publish(odom_msg)

            # TF odom -> base_link
            #t = TransformStamped()
            #t.header.stamp = now.to_msg()
            #t.header.frame_id = 'odom'
            #t.child_frame_id = 'base_link'
            #t.transform.translation.x = float(self.x)
            #t.transform.translation.y = float(self.y)
            #t.transform.translation.z = 0.0
            #t.transform.rotation.x = q[0]
            #t.transform.rotation.y = q[1]
            #t.transform.rotation.z = q[2]
            #t.transform.rotation.w = q[3]
            #self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error en lectura serial/odom: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

