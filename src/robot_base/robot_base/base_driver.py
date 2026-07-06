#!/usr/bin/env python3
import math
import serial

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')

        # Parámetros
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

        # Serial (con reintento periódico si falla)
        self.ser = None
        self._open_serial()

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Estado odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # La primera lectura solo inicializa los contadores: el Arduino puede
        # llevar ticks acumulados y el delta inicial sería gigante.
        self.ticks_initialized = False
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        self.stop_sent = True  # arranca detenido; no hace falta mandar STOP

        # Timers
        self.timer = self.create_timer(0.02, self.read_serial)     # ~50 Hz
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)
        self.reconnect_timer = self.create_timer(2.0, self._reconnect_check)

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.ser.reset_input_buffer()
            self.ticks_initialized = False
            self.get_logger().info(f"Serial abierto en {self.port} @ {self.baud} baud")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"No se pudo abrir {self.port}: {e}")

    def _reconnect_check(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warning(f"Reintentando abrir {self.port}...")
            self._open_serial()

    def _serial_write(self, data: bytes) -> bool:
        if not (self.ser and self.ser.is_open):
            return False
        try:
            self.ser.write(data)
            return True
        except Exception as e:
            self.get_logger().error(f"Error escribiendo serial: {e}")
            self._close_serial()
            return False

    def _close_serial(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        omega = float(msg.angular.z)
        L = self.base_width

        v_left = vx - omega * (L / 2.0)
        v_right = vx + omega * (L / 2.0)

        K_PWM = 255.0 / 1.0  # ganancia empírica
        pwm_left = int(max(min(v_left * K_PWM, 255.0), -255.0))
        pwm_right = int(max(min(v_right * K_PWM, 255.0), -255.0))

        cmd = f"{pwm_left},{pwm_right}\n"
        if self._serial_write(cmd.encode()):
            self.last_cmd_time = self.get_clock().now()
            self.stop_sent = False
        else:
            self.get_logger().warning("Serial no disponible para enviar cmd_vel")

    def watchdog_check(self):
        # STOP (una sola vez) si no llega /cmd_vel en cmd_timeout
        if self.stop_sent:
            return
        if (self.get_clock().now() - self.last_cmd_time) > Duration(seconds=self.cmd_timeout):
            if self._serial_write(b"0,0\n"):
                self.stop_sent = True
                self.get_logger().warning("Watchdog: sin cmd_vel, motores detenidos")

    def read_serial(self):
        if not (self.ser and self.ser.is_open):
            return
        try:
            # Drenar todo lo pendiente y quedarse con la última línea válida:
            # los ticks son acumulativos, las líneas intermedias no aportan.
            last_valid = None
            while True:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    break
                if self.verbose_serial:
                    self.get_logger().debug(f"SER:{line}")
                if line != 'ACK':
                    last_valid = line
                if self.ser.in_waiting == 0:
                    break
            if last_valid is not None:
                self._process_line(last_valid)
        except Exception as e:
            self.get_logger().error(f"Error en lectura serial/odom: {e}")
            self._close_serial()

    def _process_line(self, line: str):
        parts = line.split(',')
        if len(parts) != 2:
            return

        try:
            ticks_left = int(parts[0])
            ticks_right = int(parts[1])
        except ValueError:
            return

        now = self.get_clock().now()

        if not self.ticks_initialized:
            # Primera lectura tras (re)conexión: solo fija la referencia
            self.prev_ticks_left = ticks_left
            self.prev_ticks_right = ticks_right
            self.last_time = now
            self.ticks_initialized = True
            return

        dticks_left = ticks_left - self.prev_ticks_left
        dticks_right = ticks_right - self.prev_ticks_right
        self.prev_ticks_left = ticks_left
        self.prev_ticks_right = ticks_right

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Descarta saltos imposibles (reset/overflow del contador del Arduino)
        max_dticks = self.ticks_per_revolution * 5 * dt  # ~5 rev/s de tope físico
        if abs(dticks_left) > max_dticks or abs(dticks_right) > max_dticks:
            self.get_logger().warning(
                f"Salto de ticks descartado (L={dticks_left}, R={dticks_right}); re-sincronizando")
            return

        dist_per_tick = (2.0 * math.pi * self.wheel_radius) / float(self.ticks_per_revolution)
        dist_left = dticks_left * dist_per_tick
        dist_right = dticks_right * dist_per_tick

        d = (dist_left + dist_right) / 2.0
        dtheta = (dist_right - dist_left) / self.base_width

        # Integración de punto medio: proyecta con la orientación intermedia
        theta_mid = self.theta + dtheta / 2.0
        self.x += d * math.cos(theta_mid)
        self.y += d * math.sin(theta_mid)
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

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


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
