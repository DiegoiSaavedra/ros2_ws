#!/usr/bin/env python3
import math
import threading

import serial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler


class BaseDriver(Node):
    """Driver de la base diferencial.

    Un hilo dedicado lee el serial con lecturas bloqueantes y un buffer
    propio: solo se procesan lineas completas (terminadas en \\n), con
    timestamp real de llegada. Los comandos y el watchdog corren en el
    executor y escriben al puerto protegidos por un lock.
    """

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

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Estado odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # La primera lectura solo inicializa los contadores: el ESP32 puede
        # llevar ticks acumulados y el delta inicial sería gigante.
        self.ticks_initialized = False
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        self.stop_sent = True  # arranca detenido; no hace falta mandar STOP

        # Serial compartido entre el hilo lector y el executor
        self.ser = None
        self._ser_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)

    # ---------- Serial ----------
    def _open_serial(self):
        try:
            with self._ser_lock:
                # exclusive: si otro proceso ya tiene el puerto, falla aquí
                # en vez de pelear silenciosamente por los datos.
                self.ser = serial.Serial(self.port, self.baud,
                                         timeout=0.05, exclusive=True)
                self.ser.reset_input_buffer()
            self.ticks_initialized = False
            self.get_logger().info(f"Serial abierto en {self.port} @ {self.baud} baud")
            return True
        except Exception as e:
            with self._ser_lock:
                self.ser = None
            self.get_logger().error(f"No se pudo abrir {self.port}: {e}",
                                    throttle_duration_sec=10.0)
            return False

    def _close_serial(self):
        with self._ser_lock:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _serial_write(self, data: bytes) -> bool:
        with self._ser_lock:
            if not (self.ser and self.ser.is_open):
                return False
            try:
                self.ser.write(data)
                return True
            except Exception as e:
                self.get_logger().error(f"Error escribiendo serial: {e}")
        self._close_serial()
        return False

    # ---------- Hilo lector ----------
    def _reader_loop(self):
        buf = bytearray()
        while not self._stop_event.is_set():
            ser = self.ser
            if ser is None or not ser.is_open:
                if not self._open_serial():
                    self._stop_event.wait(2.0)
                buf.clear()
                continue
            try:
                chunk = ser.read(max(1, ser.in_waiting))
            except Exception as e:
                self.get_logger().error(f"Error leyendo serial: {e}")
                self._close_serial()
                continue
            if not chunk:
                continue
            buf += chunk
            if b'\n' not in buf:
                if len(buf) > 1024:
                    self.get_logger().warning("Buffer serial sin saltos de línea; descartando")
                    buf.clear()
                continue

            *lines, rest = buf.split(b'\n')
            buf = bytearray(rest)

            # Los ticks son acumulativos: si llegaron varias líneas juntas
            # (ráfaga tras una pausa), basta procesar la última válida.
            last_valid = None
            for raw in lines:
                line = raw.decode(errors='ignore').strip()
                if not line:
                    continue
                if self.verbose_serial:
                    self.get_logger().debug(f"SER:{line}")
                if line != 'ACK':
                    last_valid = line
            if last_valid is not None:
                try:
                    self._process_line(last_valid)
                except Exception as e:
                    self.get_logger().error(f"Error procesando odometría: {e}")

    # ---------- Comandos ----------
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
            self.get_logger().warning("Serial no disponible para enviar cmd_vel",
                                      throttle_duration_sec=5.0)

    def watchdog_check(self):
        # STOP (una sola vez) si no llega /cmd_vel en cmd_timeout
        if self.stop_sent:
            return
        if (self.get_clock().now() - self.last_cmd_time) > Duration(seconds=self.cmd_timeout):
            if self._serial_write(b"0,0\n"):
                self.stop_sent = True
                self.get_logger().warning("Watchdog: sin cmd_vel, motores detenidos")

    # ---------- Odometría ----------
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

        # Descarta saltos imposibles (reset/overflow del contador del ESP32)
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
        # vy=0 con varianza chica: restricción no-holonómica (el robot
        # no puede moverse de lado); el EKF la fusiona como medición.
        odom_msg.twist.covariance = [
            0.05, 0,     0,    0,    0,    0,
            0,    0.001, 0,    0,    0,    0,
            0,    0,     1e6,  0,    0,    0,
            0,    0,     0,    1e6,  0,    0,
            0,    0,     0,    0,    1e6,  0,
            0,    0,     0,    0,    0,    0.1,
        ]

        self.odom_pub.publish(odom_msg)

    # ---------- Apagado ----------
    def shutdown(self):
        # STOP a los motores pase lo que pase: sin esto, si el nodo muere
        # en movimiento el ESP32 se queda con el último PWM aplicado.
        self._serial_write(b"0,0\n")
        self._stop_event.set()
        self._reader.join(timeout=1.0)
        self._close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
