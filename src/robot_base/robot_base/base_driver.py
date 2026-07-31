#!/usr/bin/env python3
import fcntl
import glob
import math
import sys
import threading
import time

import serial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler

from robot_base.safety import (
    DEFAULT_CMD_VEL_TIMEOUT,
    DEFAULT_DRIVE_DEADBAND,
    DEFAULT_MAX_WHEEL_SPEED,
    DEFAULT_MIN_WHEEL_SPEED,
    DEFAULT_PIVOT_DEADBAND,
    DEFAULT_REVERSE_DEADBAND,
    DEFAULT_SERIAL_RX_TIMEOUT,
    MAX_PWM,
    VELOCITY_EPSILON,
    WheelSpeedPI,
    check_deadband,
    command_timed_out,
    parse_encoder_line,
    positive_timeout,
    require_finite,
    twist_to_wheel_velocities,
    velocity_to_pwm,
)

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
        self.declare_parameter('cmd_vel_timeout', DEFAULT_CMD_VEL_TIMEOUT)
        self.declare_parameter('serial_rx_timeout', DEFAULT_SERIAL_RX_TIMEOUT)
        self.declare_parameter('verbose_serial', False)         # logs crudos
        # Umbrales de arranque reales; ver comentario en safety.py.
        self.declare_parameter('pwm_deadband_drive', DEFAULT_DRIVE_DEADBAND)
        self.declare_parameter('pwm_deadband_reverse', DEFAULT_REVERSE_DEADBAND)
        self.declare_parameter('pwm_deadband_pivot', DEFAULT_PIVOT_DEADBAND)
        self.declare_parameter('min_wheel_speed', DEFAULT_MIN_WHEEL_SPEED)
        self.declare_parameter('max_wheel_speed', DEFAULT_MAX_WHEEL_SPEED)
        self.declare_parameter('wheel_speed_pi_enabled', True)
        self.declare_parameter('wheel_speed_pi_kp', 800.0)
        self.declare_parameter('wheel_speed_pi_ki', 250.0)
        self.declare_parameter('wheel_speed_pi_correction_limit', 35.0)
        self.declare_parameter('wheel_speed_pi_integral_limit', 0.10)
        self.declare_parameter('wheel_speed_target_scale', 0.62)
        self.declare_parameter('left_feedforward_scale', 0.87)
        self.declare_parameter('right_feedforward_scale', 1.00)
        self.declare_parameter('wheel_speed_feedback_timeout', 0.15)
        self.declare_parameter('wheel_breakaway_enabled', True)
        self.declare_parameter('wheel_breakaway_pwm', 226)
        self.declare_parameter('wheel_breakaway_speed', 0.03)
        self.declare_parameter('wheel_breakaway_samples', 3)
        self.declare_parameter('wheel_breakaway_timeout', 0.25)
        self.declare_parameter('wheel_breakaway_pivot_only', True)

        self.port = self.get_parameter(
            'port').get_parameter_value().string_value
        self.baud = self.get_parameter(
            'baud').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value
        self.base_width = self.get_parameter(
            'base_width').get_parameter_value().double_value
        self.ticks_per_revolution = self.get_parameter(
            'ticks_per_revolution').get_parameter_value().integer_value
        raw_cmd_timeout = self.get_parameter(
            'cmd_vel_timeout').get_parameter_value().double_value
        raw_rx_timeout = self.get_parameter(
            'serial_rx_timeout').get_parameter_value().double_value
        self.cmd_vel_timeout = positive_timeout(
            raw_cmd_timeout, DEFAULT_CMD_VEL_TIMEOUT)
        self.serial_rx_timeout = positive_timeout(
            raw_rx_timeout, DEFAULT_SERIAL_RX_TIMEOUT)
        self.verbose_serial = self.get_parameter(
            'verbose_serial').get_parameter_value().bool_value
        self.pwm_deadband_drive = check_deadband(self.get_parameter(
            'pwm_deadband_drive').get_parameter_value().double_value)
        self.pwm_deadband_reverse = check_deadband(self.get_parameter(
            'pwm_deadband_reverse').get_parameter_value().double_value)
        self.pwm_deadband_pivot = check_deadband(self.get_parameter(
            'pwm_deadband_pivot').get_parameter_value().double_value)
        self.min_wheel_speed = require_finite(self.get_parameter(
            'min_wheel_speed').get_parameter_value().double_value,
            'min_wheel_speed')
        self.max_wheel_speed = require_finite(self.get_parameter(
            'max_wheel_speed').get_parameter_value().double_value,
            'max_wheel_speed')
        if self.max_wheel_speed <= 0.0:
            raise ValueError("max_wheel_speed debe ser positivo")
        if not 0.0 <= self.min_wheel_speed < self.max_wheel_speed:
            raise ValueError(
                "min_wheel_speed debe estar entre 0 y max_wheel_speed")
        self.wheel_speed_pi_enabled = self.get_parameter(
            'wheel_speed_pi_enabled').get_parameter_value().bool_value
        self.wheel_speed_pi_kp = self.get_parameter(
            'wheel_speed_pi_kp').get_parameter_value().double_value
        self.wheel_speed_pi_ki = self.get_parameter(
            'wheel_speed_pi_ki').get_parameter_value().double_value
        self.wheel_speed_pi_correction_limit = self.get_parameter(
            'wheel_speed_pi_correction_limit').get_parameter_value().double_value
        self.wheel_speed_pi_integral_limit = self.get_parameter(
            'wheel_speed_pi_integral_limit').get_parameter_value().double_value
        self.wheel_speed_target_scale = self.get_parameter(
            'wheel_speed_target_scale').get_parameter_value().double_value
        self.left_feedforward_scale = self.get_parameter(
            'left_feedforward_scale').get_parameter_value().double_value
        self.right_feedforward_scale = self.get_parameter(
            'right_feedforward_scale').get_parameter_value().double_value
        raw_feedback_timeout = self.get_parameter(
            'wheel_speed_feedback_timeout').get_parameter_value().double_value
        self.wheel_speed_feedback_timeout = positive_timeout(
            raw_feedback_timeout, 0.15)
        self.wheel_breakaway_pwm = self.get_parameter(
            'wheel_breakaway_pwm').get_parameter_value().integer_value
        self.wheel_breakaway_enabled = self.get_parameter(
            'wheel_breakaway_enabled').get_parameter_value().bool_value
        self.wheel_breakaway_speed = self.get_parameter(
            'wheel_breakaway_speed').get_parameter_value().double_value
        self.wheel_breakaway_samples = self.get_parameter(
            'wheel_breakaway_samples').get_parameter_value().integer_value
        self.wheel_breakaway_timeout = self.get_parameter(
            'wheel_breakaway_timeout').get_parameter_value().double_value
        self.wheel_breakaway_pivot_only = self.get_parameter(
            'wheel_breakaway_pivot_only').get_parameter_value().bool_value

        if self.cmd_vel_timeout != raw_cmd_timeout:
            self.get_logger().error(
                f"cmd_vel_timeout inválido ({raw_cmd_timeout!r}); "
                f"se usa el valor seguro {self.cmd_vel_timeout:.3f} s")
        if self.serial_rx_timeout != raw_rx_timeout:
            self.get_logger().error(
                f"serial_rx_timeout inválido ({raw_rx_timeout!r}); "
                f"se usa el valor seguro {self.serial_rx_timeout:.3f} s")
        require_finite(self.wheel_speed_target_scale, 'wheel_speed_target_scale')
        if self.wheel_speed_target_scale <= 0.0:
            raise ValueError(
                "wheel_speed_target_scale debe ser positivo")
        require_finite(self.left_feedforward_scale, 'left_feedforward_scale')
        require_finite(self.right_feedforward_scale, 'right_feedforward_scale')
        if (
            self.left_feedforward_scale <= 0.0
            or self.right_feedforward_scale <= 0.0
        ):
            raise ValueError("Los scales de feedforward deben ser positivos")
        require_finite(self.wheel_breakaway_speed, 'wheel_breakaway_speed')
        require_finite(self.wheel_breakaway_timeout, 'wheel_breakaway_timeout')
        if not 0 < self.wheel_breakaway_pwm <= MAX_PWM:
            raise ValueError("wheel_breakaway_pwm fuera de rango")
        if self.wheel_breakaway_speed <= 0.0:
            raise ValueError("wheel_breakaway_speed debe ser positivo")
        if self.wheel_breakaway_samples <= 0:
            raise ValueError("wheel_breakaway_samples debe ser positivo")
        if self.wheel_breakaway_timeout <= 0.0:
            raise ValueError("wheel_breakaway_timeout debe ser positivo")
        self.left_speed_pi = WheelSpeedPI(
            self.wheel_speed_pi_kp,
            self.wheel_speed_pi_ki,
            self.wheel_speed_pi_correction_limit,
            self.wheel_speed_pi_integral_limit,
        )
        self.right_speed_pi = WheelSpeedPI(
            self.wheel_speed_pi_kp,
            self.wheel_speed_pi_ki,
            self.wheel_speed_pi_correction_limit,
            self.wheel_speed_pi_integral_limit,
        )

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.pi_diag_pub = self.create_publisher(
            Float64MultiArray, 'wheel_pi_state', 20)
        self.breakaway_diag_pub = self.create_publisher(
            Float64MultiArray, 'wheel_breakaway_state', 20)
        self.create_subscription(
            Twist, 'cmd_vel_safe', self.cmd_vel_callback, 10)

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

        # Estado de seguridad: siempre comienza detenido y sin permiso para
        # aceptar movimiento hasta abrir el enlace, enviar STOP y recibir una
        # línea válida de telemetría posterior a la conexión.
        self._state_lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._last_log_times = {}
        self.desired_linear_x = 0.0
        self.desired_angular_z = 0.0
        self.pwm_left = 0
        self.pwm_right = 0
        self.measured_left_speed = 0.0
        self.measured_right_speed = 0.0
        self.last_wheel_feedback_monotonic = None
        self.pi_last_target_left = 0.0
        self.pi_last_target_right = 0.0
        self.breakaway_command_active = False
        self.breakaway_active = False
        self.breakaway_started_at = None
        self.breakaway_last_feedback_at = None
        self.breakaway_consecutive_samples = 0
        self.breakaway_fault_latched = False
        self.last_cmd_monotonic = time.monotonic()
        self._stopped = True
        self._accept_motion = False
        self._shutdown_started = False
        self._odom_ever_published = False

        # Serial compartido entre el hilo lector y el executor
        self.ser = None
        self._ser_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)
        self._reader.start()

    # ---------- Serial ----------
    def _safe_log(self, level: str, message: str,
                  throttle_duration_sec: float = None) -> None:
        """Registra sin permitir que un fallo del logger oculte la parada."""
        if throttle_duration_sec is not None:
            now = time.monotonic()
            key = (level, message[:120])
            with self._log_lock:
                last = self._last_log_times.get(key)
                if last is not None and now - last < throttle_duration_sec:
                    return
                self._last_log_times[key] = now

        try:
            context_is_valid = rclpy.ok(context=self.context)
        except Exception:
            context_is_valid = False

        if not context_is_valid:
            self._stderr_log(level, message)
            return

        try:
            logger = self.get_logger()
            if level == 'error':
                logger.error(message)
            elif level == 'warning':
                logger.warning(message)
            elif level == 'info':
                logger.info(message)
            else:
                logger.debug(message)
        except Exception as log_error:
            self._stderr_log(
                'error',
                f"fallo de log ({type(log_error).__name__}: {log_error}); "
                f"mensaje original: {message}",
            )

    @staticmethod
    def _stderr_log(level: str, message: str) -> None:
        try:
            sys.stderr.write(f"base_driver [{level}]: {message}\n")
            sys.stderr.flush()
        except Exception:
            return

    @staticmethod
    def _exception_detail(place: str, error: Exception) -> str:
        return f"{place}: {type(error).__name__}: {error}"

    def _ros_context_is_valid(self) -> bool:
        try:
            return rclpy.ok(context=self.context)
        except Exception:
            return False

    def _set_stopped_state(self, accept_motion=None) -> None:
        with self._state_lock:
            self.desired_linear_x = 0.0
            self.desired_angular_z = 0.0
            self.pwm_left = 0
            self.pwm_right = 0
            self.left_speed_pi.reset()
            self.right_speed_pi.reset()
            self.pi_last_target_left = 0.0
            self.pi_last_target_right = 0.0
            self.measured_left_speed = 0.0
            self.measured_right_speed = 0.0
            self.last_wheel_feedback_monotonic = None
            self.breakaway_command_active = False
            self.breakaway_active = False
            self.breakaway_started_at = None
            self.breakaway_last_feedback_at = None
            self.breakaway_consecutive_samples = 0
            self._stopped = True
            if accept_motion is not None:
                self._accept_motion = bool(accept_motion)

    def _set_link_ready(self) -> None:
        """Habilita únicamente comandos nuevos, sin recuperar ningún PWM."""
        with self._state_lock:
            if not self._accept_motion:
                self.last_cmd_monotonic = time.monotonic()
                self._accept_motion = True

    def _motion_is_allowed(self) -> bool:
        with self._state_lock:
            return self._accept_motion and not self._shutdown_started

    def emergency_stop(self, reason: str, *, level: str = 'warning',
                       repeat: int = 1) -> None:
        """Pasa el estado a cero e intenta transmitir STOP sin propagar errores."""
        self._set_stopped_state()
        sent = False
        attempts = max(1, int(repeat))
        for attempt in range(attempts):
            try:
                if not self._serial_write(b"0,0\n"):
                    break
                sent = True
            except Exception as error:
                self._set_stopped_state(accept_motion=False)
                self._safe_log(
                    'error', self._exception_detail(
                        'emergency_stop/escritura', error), 5.0)
                break
            if attempt + 1 < attempts:
                time.sleep(0.01)

        suffix = "STOP enviado" if sent else "estado local en cero; enlace no disponible"
        self._safe_log(
            level, f"Parada de seguridad: {reason} ({suffix})", 5.0)

    def _open_serial(self):
        candidate = None
        try:
            # exclusive: si otro proceso ya tiene el puerto, falla aquí
            # en vez de pelear silenciosamente por los datos.
            candidate = serial.Serial(
                self.port,
                self.baud,
                timeout=0.05,
                write_timeout=0.05,
                exclusive=True,
            )
            candidate.reset_input_buffer()
            # STOP se transmite antes de exponer el puerto a callbacks.
            candidate.write(b"0,0\n")
            candidate.flush()
            with self._ser_lock:
                self.ser = candidate
            self.ticks_initialized = False
            self._set_stopped_state(accept_motion=False)
            self._safe_log(
                'info', f"Serial abierto en {self.port} @ {self.baud} baud; "
                "STOP enviado, esperando telemetría válida")
            return True
        except Exception as e:
            self._set_stopped_state(accept_motion=False)
            with self._ser_lock:
                self.ser = None
            if candidate is not None:
                try:
                    candidate.close()
                except Exception as close_error:
                    self._safe_log(
                        'error', self._exception_detail(
                            '_open_serial/cierre de candidato', close_error), 10.0)
            self._safe_log(
                'error', self._exception_detail(
                    f"_open_serial({self.port})", e), 10.0)
            return False

    def _usb_reset_esp32(self) -> bool:
        # El stack USB del S2 Mini se cuelga a veces (3 veces el 9-jul,
        # sobre todo tras cortes de energia): el puerto sigue abierto y
        # enumerado pero no fluye ningun dato, y cerrar/reabrir no alcanza.
        # El unico remedio sin tocar cables es un reset USB del dispositivo
        # (equivale a desenchufar y enchufar). Requiere permiso de escritura
        # sobre /dev/bus/usb/... (regla udev 99-esp32-usbreset.rules).
        USBDEVFS_RESET = 21780
        VID, PID = '303a', '80c2'  # LOLIN S2 Mini
        self._set_stopped_state(accept_motion=False)
        try:
            for dev in glob.glob('/sys/bus/usb/devices/*'):
                try:
                    with open(dev + '/idVendor') as f:
                        vid = f.read().strip()
                    with open(dev + '/idProduct') as f:
                        pid = f.read().strip()
                except OSError:
                    continue
                if (vid, pid) != (VID, PID):
                    continue
                with open(dev + '/busnum') as f:
                    bus = int(f.read())
                with open(dev + '/devnum') as f:
                    num = int(f.read())
                with open('/dev/bus/usb/%03d/%03d' % (bus, num), 'wb') as f:
                    fcntl.ioctl(f, USBDEVFS_RESET, 0)
                self._safe_log('warning', "Reset USB enviado al ESP32")
                return True
            self._safe_log('error', "ESP32 no encontrado en el bus USB", 5.0)
        except Exception as e:
            self._safe_log(
                'error', self._exception_detail('_usb_reset_esp32', e), 5.0)
        return False

    def _close_serial(self, reason: str = 'cierre solicitado'):
        self._set_stopped_state(accept_motion=False)
        with self._ser_lock:
            serial_port = self.ser
            self.ser = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception as error:
                self._safe_log(
                    'error', self._exception_detail(
                        f"_close_serial/{reason}", error), 5.0)

    def _serial_write(self, data: bytes) -> bool:
        error = None
        with self._ser_lock:
            if not (self.ser and self.ser.is_open):
                return False
            try:
                self.ser.write(data)
                return True
            except Exception as e:
                error = e
        self._set_stopped_state(accept_motion=False)
        self._safe_log(
            'error', self._exception_detail('_serial_write', error), 5.0)
        self._close_serial('fallo de escritura')
        return False

    # ---------- Hilo lector ----------
    def _reader_loop(self):
        try:
            self._reader_loop_impl()
        except Exception as error:
            self._set_stopped_state(accept_motion=False)
            self.emergency_stop(
                self._exception_detail('_reader_loop/no controlado', error),
                level='error')
            self._close_serial('excepción no controlada del lector')

    def _reader_loop_impl(self):
        buf = bytearray()
        last_rx = time.monotonic()
        last_reset = 0.0
        open_failures = 0
        rx_fault_active = False
        while not self._stop_event.is_set():
            ser = self.ser
            if ser is None or not ser.is_open:
                if not self._open_serial():
                    # Cuando el stack USB del S2 Mini se cuelga, /dev/esp32
                    # sigue existiendo pero toda escritura da Write timeout y
                    # reabrir nunca funciona: hasta ahora habia que desenchufar
                    # el cable a mano. Tras 3 intentos fallidos se usa el mismo
                    # reset USB que ya existia para la telemetria ausente.
                    open_failures += 1
                    now = time.monotonic()
                    if open_failures >= 3 and now - last_reset > 10.0:
                        self._safe_log(
                            'error',
                            f"{open_failures} aperturas fallidas de "
                            f"{self.port}; se intentará reset USB")
                        try:
                            self._usb_reset_esp32()
                        except Exception as error:
                            self._safe_log(
                                'error', self._exception_detail(
                                    '_reader_loop/reset USB en apertura',
                                    error), 10.0)
                        last_reset = time.monotonic()
                        open_failures = 0
                    self._stop_event.wait(2.0)
                else:
                    open_failures = 0
                buf.clear()
                last_rx = time.monotonic()
                rx_fault_active = False
                continue
            try:
                chunk = ser.read(max(1, ser.in_waiting))
            except Exception as e:
                self.emergency_stop(
                    self._exception_detail('_reader_loop/read', e), level='error')
                self._close_serial('fallo de lectura')
                continue
            if not chunk:
                # El firmware streamea ticks a 20 Hz siempre: puerto abierto
                # sin datos implica que no se puede verificar el estado físico.
                now = time.monotonic()
                if now - last_rx >= self.serial_rx_timeout and not rx_fault_active:
                    self._set_stopped_state(accept_motion=False)
                    self.emergency_stop(
                        f"sin telemetría del ESP32 durante "
                        f"{now - last_rx:.3f} s", level='error')
                    rx_fault_active = True
                # Se conserva el auto-rescate existente, pero siempre después
                # de haber ordenado STOP y como máximo una vez cada 10 s.
                if now - last_rx > 3.0 and now - last_reset > 10.0:
                    self.emergency_stop(
                        "3 s sin datos del ESP32; se intentará reset USB",
                        level='error')
                    self._close_serial('telemetría ausente antes de reset USB')
                    try:
                        self._usb_reset_esp32()
                    except Exception as error:
                        self.emergency_stop(
                            self._exception_detail(
                                '_reader_loop/reset USB', error), level='error')
                    last_reset = time.monotonic()
                    last_rx = last_reset
                continue
            last_rx = time.monotonic()
            buf += chunk
            if b'\n' not in buf:
                if len(buf) > 1024:
                    self._set_stopped_state(accept_motion=False)
                    self.emergency_stop(
                        "buffer serial excedió 1024 bytes sin paquete completo",
                        level='error')
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
                    ticks_left, ticks_right = parse_encoder_line(last_valid)
                    self._process_ticks(ticks_left, ticks_right)
                    self._set_link_ready()
                    rx_fault_active = False
                except Exception as e:
                    self._set_stopped_state(accept_motion=False)
                    self.emergency_stop(
                        self._exception_detail(
                            f"_reader_loop/paquete {last_valid!r}", e),
                        level='error')

    # ---------- Comandos ----------
    def _publish_pi_diagnostic(
        self,
        target_left: float,
        target_right: float,
        pwm_left: int,
        pwm_right: int,
        now: float,
    ) -> None:
        with self._state_lock:
            measured_left = self.measured_left_speed
            measured_right = self.measured_right_speed
            feedback_time = self.last_wheel_feedback_monotonic
        feedback_age = (
            -1.0 if feedback_time is None else now - feedback_time)
        feedback_timeout = (
            feedback_time is None
            or feedback_age > self.wheel_speed_feedback_timeout)
        message = Float64MultiArray()
        # Orden estable:
        # t, targets L/R, medidas L/R, errores L/R, P L/R,
        # integrales crudas L/R, términos I L/R, correcciones L/R,
        # PWM L/R, saturación L/R, edad feedback, timeout feedback.
        message.data = [
            now,
            target_left * self.wheel_speed_target_scale,
            target_right * self.wheel_speed_target_scale,
            measured_left,
            measured_right,
            self.left_speed_pi.error,
            self.right_speed_pi.error,
            self.left_speed_pi.proportional,
            self.right_speed_pi.proportional,
            self.left_speed_pi.integral,
            self.right_speed_pi.integral,
            self.left_speed_pi.integral_term,
            self.right_speed_pi.integral_term,
            self.left_speed_pi.correction,
            self.right_speed_pi.correction,
            float(pwm_left),
            float(pwm_right),
            float(self.left_speed_pi.saturated),
            float(self.right_speed_pi.saturated),
            feedback_age,
            float(feedback_timeout),
        ]
        self.pi_diag_pub.publish(message)

    def _publish_breakaway_diagnostic(
        self,
        event: int,
        now: float,
        measured_left: float,
        measured_right: float,
        pwm_before_left: int,
        pwm_before_right: int,
        pwm_after_left: int,
        pwm_after_right: int,
    ) -> None:
        started_at = (
            -1.0 if self.breakaway_started_at is None
            else self.breakaway_started_at)
        duration = (
            0.0 if self.breakaway_started_at is None
            else max(0.0, now - self.breakaway_started_at))
        message = Float64MultiArray()
        # event: 1=inicio, 2=muestra, 3=transición, 4=fallo.
        message.data = [
            now,
            float(event),
            float(self.breakaway_active),
            started_at,
            duration,
            float(self.breakaway_consecutive_samples),
            measured_left,
            measured_right,
            float(pwm_before_left),
            float(pwm_before_right),
            float(pwm_after_left),
            float(pwm_after_right),
        ]
        self.breakaway_diag_pub.publish(message)

    def _breakaway_output(
        self,
        target_left: float,
        target_right: float,
        feedforward_left: int,
        feedforward_right: int,
        now: float,
        is_pivot: bool,
    ):
        """Devuelve PWM de breakaway, transición o None para control PI."""
        if not self.wheel_breakaway_enabled:
            return None

        command_nonzero = (
            abs(target_left) >= VELOCITY_EPSILON
            or abs(target_right) >= VELOCITY_EPSILON
        )
        if not command_nonzero:
            self.breakaway_command_active = False
            self.breakaway_active = False
            self.breakaway_started_at = None
            self.breakaway_last_feedback_at = None
            self.breakaway_consecutive_samples = 0
            self.breakaway_fault_latched = False
            return None

        # El golpe se limita al pivote (30-jul-2026). Es el unico arranque
        # donde el PWM de romper la friccion esta muy por encima del de
        # mantener el movimiento: 200 contra ~100, porque la rueda loca tiene
        # que reorientarse 180 grados. En avance el deadband de 100 ya
        # arranca las dos ruedas, asi que meter 226 ahi no arregla nada y si
        # cambiaria el comportamiento del pasillo, que quedo validado en
        # becef4a. No se toca el latch de fallo: solo lo limpia un comando
        # cero, igual que antes.
        if self.wheel_breakaway_pivot_only and not is_pivot:
            self.breakaway_command_active = False
            self.breakaway_active = False
            self.breakaway_started_at = None
            self.breakaway_last_feedback_at = None
            self.breakaway_consecutive_samples = 0
            return None

        if self.breakaway_fault_latched:
            return "failure"

        with self._state_lock:
            measured_left = self.measured_left_speed
            measured_right = self.measured_right_speed
            feedback_at = self.last_wheel_feedback_monotonic
        pwm_left_breakaway = (
            0 if abs(target_left) < VELOCITY_EPSILON
            else int(math.copysign(self.wheel_breakaway_pwm, target_left)))
        pwm_right_breakaway = (
            0 if abs(target_right) < VELOCITY_EPSILON
            else int(math.copysign(self.wheel_breakaway_pwm, target_right)))

        if not self.breakaway_command_active:
            self.breakaway_command_active = True
            self.breakaway_active = True
            self.breakaway_started_at = now
            self.breakaway_last_feedback_at = feedback_at
            self.breakaway_consecutive_samples = 0
            self.left_speed_pi.reset()
            self.right_speed_pi.reset()
            self.pi_last_target_left = 0.0
            self.pi_last_target_right = 0.0
            self._publish_breakaway_diagnostic(
                1, now, measured_left, measured_right,
                pwm_left_breakaway, pwm_right_breakaway,
                pwm_left_breakaway, pwm_right_breakaway)
            return pwm_left_breakaway, pwm_right_breakaway

        if not self.breakaway_active:
            return None

        if (
            feedback_at is not None
            and feedback_at != self.breakaway_last_feedback_at
        ):
            self.breakaway_last_feedback_at = feedback_at
            if (
                measured_left * target_left > 0.0
                and abs(measured_left) > self.wheel_breakaway_speed
                and measured_right * target_right > 0.0
                and abs(measured_right) > self.wheel_breakaway_speed
            ):
                self.breakaway_consecutive_samples += 1
            else:
                self.breakaway_consecutive_samples = 0

        elapsed = now - self.breakaway_started_at
        if (
            self.breakaway_consecutive_samples >= self.wheel_breakaway_samples
            and elapsed <= self.wheel_breakaway_timeout
        ):
            # El trim ya viene aplicado SOBRE LA VELOCIDAD dentro del
            # feedforward (ver cmd_vel_callback). Volver a multiplicarlo aqui
            # lo aplicaba dos veces: 0.915^2 = 0.837 en la rueda izquierda.
            # Corregido el 30-jul-2026.
            post_left = feedforward_left
            post_right = feedforward_right
            self.breakaway_active = False
            self._publish_breakaway_diagnostic(
                3, now, measured_left, measured_right,
                pwm_left_breakaway, pwm_right_breakaway,
                post_left, post_right)
            # El sosten siempre queda por debajo del golpe (226), asi que la
            # transicion nunca es un salto ascendente de PWM.
            return post_left, post_right

        if elapsed >= self.wheel_breakaway_timeout:
            self.breakaway_active = False
            self.breakaway_command_active = False
            self.breakaway_fault_latched = True
            self._publish_breakaway_diagnostic(
                4, now, measured_left, measured_right,
                pwm_left_breakaway, pwm_right_breakaway, 0, 0)
            return "failure"

        self._publish_breakaway_diagnostic(
            2, now, measured_left, measured_right,
            pwm_left_breakaway, pwm_right_breakaway,
            pwm_left_breakaway, pwm_right_breakaway)
        return pwm_left_breakaway, pwm_right_breakaway

    def _apply_wheel_speed_pi(
        self,
        target_left: float,
        target_right: float,
        feedforward_left: int,
        feedforward_right: int,
        now: float,
    ):
        """Corrige cada PWM con su propio PI y feedback fresco de encoders."""
        if not self.wheel_speed_pi_enabled:
            return feedforward_left, feedforward_right

        if (
            abs(target_left) < VELOCITY_EPSILON
            and abs(target_right) < VELOCITY_EPSILON
        ):
            self.left_speed_pi.reset()
            self.right_speed_pi.reset()
            self.pi_last_target_left = 0.0
            self.pi_last_target_right = 0.0
            return 0, 0

        with self._state_lock:
            measured_left = self.measured_left_speed
            measured_right = self.measured_right_speed
            feedback_time = self.last_wheel_feedback_monotonic

        if (
            feedback_time is None
            or now - feedback_time > self.wheel_speed_feedback_timeout
            or max(abs(measured_left), abs(measured_right))
            < VELOCITY_EPSILON
        ):
            self.left_speed_pi.reset()
            self.right_speed_pi.reset()
            return feedforward_left, feedforward_right

        if target_left * self.pi_last_target_left < 0.0:
            self.left_speed_pi.reset()
        if target_right * self.pi_last_target_right < 0.0:
            self.right_speed_pi.reset()
        self.pi_last_target_left = target_left
        self.pi_last_target_right = target_right

        scaled_left = target_left * self.wheel_speed_target_scale
        scaled_right = target_right * self.wheel_speed_target_scale
        if abs(target_left) < VELOCITY_EPSILON:
            self.left_speed_pi.reset()
            correction_left = 0.0
            feedforward_left = 0
        else:
            correction_left = self.left_speed_pi.update(
                scaled_left, measured_left, now)
        if abs(target_right) < VELOCITY_EPSILON:
            self.right_speed_pi.reset()
            correction_right = 0.0
            feedforward_right = 0
        else:
            correction_right = self.right_speed_pi.update(
                scaled_right, measured_right, now)

        pwm_left = int(round(feedforward_left + correction_left))
        pwm_right = int(round(feedforward_right + correction_right))
        pwm_left = max(-MAX_PWM, min(MAX_PWM, pwm_left))
        pwm_right = max(-MAX_PWM, min(MAX_PWM, pwm_right))
        return pwm_left, pwm_right

    def cmd_vel_callback(self, msg: Twist):
        try:
            vx = float(msg.linear.x)
            omega = float(msg.angular.z)
            if not math.isfinite(vx) or not math.isfinite(omega):
                self.emergency_stop(
                    f"Twist no finito: linear.x={vx!r}, angular.z={omega!r}",
                    level='error')
                return

            if not self._motion_is_allowed():
                self.emergency_stop(
                    "Twist recibido antes de una conexión y telemetría válidas",
                    level='error')
                return

            # Un deadband por sentido de movimiento: la rueda loca tiene que
            # reorientarse para pivotar o para invertir la marcha, y eso sube
            # mucho el PWM de arranque (200 y 160 medidos, contra 100 en
            # avance). El calculo puro garantiza que cero sigue en cero.
            is_pivot = abs(vx) < 0.02
            if is_pivot:
                deadband = self.pwm_deadband_pivot
            elif vx < 0.0:
                deadband = self.pwm_deadband_reverse
            else:
                deadband = self.pwm_deadband_drive
            v_left, v_right = twist_to_wheel_velocities(
                vx, omega, self.base_width, self.max_wheel_speed)
            # El trim corrige la asimetria mecanica entre ruedas y se aplica
            # sobre la VELOCIDAD, no sobre el PWM: la curva PWM->velocidad no
            # pasa por el origen, asi que recortar un 8% de PWM no recorta un
            # 8% de velocidad. v_left/v_right se conservan sin escalar como
            # objetivo real de la rueda para el PI y el breakaway.
            feedforward_left = velocity_to_pwm(
                v_left * self.left_feedforward_scale,
                deadband, self.min_wheel_speed, self.max_wheel_speed)
            feedforward_right = velocity_to_pwm(
                v_right * self.right_feedforward_scale,
                deadband, self.min_wheel_speed, self.max_wheel_speed)
            control_now = time.monotonic()
            breakaway = self._breakaway_output(
                v_left,
                v_right,
                feedforward_left,
                feedforward_right,
                control_now,
                is_pivot,
            )
            if breakaway == "failure":
                self._publish_pi_diagnostic(
                    v_left, v_right, 0, 0, control_now)
                self.emergency_stop(
                    "BREAKAWAY_FAILURE: una rueda no superó 0.03 m/s "
                    "en 0.25 s",
                    level='error',
                )
                return
            if breakaway is not None:
                pwm_left, pwm_right = breakaway
            else:
                # El trim ya está aplicado en el feedforward (sobre la
                # velocidad). La corrección PI conserva su escala completa.
                pwm_left = feedforward_left
                pwm_right = feedforward_right
                pwm_left, pwm_right = self._apply_wheel_speed_pi(
                    v_left,
                    v_right,
                    pwm_left,
                    pwm_right,
                    control_now,
                )

            # Defensa final antes de formar el paquete y convertir a hardware.
            require_finite(pwm_left, 'left PWM')
            require_finite(pwm_right, 'right PWM')
            cmd = f"{pwm_left},{pwm_right}\n"
            if not self._serial_write(cmd.encode('ascii')):
                self.emergency_stop(
                    "no se pudo transmitir el comando de velocidad",
                    level='error')
                return

            now = time.monotonic()
            accepted = False
            with self._state_lock:
                if self._accept_motion and not self._shutdown_started:
                    self.desired_linear_x = vx
                    self.desired_angular_z = omega
                    self.pwm_left = pwm_left
                    self.pwm_right = pwm_right
                    self.last_cmd_monotonic = now
                    self._stopped = pwm_left == 0 and pwm_right == 0
                    accepted = True
            if not accepted:
                self.emergency_stop(
                    "la conexión dejó de ser válida durante el callback",
                    level='error')
            else:
                self._publish_pi_diagnostic(
                    v_left, v_right, pwm_left, pwm_right, now)
        except Exception as error:
            self.emergency_stop(
                self._exception_detail('cmd_vel_callback', error), level='error')

    def watchdog_check(self):
        try:
            now = time.monotonic()
            with self._state_lock:
                stopped = self._stopped
                last_command = self.last_cmd_monotonic
            if not stopped and command_timed_out(
                    now, last_command, self.cmd_vel_timeout):
                self.emergency_stop(
                    f"watchdog: sin Twist válido durante "
                    f"{now - last_command:.3f} s")

            # Si el enlace con el ESP32 cae (se cuelga su stack USB), dejan de
            # llegar ticks y /odom deja de publicarse. El EKF no se detiene por
            # eso: sigue extrapolando la ultima velocidad, asi que en Foxglove
            # el robot "avanza" estando quieto, el TF odom->base_link se va, y
            # slam_toolbox intenta encajar scans identicos en poses distintas y
            # arruina el mapa. Publicando twist cero mientras el enlace no es
            # valido, el EKF recibe "estoy quieto" y la estimacion se congela.
            if (
                self._odom_ever_published
                and not self._shutdown_started
                and not self._motion_is_allowed()
            ):
                self._publish_odom(self.get_clock().now(), 0.0, 0.0)
        except Exception as error:
            self.emergency_stop(
                self._exception_detail('watchdog_check', error), level='error')

    # ---------- Odometría ----------
    def _process_ticks(self, ticks_left: int, ticks_right: int):
        # El manejador de señales de rclpy puede invalidar el contexto antes
        # de que spin() retorne. El hilo serie no debe publicar durante esa
        # breve ventana de cierre.
        if self._shutdown_started or not self._ros_context_is_valid():
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
        with self._state_lock:
            self.measured_left_speed = dist_left / dt
            self.measured_right_speed = dist_right / dt
            self.last_wheel_feedback_monotonic = time.monotonic()

        self._publish_odom(now, vx, vth)

    def _publish_odom(self, now, vx: float, vth: float) -> None:
        """Publica la pose acumulada con la velocidad indicada."""
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

        if self._shutdown_started or not self._ros_context_is_valid():
            return
        self.odom_pub.publish(odom_msg)
        self._odom_ever_published = True

    # ---------- Apagado ----------
    def shutdown(self, reason: str = 'shutdown'):
        with self._state_lock:
            if self._shutdown_started:
                return
            self._shutdown_started = True
            self._accept_motion = False

        # Tres STOP con pausas breves; emergency_stop es idempotente y no
        # propaga errores aunque el contexto ROS o el puerto ya no existan.
        self.emergency_stop(reason, level='info', repeat=3)
        self._stop_event.set()
        if self._reader.is_alive():
            self._reader.join(timeout=0.25)
        self._close_serial(reason)

    def destroy_node(self):
        self.shutdown('destroy_node')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BaseDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.shutdown('KeyboardInterrupt')
    except ExternalShutdownException:
        if node is not None:
            node.shutdown('SIGINT/SIGTERM o cierre externo')
    except Exception as error:
        if node is not None:
            node.shutdown(node._exception_detail('main', error))
        raise
    finally:
        if node is not None:
            node.shutdown('bloque finally de main')
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception as error:
            sys.stderr.write(
                f"base_driver: error finalizando ROS: "
                f"{type(error).__name__}: {error}\n")


if __name__ == '__main__':
    main()
