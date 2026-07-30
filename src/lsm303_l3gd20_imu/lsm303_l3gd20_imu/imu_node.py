#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64, Header

try:
    from smbus2 import SMBus
except Exception:
    SMBus = None

# ================= I2C addresses =================
ADDR_ACC = 0x19  # LSM303DLHC accelerometer
ADDR_MAG = 0x1E  # LSM303DLHC magnetometer
ADDR_GYR = 0x69  # gyro (este hardware responde en 0x69; ver imu_params.yaml)

# ============== LSM303DLHC registers =============
# Accelerometer
CTRL_REG1_A = 0x20
CTRL_REG4_A = 0x23
OUT_X_L_A   = 0x28  # need auto-increment

# Magnetometer
CRA_REG_M = 0x00
CRB_REG_M = 0x01
MR_REG_M  = 0x02
OUT_X_H_M = 0x03  # order: X, Z, Y (H,L)

# ================ L3GD20 registers ===============
CTRL_REG1_G = 0x20
CTRL_REG4_G = 0x23
OUT_X_L_G   = 0x28  # need auto-increment

G_TO_MS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0

STATIONARY_CONFIRMATION_S = 2.0
ODOM_STALE_TIMEOUT_S = 0.25
COMMAND_ACTIVE_TIMEOUT_S = 0.5
ZERO_VELOCITY_TOLERANCE = 1e-9
GYRO_BIAS_TIME_CONSTANT_S = 60.0
GYRO_RAW_FILTER_TIME_CONSTANT_S = 2.0
GYRO_BIAS_MAX_RATE_DPS_PER_S = 0.02 / 60.0
BIAS_LOG_INTERVAL_S = 60.0

@dataclass
class BiasScale:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    mx_bias: float = 0.0
    my_bias: float = 0.0
    mz_bias: float = 0.0
    mx_scale: float = 1.0
    my_scale: float = 1.0
    mz_scale: float = 1.0


@dataclass
class StationaryGyroBiasAdapter:
    """Adapta lentamente gyro_z sin decidir por sí mismo si hay reposo."""

    bias_dps: float
    time_constant_s: float = GYRO_BIAS_TIME_CONSTANT_S
    raw_filter_time_constant_s: float = GYRO_RAW_FILTER_TIME_CONSTANT_S
    max_rate_dps_per_s: float = GYRO_BIAS_MAX_RATE_DPS_PER_S
    active: bool = False
    samples: int = 0
    last_update: float = None
    raw_z_ema_dps: float = None

    def set_stationary(self, active: bool, now: float) -> None:
        self.active = active
        self.last_update = now if active else None
        self.raw_z_ema_dps = self.bias_dps if active else None

    def update(self, raw_z_dps: float, now: float):
        """Devuelve (bias, delta); congela la estimación fuera de reposo."""
        if not self.active:
            return self.bias_dps, 0.0
        if not math.isfinite(raw_z_dps):
            self.last_update = now
            return self.bias_dps, 0.0
        if self.last_update is None:
            self.last_update = now
            return self.bias_dps, 0.0

        dt = now - self.last_update
        self.last_update = now
        if not math.isfinite(dt) or dt <= 0.0:
            return self.bias_dps, 0.0

        raw_alpha = -math.expm1(-dt / self.raw_filter_time_constant_s)
        self.raw_z_ema_dps += raw_alpha * (
            raw_z_dps - self.raw_z_ema_dps)
        bias_alpha = -math.expm1(-dt / self.time_constant_s)
        requested_delta = bias_alpha * (
            self.raw_z_ema_dps - self.bias_dps)
        maximum_delta = self.max_rate_dps_per_s * dt
        delta = max(-maximum_delta, min(maximum_delta, requested_delta))
        self.bias_dps += delta
        self.samples += 1
        return self.bias_dps, delta


def stationary_evidence_is_valid(
        linear_x: float, angular_z: float, odom_age_s: float,
        command_active: bool) -> bool:
    """Evalúa reposo solo con encoders/odom y comandos, nunca con el gyro."""
    values = (linear_x, angular_z, odom_age_s)
    if not all(math.isfinite(value) for value in values):
        return False
    return (
        odom_age_s <= ODOM_STALE_TIMEOUT_S and
        abs(linear_x) <= ZERO_VELOCITY_TOLERANCE and
        abs(angular_z) <= ZERO_VELOCITY_TOLERANCE and
        not command_active
    )


def gyro_z_for_ekf(corrected_z_rad_s: float,
                   stationary_confirmed: bool) -> float:
    """Anula exclusivamente la tasa publicada durante reposo confirmado."""
    return 0.0 if stationary_confirmed else corrected_z_rad_s


class IMUNode(Node):
    def __init__(self):
        super().__init__('lsm303_l3gd20_imu')
        if SMBus is None:
            raise RuntimeError('smbus2 no disponible. Instala python3-smbus2')

        # Declarar todos los parámetros para que ROS2 los reconozca desde el archivo YAML
        self.bus_num = self.declare_parameter('bus', 1).get_parameter_value().integer_value
        self.addr_acc = self.declare_parameter('addr_acc', ADDR_ACC).get_parameter_value().integer_value
        self.addr_mag = self.declare_parameter('addr_mag', ADDR_MAG).get_parameter_value().integer_value
        self.addr_gyr = self.declare_parameter('addr_gyr', ADDR_GYR).get_parameter_value().integer_value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value
        self.rate_hz = self.declare_parameter('rate', 100.0).get_parameter_value().double_value

        self.bias = BiasScale(
            ax=self.declare_parameter('accel_bias_x', 0.0).get_parameter_value().double_value,
            ay=self.declare_parameter('accel_bias_y', 0.0).get_parameter_value().double_value,
            az=self.declare_parameter('accel_bias_z', 0.0).get_parameter_value().double_value,
            gx=self.declare_parameter('gyro_bias_x', 0.0).get_parameter_value().double_value,
            gy=self.declare_parameter('gyro_bias_y', 0.0).get_parameter_value().double_value,
            gz=self.declare_parameter('gyro_bias_z', 0.0).get_parameter_value().double_value,
            mx_bias=self.declare_parameter('mag_bias_x', 0.0).get_parameter_value().double_value,
            my_bias=self.declare_parameter('mag_bias_y', 0.0).get_parameter_value().double_value,
            mz_bias=self.declare_parameter('mag_bias_z', 0.0).get_parameter_value().double_value,
            mx_scale=self.declare_parameter('mag_scale_x', 1.0).get_parameter_value().double_value,
            my_scale=self.declare_parameter('mag_scale_y', 1.0).get_parameter_value().double_value,
            mz_scale=self.declare_parameter('mag_scale_z', 1.0).get_parameter_value().double_value,
        )

        self.get_logger().info(
            f"Bias gyro (dps): x={self.bias.gx} y={self.bias.gy} z={self.bias.gz}")

        self.auto_bias = self.declare_parameter(
            'gyro_auto_bias_on_start', True).get_parameter_value().bool_value

        self.bus = SMBus(self.bus_num)
        self._init_accel()
        self._init_mag()
        self._init_gyro()

        if self.auto_bias:
            self._auto_calibrate_gyro()

        self.initial_gyro_z_bias = self.bias.gz
        self.gyro_z_bias_adapter = StationaryGyroBiasAdapter(self.bias.gz)
        self.stationary = False
        self.stationary_candidate_since = None
        self.stationary_entries = 0
        self.stationary_exits = 0
        self.last_odom_monotonic = None
        self.odom_linear_x = float('nan')
        self.odom_angular_z = float('nan')
        self.last_nonzero_command = {
            'cmd_vel_safe': float('-inf'),
            'cmd_vel_nav': float('-inf'),
        }
        self.last_bias_log = time.monotonic()

        self.get_logger().info(
            f'Bias gyro_z inicial adaptativo: {self.bias.gz:.6f} dps')

        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'mag', 10)
        self.pub_gyro_z_raw = self.create_publisher(
            Float64, 'imu/gyro_z_raw', 10)
        self.pub_gyro_z_bias = self.create_publisher(
            Float64, 'imu/gyro_z_bias', 10)
        self.pub_gyro_z_corrected = self.create_publisher(
            Float64, 'imu/gyro_z_corrected', 10)

        self.create_subscription(Odometry, 'odom', self._odom_callback, 20)
        self.create_subscription(
            Twist, 'cmd_vel_safe',
            lambda msg: self._cmd_vel_callback('cmd_vel_safe', msg), 10)
        self.create_subscription(
            Twist, 'cmd_vel_nav',
            lambda msg: self._cmd_vel_callback('cmd_vel_nav', msg), 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._loop)
        self.get_logger().info('IMU node inicializado')

    # ---------- Init devices ----------
    def _init_accel(self):
        # 0b01100111 = ODR 200 Hz, ejes XYZ on (se lee a 100 Hz: sin duplicados)
        self._write(self.addr_acc, CTRL_REG1_A, 0b01100111)
        self._write(self.addr_acc, CTRL_REG4_A, 0b00000000)

    def _init_mag(self):
        # 0b00011000 = 75 Hz de data rate (antes 15 Hz, se leia repetido)
        self._write(self.addr_mag, CRA_REG_M, 0b00011000)
        self._write(self.addr_mag, CRB_REG_M, 0b00100000)
        self._write(self.addr_mag, MR_REG_M, 0b00000000)

    def _init_gyro(self):
        # 0b01001111 = ODR 190 Hz, ejes XYZ on (se lee a 100 Hz: sin duplicados)
        self._write(self.addr_gyr, CTRL_REG1_G, 0b01001111)
        self._write(self.addr_gyr, CTRL_REG4_G, 0b00000000)

    def _auto_calibrate_gyro(self, duration=2.0):
        # El bias del gyro cambia con la temperatura; el valor del YAML
        # queda desactualizado en horas. Como el robot siempre arranca
        # quieto, se mide aqui y se reemplaza el bias configurado.
        n = 0
        sx = sy = sz = 0.0
        sq = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            b = self._read_block(self.addr_gyr, OUT_X_L_G, 6)
            x = self._twos_comp_16(b[1], b[0]) * 0.00875
            y = self._twos_comp_16(b[3], b[2]) * 0.00875
            z = self._twos_comp_16(b[5], b[4]) * 0.00875
            sx += x; sy += y; sz += z
            sq += z * z
            n += 1
            time.sleep(0.005)
        if n < 50:
            self.get_logger().warning('Auto-bias: pocas muestras; se usa el bias del YAML')
            return
        mz = sz / n
        std_z = math.sqrt(max(sq / n - mz * mz, 0.0))
        if std_z > 0.5:  # dps: hay vibracion/movimiento, no es confiable
            self.get_logger().warning(
                f'Auto-bias: robot en movimiento (desv={std_z:.2f} dps); se usa el bias del YAML')
            return
        old = (self.bias.gx, self.bias.gy, self.bias.gz)
        self.bias.gx = sx / n
        self.bias.gy = sy / n
        self.bias.gz = mz
        self.get_logger().info(
            f'Auto-bias gyro (dps): {old} -> '
            f'({self.bias.gx:.4f}, {self.bias.gy:.4f}, {self.bias.gz:.4f}) con {n} muestras')

    # ---------- I2C helpers ----------
    def _write(self, addr, reg, val):
        self.bus.write_byte_data(addr, reg, val)

    def _read_block(self, addr, reg, length):
        return self.bus.read_i2c_block_data(addr, reg | 0x80, length)

    # ---------- Conversions ----------
    @staticmethod
    def _twos_comp_16(msb, lsb):
        val = (msb << 8) | lsb
        if val & 0x8000:
            val -= 0x10000
        return val

    def read_accel_ms2(self):
        b = self._read_block(self.addr_acc, OUT_X_L_A, 6)
        x = self._twos_comp_16(b[1], b[0]) >> 4
        y = self._twos_comp_16(b[3], b[2]) >> 4
        z = self._twos_comp_16(b[5], b[4]) >> 4
        ax = (x * 0.001 - self.bias.ax) * G_TO_MS2
        ay = (y * 0.001 - self.bias.ay) * G_TO_MS2
        az = (z * 0.001 - self.bias.az) * G_TO_MS2
        return ax, ay, az

    def read_gyro_dps_raw(self):
        b = self._read_block(self.addr_gyr, OUT_X_L_G, 6)
        x = self._twos_comp_16(b[1], b[0])
        y = self._twos_comp_16(b[3], b[2])
        z = self._twos_comp_16(b[5], b[4])
        return x * 0.00875, y * 0.00875, z * 0.00875

    def correct_gyro_rads(self, gx_dps, gy_dps, gz_dps):
        # 250 dps full-scale -> 8.75 mdps/LSB. El bias se resta en dps.
        gx = (gx_dps - self.bias.gx) * DEG_TO_RAD
        gy = (gy_dps - self.bias.gy) * DEG_TO_RAD
        gz = (gz_dps - self.bias.gz) * DEG_TO_RAD
        return gx, gy, gz

    # ---------- Reposo físico y bias adaptativo de gyro_z ----------
    def _odom_callback(self, msg: Odometry):
        now = time.monotonic()
        self.last_odom_monotonic = now
        self.odom_linear_x = float(msg.twist.twist.linear.x)
        self.odom_angular_z = float(msg.twist.twist.angular.z)
        if (
                abs(self.odom_linear_x) > ZERO_VELOCITY_TOLERANCE or
                abs(self.odom_angular_z) > ZERO_VELOCITY_TOLERANCE):
            self._leave_stationary('movimiento de encoders', now)

    def _cmd_vel_callback(self, source: str, msg: Twist):
        now = time.monotonic()
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.last_nonzero_command[source] = now
            self._leave_stationary(f'{source} no finito', now)
            return
        if (
                abs(linear_x) > ZERO_VELOCITY_TOLERANCE or
                abs(angular_z) > ZERO_VELOCITY_TOLERANCE):
            self.last_nonzero_command[source] = now
            self._leave_stationary(f'{source} activo', now)

    def _command_is_active(self, now: float) -> bool:
        return any(
            now - last_time <= COMMAND_ACTIVE_TIMEOUT_S
            for last_time in self.last_nonzero_command.values())

    def _stationary_evidence(self, now: float) -> bool:
        if self.last_odom_monotonic is None:
            return False
        return stationary_evidence_is_valid(
            self.odom_linear_x,
            self.odom_angular_z,
            now - self.last_odom_monotonic,
            self._command_is_active(now),
        )

    def _leave_stationary(self, reason: str, now: float) -> None:
        self.stationary_candidate_since = None
        if not self.stationary:
            return
        self.stationary = False
        self.stationary_exits += 1
        self.gyro_z_bias_adapter.set_stationary(False, now)
        self.get_logger().info(
            f'REPOSO_SALIÓ: {reason}; bias_z={self.bias.gz:.6f} dps; '
            f'muestras={self.gyro_z_bias_adapter.samples}')

    def _update_stationary_state(self, now: float) -> None:
        if not self._stationary_evidence(now):
            self._leave_stationary('evidencia de reposo inválida', now)
            return

        if self.stationary:
            return
        if self.stationary_candidate_since is None:
            self.stationary_candidate_since = now
            return
        if now - self.stationary_candidate_since < STATIONARY_CONFIRMATION_S:
            return

        self.stationary = True
        self.stationary_entries += 1
        self.gyro_z_bias_adapter.set_stationary(True, now)
        self.last_bias_log = now
        self.get_logger().info(
            f'REPOSO_ENTRÓ: encoders/odom en cero durante '
            f'{STATIONARY_CONFIRMATION_S:.1f} s, sin comando activo; '
            f'bias_z={self.bias.gz:.6f} dps')

    def _adapt_gyro_z_bias(self, raw_z_dps: float, now: float) -> None:
        new_bias, _ = self.gyro_z_bias_adapter.update(raw_z_dps, now)
        self.bias.gz = new_bias
        if self.stationary and now - self.last_bias_log >= BIAS_LOG_INTERVAL_S:
            self.get_logger().info(
                f'Bias gyro_z actualizado: {self.bias.gz:.6f} dps; '
                f'muestras={self.gyro_z_bias_adapter.samples}')
            self.last_bias_log = now

    def log_bias_summary(self):
        self.get_logger().info(
            f'Bias gyro_z final: {self.bias.gz:.6f} dps '
            f'(inicial={self.initial_gyro_z_bias:.6f}); '
            f'muestras={self.gyro_z_bias_adapter.samples}; '
            f'REPOSO_ENTRÓ={self.stationary_entries}; '
            f'REPOSO_SALIÓ={self.stationary_exits}')

    def read_mag_uT(self):
        b = self.bus.read_i2c_block_data(self.addr_mag, OUT_X_H_M, 6)
        x = self._twos_comp_16(b[0], b[1])
        z = self._twos_comp_16(b[2], b[3])
        y = self._twos_comp_16(b[4], b[5])
        mx = ((x / 1100.0) * 100.0 - self.bias.mx_bias) * self.bias.mx_scale
        my = ((y / 1100.0) * 100.0 - self.bias.my_bias) * self.bias.my_scale
        mz = ((z / 980.0)  * 100.0 - self.bias.mz_bias) * self.bias.mz_scale
        return mx, my, mz

    # ---------- Main loop ----------
    def _loop(self):
        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = self.frame_id

        try:
            ax, ay, az = self.read_accel_ms2()
            gx_raw_dps, gy_raw_dps, gz_raw_dps = self.read_gyro_dps_raw()
        except Exception as e:
            self.get_logger().error(f'Error leyendo accel/gyro: {e}')
            return

        monotonic_now = time.monotonic()
        self._update_stationary_state(monotonic_now)
        if math.isfinite(gz_raw_dps):
            self._adapt_gyro_z_bias(gz_raw_dps, monotonic_now)
        gx, gy, gz_corrected = self.correct_gyro_rads(
            gx_raw_dps, gy_raw_dps, gz_raw_dps)
        gz = gyro_z_for_ekf(gz_corrected, self.stationary)

        msg = Imu()
        msg.header = header
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration_covariance = [0.04,0,0, 0,0.04,0, 0,0,0.04]
        msg.angular_velocity_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]
        msg.orientation_covariance[0] = -1.0
        self.pub_imu.publish(msg)

        raw_z_msg = Float64()
        raw_z_msg.data = gz_raw_dps * DEG_TO_RAD
        self.pub_gyro_z_raw.publish(raw_z_msg)
        bias_z_msg = Float64()
        bias_z_msg.data = self.bias.gz * DEG_TO_RAD
        self.pub_gyro_z_bias.publish(bias_z_msg)
        corrected_z_msg = Float64()
        corrected_z_msg.data = gz_corrected
        self.pub_gyro_z_corrected.publish(corrected_z_msg)

        try:
            mx, my, mz = self.read_mag_uT()
            m = MagneticField()
            m.header = header
            m.magnetic_field.x = mx * 1e-6
            m.magnetic_field.y = my * 1e-6
            m.magnetic_field.z = mz * 1e-6
            m.magnetic_field_covariance = [1e-6,0,0, 0,1e-6,0, 0,0,1e-6]
            self.pub_mag.publish(m)
        except Exception as e:
            self.get_logger().warn(f'Error leyendo magnetómetro: {e}')


def main():
    rclpy.init()
    node = None
    try:
        node = IMUNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        if node is not None:
            node.log_bias_summary()
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
