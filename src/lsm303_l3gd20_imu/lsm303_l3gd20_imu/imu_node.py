#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

try:
    from smbus2 import SMBus
except Exception:
    SMBus = None

# ================= I2C addresses =================
ADDR_ACC = 0x19  # LSM303DLHC accelerometer
ADDR_MAG = 0x1E  # LSM303DLHC magnetometer
ADDR_GYR = 0x6B  # L3GD20 gyro (0x6A if SA0=GND)

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

        # Imprime el valor del bias que realmente se cargó para verificar
        self.get_logger().info(f"========== VALOR DE GYRO_BIAS_Z CARGADO: {self.bias.gz} ==========")

        self.bus = SMBus(self.bus_num)
        self._init_accel()
        self._init_mag()
        self._init_gyro()

        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'mag', 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._loop)
        self.get_logger().info('IMU node inicializado')

    # ---------- Init devices ----------
    def _init_accel(self):
        self._write(self.addr_acc, CTRL_REG1_A, 0b01010111)
        self._write(self.addr_acc, CTRL_REG4_A, 0b00000000)

    def _init_mag(self):
        self._write(self.addr_mag, CRA_REG_M, 0b00010000)
        self._write(self.addr_mag, CRB_REG_M, 0b00100000)
        self._write(self.addr_mag, MR_REG_M, 0b00000000)

    def _init_gyro(self):
        self._write(self.addr_gyr, CTRL_REG1_G, 0b00001111)
        self._write(self.addr_gyr, CTRL_REG4_G, 0b00000000)

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

    def read_gyro_rads(self):
        b = self._read_block(self.addr_gyr, OUT_X_L_G, 6)
        x = self._twos_comp_16(b[1], b[0])
        y = self._twos_comp_16(b[3], b[2])
        z = self._twos_comp_16(b[5], b[4])
        # 250 dps full‑scale → 8.75 mdps/LSB
        # NOTA: La corrección del bias debe hacerse en las mismas unidades que la lectura
        gx = (x * 0.00875 - self.bias.gx) * DEG_TO_RAD
        gy = (y * 0.00875 - self.bias.gy) * DEG_TO_RAD
        gz = (z * 0.00875 - self.bias.gz) * DEG_TO_RAD
        return gx, gy, gz

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
            gx, gy, gz = self.read_gyro_rads()
        except Exception as e:
            self.get_logger().error(f'Error leyendo accel/gyro: {e}')
            return

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
    node = IMUNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
