import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
class ImuCovFix(Node):
    def __init__(self):
        super().__init__('imu_cov_fix')
        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/imu/data_fixed', 10)
        self.orient = [0.02, 0.02, 0.01]  # rad^2 (roll,pitch,yaw)
        self.gyro   = [0.01, 0.01, 0.01]  # (rad/s)^2
        self.acc    = [0.04, 0.04, 0.04]  # (m/s^2)^2
    def cb(self, msg):
        m = Imu()
        m.header = msg.header
        m.orientation = msg.orientation
        m.angular_velocity = msg.angular_velocity
        m.linear_acceleration = msg.linear_acceleration
        oc=[0.0]*9; oc[0]=self.orient[0]; oc[4]=self.orient[1]; oc[8]=self.orient[2]
        gc=[0.0]*9; gc[0]=self.gyro[0];   gc[4]=self.gyro[1];   gc[8]=self.gyro[2]
        ac=[0.0]*9; ac[0]=self.acc[0];    ac[4]=self.acc[1];    ac[8]=self.acc[2]
        m.orientation_covariance=oc
        m.angular_velocity_covariance=gc
        m.linear_acceleration_covariance=ac
        self.pub.publish(m)
def main():
    rclpy.init(); node=ImuCovFix(); rclpy.spin(node); rclpy.shutdown()
if __name__ == '__main__': main()
