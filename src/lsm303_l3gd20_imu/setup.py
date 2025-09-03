from setuptools import setup

package_name = 'lsm303_l3gd20_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
        ('share/' + package_name + '/config', ['config/imu_params.yaml', 'config/madgwick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 driver for LSM303DLHC + L3GD20 over I2C',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = lsm303_l3gd20_imu.imu_node:main',
            'mag_calibrator = lsm303_l3gd20_imu.calib_utils:mag_cal_cli',
        ],
    },
)
