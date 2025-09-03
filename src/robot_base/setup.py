from setuptools import setup
from glob import glob
import os

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': ''},
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',  glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config',  glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/rviz',    glob(os.path.join('rviz',   '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='diego@todo.todo',
    description='Nodo Python que controla la base diferencial y publica odometria y TF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = robot_base.base_driver:main',
            'imu_cov_fix = robot_base.imu_cov_fix:main',
        ],
    },
)
