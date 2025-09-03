#!/usr/bin/env python3
# Wrapper para lanzar el calibrador desde ros2 run
from lsm303_l3gd20_imu.calib_utils import mag_cal_cli

if __name__ == "__main__":
    mag_cal_cli()
