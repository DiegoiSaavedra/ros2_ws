#!/usr/bin/env python3
import sys
import time
import math
import json
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except Exception:
    SMBus = None

ADDR_MAG = 0x1E
CRA_REG_M = 0x00
CRB_REG_M = 0x01
MR_REG_M  = 0x02
OUT_X_H_M = 0x03

@dataclass
class MagCalib:
    bias_x: float
    bias_y: float
    bias_z: float
    scale_x: float
    scale_y: float
    scale_z: float


def read_mag_raw(bus, addr=ADDR_MAG):
    b = bus.read_i2c_block_data(addr, OUT_X_H_M, 6)
    def tc(msb, lsb):
        v = (msb<<8)|lsb
        return v-65536 if v&0x8000 else v
    x = tc(b[0], b[1])
    z = tc(b[2], b[3])
    y = tc(b[4], b[5])
    return x, y, z


def simple_minmax_calibration(samples):
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    zs = [s[2] for s in samples]
    bx = (max(xs)+min(xs))/2.0
    by = (max(ys)+min(ys))/2.0
    bz = (max(zs)+min(zs))/2.0
    sx = (max(xs)-min(xs))/2.0
    sy = (max(ys)-min(ys))/2.0
    sz = (max(zs)-min(zs))/2.0
    s_avg = (sx+sy+sz)/3.0
    return MagCalib(
        bias_x=bx, bias_y=by, bias_z=bz,
        scale_x=s_avg/sx if sx else 1.0,
        scale_y=s_avg/sy if sy else 1.0,
        scale_z=s_avg/sz if sz else 1.0,
    )


def mag_cal_cli():
    if SMBus is None:
        print('Instala python3-smbus2', file=sys.stderr)
        return
    bus = SMBus(1)
    # Init MAG
    bus.write_byte_data(ADDR_MAG, CRA_REG_M, 0b00010000)  # 15 Hz
    bus.write_byte_data(ADDR_MAG, CRB_REG_M, 0b00100000)  # 1.3 G
    bus.write_byte_data(ADDR_MAG, MR_REG_M,  0)           # continuous

    print('Gira el sensor en todas las orientaciones durante ~30s...')
    samples = []
    t0 = time.time()
    while time.time() - t0 < 30.0:
        samples.append(read_mag_raw(bus))
        time.sleep(0.02)

    cal = simple_minmax_calibration(samples)
    print('\nResultados (para imu_params.yaml):')
    print(json.dumps({
        'mag_bias_x': cal.bias_x,
        'mag_bias_y': cal.bias_y,
        'mag_bias_z': cal.bias_z,
        'mag_scale_x': cal.scale_x,
        'mag_scale_y': cal.scale_y,
        'mag_scale_z': cal.scale_z,
    }, indent=2))

if __name__ == '__main__':
    mag_cal_cli()
