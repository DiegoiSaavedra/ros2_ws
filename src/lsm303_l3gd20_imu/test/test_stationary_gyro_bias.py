import math

from lsm303_l3gd20_imu.imu_node import (
    GYRO_BIAS_MAX_RATE_DPS_PER_S,
    StationaryGyroBiasAdapter,
    gyro_z_for_ekf,
    stationary_evidence_is_valid,
)


def test_stationary_evidence_requires_fresh_zero_odom_and_no_command():
    assert stationary_evidence_is_valid(0.0, 0.0, 0.05, False)
    assert not stationary_evidence_is_valid(0.01, 0.0, 0.05, False)
    assert not stationary_evidence_is_valid(0.0, 0.01, 0.05, False)
    assert not stationary_evidence_is_valid(0.0, 0.0, 0.30, False)
    assert not stationary_evidence_is_valid(0.0, 0.0, 0.05, True)
    assert not stationary_evidence_is_valid(math.nan, 0.0, 0.05, False)


def test_bias_is_frozen_outside_stationary_state():
    adapter = StationaryGyroBiasAdapter(-0.8)
    bias, delta = adapter.update(-0.7, 10.0)
    assert bias == -0.8
    assert delta == 0.0
    assert adapter.samples == 0

    adapter.set_stationary(False, 1.0)
    bias, delta = adapter.update(-0.7, 2.0)
    assert bias == -0.8
    assert delta == 0.0
    assert adapter.samples == 0


def test_bias_adapts_slowly_and_respects_rate_limit():
    adapter = StationaryGyroBiasAdapter(0.0)
    adapter.set_stationary(True, 0.0)
    bias, delta = adapter.update(100.0, 1.0)
    assert math.isclose(delta, GYRO_BIAS_MAX_RATE_DPS_PER_S)
    assert math.isclose(bias, delta)
    assert adapter.samples == 1


def test_bias_rejects_invalid_samples_and_freezes_after_motion():
    adapter = StationaryGyroBiasAdapter(-0.8)
    adapter.set_stationary(True, 0.0)
    bias, delta = adapter.update(math.nan, 1.0)
    assert bias == -0.8
    assert delta == 0.0
    assert adapter.samples == 0


def test_bias_converges_after_filtering_zero_mean_sensor_noise():
    adapter = StationaryGyroBiasAdapter(-0.785)
    adapter.set_stationary(True, 0.0)
    target = -0.802
    now = 0.0
    for sample in range(60000):
        now += 0.01
        noise = 0.30 if sample % 2 else -0.30
        adapter.update(target + noise, now)

    assert abs(adapter.bias_dps - target) < 0.001


def test_gyro_z_is_zero_only_during_confirmed_stationary_state():
    corrected_z = 0.42
    assert gyro_z_for_ekf(corrected_z, True) == 0.0
    assert gyro_z_for_ekf(corrected_z, False) == corrected_z
    assert gyro_z_for_ekf(-corrected_z, False) == -corrected_z
