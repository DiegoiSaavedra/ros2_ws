import math
import unittest

from robot_base.safety import (
    DEFAULT_CMD_VEL_TIMEOUT,
    DEFAULT_DRIVE_DEADBAND,
    DEFAULT_MAX_WHEEL_SPEED,
    DEFAULT_MIN_WHEEL_SPEED,
    DEFAULT_PIVOT_DEADBAND,
    MAX_PWM,
    WheelSpeedPI,
    command_timed_out,
    parse_encoder_line,
    positive_timeout,
    require_finite,
    twist_to_wheel_velocities,
    velocity_to_pwm,
)


class TestBaseDriverSafety(unittest.TestCase):
    def test_require_finite_rejects_nan_and_infinity(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    require_finite(value, 'test')

    def test_zero_twist_always_produces_zero_pwm(self):
        left, right = twist_to_wheel_velocities(
            0.0, 0.0, 0.145, DEFAULT_MAX_WHEEL_SPEED)
        for deadband in (DEFAULT_DRIVE_DEADBAND, DEFAULT_PIVOT_DEADBAND):
            with self.subTest(deadband=deadband):
                self.assertEqual(
                    velocity_to_pwm(left, deadband, DEFAULT_MIN_WHEEL_SPEED,
                                    DEFAULT_MAX_WHEEL_SPEED), 0)
                self.assertEqual(
                    velocity_to_pwm(right, deadband, DEFAULT_MIN_WHEEL_SPEED,
                                    DEFAULT_MAX_WHEEL_SPEED), 0)

    def test_any_nonzero_command_reaches_the_breakaway_threshold(self):
        # La regresion que dejaba a Nav2 zumbando: con el mapeo viejo un
        # comando chico daba PWM 95 y la base no arrancaba.
        for deadband in (DEFAULT_DRIVE_DEADBAND, DEFAULT_PIVOT_DEADBAND):
            for velocity in (0.006, 0.05, 0.18):
                with self.subTest(deadband=deadband, velocity=velocity):
                    pwm = velocity_to_pwm(
                        velocity, deadband, DEFAULT_MIN_WHEEL_SPEED,
                        DEFAULT_MAX_WHEEL_SPEED)
                    self.assertGreaterEqual(pwm, int(deadband))
                    self.assertLessEqual(pwm, MAX_PWM)

    def test_pwm_saturates_and_keeps_sign(self):
        forward = velocity_to_pwm(
            10.0, DEFAULT_DRIVE_DEADBAND, DEFAULT_MIN_WHEEL_SPEED,
            DEFAULT_MAX_WHEEL_SPEED)
        backward = velocity_to_pwm(
            -10.0, DEFAULT_DRIVE_DEADBAND, DEFAULT_MIN_WHEEL_SPEED,
            DEFAULT_MAX_WHEEL_SPEED)
        self.assertEqual(forward, MAX_PWM)
        self.assertEqual(backward, -MAX_PWM)

    def test_twist_conversion_is_differential(self):
        left, right = twist_to_wheel_velocities(0.1, 0.5, 0.2, 1.0)
        self.assertAlmostEqual(left, 0.05)
        self.assertAlmostEqual(right, 0.15)

    def test_wheel_saturation_preserves_ratio(self):
        left, right = twist_to_wheel_velocities(0.3, 1.0, 0.2, 0.2)
        self.assertAlmostEqual(right, 0.2)
        self.assertAlmostEqual(left / right, 0.5)

    def test_calculated_values_must_be_finite(self):
        with self.assertRaises(ValueError):
            twist_to_wheel_velocities(math.nan, 0.0, 0.145, 0.2)
        with self.assertRaises(ValueError):
            velocity_to_pwm(math.inf, DEFAULT_DRIVE_DEADBAND,
                            DEFAULT_MIN_WHEEL_SPEED, DEFAULT_MAX_WHEEL_SPEED)

    def test_timeout_uses_safe_boundary(self):
        self.assertFalse(command_timed_out(10.24, 10.0, 0.25))
        self.assertTrue(command_timed_out(10.25, 10.0, 0.25))
        self.assertTrue(command_timed_out(9.0, 10.0, 0.25))
        self.assertTrue(command_timed_out(math.nan, 10.0, 0.25))

    def test_invalid_timeout_falls_back_to_safe_default(self):
        for value in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(value=value):
                self.assertEqual(
                    positive_timeout(value, DEFAULT_CMD_VEL_TIMEOUT),
                    DEFAULT_CMD_VEL_TIMEOUT,
                )

    def test_encoder_packet_requires_exact_valid_fields(self):
        self.assertEqual(parse_encoder_line('123,-456'), (123, -456))
        for line in ('123', '1,2,3', 'left,right', ''):
            with self.subTest(line=line):
                with self.assertRaises(ValueError):
                    parse_encoder_line(line)

    def test_wheel_pi_corrects_each_wheel_independently(self):
        left = WheelSpeedPI(800.0, 250.0, 35.0, 0.10)
        right = WheelSpeedPI(800.0, 250.0, 35.0, 0.10)
        left_correction = left.update(0.11, 0.12, 10.0)
        right_correction = right.update(0.11, 0.10, 10.0)
        self.assertLess(left_correction, 0.0)
        self.assertGreater(right_correction, 0.0)

    def test_wheel_pi_anti_windup_and_reset(self):
        pi = WheelSpeedPI(800.0, 250.0, 35.0, 0.10)
        for index in range(100):
            correction = pi.update(0.2, 0.0, 10.0 + index * 0.1)
            self.assertLessEqual(abs(correction), 35.0)
        self.assertEqual(pi.integral, 0.0)
        pi.reset()
        self.assertEqual(pi.integral, 0.0)
        self.assertIsNone(pi.last_time)


if __name__ == '__main__':
    unittest.main()
