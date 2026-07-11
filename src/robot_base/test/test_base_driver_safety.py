import math
import unittest

from robot_base.safety import (
    DEFAULT_CMD_VEL_TIMEOUT,
    command_timed_out,
    parse_encoder_line,
    positive_timeout,
    require_finite,
    twist_to_wheel_velocities,
    velocity_to_pwm,
    wheel_speed_limit,
)


class TestBaseDriverSafety(unittest.TestCase):
    def test_require_finite_rejects_nan_and_infinity(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    require_finite(value, 'test')

    def test_zero_twist_always_produces_zero_pwm(self):
        maximum = wheel_speed_limit(85.0)
        left, right = twist_to_wheel_velocities(0.0, 0.0, 0.145, maximum)
        self.assertEqual(velocity_to_pwm(left, 85.0), 0)
        self.assertEqual(velocity_to_pwm(right, 85.0), 0)

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
            velocity_to_pwm(math.inf, 55.0)

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


if __name__ == '__main__':
    unittest.main()
