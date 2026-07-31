# pyright: reportMissingImports=false

import unittest

from control.motor_calibration import center_motor_range


class CenteredMotorCalibrationTest(unittest.TestCase):
    def test_endpoints_define_midpoint_without_visual_scale(self) -> None:
        result = center_motor_range(922, 3347, minimum_span_counts=500)

        self.assertEqual(result.endpoint_min_raw, 922)
        self.assertEqual(result.endpoint_max_raw, 3347)
        self.assertEqual(result.midpoint_raw, 2134)
        self.assertEqual(result.homing_offset, 87)
        self.assertEqual(result.range_min, 835)
        self.assertEqual(result.range_max, 3260)
        self.assertEqual(result.span_counts, 2425)
        self.assertFalse(result.wraps_encoder_zero)
        self.assertAlmostEqual((result.range_min + result.range_max) / 2, 2047.5)

    def test_endpoints_across_encoder_zero_use_long_physical_arc(self) -> None:
        result = center_motor_range(2751, 1372, minimum_span_counts=1800)

        self.assertEqual(result.span_counts, 2717)
        self.assertTrue(result.wraps_encoder_zero)
        self.assertEqual(result.midpoint_raw, 13)
        self.assertEqual(result.homing_offset, -2034)
        self.assertEqual(result.range_min, 689)
        self.assertEqual(result.range_max, 3406)
        self.assertAlmostEqual((result.range_min + result.range_max) / 2, 2047.5)

    def test_endpoint_order_does_not_matter(self) -> None:
        forward = center_motor_range(669, 3237, minimum_span_counts=500)
        reverse = center_motor_range(3237, 669, minimum_span_counts=500)

        self.assertEqual(forward, reverse)

    def test_too_small_span_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "跨度"):
            center_motor_range(1900, 2100, minimum_span_counts=500)

    def test_encoder_values_must_be_12_bit(self) -> None:
        with self.assertRaisesRegex(ValueError, "0..4095"):
            center_motor_range(-1, 3000)
        with self.assertRaisesRegex(ValueError, "0..4095"):
            center_motor_range(1000, 4096)


if __name__ == "__main__":
    unittest.main()
