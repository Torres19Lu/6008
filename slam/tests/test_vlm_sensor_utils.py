#!/usr/bin/env python3

import unittest

import numpy as np

from vlm_sensor_utils import depth_image_to_meters


class DepthImageConversionTest(unittest.TestCase):
    def test_gazebo_float_depth_stays_in_metres(self):
        source = np.asarray([[0.5, 2.75]], dtype=np.float32)
        converted = depth_image_to_meters(source, "32FC1")
        np.testing.assert_allclose(converted, source)
        self.assertEqual(converted.dtype, np.float32)

    def test_realsense_uint16_millimetres_become_metres(self):
        source = np.asarray([[500, 2750]], dtype=np.uint16)
        converted = depth_image_to_meters(source, "16UC1")
        np.testing.assert_allclose(converted, [[0.5, 2.75]])
        self.assertEqual(converted.dtype, np.float32)

    def test_unknown_encoding_is_rejected(self):
        with self.assertRaises(ValueError):
            depth_image_to_meters(np.zeros((1, 1)), "8UC1")


if __name__ == "__main__":
    unittest.main()
