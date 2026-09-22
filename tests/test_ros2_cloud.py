import unittest

import numpy as np

from sentinel.integrations.ros2_cloud import transform_xyz


class ROS2CloudTransformTests(unittest.TestCase):
    def test_identity_and_translation(self):
        points = [[1., 2., 3.], [-2., 0., 4.]]
        mapped = transform_xyz(points, [10., -3., 1.], [0., 0., 0., 1.])
        np.testing.assert_allclose(mapped, [[11., -1., 4.], [8., -3., 5.]])

    def test_quarter_turn_about_up_axis(self):
        s = np.sqrt(0.5)
        mapped = transform_xyz([[1., 0., 0.]], [0., 0., 0.], [0., 0., s, s])
        np.testing.assert_allclose(mapped, [[0., 1., 0.]], atol=1e-12)

    def test_invalid_transform_rejected(self):
        with self.assertRaises(ValueError):
            transform_xyz([[1., 2., 3.]], [0., 0., 0.], [0., 0., 0., 0.])
        with self.assertRaises(ValueError):
            transform_xyz([[float("nan"), 2., 3.]], [0., 0., 0.], [0., 0., 0., 1.])


if __name__ == "__main__":
    unittest.main()
