import unittest

import numpy as np

from airsim.types import CollisionInfo


class TestCollisionInfoDefaults(unittest.TestCase):
    def test_timestamp_uint64_default(self):
        c = CollisionInfo()
        self.assertTrue(isinstance(c.time_stamp, (int, np.integer)))
        self.assertEqual(int(c.time_stamp), 0)
        self.assertFalse(c.has_collided)


if __name__ == "__main__":
    unittest.main()
