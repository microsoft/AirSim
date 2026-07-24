import unittest

from airsim.types import ImageResponse


class TestImageResponseDefaults(unittest.TestCase):
    def test_default_buffers_are_empty_iterables(self):
        r = ImageResponse()
        self.assertIsInstance(r.image_data_uint8, (bytes, bytearray))
        self.assertEqual(len(r.image_data_uint8), 0)
        self.assertIsInstance(r.image_data_float, list)
        self.assertEqual(len(r.image_data_float), 0)
        # Sample code paths
        self.assertEqual(len(r.image_data_uint8), 0)
        import numpy as np
        arr = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
        self.assertEqual(arr.size, 0)


if __name__ == "__main__":
    unittest.main()
