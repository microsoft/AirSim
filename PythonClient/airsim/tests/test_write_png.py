import os
import tempfile
import unittest

import numpy as np

from airsim.utils import write_png


class WritePngTests(unittest.TestCase):
    def test_rejects_empty_filename(self):
        with self.assertRaises(ValueError):
            write_png("", np.zeros((2, 2), dtype=np.uint8))

    def test_rejects_none_image(self):
        with self.assertRaises(ValueError):
            write_png("x.png", None)

    def test_rejects_bad_ndim(self):
        with self.assertRaises(ValueError):
            write_png("x.png", np.zeros((2,), dtype=np.uint8))

    def test_writes_when_cv2_available(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("opencv not installed")
        img = np.zeros((4, 4, 3), dtype=np.uint8)
        img[0, 0] = (0, 0, 255)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "t.png")
            write_png(path, img)
            self.assertTrue(os.path.isfile(path))
            self.assertGreater(os.path.getsize(path), 0)


if __name__ == "__main__":
    unittest.main()
