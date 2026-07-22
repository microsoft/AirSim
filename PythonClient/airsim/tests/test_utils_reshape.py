"""Offline tests for list_to_2d_float_array guards."""
import importlib.util
import os
import sys
import types as pytypes
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_UTILS = os.path.join(os.path.dirname(_HERE), "utils.py")


def _load_utils():
    pkg = pytypes.ModuleType("airsim")
    pkg.__path__ = [os.path.dirname(_HERE)]
    sys.modules["airsim"] = pkg
    stub = pytypes.ModuleType("airsim.types")
    sys.modules["airsim.types"] = stub
    uspec = importlib.util.spec_from_file_location("airsim.utils", _UTILS)
    umod = importlib.util.module_from_spec(uspec)
    sys.modules["airsim.utils"] = umod
    uspec.loader.exec_module(umod)
    return umod


_u = _load_utils()


class TestListTo2d(unittest.TestCase):
    def test_happy_path(self):
        out = _u.list_to_2d_float_array([1, 2, 3, 4], 2, 2)
        self.assertEqual(out.shape, (2, 2))
        self.assertEqual(out.dtype, np.float32)
        self.assertAlmostEqual(float(out[0, 0]), 1.0)

    def test_empty_zero_by_zero(self):
        out = _u.list_to_2d_float_array([], 0, 0)
        self.assertEqual(out.shape, (0, 0))

    def test_length_mismatch(self):
        with self.assertRaises(ValueError) as ctx:
            _u.list_to_2d_float_array([1, 2, 3], 2, 2)
        self.assertIn("flat length", str(ctx.exception))

    def test_negative_dim(self):
        with self.assertRaises(ValueError):
            _u.list_to_2d_float_array([], -1, 1)


if __name__ == "__main__":
    unittest.main()
