"""Vector3r/Quaternionr scalar ops must work under NumPy 2 (no np.sctypes)."""
import importlib.util
import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_TYPES = os.path.join(os.path.dirname(_HERE), "types.py")


def _load_types():
    spec = importlib.util.spec_from_file_location("airsim_types_under_test", _TYPES)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_types = _load_types()
Vector3r = _types.Vector3r
Quaternionr = _types.Quaternionr


class TestNumpy2Scalars(unittest.TestCase):
    def test_vector3_mul_python_float(self):
        v = Vector3r(1.0, 2.0, 3.0) * 2.0
        self.assertEqual((v.x_val, v.y_val, v.z_val), (2.0, 4.0, 6.0))

    def test_vector3_mul_numpy_float32(self):
        v = Vector3r(1.0, -1.0, 0.5) * np.float32(2.0)
        self.assertAlmostEqual(float(v.x_val), 2.0, places=5)
        self.assertAlmostEqual(float(v.y_val), -2.0, places=5)
        self.assertAlmostEqual(float(v.z_val), 1.0, places=5)

    def test_vector3_div_numpy_int64(self):
        v = Vector3r(2.0, 4.0, 6.0) / np.int64(2)
        self.assertEqual(
            (float(v.x_val), float(v.y_val), float(v.z_val)),
            (1.0, 2.0, 3.0),
        )

    def test_vector3_mul_rejects_str(self):
        with self.assertRaises(TypeError):
            _ = Vector3r(1, 2, 3) * "2"

    def test_quaternion_div_float(self):
        q = Quaternionr(2.0, 4.0, 6.0, 8.0) / 2.0
        self.assertEqual(
            (q.x_val, q.y_val, q.z_val, q.w_val),
            (1.0, 2.0, 3.0, 4.0),
        )

    def test_quaternion_div_numpy_uint(self):
        q = Quaternionr(2.0, 4.0, 6.0, 8.0) / np.uint32(2)
        self.assertEqual(
            (float(q.x_val), float(q.y_val), float(q.z_val), float(q.w_val)),
            (1.0, 2.0, 3.0, 4.0),
        )


if __name__ == "__main__":
    unittest.main()
