"""Offline Vector3r/Quaternionr numeric-operator checks (no sim)."""
from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path

import numpy as np


def _load_types():
    if "msgpackrpc" not in sys.modules:
        sys.modules["msgpackrpc"] = types.ModuleType("msgpackrpc")
    root = Path(__file__).resolve().parents[1] / "airsim" / "types.py"
    spec = importlib.util.spec_from_file_location("airsim_types_ops_under_test", root)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class TestVectorQuatNumericOps(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.t = _load_types()

    def test_vector_scale_python_and_numpy_scalars(self):
        v = self.t.Vector3r(2.0, 4.0, 6.0)
        self.assertEqual(tuple(v * 2), (4.0, 8.0, 12.0))
        self.assertEqual(tuple(v / 2), (1.0, 2.0, 3.0))
        self.assertEqual(tuple(v * np.float32(0.5)), (1.0, 2.0, 3.0))
        self.assertEqual(tuple(v / np.int64(2)), (1.0, 2.0, 3.0))

    def test_bool_not_numeric_scalar(self):
        v = self.t.Vector3r(1.0, 2.0, 3.0)
        with self.assertRaises(TypeError):
            _ = v * True
        with self.assertRaises(TypeError):
            _ = v / False
        q = self.t.Quaternionr(0, 0, 0, 1)
        with self.assertRaises(TypeError):
            _ = q / True

    def test_quaternion_sub_and_scale(self):
        a = self.t.Quaternionr(1, 2, 3, 4)
        b = self.t.Quaternionr(1, 1, 1, 1)
        d = a - b
        self.assertEqual(tuple(d), (0.0, 1.0, 2.0, 3.0))
        s = a / 2.0
        self.assertEqual(tuple(s), (0.5, 1.0, 1.5, 2.0))

    def test_no_sctypes_dependency(self):
        src = (Path(__file__).resolve().parents[1] / "airsim" / "types.py").read_text()
        self.assertNotIn("np.sctypes", src)
        self.assertTrue(hasattr(self.t, "_is_numeric_scalar"))
        self.assertFalse(self.t._is_numeric_scalar(True))
        self.assertTrue(self.t._is_numeric_scalar(3))
        self.assertTrue(self.t._is_numeric_scalar(np.float64(1.25)))


if __name__ == "__main__":
    unittest.main()
