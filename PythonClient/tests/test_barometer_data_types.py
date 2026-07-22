"""Offline checks for BarometerData scalars (no AirSim sim required)."""
from __future__ import annotations

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path


def _load_types():
    # Stub msgpackrpc so types.py imports cleanly without the RPC package.
    if "msgpackrpc" not in sys.modules:
        sys.modules["msgpackrpc"] = types.ModuleType("msgpackrpc")
    root = Path(__file__).resolve().parents[1] / "airsim" / "types.py"
    spec = importlib.util.spec_from_file_location("airsim_types_under_test", root)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class TestBarometerDataTypes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.types = _load_types()

    def test_defaults_are_real_scalars(self):
        b = self.types.BarometerData()
        self.assertIsInstance(b.altitude, float)
        self.assertIsInstance(b.pressure, float)
        self.assertIsInstance(b.qnh, float)
        self.assertTrue(math.isfinite(b.altitude))
        self.assertTrue(math.isfinite(b.pressure))
        self.assertTrue(math.isfinite(b.qnh))
        # Must not be pose-like defaults (historical bug).
        self.assertFalse(isinstance(b.altitude, self.types.Quaternionr))
        self.assertFalse(isinstance(b.pressure, self.types.Vector3r))
        self.assertFalse(isinstance(b.qnh, self.types.Vector3r))

    def test_assign_numeric_values(self):
        b = self.types.BarometerData()
        b.altitude = 120.5
        b.pressure = 101325.0
        b.qnh = 1013.25
        self.assertEqual(b.altitude, 120.5)
        self.assertEqual(b.pressure, 101325.0)
        self.assertAlmostEqual(b.qnh, 1013.25)


if __name__ == "__main__":
    unittest.main()
