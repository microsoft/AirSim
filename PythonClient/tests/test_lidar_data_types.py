"""Offline checks for LidarData list field defaults (no AirSim sim required)."""
from __future__ import annotations

import importlib.util
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


class TestLidarDataTypes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.types = _load_types()

    def test_defaults_are_empty_lists(self):
        data = self.types.LidarData()
        self.assertIsInstance(data.point_cloud, list)
        self.assertIsInstance(data.segmentation, list)
        self.assertEqual(len(data.point_cloud), 0)
        self.assertEqual(len(data.segmentation), 0)
        # Historical bug: scalar defaults break sample len()/slice code paths.
        self.assertFalse(isinstance(data.point_cloud, float))
        self.assertFalse(isinstance(data.segmentation, int))

    def test_assign_point_cloud_and_segmentation(self):
        data = self.types.LidarData()
        data.point_cloud = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        data.segmentation = [10, 20]
        self.assertEqual(len(data.point_cloud), 6)
        self.assertEqual(data.point_cloud[0:3], [1.0, 2.0, 3.0])
        self.assertEqual(data.segmentation, [10, 20])
        # Triplet walk matches public sample clients (drone_lidar / point_cloud).
        triplets = [
            data.point_cloud[i : i + 3]
            for i in range(0, len(data.point_cloud), 3)
        ]
        self.assertEqual(triplets, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])


if __name__ == "__main__":
    unittest.main()
