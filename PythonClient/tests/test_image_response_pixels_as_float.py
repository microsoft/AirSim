"""Offline check: ImageResponse.pixels_as_float is a bool (matches C++ RPC)."""
from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path


def _load_types():
    if "msgpackrpc" not in sys.modules:
        sys.modules["msgpackrpc"] = types.ModuleType("msgpackrpc")
    root = Path(__file__).resolve().parents[1] / "airsim" / "types.py"
    spec = importlib.util.spec_from_file_location("airsim_types_under_test", root)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class TestImageResponsePixelsAsFloat(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.types = _load_types()

    def test_pixels_as_float_is_bool_false(self):
        r = self.types.ImageResponse()
        self.assertIsInstance(r.pixels_as_float, bool)
        self.assertIs(r.pixels_as_float, False)


if __name__ == "__main__":
    unittest.main()
