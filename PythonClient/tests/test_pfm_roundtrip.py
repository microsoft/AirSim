"""Offline PFM read/write roundtrip (no sim, no matplotlib)."""
from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


def _load_pfm():
    root = Path(__file__).resolve().parents[1] / "airsim" / "pfm.py"
    spec = importlib.util.spec_from_file_location("airsim_pfm_under_test", root)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class TestPfmRoundtrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pfm = _load_pfm()

    def test_gray_roundtrip(self):
        img = np.arange(12, dtype=np.float32).reshape(3, 4)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "g.pfm"
            self.pfm.write_pfm(str(path), img, scale=1)
            data, scale = self.pfm.read_pfm(str(path))
        self.assertEqual(data.shape, (3, 4))
        self.assertTrue(np.allclose(data, img))
        self.assertAlmostEqual(scale, 1.0)

    def test_color_roundtrip(self):
        img = np.linspace(0, 1, 24, dtype=np.float32).reshape(2, 4, 3)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "c.pfm"
            self.pfm.write_pfm(str(path), img, scale=1)
            data, scale = self.pfm.read_pfm(str(path))
        self.assertEqual(data.shape, (2, 4, 3))
        self.assertTrue(np.allclose(data, img))

    def test_rejects_non_float32(self):
        img = np.zeros((2, 2), dtype=np.float64)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "bad.pfm"
            with self.assertRaises(Exception):
                self.pfm.write_pfm(str(path), img)


if __name__ == "__main__":
    unittest.main()
