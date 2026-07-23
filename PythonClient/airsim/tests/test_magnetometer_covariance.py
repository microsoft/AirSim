import unittest

import numpy as np

# Import without full client stack when msgpackrpc missing
try:
    from airsim.types import MagnetometerData, Vector3r
except Exception:
    import sys, types
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    if 'msgpackrpc' not in sys.modules:
        sys.modules['msgpackrpc'] = types.ModuleType('msgpackrpc')
    from airsim.types import MagnetometerData, Vector3r


class MagnetometerCovarianceTests(unittest.TestCase):
    def test_default_is_list(self):
        m = MagnetometerData()
        self.assertIsInstance(m.magnetic_field_covariance, list)
        self.assertEqual(m.magnetic_field_covariance, [])

    def test_from_msgpack_list(self):
        payload = {
            'time_stamp': np.uint64(1),
            'magnetic_field_body': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.1},
            'magnetic_field_covariance': [1.0] * 9,
        }
        m = MagnetometerData.from_msgpack(payload)
        self.assertEqual(len(m.magnetic_field_covariance), 9)
        self.assertIsInstance(m.magnetic_field_covariance, list)


if __name__ == '__main__':
    unittest.main()
