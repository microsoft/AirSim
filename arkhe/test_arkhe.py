import unittest
import numpy as np
from arkhe.arkhe_types import CIEF, HexVoxel
from arkhe.hsi import HSI
from arkhe.fusion import FusionEngine
from arkhe.simulation import MorphogeneticSimulation

class TestArkhe(unittest.TestCase):
    def test_cief_init(self):
        genome = CIEF(c=1.0, i=0.5, e=0.2, f=0.1)
        self.assertEqual(genome.c, 1.0)
        self.assertEqual(genome.f, 0.1)
        arr = genome.to_array()
        self.assertEqual(arr.shape, (4,))

    def test_hsi_coordinates(self):
        hsi = HSI(size=1.0)
        # Cartesian (0,0,0) should be hex (0,0,0,0)
        coords = hsi.cartesian_to_hex(0, 0, 0)
        self.assertEqual(coords, (0, 0, 0, 0))

        # Test back and forth
        x, y, z = 10.5, -5.2, 2.0
        coords = hsi.cartesian_to_hex(x, y, z)
        x2, y2, z2 = hsi.hex_to_cartesian(*coords)
        # Allow some margin due to discretization
        self.assertLess(abs(x - x2), 2.0)
        self.assertLess(abs(y - y2), 2.0)

    def test_fusion_lidar(self):
        hsi = HSI(size=1.0)
        fusion = FusionEngine(hsi)
        points = np.array([[0, 0, 0], [1, 1, 0]])
        fusion.fuse_lidar(points)
        self.assertIn((0, 0, 0, 0), hsi.voxels)
        self.assertGreater(hsi.voxels[(0, 0, 0, 0)].genome.c, 0)

    def test_simulation_step(self):
        hsi = HSI(size=1.0)
        sim = MorphogeneticSimulation(hsi)
        # Add a voxel with some B state
        voxel = hsi.get_voxel((0, 0, 0, 0))
        voxel.rd_state = (0.5, 0.5)

        sim.step(dt=0.1)
        # Check that state changed
        self.assertNotEqual(voxel.rd_state, (0.5, 0.5))

    def test_coherence_phi(self):
        hsi = HSI(size=1.0)
        fusion = FusionEngine(hsi)
        voxel = hsi.get_voxel((0, 0, 0, 0))
        # Pure state should have Phi_data = 1.0
        voxel.genome = CIEF(c=1.0, i=0.0, e=0.0, f=0.0)
        fusion.update_voxel_coherence()
        self.assertAlmostEqual(voxel.phi_data, 1.0, places=5)
        # Total phi should be average of data and field (0 initially)
        self.assertAlmostEqual(voxel.phi, 0.5, places=5)

if __name__ == "__main__":
    unittest.main()
