import unittest
import numpy as np
from arkhe.arkhe_types import CIEF, HexVoxel
from arkhe.hsi import HSI
from arkhe.fusion import FusionEngine
from arkhe.simulation import MorphogeneticSimulation
from arkhe.consensus import QuantumPaxos
from arkhe.telemetry import ArkheTelemetry

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

    def test_quantumpaxos_sign(self):
        paxos = QuantumPaxos(node_id="test_node")
        report = {"data": "test"}
        signature = paxos.sign_report(report)
        self.assertIn("quantum_signature", report)
        self.assertEqual(report["quantum_signature"], signature)

    def test_telemetry_dispatch(self):
        # Test basic dispatch without needing full redis/websockets
        telemetry = ArkheTelemetry()
        report = {"event": "test"}
        # Should not crash even if redis is missing
        telemetry.dispatch_channel_a(report)

    def test_hebbian_trace(self):
        hsi = HSI(size=1.0)
        sim = MorphogeneticSimulation(hsi)
        v1 = hsi.get_voxel((0,0,0,0))
        v2 = hsi.get_voxel((1,-1,0,0))

        sim.on_hex_boundary_crossed(v1, v2)

        self.assertEqual(len(v1.hebbian_trace), 1)
        self.assertEqual(v1.hebbian_trace[0][1], "entity_exited")
        self.assertEqual(len(v2.hebbian_trace), 1)
        self.assertEqual(v2.hebbian_trace[0][1], "entity_entered")

    def test_immune_system(self):
        hsi = HSI(size=1.0)
        sim = MorphogeneticSimulation(hsi)
        v = hsi.get_voxel((0,0,0,0))
        # Create a massive divergence by changing genome (affects intention F)
        v.genome.i = 1.0
        v.genome.e = 1.0
        # sim.step will calculate intention_derivative based on (i+e)/2

        sim.step(dt=1.0)
        # Should be isolated due to high dF (1.0)
        self.assertTrue(v.is_isolated)

    def test_rehabilitation(self):
        hsi = HSI(size=1.0)
        sim = MorphogeneticSimulation(hsi)
        v = hsi.get_voxel((0,0,0,0))
        v.is_isolated = True
        v.genome.i = 0.0
        v.genome.e = 0.0
        v.intention_amplitude = 0.0 # Stable state

        # Needs to reach REHAB_GOAL (0.74) with score += dt * 0.1
        for _ in range(8):
            sim.step(dt=1.0)

        # Should be rehabilitated after ~8 steps
        self.assertFalse(v.is_isolated)

    def test_biogenesis_engine(self):
        from arkhe.biogenesis import BioGenesisEngine
        engine = BioGenesisEngine(num_agents=10)
        self.assertEqual(len(engine.agents), 10)
        engine.update(dt=0.1)
        self.assertGreaterEqual(engine.simulation_time, 0.1)

    def test_cultural_dna(self):
        from arkhe.biogenesis import BioGenesisEngine
        engine = BioGenesisEngine(num_agents=10)
        dna = engine.extract_cultural_dna(list(engine.agents.values()))
        self.assertEqual(len(dna), 4)

        engine2 = BioGenesisEngine(num_agents=0)
        engine2.add_agents(5, base_weights=dna)
        self.assertEqual(len(engine2.agents), 5)

if __name__ == "__main__":
    unittest.main()
