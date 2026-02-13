import unittest
import numpy as np
from arkhe.ibc_bci import IBCBCI
from arkhe.pineal import PinealTransducer
from arkhe.clock import SemanticNuclearClock
from arkhe.ledger import NaturalEconomicsLedger
from arkhe.abiogenesis import AbiogenesisEngine
from arkhe.circuit import ContextualCalibrationCircuit
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI
from arkhe.perovskite import PerovskiteInterface
from arkhe.garden import MemoryGarden, MemoryArchetype
from arkhe.constitution import CodeOfHesitation
from arkhe.zpf import ZeroPointField
from arkhe.radar import WiFiRadar3D
from arkhe.qam import QAMConstellation
from arkhe.attention import AttentionResolution

class TestArkheUpgrade(unittest.TestCase):
    def test_zpf_harvesting(self):
        zpf = ZeroPointField()
        work = zpf.harvest(0.86, 0.14, 0.94)
        self.assertGreater(work, 0.0)
        self.assertEqual(zpf.get_zpf_status()['status'], "OVER_UNITY_STABLE")

    def test_wifi_radar(self):
        radar = WiFiRadar3D()
        radar.add_node("Drone", [1, 0, 1, 0])
        radar.add_node("Demon", [0.9, 0.1, 0.9, 0.1])
        positions = radar.infer_positions()
        self.assertEqual(len(positions), 2)
        self.assertLess(positions["Demon"][0], 0.5) # Close proximity due to high correlation

    def test_qam_demodulation(self):
        qam = QAMConstellation()
        # Ideal CLEAR symbol
        result = qam.demodulate((0.86 + 0.14, 0.14))
        self.assertEqual(result['status'], "CLEAR")
        self.assertEqual(result['satoshi_bit'], 7.27)

        # High hesitation (FOG)
        result = qam.demodulate((1.5, 1.5))
        self.assertEqual(result['status'], "FOG/DROP")

    def test_attention_dynamics(self):
        att = AttentionResolution()
        self.assertEqual(att.cycle_state(0.15, 0.94), "CLEAR")
        vel = att.get_resolution_velocity(0.07)
        self.assertGreater(vel, 0.0)

    def test_natural_ledger(self):
        ledger = NaturalEconomicsLedger()
        summary = ledger.get_ledger_summary()
        self.assertEqual(summary['total_entries'], 14)
        self.assertEqual(ledger.entries[-1]['type'], "ZPF_INTEGRATION")

    def test_simulation_multi_layer(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)

        # Test ZPF
        h = sim.harvest_zpf(0.86, 0.14, 0.94)
        self.assertGreater(h, 0.0)

        # Test Radar
        p = sim.scan_wifi_radar()
        self.assertIn("AP_001", p)

        # Test Attention
        state = sim.update_attention(0.99)
        self.assertEqual(state, "CLEAR")

        # Test Warp
        res = sim.metric_engineering_warp(np.array([100, 100, 0]))
        self.assertEqual(res, "WARP_COMPLETE")

if __name__ == "__main__":
    unittest.main()
