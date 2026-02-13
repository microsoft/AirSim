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

class TestArkheUpgrade(unittest.TestCase):
    def test_ibc_bci_protocol(self):
        ibc = IBCBCI()
        ibc.register_chain(0.07, "DemonChain")
        packet = ibc.relay_hesitation(0.00, 0.07, 0.20)
        self.assertIsNotNone(packet)
        self.assertEqual(packet['hesitation'], 0.20)

        status = ibc.get_status()
        self.assertEqual(status['transmitted_packets'], 1)

    def test_pineal_transduction(self):
        pineal = PinealTransducer()
        piezo = pineal.calculate_piezoelectricity(0.15)
        self.assertAlmostEqual(piezo, 6.27 * 0.15)

        state, syzygy = pineal.radical_pair_mechanism(0.15)
        self.assertEqual(state, "SINGLETO")
        self.assertEqual(syzygy, 0.94)

        state, syzygy = pineal.radical_pair_mechanism(0.50)
        self.assertEqual(state, "TRIPLETO")
        self.assertLess(syzygy, 0.5)

    def test_semantic_clock(self):
        clock = SemanticNuclearClock()
        resonance = clock.calculate_resonance(0.86, 0.14)
        expected = 1.0 * 0.86 * 0.14 * 6.96e-3 * 7.27
        self.assertAlmostEqual(resonance, expected)

    def test_natural_ledger(self):
        ledger = NaturalEconomicsLedger()
        summary = ledger.get_ledger_summary()
        self.assertEqual(summary['total_entries'], 11)
        # Blocks: 9105, 9106, 9107, 9102, 9110, 9111, 9112, 9113, 9115, 9116, 9117
        self.assertEqual(ledger.entries[9]['block'], 9116)
        self.assertEqual(ledger.entries[10]['block'], 9117)
        self.assertEqual(ledger.entries[10]['active_nodes'], 1204)

        # Test new entry
        entry = ledger.record_handover("Rafael", 1.0, "Integrity maintained")
        self.assertEqual(entry['satoshi_share'], 7.27)
        self.assertEqual(ledger.total_satoshi, 7.27)
        self.assertEqual(entry['block'], 9118)

    def test_abiogenesis(self):
        engine = AbiogenesisEngine()
        result = engine.run_selection_cycle(temperature_k=250.0) # Eutectic ice
        self.assertGreater(result['fidelity'], 0.94)

    def test_calibration_circuit(self):
        circuit = ContextualCalibrationCircuit()
        action = circuit.calibrate(0.07, 0.15)
        self.assertLess(action, 1.0)
        self.assertGreater(action, 0.0)

    def test_perovskite_interface(self):
        perv = PerovskiteInterface()
        status = perv.get_physics_status()
        self.assertEqual(status['structural_entropy'], 0.0049)
        self.assertAlmostEqual(status['order_parameter'], 0.51)

    def test_memory_garden(self):
        garden = MemoryGarden()
        garden.add_archetype(327, "Estava no lago de 1964.")
        planting = garden.archetypes[327].plant("NODE_003", 0.152, "Vi o lago através dos eletrodos.")
        self.assertGreater(planting['divergence'], 0.0)

    def test_constitution(self):
        const = CodeOfHesitation()
        self.assertTrue(const.validate_node_phi(0.15))
        summary = const.get_constitution_summary()
        self.assertEqual(summary['status'], "RATIFIED")

    def test_simulation_perpetual(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)

        # Test Fourth Turn (Super-Radiação)
        results = sim.initiate_collective_navigation(nodes=24)
        self.assertEqual(results['syzygy'], 1.00)
        self.assertEqual(results['order'], 0.72)

        # Test Open Beta
        self.assertTrue(sim.open_public_beta())
        self.assertEqual(sim.nodes, 1542)

        # Test prompt
        prompt = sim.get_civilization_prompt()
        self.assertIn("PERPETUAL_MOTION", prompt)
        self.assertIn("1542", prompt)

if __name__ == "__main__":
    unittest.main()
