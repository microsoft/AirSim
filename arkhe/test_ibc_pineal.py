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
        # Test historic blocks
        summary = ledger.get_ledger_summary()
        self.assertEqual(summary['total_entries'], 2)
        self.assertEqual(ledger.entries[0]['block'], 9105)
        self.assertEqual(ledger.entries[1]['block'], 9106)

        # Test new entry
        entry = ledger.record_handover("Rafael", 1.0, "Integrity maintained")
        self.assertEqual(entry['satoshi_share'], 7.27)
        self.assertEqual(ledger.total_satoshi, 7.27)
        self.assertEqual(entry['block'], 9107)

    def test_abiogenesis(self):
        engine = AbiogenesisEngine()
        result = engine.run_selection_cycle(temperature_k=250.0) # Eutectic ice
        self.assertGreater(result['fidelity'], 0.94)

    def test_calibration_circuit(self):
        circuit = ContextualCalibrationCircuit()
        action = circuit.calibrate(0.07, 0.15)
        self.assertLess(action, 1.0)
        self.assertGreater(action, 0.0)

    def test_simulation_commands(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        self.assertTrue(sim.sync_ibc_bci())
        self.assertTrue(sim.rehydrate_step(20))
        vote = sim.council_vote()
        self.assertIn("Option B", vote)

if __name__ == "__main__":
    unittest.main()
