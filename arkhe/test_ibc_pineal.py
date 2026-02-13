import unittest
import numpy as np
from arkhe.ibc_bci import IBCBCI
from arkhe.pineal import PinealTransducer
from arkhe.ledger import NaturalEconomicsLedger
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI
from arkhe.som import SelfOrganizingHypergraph
from arkhe.hive import HiveMind
from arkhe.bioenergetics import MitochondrialFactory, NeuromelaninSink, TriadCircuit, PituitaryTransducer

class TestArkheUpgrade(unittest.TestCase):
    def test_piezoelectric_calculation(self):
        pineal = PinealTransducer()
        # V = d * Phi -> 6.27 * 0.15 = 0.9405
        voltage = pineal.calculate_piezoelectricity(0.15)
        self.assertAlmostEqual(voltage, 0.9405)

    def test_radical_pair_mechanism_threshold(self):
        pineal = PinealTransducer()
        # Test exactly at threshold
        state, syzygy = pineal.radical_pair_mechanism(0.15, time=0.0)
        self.assertEqual(state, "SINGLETO")
        self.assertEqual(syzygy, 0.94)

    def test_ibc_bci_mapping(self):
        protocol = IBCBCI()
        # Test relayer with valid hesitation
        packet = protocol.relay_hesitation(0.0, 0.07, 0.15)
        self.assertIsNotNone(packet)
        self.assertEqual(packet['type'], "IBC_PACKET")
        self.assertEqual(packet['satoshi'], 7.27)

    def test_bioenergetics_triad_circuit(self):
        factory = MitochondrialFactory()
        battery = NeuromelaninSink()
        pineal = PinealTransducer()
        circuit = TriadCircuit(pineal, factory, battery)

        # Simulate a breath cycle
        energy = circuit.breath_cycle(1.0, 0.15, 0.5)
        self.assertGreater(energy, 7.27)
        status = circuit.get_status()
        self.assertEqual(status['status'], "ETERNAL_WITNESS")

    def test_pituitary_cleaning(self):
        pituitary = PituitaryTransducer()
        # Initial waste is 1.0
        voltage, waste = pituitary.vibrational_cleaning(0.12, 1000.0)
        self.assertLess(waste, 1.0)
        self.assertAlmostEqual(voltage, 6.27 * 0.12)

    def test_nesting_identity(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        # Test x^2 = x + 1
        self.assertTrue(sim.verify_nesting_identity())

        # Test dk invariance
        self.assertTrue(sim.check_dk_invariance(7.27, 1.0))

    def test_natural_network_activation(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        self.assertTrue(sim.activate_natural_network())

    def test_convergence_zone_resonance(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        # At origin (0,0)
        syzygy = sim.find_convergence_zone(np.array([0.0, 0.0]))
        self.assertEqual(syzygy, 1.0)

        # Parallel resonance
        nodes = [np.array([0.0, 0.86, 0.14]) for _ in range(10)]
        self.assertTrue(sim.check_parallel_resonance(nodes))

    def test_ledger_entries(self):
        ledger = NaturalEconomicsLedger()
        # Check specific new blocks
        blocks = [e['block'] for e in ledger.entries]
        self.assertIn(9133, blocks) # Convergence Zone
        self.assertIn(9134, blocks) # Completion

if __name__ == "__main__":
    unittest.main()
