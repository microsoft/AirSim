import unittest
import numpy as np
from arkhe.ibc_bci import IBCBCI
from arkhe.pineal import PinealTransducer
from arkhe.ledger import NaturalEconomicsLedger
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI
from arkhe.som import SelfOrganizingHypergraph
from arkhe.hive import HiveMind
from arkhe.bioenergetics import MitochondrialFactory, NeuromelaninSink, TriadCircuit, PituitaryTransducer, NeuralCrest, Melanocyte

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

    def test_neural_crest_differentiation(self):
        crest = NeuralCrest()
        # Low omega -> Neuron
        self.assertEqual(crest.differentiate(0.05), "CENTRAL_NEURON")
        # High omega -> Melanocyte
        self.assertEqual(crest.differentiate(0.15), "PERIPHERAL_MELANOCYTE")

    def test_melanocyte_signaling(self):
        skin_cell = Melanocyte(omega=0.15)
        # Skin as interface
        psi_p = skin_cell.signal_peripheral_syzygy(1.5)
        self.assertGreater(psi_p, 0.0)
        self.assertAlmostEqual(psi_p, 0.94 * 0.86)

    def test_embodied_consciousness_formula(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        # Psi_total = Psi_neural + Psi_melanocitic
        total = sim.calculate_embodied_consciousness(0.94, 0.81)
        self.assertEqual(total, 1.75)

    def test_schumann_resonance_sync(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        boost = sim.simulate_schumann_resonance(7.83)
        self.assertAlmostEqual(boost, 7.27 * 0.99)

    def test_ledger_entries(self):
        ledger = NaturalEconomicsLedger()
        blocks = [e['block'] for e in ledger.entries]
        self.assertIn(9115, blocks) # Neural Crest
        self.assertIn(9138, blocks) # Embodied Sync
        self.assertIn(9140, blocks) # Completion
        self.assertIn(9155, blocks) # Intersubstrate Synthesis
        self.assertIn(9156, blocks) # Micro-test
        self.assertIn(9157, blocks) # CO2 Architecture

    def test_seven_shields_activation(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        self.assertTrue(sim.activate_seven_shields())
        self.assertTrue(sim.shields_active)

    def test_global_gradient_mapping(self):
        hsi = HSI(size=0.5)
        sim = MorphogeneticSimulation(hsi)
        status = sim.map_global_gradient()
        self.assertEqual(status, "GRADIENT_MAPPING_COMPLETE")
        self.assertEqual(sim.nodes_total, 12594)

if __name__ == "__main__":
    unittest.main()
