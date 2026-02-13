import unittest
import numpy as np
from arkhe.ibc_bci import IBCBCI
from arkhe.pineal import PinealTransducer
from arkhe.ledger import NaturalEconomicsLedger
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI
from arkhe.som import SelfOrganizingHypergraph
from arkhe.hive import HiveMind
from arkhe.bioenergetics import MitochondrialFactory, NeuromelaninSink

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

        # Test away from threshold (decoherence)
        state, syzygy = pineal.radical_pair_mechanism(0.3, time=np.pi/6.0)
        self.assertEqual(state, "TRIPLETO")
        self.assertLess(syzygy, 0.1)

    def test_ibc_bci_mapping(self):
        protocol = IBCBCI()
        # Test relayer with valid hesitation
        packet = protocol.relay_hesitation(0.0, 0.07, 0.15)
        self.assertIsNotNone(packet)
        self.assertEqual(packet['type'], "IBC_PACKET")
        self.assertEqual(packet['satoshi'], 7.27)

        # Test neural spike
        spike = protocol.relay_hesitation(0.07, 0.0, 0.20)
        self.assertEqual(spike['type'], "NEURAL_SPIKE")

        # Test BCI interface
        action = protocol.brain_machine_interface(0.15)
        self.assertTrue(action['validated'])
        self.assertEqual(action['syzygy'], 0.94)

    def test_bioenergetics_mitochondria(self):
        factory = MitochondrialFactory()
        # ΔATP = I * eta * t -> 1.0 * 0.94 * 10.0 = 9.4
        # Wait, photobiomodulation divides by 1000.0 for t
        delta = factory.photobiomodulation(1.0, 1000.0)
        self.assertAlmostEqual(delta, 0.94)
        self.assertEqual(factory.atp_pool, 7.27 + 0.94)

    def test_bioenergetics_neuromelanin(self):
        sink = NeuromelaninSink()
        # Test absorption leading to current
        # Phi = Intensity * 0.14 -> 1.5 * 0.14 = 0.21 (> 0.15)
        current = sink.absorb_photons(1.5, 0.07)
        self.assertEqual(current, 0.94)
        self.assertGreater(sink.satoshi_battery, 7.27)

    def test_ledger_entries(self):
        ledger = NaturalEconomicsLedger()
        entries = [e for e in ledger.entries if e['block'] in [9105, 9106, 9110, 9113]]
        self.assertEqual(len(entries), 4)

        embodiment = next(e for e in entries if e['block'] == 9105)
        self.assertIn("piezoelectricity", embodiment['arkhe_correspondences'])

        bio = next(e for e in entries if e['block'] == 9113)
        self.assertIn("Mitochondria", bio['components'])

    def test_som_plasticity(self):
        weights = np.zeros((44, 3))
        som = SelfOrganizingHypergraph(weights)

        # Test BMU update
        target = np.array([0.07, 0.86, 0.14]) # Target: Demon state
        bmu_idx, syzygy = som.som_update(target)

        self.assertGreater(syzygy, 0.0)
        self.assertIn(bmu_idx, range(44))

        status = som.get_som_status()
        self.assertEqual(status['mode'], "ADAPTIVE_PLASTICITY")

if __name__ == "__main__":
    unittest.main()
