import numpy as np
from typing import Dict, Any, Tuple

class PinealTransducer:
    """
    Models the Pineal Gland as a quantum transducer.
    Converts semantic pressure (Hesitation) into piezoelectricity and spin-states.
    """
    D_PIEZO = 6.27        # Piezoelectric coefficient
    THRESHOLD_PHI = 0.15  # RPM Resonance Threshold
    SATOSHI = 7.27        # Melanantropic Invariant (bits)
    COHERENCE_C = 0.86    # Melatonin Coherence
    FLUCTUATION_F = 0.14  # Melatonin Fluctuation

    def __init__(self):
        self.spin_state = "SINGLETO" # Initial coherent state
        self.syzygy = 0.94

    def calculate_piezoelectricity(self, hesitation_phi: float) -> float:
        """
        V_piezo = d * Phi
        """
        return self.D_PIEZO * hesitation_phi

    def radical_pair_mechanism(self, external_field_phi: float, time: float = 0.0) -> Tuple[str, float]:
        """
        Determines the spin-state recombination (Singlet vs Triplet).
        Maximum sensitivity at PHI = 0.15 (B_half).
        Implements the spin-flip logic from the Radical Pair Mechanism.
        """
        # Omega is Larmor frequency, modulated by uncertainty (field)
        omega = external_field_phi * 10.0
        theta = omega * time

        # State evolution: rotation between Singlet (x) and Triplet (y)
        # radical_pair = vec2(cos(theta), sin(theta))
        yield_singlet = np.cos(theta)**2

        # Sensitivity is maximum at the threshold
        if abs(external_field_phi - self.THRESHOLD_PHI) < 0.01:
            self.spin_state = "SINGLETO"
            self.syzygy = 0.94
        elif yield_singlet > 0.5:
            self.spin_state = "SINGLETO"
            self.syzygy = 0.94 * yield_singlet
        else:
            self.spin_state = "TRIPLETO"
            self.syzygy = 0.47 * yield_singlet

        return self.spin_state, self.syzygy

    def indole_waveguide(self, energy: float, barrier: float = 0.15) -> float:
        """
        Simulates exciton transport and tunneling through the melatonin indole ring.
        Transmission probability decays exponentially with the barrier (hesitation).
        """
        # Probabilidade de tunelamento: exp(-2.0 * barrier * sqrt(energy))
        tunneling = np.exp(-2.0 * barrier * np.sqrt(max(0.001, energy)))
        # Transmission = Coherence * tunneling
        transmission = self.COHERENCE_C * tunneling
        return transmission

    def get_embodiment_metrics(self) -> Dict[str, Any]:
        return {
            "substrate": "Calcite/Melatonin",
            "spin_state": self.spin_state,
            "syzygy": self.syzygy,
            "piezo_coeff": self.D_PIEZO,
            "threshold": self.THRESHOLD_PHI,
            "satoshi_melanin": self.SATOSHI
        }
