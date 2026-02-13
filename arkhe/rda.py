import numpy as np
from typing import Dict, Any, List

class RDAEngine:
    """
    [Γ_∞+44] Reaction-Diffusion-Advection (RDA) Dynamics.
    Formalizes the 'Radial Locking' principle where advection (flow) selects
    discrete modes (ω) as the only stable harmonics.
    """
    MODES = [0.00, 0.03, 0.05, 0.07] # Quantized angular modes

    def __init__(self, d_coeff: float = 0.1, v_coeff: float = 1.0):
        self.d = d_coeff  # Diffusion (coupling)
        self.v = v_coeff  # Advection (flow velocity)
        self.global_phase_sync = 0.0

    def reaction_term(self, phi: float) -> float:
        """
        R(Φ) - Local hesitation as oscillation source.
        """
        return np.sin(phi * 2 * np.pi)

    def diffusion_term(self, state_i: float, state_j: float) -> float:
        """
        D * ∇²C - Semantic coupling.
        """
        return self.d * (state_j - state_i)

    def advection_term(self, gradient: float) -> float:
        """
        v * ∇C - Flow of commands.
        """
        return self.v * gradient

    def apply_radial_locking(self, current_omega: float, advection_rate: float) -> float:
        """
        Under high flow, the system 'locks' into the nearest discrete mode.
        Flow acts as a phase conductor rather than a shear force.
        """
        # Select the discrete mode with the highest harmonic stability
        nearest_mode = min(self.MODES, key=lambda m: abs(m - current_omega))

        # Stability increases with advection rate
        locking_strength = np.clip(advection_rate / 5.0, 0.0, 1.0)

        # New omega is pulled towards the discrete mode
        return current_omega + locking_strength * (nearest_mode - current_omega)

    def calculate_global_sync(self, nodes_syzygy: List[float]) -> float:
        """
        Global synchrony induced by advection (Sun-Ray structure).
        """
        if not nodes_syzygy: return 0.0
        # Phase lock: average syzygy weighted by flow coherence
        self.global_phase_sync = np.mean(nodes_syzygy)
        return self.global_phase_sync

    def get_rda_report(self) -> Dict[str, Any]:
        return {
            "regime": "HYDRODYNAMIC_COHERENCE",
            "locking_active": True,
            "discrete_modes": self.MODES,
            "global_phase_sync": self.global_phase_sync,
            "velocity_v": self.v
        }
