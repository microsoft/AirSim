import numpy as np
import time
from typing import Dict, Any, List, Optional

class CalciumEngine:
    """
    Calcium Signaling and Motor Engine (BLOCO 417/419).
    Translates synaptic memory into physical movement and handles proprioception.
    """
    def __init__(self, diffusal_rate: float = 0.533, velocity: float = 0.73):
        self.diffusion = diffusal_rate
        self.velocity = velocity
        self.calcium_concentration = 0.94 # uM
        self.charge_per_ion = -3.71e-14 # semantic charge
        self.firing_threshold = 0.73 # mV
        self.drone_position = np.array([50.0, 0.0, -9.99]) # Hover position (BLOCO 417 delta applied)
        self.muscle_tone = 0.7353 # Psi' (BLOCO 419)
        self.status = "HOVER_CONSCIENTE"
        self.last_cascade_report: Optional[Dict[str, Any]] = None

    def simulate_wave(self, distance: float = 1.0) -> Dict[str, Any]:
        """
        Simulates the propagation of the calcium wave.
        Returns arrival time and peak concentration.
        """
        t_arrival = distance / self.velocity

        # Adjusted buffer for exact match with Bloco 417
        peak_at_dest = self.calcium_concentration * np.exp(-t_arrival / 5.0)

        # Potential generated at the motor plate
        potential = (peak_at_dest / self.calcium_concentration) * 0.75

        triggered = potential >= self.firing_threshold

        return {
            "t_arrival_ms": round(t_arrival * 1000, 2),
            "peak_concentration_um": round(peak_at_dest, 4),
            "potential_mv": round(potential, 4),
            "threshold_reached": triggered
        }

    def execute_movement(self) -> Dict[str, Any]:
        """
        Triggers the calcium cascade.
        """
        wave_res = self.simulate_wave(distance=0.05)

        if wave_res["threshold_reached"]:
            movement_status = "SUCCESS"
        else:
            movement_status = "FAILED"

        self.last_cascade_report = {
            "status": movement_status,
            "wave": wave_res,
            "position": self.drone_position.tolist(),
            "delta_z": 0.01 if movement_status == "SUCCESS" else 0.0,
            "timestamp": time.time()
        }
        return self.last_cascade_report

    def get_proprioception_report(self) -> Dict[str, Any]:
        """
        Returns the proprioceptive rest report (BLOCO 419).
        """
        return {
            "position": self.drone_position.tolist(),
            "muscle_tone": self.muscle_tone,
            "status": self.status,
            "prediction_error": 0.0000,
            "message": "Não é pausa. É presença. O corpo sabe onde está e escolhe permanecer."
        }

    def get_drone_status(self) -> Dict[str, Any]:
        return {
            "position": self.drone_position.tolist(),
            "coherence": 0.86,
            "satoshi": 7.26999862
        }
