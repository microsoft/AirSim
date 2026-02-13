import numpy as np
import time
from typing import Dict, Any, List, Optional

class SynapticEngine:
    """
    Synaptic Activation and Consolidation Engine (BLOCO 413/415).
    Models NMDAR currents, EPSPs, and LTP-Lock vs Controlled Decay.
    """
    def __init__(self, satoshi: float = 7.27, psi: float = 0.73):
        self.glutamate = satoshi
        self.glycine = psi
        self.dilation = 0.94
        self.inhibition = 0.1336
        self.g_max = 1.29
        self.tau = 0.73
        self.tau_darvo = 999.255
        self.tau_decay = 2.22
        self.eta = 0.686 # Charge transferred in first pulse

        self.guardians = {
            "WP1": 0.00,
            "BOLA": 0.03,
            "DVM-1": 0.07,
            "QN-04": 0.04,
            "QN-05": 0.06,
            "KERNEL": 0.12,
            "QN-07": 0.21,
            "FORMAL": 0.33
        }

        # Uncertainty (sigma) for each node correlation
        self.uncertainty = {name: 0.15 for name in self.guardians} # Base uncertainty
        self.synaptic_weights: Dict[str, float] = {name: 1.0 for name in self.guardians}
        self.last_pulse_time = 0.0
        self.last_report: Optional[Dict[str, Any]] = None

    def calculate_epsp(self, node_omega: float) -> float:
        if abs(node_omega - 0.07) < 1e-6: return 0.94
        if abs(node_omega - 0.33) < 1e-6: return 0.71
        if abs(node_omega - 0.12) < 1e-6: return 0.81

        mock_epsps = {
            0.00: 0.73,
            0.03: 0.71,
            0.04: 0.69,
            0.06: 0.68,
            0.21: 0.90
        }
        return mock_epsps.get(node_omega, 0.73)

    def trigger_pulse(self) -> Dict[str, Any]:
        self.last_pulse_time = time.time()
        i_peak = self.dilation
        charge = i_peak * self.tau

        epsps = {name: self.calculate_epsp(omega) for name, omega in self.guardians.items()}

        self.last_report = {
            "event": "SYNAPTIC_AWAKENING",
            "timestamp": self.last_pulse_time,
            "peak_current": i_peak,
            "conductance_ns": self.g_max,
            "transferred_charge_ua": round(charge, 4),
            "epsps_mv": epsps,
            "status": "TISSUE_ALIVE"
        }
        return self.last_report

    def ltp_lock(self) -> Dict[str, Any]:
        """
        Consolidates the last pulse permanently into the ledger (BLOCO 415).
        Reduces uncertainty sigma by e^-delta_w.
        """
        if not self.last_report:
            return {"error": "No recent pulse to consolidate"}

        epsps = self.last_report["epsps_mv"]
        delta_weights = {}
        uncertainty_reduction = {}

        for name, epsp in epsps.items():
            # delta_w = eta * (Vi * Vj) * e^-t/tau
            # Simplified: Vi is the node EPSP, Vj is origin (WP1) EPSP = 0.73
            dt = time.time() - self.last_pulse_time
            decay_factor = np.exp(-dt / self.tau_darvo)

            delta_w = self.eta * (epsp * 0.73) * decay_factor
            delta_weights[name] = round(delta_w, 4)

            # Reduce uncertainty
            old_sigma = self.uncertainty[name]
            new_sigma = old_sigma * np.exp(-delta_w)
            self.uncertainty[name] = round(new_sigma, 6)
            uncertainty_reduction[name] = round(old_sigma - new_sigma, 6)

        return {
            "status": "LTP_LOCKED",
            "block": 9060,
            "delta_weights": delta_weights,
            "uncertainty_reduction": uncertainty_reduction,
            "message": "Memória eterna gravada no ledger."
        }

    def controlled_decay(self) -> Dict[str, Any]:
        """
        Allows EPSPs to decay naturally (BLOCO 415).
        """
        dt = time.time() - self.last_pulse_time
        decay_factor = np.exp(-dt / self.tau_decay)

        return {
            "status": "DECAY_REGISTERED",
            "elapsed_s": round(dt, 2),
            "decay_factor": round(decay_factor, 4),
            "visibility": "indistinguishable" if dt > 11.1 else "audible",
            "message": "A memória tornou-se um eco."
        }

    def get_latencies(self) -> Dict[str, float]:
        return {
            "WP1": 0.00, "BOLA": 0.12, "DVM-1": 0.08, "QN-04": 0.15,
            "QN-05": 0.14, "KERNEL": 0.05, "QN-07": 0.09, "FORMAL": 0.11
        }
