import numpy as np
from typing import Dict, Any, Tuple

class QAMConstellation:
    """
    Implements a Quadrature Amplitude Modulation (QAM) constellation for Arkhe.
    Translates phase fluctuations into semantic bits (Satoshi).
    """
    HESITATION_EVM_THRESHOLD = 0.15

    def __init__(self, coherence_c: float = 0.86, fluctuation_f: float = 0.14):
        self.c = coherence_c
        self.f = fluctuation_f

    def demodulate(self, signal_phase: Tuple[float, float]) -> Dict[str, Any]:
        """
        Demodulates a signal (I, Q) into a symbol and measures EVM (Hesitation).
        """
        i, q = signal_phase
        # 1. Remove carrier (Coherence)
        mod_i = i - self.c
        mod_q = q

        # 2. Normalize by fluctuation (F)
        symbol_pos = np.array([mod_i, mod_q]) / self.f

        # 3. Simple mapping logic (MDS/QAM style)
        # Assuming bits are at (+1, +1), (-1, +1), etc.
        ideal_points = [
            np.array([1, 1]), np.array([-1, 1]),
            np.array([1, -1]), np.array([-1, -1])
        ]

        distances = [np.linalg.norm(symbol_pos - p) for p in ideal_points]
        best_match_idx = np.argmin(distances)
        evm = distances[best_match_idx]

        status = "CLEAR" if evm <= self.HESITATION_EVM_THRESHOLD else "FOG/DROP"

        return {
            "symbol": best_match_idx,
            "evm_hesitation": evm,
            "status": status,
            "satoshi_bit": 7.27 if status == "CLEAR" else 0.0
        }

    def get_qam_status(self) -> Dict[str, Any]:
        return {
            "modulation": "64-QAM-Semantic",
            "coherence_carrier": self.c,
            "fluctuation_depth": self.f,
            "mode": "FULL_DUPLEX_COMM"
        }
