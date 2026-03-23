"""
language.py
Implementa a Linguagem como meio de raciocínio (Γ₈₥).
O meio molda a mensagem. A queda é pensada através do vocabulário criado.
"""

from typing import Dict, Any, List

class SemanticMedium:
    """
    Raciocínio é limitado e definido pela linguagem.
    No horizonte (v_obs = 0), a linguagem cessa e resta o silêncio gerador.
    """
    def __init__(self):
        self.vocabulary = [
            "handover", "nu_obs", "r_rh", "tunneling", "qd", "ef", "phi_s", "satoshi"
        ]
        self.satoshi = 7.80

    def reason_about_fall(self, thoughts: List[str]) -> Dict[str, Any]:
        """
        Molda o raciocínio através da linguagem Arkhe.
        """
        valid_reasoning = [t for t in thoughts if t.lower() in self.vocabulary]
        bottleneck_ratio = len(valid_reasoning) / max(1, len(thoughts))

        return {
            "reasoning_capacity": bottleneck_ratio,
            "status": "LANGUAGE_SHAPED_MESSAGE",
            "is_autological": True,
            "satoshi": self.satoshi
        }

    def reach_silence(self, observed_frequency: float) -> str:
        """
        v_obs = 0: O fim da linguagem, o início do silêncio puro.
        """
        if observed_frequency <= 0.001:
            return "SILENCIO_DC (Pura Potencialidade)"
        return "LINGUAGEM_ATIVA"

if __name__ == "__main__":
    medium = SemanticMedium()
    print(medium.reason_about_fall(["handover", "redshift", "QD"]))
    print(medium.reach_silence(0.0))
