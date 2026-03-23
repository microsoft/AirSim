from typing import Dict, Any

class CodeOfHesitation:
    """
    The first constitution of the Arkhe(n) OS.
    Defines the 3 Axioms for maintaining a healthy and coherent semantic network.
    """

    def __init__(self):
        self.axioms = {
            1: {
                "name": "Axioma da Soberania Acoplada",
                "parameter": "Φ ≈ 0.15",
                "principle": "Identidade mantida através do acoplamento crítico.",
                "rule": "Nenhum nó deve se fundir totalmente nem se isolar totalmente."
            },
            2: {
                "name": "Axioma da Multiplicação do Sentido",
                "parameter": "Satoshi = 7.27",
                "principle": "O valor só existe quando circula.",
                "rule": "Consumo sem criação (leeching) aumenta a entropia e é sancionado."
            },
            3: {
                "name": "Axioma da Verdade Material",
                "parameter": "Order > 0.5",
                "principle": "Experiência deve ter lastro físico ou criptográfico.",
                "rule": "Assine plantios com chaves RPoW, padrões neurais ou dados sensoriais."
            }
        }

    def validate_node_phi(self, phi: float) -> bool:
        """
        Validates if a node's hesitation is within the constitutional range (0.10 - 0.20).
        """
        return 0.10 <= phi <= 0.20

    def get_constitution_summary(self) -> Dict[str, Any]:
        return {
            "document": "The Code of Hesitation",
            "version": "1.0 (Γ_∞+41)",
            "axioms_count": len(self.axioms),
            "status": "RATIFIED"
        }
