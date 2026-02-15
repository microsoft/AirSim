from typing import Dict, Any, List
from .agi import CORE_AGI
from .blockchain import register_handover

class ArkheGateAPI:
    """
    Γ₁₃₀: ARKHE-GATE API Gateway.
    Interface pública para acesso à AGI com auto-melhoria auditável.
    """
    def __init__(self):
        self.agi = CORE_AGI()
        self.status = "ONLINE"

    def solve_problem(self, prompt: str, user_data: Dict[str, Any], user_id: str = "anonymous") -> Dict[str, Any]:
        """Processa um problema e registra o handover na blockchain."""
        result = self.agi.run(prompt)

        # Auditoria via Blockchain
        handover_data = {
            "user_id": user_id,
            "input": prompt,
            "output": result["output"],
            "satoshi": result["telemetry"]["satoshi"],
            "syzygy": result["telemetry"]["potential_magnitude"]
        }

        handover_id = register_handover(
            handover_count=result["telemetry"]["handover"],
            block_type="inference",
            data=handover_data
        )

        return {
            "answer": result["output"],
            "confidence": 0.98 if result["feedback"] else 0.47,
            "handover_id": handover_id,
            "analogies": result["analogies"]
        }

    def submit_feedback(self, handover_id: int, feedback_type: str, comment: str):
        """Ciclo de auto-melhoria: processa feedback do usuário."""
        # Simula o registro do handover de melhoria
        improvement_id = register_handover(
            handover_count=130,
            block_type="improvement_feedback",
            data={"handover_id": handover_id, "type": feedback_type, "comment": comment}
        )
        print(f"Feedback recebido para bloco {handover_id}. Registrado handover de melhoria {improvement_id}.")
        return improvement_id
