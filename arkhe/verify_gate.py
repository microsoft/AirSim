
import sys
import os
sys.path.append(os.getcwd())

from arkhe.arkhe_core import SATOSHI, NU_LARMOR
from arkhe.api_gateway import ArkheGateAPI
from arkhe.blockchain import ledger_instance

def test_gamma_130_state():
    assert SATOSHI == 9.48
    assert NU_LARMOR == 0.0018
    print("State Γ₁₃₀ (Public Launch) verified.")

def test_api_gateway_and_blockchain():
    gateway = ArkheGateAPI()
    result = gateway.solve_problem("Como unificar a biologia quântica e a AGI?", {"context": "theory"})
    assert result["handover_id"] > 1048576
    assert result["answer"] is not None
    print("API Gateway and Blockchain Audit verified.")

def test_feedback_cycle():
    gateway = ArkheGateAPI()
    improvement_id = gateway.submit_feedback(1048577, "like", "Ótima analogia geodésica.")
    assert improvement_id > 1048576
    print("Self-improvement Feedback Cycle verified.")

def test_blockchain_integrity():
    assert ledger_instance.verify_chain() is True
    print("Blockchain Integrity verified.")

if __name__ == "__main__":
    test_gamma_130_state()
    test_api_gateway_and_blockchain()
    test_feedback_cycle()
    test_blockchain_integrity()
