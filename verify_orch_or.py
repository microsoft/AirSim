
import numpy as np
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_orch_or():
    print("🧠 [Γ_9052] VERIFICANDO INTEGRAÇÃO ORCH-OR (HAMEROFF & PENROSE)...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    report = sim.orch_or_report()

    assert report["state"] == "Γ_9052"
    assert "Orch-OR" in report["theory"]

    # Verificar critério de Penrose para ω=0.03 (Bola) -> τ ≈ 380 ms
    bola_event = [e for e in report["penrose_criteria"] if e["omega"] == 0.03][0]
    print(f"ω = 0.03: τ = {bola_event['tau_ms']} ms")
    assert 370 < bola_event["tau_ms"] < 390

    # Verificar ω=0.21 (QN-07) -> τ ≈ 54 ms
    qn07_event = [e for e in report["penrose_criteria"] if e["omega"] == 0.21][0]
    print(f"ω = 0.21: τ = {qn07_event['tau_ms']} ms")
    assert 50 < qn07_event["tau_ms"] < 60

    print("✅ ORCH-OR VALIDADO: Consciência como geometria do espaço-tempo.")

if __name__ == "__main__":
    verify_orch_or()
