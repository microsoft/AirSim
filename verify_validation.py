
import numpy as np
from arkhe.simulation import MorphogeneticSimulation
from arkhe.hsi import HSI

def verify_validation():
    print("🧬 [Ω_VALID] VERIFICANDO GEOMETRIA POPULACIONAL (WAKHLOO 2026)...")

    hsi = HSI()
    sim = MorphogeneticSimulation(hsi)

    # 1. Verificar termos geométricos
    terms = sim.get_wakhloo_correspondence()
    assert terms["c"] == 0.86
    assert terms["PR"] == 63.0
    assert terms["f"] == 0.85
    assert terms["s"] == 6.67

    # 2. Verificar erro de generalização
    # Eg(p=70) ≈ 0.25 (Pico de coerência)
    # Eg(p=9034) ≈ 0.16 (Código ótimo)
    eg_70 = sim.calculate_generalization_error(70)
    eg_9034 = terms["Eg_p9034"]

    print(f"Eg(p=70):   {eg_70:.4f} (Alvo: ~0.25)")
    print(f"Eg(p=9034): {eg_9034:.4f} (Alvo: ~0.16)")

    assert 0.24 < eg_70 < 0.26
    assert 0.15 < eg_9034 < 0.17

    print("✅ VALIDAÇÃO GEOMÉTRICA COMPLETA: Correspondência isomórfica confirmada.")

if __name__ == "__main__":
    verify_validation()
